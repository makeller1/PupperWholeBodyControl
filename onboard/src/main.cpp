#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <CommandInterpreter.h>
#include <Streaming.h>
#include "DriveSystem.h"
#include "Utils.h"
#include "ICM_20948.h"
#include "Calibration.h"
#include <EEPROM.h>

const uint32_t CONTROL_DELAY = 500;//1000;  // micros - (soo small of a delay results in a CAN error)
const uint32_t OBSERVE_DELAY = 500;//1000; // micros
const uint32_t IMU_DELAY = 500;//1000; // micros

const float MAX_CURRENT_DEFAULT = 2.0; // Amps - Default saturation value, is changed by run_djipupper

const bool send_robot_states = true; // This is needed to send pupper states over serial - mathew
bool print_debug_info = false; 

const bool ECHO_COMMANDS = false; // Set true for debugging
const bool AHRS_ENABLED = true; // Are we running with the AHRS/IMU enabled?

// Calibration parameters
float b_gyr_x;
float b_gyr_y;
float b_gyr_z;
float b_mag_x;
float b_mag_y;
float b_mag_z;
float s_mag_x;
float s_mag_y;
float s_mag_z;

////////////////////// SETUP IMU ////////////////////////
#define USE_SPI       // Uncomment this to use SPI
#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 10     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0
#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif
// Quaternions
std::array<float,4> quats = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
float qDot1, qDot2, qDot3, qDot4; // w, x, y, z time derivative of quaternion

// TODO: Fix this madness below
std::array<float, 4> MadgwickUpdate(float gx, float gy, float gz, 
                                   float ax, float ay, float az, 
                                   float mx, float my, float mz, 
                                   float q0, float q1, float q2, float q3,
                                   float& qDot1, float& qDot2, float& qDot3,float& qDot4,
                                   float dt, float beta);

////////////////////// END SETUP ///////////////////////

DriveSystem drive;

const uint32_t kLogSize = 1000;
const uint32_t kNumAttributes = 7 * 12 + 1;

// Example json message with default start and stop characters: <{"kp":2.0}>
// use_msgpack: true, use default arguments for the rest
CommandInterpreter interpreter(true);
DrivePrintOptions options;

float dt = 0.0f;
uint32_t last_command_us;
uint32_t last_print_us;
uint32_t last_cal_print_us;
uint32_t last_imu_update_us;
uint32_t last_imu_read_us;

void setup(void) {
  Serial.begin(115200);
  pinMode(13, OUTPUT);

  // Wait before turning on. This allows the motors to boot up.
  for (int i = 0; i < 5; i++) {
    digitalWrite(13, HIGH);
    delay(125);
    digitalWrite(13, LOW);
    delay(125);
    Beep(i, 0, 2);
  }

  // Read calibration parameters from EEPROM
  std::array<float,9> calib_params = ReadParams();
  b_gyr_x = calib_params[0];
  b_gyr_y = calib_params[1];
  b_gyr_z = calib_params[2];
  b_mag_x = calib_params[3];
  b_mag_y = calib_params[4];
  b_mag_z = calib_params[5];
  s_mag_x = calib_params[6];
  s_mag_y = calib_params[7];
  s_mag_z = calib_params[8];

  if (AHRS_ENABLED){
    // Setup AHRS

    bool initialized = false;
    while( !initialized )
    {
      #ifdef USE_SPI
        SPI_PORT.begin();
        myICM.begin(CS_PIN, SPI_PORT);
      #else
        WIRE_PORT.begin();
        WIRE_PORT.setClock(400000);
        myICM.begin(WIRE_PORT, AD0_VAL);
      #endif

      if( myICM.status != ICM_20948_Stat_Ok )
      {
        delay(500);
        Serial << "ICM not working" << endl;
      }else
      {
        initialized = true;
      }
    }

    // Fast compute of initial quaternion by running filter with high gain on bootup
    uint32_t i = 0;
    uint32_t N = 5000;
    float beta_init = 10.0f;
    while (i < N){
      if(myICM.dataReady())
      {
        beta_init = 10.0f*( 1 - ((float)(i)/float(N)) ) + 0.1f;
        myICM.getAGMT();
        dt = (micros() - last_imu_update_us)/1000000.0f; 
        quats = MadgwickUpdate(myICM.gyrX()-b_gyr_x, myICM.gyrY()-b_gyr_y, myICM.gyrZ()-b_gyr_z, myICM.accX(), myICM.accY(), myICM.accZ(), 
                              (myICM.magX()-b_mag_x)*s_mag_x, -(myICM.magY()-b_mag_y)*s_mag_y, -(myICM.magZ()-b_mag_z)*s_mag_z,
                              quats[0], quats[1], quats[2], quats[3], qDot1, qDot2, qDot3, qDot4, dt, beta_init);
        last_imu_update_us = micros();
        i += 1;
      }
    }
    ////////////////////////////// END IMU SETUP ///////////////////////////////////////////
  }

  last_command_us = micros();
  last_print_us = micros();
  last_cal_print_us = micros();
  last_imu_update_us = micros();
  last_imu_read_us = micros();

  // Runtime config
  drive.SetMaxCurrent(MAX_CURRENT_DEFAULT);
  options.delimiter = ',';
  options.positions = true;
  options.velocities = true;
  options.currents = true; // last actual current
  if (!AHRS_ENABLED){
    options.quaternion = false;
  }

  interpreter.Flush();
  drive.SetTorqueControl(); 
  drive.ZeroCurrentPosition();
}

void loop() 
{
  drive.CheckForCANMessages(); // Messages from motor controllers
  CheckResult r = interpreter.CheckForMessages(); // Messages from Python workstation
  if (r.flag == CheckResultFlag::kNewCommand) {
    if (r.new_torque) 
    {
      auto trqs = interpreter.LatestTorqueCommand();
      drive.SetTorques(Utils::ArrayToVector<12, 12>(trqs));
    }
    if (r.trq_mode_set)
    {
      interpreter.Flush();
      // Set motors to idle
      BLA::Matrix<12> motor_currents;
      motor_currents.Fill(0);
      drive.SetTorques(motor_currents);
      // Start torque control mode
      drive.SetTorqueControl();
    }
    if (r.new_max_current) 
    {
      drive.SetMaxCurrent(interpreter.LatestMaxCurrent());
    }
    if (r.do_zero) 
    {
      drive.ZeroCurrentPosition();
      if (ECHO_COMMANDS) 
      {
        Serial << "Setting current position as the zero point" << endl;
      }
    }
    if (r.new_debug) 
    {
      print_debug_info = interpreter.LatestDebug();
    }
    if (r.calibrate)
    {
      std::array<float,6> gyro_mag_meas = {myICM.magX(), myICM.magY(), myICM.magZ(), myICM.gyrX(), myICM.gyrY(), myICM.gyrZ()};
      bool cal_complete = CalibrateIMU(gyro_mag_meas);

      if (cal_complete)
      {
        drive.SetIdle();
      }
      else
      {
        drive.SetCalibrationControl();
      }
    }
  }

  // Update control commands
  if (micros() - last_command_us >= CONTROL_DELAY) 
  {
    // TODO: characterize maximum lag between control updates
    drive.Update();
    last_command_us = micros();
  }

  // Filter IMU data and send to DriveSystem
  if (AHRS_ENABLED)
  {
    // Update filter
    dt = (micros() - last_imu_update_us)/1000000.0f; 
    quats = MadgwickUpdate(myICM.gyrX()-b_gyr_x, myICM.gyrY()-b_gyr_y, myICM.gyrZ()-b_gyr_z, myICM.accX(), myICM.accY(), myICM.accZ(), 
                              (myICM.magX()-b_mag_x)*s_mag_x, -(myICM.magY()-b_mag_y)*s_mag_y, -(myICM.magZ()-b_mag_z)*s_mag_z,
                              quats[0], quats[1], quats[2], quats[3], qDot1, qDot2, qDot3, qDot4, dt, 0.1f);

    // Send quaternion values to DriveSystem
    drive.SetQuaternions(quats[0],quats[1],quats[2],quats[3], qDot1, qDot2, qDot3, qDot4);
    last_imu_update_us = micros();
    if(myICM.dataReady())// && micros() - last_imu_read_us >= IMU_DELAY)
    {
      // Serial << "IMU time since last read (ms): " << (micros()-last_imu_read_us)*1E-3 << endl;
      Utils::tic();
      // Read IMU data (takes 0.98 ms with i2c, 0.09 ms with SPI)
      myICM.getAGMT();
      drive.ahrs_ms = Utils::toc();
      // Serial << "IMU time to read elapsed (ms): " << drive.ahrs_ms << endl;
      last_imu_read_us = micros();
    }
  }

  // Send measurements to workstation
  if (send_robot_states){
    if (micros() - last_print_us >= OBSERVE_DELAY) {
      drive.PrintMsgPackStatus(options);
      last_print_us = micros();
    }
  }
}
