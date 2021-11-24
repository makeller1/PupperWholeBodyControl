#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <CommandInterpreter.h>
#include <Streaming.h>
#include "DriveSystem.h"
#include "Utils.h"
#include "ICM_20948.h"
#include "Calibration.h"
#include <EEPROM.h>

const int CONTROL_DELAY = 1000;//1000;  // micros - mathew (Setting this to 10 results in CAN error)
const int OBSERVE_DELAY = 1000;//1000; // micros
const float MAX_CURRENT = 1.0; // Amps - Default saturation value, is changed by run_djipupper

const bool send_robot_states = true; // This is needed to send motor / IMU data over serial - mathew
const bool print_cal_params = false; // Used to print the calibration parameters stored in persistent mem
bool print_debug_info = false; 

const bool ECHO_COMMANDS = false; // Set true for debugging
const bool AHRS_ENABLED = true; // Are we running with the AHRS/IMU enabled?

// Calibration parameters
float b_gyr_x = 0.0f;
float b_gyr_y = 0.0f;
float b_gyr_z = 0.0f;
float b_mag_x = 0.0f;
float b_mag_y = 0.0f;
float b_mag_z = 0.0f;
float s_mag_x = 0.0f;
float s_mag_y = 0.0f;
float s_mag_z = 0.0f;

////////////////////// SETUP IMU ////////////////////////
#define WIRE_PORT Wire
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0
ICM_20948_I2C myICM;  //create an ICM_20948_I2C object

// Quaternions
std::array<float,4> quats = {1.0f, 0.0f, 0.0f, 0.0f};

float lastUpdate = 0.0f;
float now = 0.0f;
float dt = 0.0f;
bool debug = false;

std::array<float, 4> MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, 
                    float mx, float my, float mz, float q0, float q1, float q2, float q3, float dt);

////////////////////// END SETUP ///////////////////////

DriveSystem drive;

const uint32_t kLogSize = 1000;
const uint32_t kNumAttributes = 7 * 12 + 1;

// Example json message with default start and stop characters: <{"kp":2.0}>
// use_msgpack: true, use default arguments for the rest
CommandInterpreter interpreter(true);
DrivePrintOptions options;

long last_command_ts;
long last_print_ts;
long last_cal_print_ts;
long last_header_ts;

void setup(void) {
  Serial.begin(115200);
  pinMode(13, OUTPUT);

  // Wait 5 seconds before turning on. This allows the motors to boot up.
  for (int i = 0; i < 20; i++) {
    digitalWrite(13, HIGH);
    delay(125);
    digitalWrite(13, LOW);
    delay(125);
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
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    bool initialized = false;
    while( !initialized )
    {
      myICM.begin( WIRE_PORT, AD0_VAL );

      if( myICM.status != ICM_20948_Stat_Ok )
      {
        delay(500);
        Serial << "ICM not working" << endl;
      }else
      {
        initialized = true;
      }
    }
    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
    myDLPcfg.a = acc_d5bw7_n8bw3; // -3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
    myDLPcfg.g = gyr_d361bw4_n376bw5; // -3 db bandwidth is 361.4 hz and nyquist bandwidth is 376.5 hz
                                            
    myICM.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    if( myICM.status != ICM_20948_Stat_Ok)
    {
      // Serial.print(F("setDLPcfg returned: "));
      Serial.println(myICM.statusString());
    }
    // Enable DLPF
    myICM.enableDLPF( ICM_20948_Internal_Acc, true );
    myICM.enableDLPF( ICM_20948_Internal_Gyr, true );

    //////////////////////////////// END IMU SETUP ///////////////////////////////////////////
  }

  last_command_ts = micros();
  last_print_ts = millis();
  last_header_ts = millis();

  // Runtime config
  drive.SetMaxCurrent(MAX_CURRENT);
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
  drive.CheckForCANMessages();
  CheckResult r = interpreter.CheckForMessages();
  if (r.flag == CheckResultFlag::kNewCommand) {
    if (r.new_torque) 
    {
      auto trqs = interpreter.LatestTorqueCommand();
      drive.SetTorques(Utils::ArrayToVector<12, 12>(trqs));
      if (ECHO_COMMANDS) 
      {
        Serial << "Torque command: "
               << interpreter.LatestTorqueCommand();
      }
    }
    if (r.trq_mode_set)
    {
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
  if (micros() - last_command_ts >= CONTROL_DELAY) 
  {
    // TODO: characterize maximum lag between control updates
    drive.Update();
    last_command_ts = micros();
  }

  // Filter IMU data and send to DriveSystem
  if (AHRS_ENABLED)
  {
    if(myICM.dataReady())
    {
      myICM.getAGMT();                // The values are only updated when you call 'getAGMT'

      // dt is throttled by the time it takes to recieve data from the IMU
      now = micros();
      dt = ((now - lastUpdate)/1000000.0f); 
      lastUpdate = now;

      // Update filter
      quats = MadgwickUpdate(myICM.gyrX()-b_gyr_x, myICM.gyrY()-b_gyr_y, myICM.gyrZ()-b_gyr_z, myICM.accX(), myICM.accY(), myICM.accZ(), 
                            (myICM.magX()-b_mag_x)*s_mag_x, -(myICM.magY()-b_mag_y)*s_mag_y, -(myICM.magZ()-b_mag_z)*s_mag_z,quats[0],quats[1],quats[2],quats[3], dt);

      // Send quaternion values to DriveSystem
      drive.SetQuaternions(quats[0],quats[1],quats[2],quats[3]);
    }
  }

  // Check parameters stored in EEPROM
  if (print_cal_params) 
  {
    std::array<float,9> calib_params = ReadParams();
    std::array<float,3> mag_data = {myICM.magX(), myICM.magY(), myICM.magZ()};
    float meanmag = CheckCalMag(mag_data, calib_params);
    if (micros() - last_cal_print_ts >= 1000000)
    {
      std::array<float,9> params = ReadParams();
      Serial << "Parameters: b_gyro: "<< params[0] << ", " << params[1] << ", " << params[2] << endl; 
      Serial << "Parameters: b_mag : "<< params[3] << ", " << params[4] << ", " << params[5] << endl;
      Serial << "Parameters: s_mag : "<< params[6] << ", " << params[7] << ", " << params[8] << endl; 
      Serial << "Normalized magnetometer magnitude: " << meanmag << endl;
      last_cal_print_ts = micros();
    }
  }

  // Send measurements to workstation
  if (send_robot_states){
    if (micros() - last_print_ts >= OBSERVE_DELAY) {
      drive.PrintMsgPackStatus(options);
      last_print_ts = micros();
    }
  }
}
