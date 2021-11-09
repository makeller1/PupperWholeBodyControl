#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <CommandInterpreter.h>
#include <Streaming.h>

#include "DataLogger.h"
#include "DriveSystem.h"
#include "Utils.h"

#include "ICM_20948.h"

////////////////////// CONFIG ///////////////////////
const int PRINT_DELAY = 5;       // millis 20hz
const int HEADER_DELAY = 5000;   // millis
const int CONTROL_DELAY = 1000;//1000;  // micros - mathew (Setting this to 10 results in CAN error)
const int OBSERVE_DELAY = 1000;//1000; // micros
const float MAX_CURRENT = .5; // Amps - Default value, is changed by run_djipupper - mathew

bool print_robot_states = true; // This is needed to send motor / IMU data over serial - mathew

bool print_debug_info = false; 
bool print_header_periodically = false;

PDGains DEFAULT_GAINS = {8.0, 2.0};

const bool ECHO_COMMANDS = false; // false; Set true for debugging - mathew
////////////////////// END CONFIG ///////////////////////

//////////////////////SETUP IMU /////////////////////
#define WIRE_PORT Wire
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0
ICM_20948_I2C myICM;  //create an ICM_20948_I2C object

// constants
// magnetometer bias: corrected_i = measurement_i - bias_i (units: uT)
const float bx = 63.4891;
const float by = .2232;
const float bz = -15.35;
// Gyro at rest noise bias: corrected_i = measurement_i - bias_i (units: deg/s)
const float bgx = .2376;
const float bgy = -.0141;
const float bgz = .1368;

float lastUpdate = 0; // uint32_t
float now = 0; // uint32_t
float dt = 0.0f;
bool debug = false;


extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

// extern "C" void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, double udt);
// extern "C" void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, double udt);
void MadgwickKrisUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);

/////////////////////////


DriveSystem drive;

const uint32_t kLogSize = 1000;
const uint32_t kNumAttributes = 7 * 12 + 1;
DataLogger<kLogSize, kNumAttributes> logger;

// Example json message with default start and stop characters: <{"kp":2.0}>
// use_msgpack: true, use default arguments for the rest
CommandInterpreter interpreter(true);
DrivePrintOptions options;

long last_command_ts;
long last_print_ts;
long last_header_ts;

void setup(void) {
  Serial.begin(115200);
  pinMode(13, OUTPUT);

  //////////////////////////////// BEGIN IMU SETUP ///////////////////////////////////////////
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while( !initialized )
  {
    myICM.begin( WIRE_PORT, AD0_VAL );

    if( myICM.status != ICM_20948_Stat_Ok )
    {
      delay(500);
    }else
    {
      initialized = true;
    }
  }
  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d5bw7_n8bw3;         // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                          // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                          // acc_d111bw4_n136bw
                                          // acc_d50bw4_n68bw8  //Note: This was still susceptible to impacts
                                          // acc_d23bw9_n34bw4
                                          // acc_d11bw5_n17bw
                                          // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                          // acc_d473bw_n499bw
    myDLPcfg.g = gyr_d361bw4_n376bw5;       // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                          // gyr_d196bw6_n229bw8
                                          // gyr_d151bw8_n187bw6
                                          // gyr_d119bw5_n154bw3
                                          // gyr_d51bw2_n73bw3
                                          // gyr_d23bw9_n35bw9
                                          // gyr_d11bw6_n17bw8
                                          // gyr_d5bw7_n8bw9
                                          // gyr_d361bw4_n376bw5
                                          
  myICM.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
  if( myICM.status != ICM_20948_Stat_Ok)
  {
    // Serial.print(F("setDLPcfg returned: "));
    Serial.println(myICM.statusString());
  }
  // Choose whether or not to use DLPF
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Acc, true );
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Gyr, true );
  // Serial.print(F("Enable DLPF for Accelerometer returned: ")); Serial.println(myICM.statusString(accDLPEnableStat));
  // Serial.print(F("Enable DLPF for Gyroscope returned: ")); Serial.println(myICM.statusString(gyrDLPEnableStat));

  //////////////////////////////// END IMU SETUP ///////////////////////////////////////////



  // Wait 5 seconds before turning on. This allows the motors to boot up.
  for (int i = 0; i < 20; i++) {
    digitalWrite(13, HIGH);
    delay(125);
    digitalWrite(13, LOW);
    delay(125);
  }

  last_command_ts = micros();
  last_print_ts = millis();
  last_header_ts = millis();

  ////////////// Runtime config /////////////////////
  drive.SetMaxCurrent(MAX_CURRENT);
  options.delimiter = ',';
  options.print_delay_millis = PRINT_DELAY;
  options.header_delay_millis = HEADER_DELAY;
  options.positions = true;
  options.velocities = true;
  options.currents = true;             // last actual current
  options.position_references = true;  // last commanded position
  options.velocity_references = false;
  options.current_references = false;
  options.last_current = true;  // last commanded current

  // Set behavioral options
  drive.SetPositionKp(DEFAULT_GAINS.kp);
  drive.SetPositionKd(DEFAULT_GAINS.kd);
  drive.SetIdle();
  drive.PrintHeader(options);

  interpreter.Flush();

  // Activating the motors on bootup is dumb but it allows you to see debug
  // information
  drive.SetActivations({1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  drive.SetMaxCurrent(0.0);

  // FOR TESTING ONLY
  // drive.SetMaxCurrent(6.0);
  // drive.ZeroCurrentPosition();
  // drive.SetCartesianPositions(drive.DefaultCartesianPositions());
  // drive.SetCartesianKp3x3({5000.0, 0, 0, 0, 0.0, 0, 0, 0, 0.0});  //[A/m]
  // drive.SetCartesianKd3x3({75.0, 0, 0, 0, 0.0, 0, 0, 0, 0.0});  // [A /
  // (m/s)]

  // FOR TESTING ONLY ////////////////////////////////////
  drive.SetMaxCurrent(6.0);
  drive.ZeroCurrentPosition();
  BLA::Matrix<12> torques_start = {0,0,0,0,0,0,0,0,0,0,0,0};
  torques_start(3) = 0;
  drive.SetTorques(torques_start); // This also puts control mode in torque control
  
  ////////////////////////////////////////////////////////

  drive.PrintHeader(options);
}

void loop() {
  drive.CheckForCANMessages();
  CheckResult r = interpreter.CheckForMessages();
  if (r.flag == CheckResultFlag::kNewCommand) {
    // Serial << "Got new command." << endl;
    if (r.new_torque) {
      auto trqs = interpreter.LatestTorqueCommand();
      drive.SetTorques(Utils::ArrayToVector<12, 12>(trqs));
      if (ECHO_COMMANDS) {
        Serial << "Torque command: "
               << interpreter.LatestTorqueCommand();
      }
    }
    if (r.new_position) { 
      Serial << "Running in joint position control now (main.cpp) - mathew" << endl;
      drive.SetJointPositions(interpreter.LatestPositionCommand());
      if (ECHO_COMMANDS) {
        Serial << "Position command: " << interpreter.LatestPositionCommand()
               << endl;
      }
    }
    if (r.new_cartesian_position) {
      drive.SetCartesianPositions(interpreter.LatestCartesianPositionCommand());
      if (ECHO_COMMANDS) {
        Serial << "Cartesian position command: "
               << interpreter.LatestCartesianPositionCommand();
      }
    }
    if (r.new_kp) {
      drive.SetPositionKp(interpreter.LatestKp());
      if (ECHO_COMMANDS) {
        Serial << "Kp: " << interpreter.LatestKp() << endl;
      }
    }
    if (r.new_kd) {
      drive.SetPositionKd(interpreter.LatestKd());
      if (ECHO_COMMANDS) {
        Serial.print("Kd: ");
        Serial.println(interpreter.LatestKd(), 4);
      }
    }
    if (r.new_cartesian_kp) {
      drive.SetCartesianKp3x3(interpreter.LatestCartesianKp3x3());
      if (ECHO_COMMANDS) {
        Serial << "Cartesian Kp: " << interpreter.LatestCartesianKp3x3()
               << endl;
      }
    }
    if (r.new_cartesian_kd) {
      drive.SetCartesianKd3x3(interpreter.LatestCartesianKd3x3());
      if (ECHO_COMMANDS) {
        Serial << "Cartesian Kd: " << interpreter.LatestCartesianKd3x3()
               << endl;
      }
    }
    if (r.new_feedforward_force) {
      auto ff = interpreter.LatestFeedForwardForce();
      drive.SetFeedForwardForce(Utils::ArrayToVector<12, 12>(ff));
      Serial << "Feed forward: " << interpreter.LatestFeedForwardForce()
             << endl;
    }
    if (r.new_max_current) {
      drive.SetMaxCurrent(interpreter.LatestMaxCurrent());
      if (ECHO_COMMANDS) {
        Serial << "Max Current: " << interpreter.LatestMaxCurrent() << endl;
      }
    }
    if (r.new_activation) {
      drive.SetActivations(interpreter.LatestActivations());
      if (ECHO_COMMANDS) {
        Serial << "Activations: " << interpreter.LatestActivations() << endl;
      }
    }
    if (r.do_zero) {
      drive.ZeroCurrentPosition();
      if (ECHO_COMMANDS) {
        Serial << "Setting current position as the zero point" << endl;
      }
    }
    if (r.do_idle) {
      //Serial << "r.do_idle message recieved by teensy. -mathew" << endl; // - mathew 
      drive.SetIdle();
      if (ECHO_COMMANDS) {
        Serial << "Setting drive to idle." << endl;
      }
    }
    if (r.new_debug) {
      print_debug_info = interpreter.LatestDebug();
    }
  }

  // Update control commands
  if (micros() - last_command_ts >= CONTROL_DELAY) {
    // TODO: characterize maximum lag between control updates
    drive.Update();
    last_command_ts = micros();
  }

  // Get IMU data
  if( myICM.dataReady() )
  {
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'



    // dt is throttled by the time it takes to recieve data from the IMU
    now = micros();
    dt = ((now - lastUpdate)/1000000.0f); 
    lastUpdate = now;

    // MadgwickAHRSupdate(myICM.gyrX()-bgx, myICM.gyrY()-bgy, myICM.gyrZ()-bgz, myICM.accX(), myICM.accY(), myICM.accZ(), myI CM.magX()-bx, myICM.magY()-by, myICM.magZ()-bz, udt);
    // MahonyAHRSupdate(myICM.gyrX()-bgx, myICM.gyrY()-bgy, myICM.gyrZ()-bgz, myICM.accX(), myICM.accY(), myICM.accZ(), myICM.magX()-bx, myICM.magY()-by, myICM.magZ()-bz, udt);
    // MahonyAHRSupdate(0, 0, 0, myICM.accX(), myICM.accY(), myICM.accZ(), myICM.magX()-bx, myICM.magY()-by, myICM.magZ()-bz, udt);
    // It's recommended by Kris to run the algorithm update 5x for each sample update: https://github.com/kriswiner/MPU9250
    // He accomplishes this by using interrupts when new data is available from the sensors.
    MadgwickKrisUpdate(myICM.gyrX()-bgx, myICM.gyrY()-bgy, myICM.gyrZ()-bgz, myICM.accX(), myICM.accY(), myICM.accZ(), myICM.magX()-bx, -(myICM.magY()-by), -(myICM.magZ()-bz), dt);

    // Run with processing to visualize orientation
    // printQuat(); // Send quaternion estimates to serial stream

    //For debugging
    // Serial.println(dt,14); // Print sampling rate
    // printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    // printScaledAGMT( myICM.agmt);   // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    // debugQuat(); // Print quaternions to serial monitor
    // types(myICM.gyrX()); // Test type of variable
    // Serial.println(myICM.accY(),7); 
    // Serial.println(myICM.gyrX()*pi/180.0f/1000.0f,7);
    // printMagnetometer(); // Used to test magnetometer calibration in MATLAB
    // printGyro(); // used to test gyro calibration

    // delay(1);
    // if(Serial.available())
    // {
    //     char input = Serial.read();
    //     if (input == 'd')
    //     {
    //       Serial.println(F("debug mode"));
    //       debug = true;
    //     }
    // }

    // if(debug == true)
    // {
    //     // Choose whether or not to use DLPF
    //     ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Acc, false );
    //     Serial.print(F("Disable DLPF for Accelerometer returned: ")); SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
    //     delay(2000);
    //     debug = false;
    // }

  }
  ///////////////////////////////////////////////////////////////////////
  // Send data to PC - the tethered WBC uses this info  - mathew
  // Serial << "\n Working: " << endl;
  if (print_robot_states){
    if (micros() - last_print_ts >= OBSERVE_DELAY) {
      // Serial << "\n Sending data to PC at : " << micros() << endl; // - mathew
      // Serial << "\n Velocity: " << drive.GetActuatorVelocity(11) << endl;
      drive.PrintMsgPackStatus(options);
      last_print_ts = micros();
    }
  }
  // else{
  //   Serial << "print_robot_states is set to false. PC is not recieving robot_states from teensy." << endl;
  // }


  if (print_debug_info) {
    if (millis() - last_print_ts >= options.print_delay_millis) {
      // drive.PrintStatus(options);
      // logger.AddData(drive.DebugData());
      drive.PrintMsgPackStatus(options);
      last_print_ts = millis();
    }
    if (print_header_periodically) {
      if (millis() - last_header_ts >= options.header_delay_millis) {
        drive.PrintHeader(options);
        last_header_ts = millis();
      }
    }
  }
}
