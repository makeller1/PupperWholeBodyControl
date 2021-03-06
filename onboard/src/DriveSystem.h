#pragma once

#include <BasicLinearAlgebra.h>
#include "C610Bus.h"
#include "RobotTypes.h"

// Enum for the various control modes: idle, position control, current control
enum class DriveControlMode {
  kTorqueControl, // setting this first to debug - mathew
  kIdle,
  kError, // for handling faults
  kCalibration
};

// Default values for which values to pass to workstation. They are modified in main.cpp.
struct DrivePrintOptions {
  bool time = true;
  bool positions = true;
  bool velocities = true;
  bool currents = true;
  bool quaternion = true; 
  char delimiter = '\t';
};

const uint8_t kNumDriveSystemDebugValues = 7*12 + 1;

// Class for controlling the 12 actuators on Pupper
class DriveSystem {
 public:
  static const size_t kNumActuators = 12; 
  // Timing troubleshooting variables
  double rcan_ms; // read can messages
  double rpy_ms; // read python messages
  double cflg_ms; // check flags
  double scc_ms; // send control commands
  double ahrs_ms; // read/filter ahrs measurements
  double spy_ms; // send messages to python
  // double 
 private:
  C610Bus<CAN1> front_bus_;
  C610Bus<CAN2> rear_bus_;

  // quaternion of sensor frame relative to global frame
  float q0_, q1_, q2_, q3_;
  // time derivative of quaternion
  float qDot0_, qDot1_, qDot2_, qDot3_;
  
  DriveControlMode control_mode_;

  ActuatorPositionVector zero_position_offset_; // used to zero pupper laying down
  ActuatorPositionVector zero_position_;

  BLA::Matrix<12> torques_; // mathew

  // Indicates which motors violate fault_velocity_
  ActuatorActivations viol_vel_mask_;

  // Indicates which motors violate fault_position_
  ActuatorActivations viol_pos_mask_;
  
  // Indicates which motor index first violated fault_position_ (-1 indicates none)
  int8_t viol_pos_mask_first_;
  // Indicates which motor index first violated fault_velocity_ (-1 indicates none)
  int8_t viol_vel_mask_first_;

  // Maximum current for current control and PD mode.
  float max_current_;

  // Maximum commandable current before system triggers a fault. Different
  // than SW saturation current.
  float fault_current_;

  // Max velocity before system errors out.
  float fault_velocity_;

  // fault_position highs for each joint (hip, shoulder, elbow, for all legs) - mathew
  float fault_position_array_high_[3];
  // fault_position lows for each joint (hip, shoulder, elbow, for all legs) - mathew
  float fault_position_array_low_[3];

  // Important direction multipliers
  std::array<float, 12> direction_multipliers_; /* */

  // Initialize the two CAN buses
  void InitializeDrive();

  // Message sequence for tracking missed messages
  unsigned long int msg_seq;

 public:
  // Construct drive system and initialize CAN buses.
  // Set position and current-control references to zero.
  DriveSystem();

  // Run one iteration through the control loop. Action depends on the current
  // mode.
  void Update();

  // Calculate motor currents for torque control - mathew
  BLA::Matrix<12> TorqueControl();

  // Check for messages on the CAN bus and run callbacks.
  void CheckForCANMessages();

  // Check for errors
  DriveControlMode CheckErrors();

  // Go into idle mode, which sends 0A to all motors.
  void SetIdle();

  // Set the measured position to the zero point for the actuators.
  void ZeroCurrentPosition();

  // Pass quaternion values
  void SetQuaternions(float q0, float q1, float q2, float q3, float qDot0, float qDot1, float qDot2, float qDot3);

  // Set ControlMode to kTorqueControl
  void SetTorqueControl();

  void SetCalibrationControl();

  // IMU calibration routine
  void CalibrateIMU(std::array<float, 12>);

  // Set torque for all motors.
  void SetTorques(BLA::Matrix<12> torques);

  // Set current level that would trigger a fault.
  void SetFaultCurrent(float fault_current);

  // Set maximum PID and current control torque.
  void SetMaxCurrent(float max_current);

  // Set activations for all the motors simultaneously.
  void SetActivations(ActuatorActivations acts);

  // Derivative control to regulate velocity
  void CommandBraking();

  // Send zero torques to the escs.
  void CommandIdle();

  // Regulates motor velocities during calibration
  void CalibrationControl();

  /*
  The ordering of the torques array goes like this:
  front-right abduction
  front-right hip
  front-right knee
  front-left ...
  back-right ...
  back-left ...
  */

  // Send torque commands to C610 escs.
  // The current argument has units amps.
  void CommandCurrents(ActuatorCurrentVector currents);

  // Get the C610 controller object corresponding to index i.
  C610 GetController(uint8_t i);

  // Returns the output shaft's position in [radians].
  float GetActuatorPosition(uint8_t i);

  // Returns all output shaft positions [radians].
  ActuatorPositionVector GetActuatorPositions();

  // Returns the output shaft's position in [radians].
  float GetRawActuatorPosition(uint8_t i);

  // Returns all output shaft positions [radians].
  ActuatorPositionVector GetRawActuatorPositions();

  // Returns the output shaft's velocity in [radians/s].
  float GetActuatorVelocity(uint8_t i);

  // Returns the motor's actual current in [A]
  float GetActuatorCurrent(uint8_t i);

  // Return the total motor power
  float GetTotalElectricalPower();

  void PrintMsgPackStatus(DrivePrintOptions options);

};