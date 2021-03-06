#include "DriveSystem.h"
#include <ArduinoJson.h>
#include <Streaming.h>
#include "Utils.h"
#include "Calibration.h"

DriveSystem::DriveSystem() : front_bus_(), rear_bus_() 
{
  control_mode_ = DriveControlMode::kIdle; //kIdle;
  fault_current_ = 10.0; // Violation sets control mode to kError

  // Fault positions measured relative to starting pose (laying down)
  fault_position_array_high_[0] = 999.99; // 0.35 hip fault position 
  fault_position_array_high_[1] = 0.25;  // shoulder fault position
  fault_position_array_high_[2] = 1.9;  // elbow fault position

  fault_position_array_low_[0] = -999.99; // -0.35 hip fault position
  fault_position_array_low_[1] = -1.0; // 0.75 shoulder fault position
  fault_position_array_low_[2] = -0.15; // elbow fault position

  fault_velocity_ = 15.0;  // Reasonable value for standing up - will need to raise for gaits

  // Default values
  torques_ = {0,0,0,0,0,0,0,0,0,0,0,0};
  max_current_ = 3.0; // Saturates current command - overridden by main.cpp
  viol_vel_mask_.fill(false);
  viol_pos_mask_.fill(false);
  viol_pos_mask_first_ = -1;
  viol_vel_mask_first_ = -1;

  // quaternion of sensor frame relative to global frame
  q0_ = 1; // w
  q1_ = 0; // x
  q2_ = 0; // y
  q3_ = 0; // z
  
  // time derivative of quaternion
  qDot0_ = 0;
  qDot1_ = 0;
  qDot2_ = 0;
  qDot3_ = 0; 

  // Message sequence for tracking missed messages
  msg_seq = 0;

  //  Currently implement the zero-offset in python
  //                              FR0, FR1, FR2                   FL0, FL1, FL2                  BR0,BR1,BR2                    BL0,BL1,BL2     
  //zero_position_offset_ = {.06539, 1.19682, 2.71176,  -.06539, -1.19682, -2.71176,   .06539, 1.19682, 2.71176,    -.06539, -1.19682, -2.71176}; // used to zero pupper laying down

  zero_position_.fill(0.0);

  std::array<float, 12> direction_multipliers = {-1, -1, 1, -1, 1, -1,
                                                 -1, -1, 1, -1, 1, -1};
  direction_multipliers_ = direction_multipliers;
  
}

void DriveSystem::CheckForCANMessages() 
{
  front_bus_.PollCAN();
  rear_bus_.PollCAN();
}

DriveControlMode DriveSystem::CheckErrors() 
{
  bool error_found = false;
  for (size_t i = 0; i < kNumActuators; i++) 
  {
    // check positions
    if (GetActuatorPosition(i) > fault_position_array_high_[i%3] || GetActuatorPosition(i) < fault_position_array_low_[i%3]) 
    {
      viol_pos_mask_[i] = true;
      error_found = true;
    }
    else
    {
      viol_pos_mask_[i] = false;
    }
    // check velocities
    if (fabsf(GetActuatorVelocity(i)) > fault_velocity_) 
    {
      viol_vel_mask_[i] = true;
      error_found = true;
    }
    else
    {
      viol_vel_mask_[i] = false;
    }
  }
  if (error_found)
  {
    if (control_mode_ == DriveControlMode::kTorqueControl)
    {
      // BeepLow(); // beep on
      // Indicate which motor violated fault pos/vel
      viol_pos_mask_first_ = -1;
      viol_vel_mask_first_ = -1;
      for (uint8_t i = 0; i < kNumActuators; i++)
      {
        if(viol_pos_mask_[i] == true)
        {
          viol_pos_mask_first_ = i;
        }
        if(viol_vel_mask_[i] == true)
        {
          viol_vel_mask_first_ = i;
        }
      }
    }
    return DriveControlMode::kError;
  }
  else
  {
    return control_mode_;
  }
}

void DriveSystem::SetIdle() 
{ 
  control_mode_ = DriveControlMode::kIdle; 
}

void DriveSystem::ZeroCurrentPosition() 
{
  zero_position_ = GetRawActuatorPositions();
}

void DriveSystem::SetQuaternions(float q0, float q1, float q2, float q3, float qDot0, float qDot1, float qDot2, float qDot3)
{
  q0_ = q0;
  q1_ = q1;
  q2_ = q2;
  q3_ = q3;
  qDot0_ = qDot0;
  qDot1_ = qDot1;
  qDot2_ = qDot2;
  qDot3_ = qDot3;
}

void DriveSystem::SetFaultCurrent(float fault_current) 
{
  fault_current_ = fault_current;
}

void DriveSystem::SetMaxCurrent(float max_current) {
  max_current_ = max_current;
}

void DriveSystem::SetTorqueControl()
{
  ZeroCurrentPosition();
  control_mode_ = DriveControlMode::kTorqueControl; 
}

void DriveSystem::SetCalibrationControl()
{
  control_mode_ = DriveControlMode::kCalibration; 
}

void DriveSystem::SetTorques(BLA::Matrix<12> torques) 
{ 
  torques_ = torques; // commanded torques
}

BLA::Matrix<12> DriveSystem::TorqueControl() 
{ 
  // Converts torques_ to currents, obeying current limit using actuator_torques

  // Convert torques to currents
  BLA::Matrix<12> motor_currents;
  motor_currents.Fill(0);

  for (int i=0; i<12; i++){
    motor_currents(i) = 1.0 * torques_(i); 
  }
  // Note: motor_currents are constrained in command_current to max_current_
  return motor_currents;
}

void DriveSystem::Update() 
{
  // If there are errors, put the system in the error state.
  if (CheckErrors() == DriveControlMode::kError) 
  {
    control_mode_ = DriveControlMode::kError;
    SetMaxCurrent(5.0f); // In case Teensy code fails
  }
  switch (control_mode_) 
  {
    case DriveControlMode::kError: 
    {
      CommandBraking();
      break;
    }
    case DriveControlMode::kTorqueControl: 
    {
      CommandCurrents(Utils::VectorToArray<12, 12>(TorqueControl()));
      break;
    }
    case DriveControlMode::kCalibration:
    {
      // Regulate motor velocity during calibration
      CalibrationControl();
      break;
    }
    case DriveControlMode::kIdle: 
    {
      CommandIdle();
      break;
    }
  }
}

void DriveSystem::CommandBraking() 
{
  // Regulate joint velocity to prevent destructive joint positions or velocities
  const float Kd_viol = 2.0;
  const float Kd_safe = 0.8; // 0.8
  ActuatorCurrentVector currents;
  currents.fill(0.0);
  for (size_t i=0; i < kNumActuators; i++)
  {
    if (viol_pos_mask_[i] || viol_vel_mask_[i])
    {
      currents[i] = -Kd_viol*GetActuatorVelocity(i);
    }
    else
    {
      currents[i] = -Kd_safe*GetActuatorVelocity(i);
    }
  }
  CommandCurrents(currents);
}

void DriveSystem::CommandIdle() 
{
  ActuatorCurrentVector currents;
  currents.fill(0.0);
  CommandCurrents(currents);
}

void DriveSystem::CalibrationControl()
{
  // Regulate joint velocity to make calibration easier
  static std::array<float,12> e_I = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Integral of velocity error
  static float last_command_ts = millis();

  float now = millis();
  float dt = (now - last_command_ts)/1000.0f; 

  const float Kd = 0.6;
  const float Ki = 1.0;

  ActuatorCurrentVector currents;
  currents.fill(0.0);
  for (size_t i=0; i < kNumActuators; i++)
  {
      float e = GetActuatorVelocity(i);
      currents[i] = -Kd*e -Ki*e_I[i];
      e_I[i] += e*dt;
  }
  CommandCurrents(currents);
  last_command_ts = now;
}

void DriveSystem::CommandCurrents(ActuatorCurrentVector currents) 
{
  ActuatorCurrentVector current_command = Utils::Constrain(currents, -max_current_, max_current_);

  if (Utils::Maximum(current_command) > fault_current_ ||
      Utils::Minimum(current_command) < -fault_current_) {
    Serial << "\nRequested current too large. Erroring out.\n" << endl;
    control_mode_ = DriveControlMode::kError;
    return;
  }

  // Convert the currents into the motors' local frames
  current_command =
      Utils::ElemMultiply(current_command, direction_multipliers_);

  // Convert from float array to int32 array in units milli amps.
  std::array<int32_t, kNumActuators> currents_mA =
      Utils::ConvertToFixedPoint(current_command, 1000);
  // Send current commands down the CAN buses (CommandTorques is a misnomer, actually passes currents)
  front_bus_.CommandTorques(currents_mA[0], currents_mA[1], currents_mA[2],
                            currents_mA[3], C610Subbus::kIDZeroToThree);
  front_bus_.CommandTorques(currents_mA[4], currents_mA[5], 0, 0,
                            C610Subbus::kIDFourToSeven);
  rear_bus_.CommandTorques(currents_mA[6], currents_mA[7], currents_mA[8],
                           currents_mA[9], C610Subbus::kIDZeroToThree);
  rear_bus_.CommandTorques(currents_mA[10], currents_mA[11], 0, 0,
                           C610Subbus::kIDFourToSeven);
}

C610 DriveSystem::GetController(uint8_t i) {
  // TODO put these constants somewhere else
  if (i >= 0 && i <= 5) {
    return front_bus_.Get(i);
  } else if (i >= 6 && i <= 11) {
    return rear_bus_.Get(i - 6);
  } else {
    Serial << "Invalid actuator index. Must be 0<=i<=11." << endl;
    control_mode_ = DriveControlMode::kError;
    return C610();
  }
}

float DriveSystem::GetRawActuatorPosition(uint8_t i) {
  return GetController(i).Position();
}

ActuatorPositionVector DriveSystem::GetRawActuatorPositions() {
  ActuatorPositionVector pos;
  for (size_t i = 0; i < pos.size(); i++) {
    pos[i] = GetRawActuatorPosition(i);
  }
  return pos;
}

float DriveSystem::GetActuatorPosition(uint8_t i) {
  return (GetRawActuatorPosition(i) - zero_position_[i]) *
         direction_multipliers_[i];
}

ActuatorPositionVector DriveSystem::GetActuatorPositions() {
  ActuatorPositionVector pos;
  for (size_t i = 0; i < pos.size(); i++) {
    pos[i] = GetActuatorPosition(i);
  }
  return pos;
}

float DriveSystem::GetActuatorVelocity(uint8_t i) {
  return GetController(i).Velocity() * direction_multipliers_[i];
}

float DriveSystem::GetActuatorCurrent(uint8_t i) {
  return GetController(i).Current() * direction_multipliers_[i];
}

float DriveSystem::GetTotalElectricalPower() {
  float power = 0.0;
  for (uint8_t i = 0; i < kNumActuators; i++) {
    power += GetController(i).ElectricalPower();
  }
  return power;
}

// Sends data to the workstation
void DriveSystem::PrintMsgPackStatus(DrivePrintOptions options) {
  // Serial << "PrintMsgPackStatus() is running" << endl; // - mathew
  StaticJsonDocument<2048> doc;
  // 21 micros to put this doc together
  doc["ts"] = micros()/1000.0; // ms
  for (uint8_t i = 0; i < kNumActuators; i++) 
  {
    if (options.positions) {
      doc["pos"][i] = GetActuatorPosition(i);
    }
    if (options.velocities) {
      doc["vel"][i] = GetActuatorVelocity(i);
    }
    if (options.currents) {
      doc["cur"][i] = GetActuatorCurrent(i);
    }
  }
  if (options.quaternion)
  { 
    doc["quat"][0] = q0_; // w
    doc["quat"][1] = q1_; // x
    doc["quat"][2] = q2_; // y
    doc["quat"][3] = q3_; // z
    doc["quatdot"][0] = qDot0_; // w
    doc["quatdot"][1] = qDot1_; // x
    doc["quatdot"][2] = qDot2_; // y
    doc["quatdot"][3] = qDot3_; // z
  }
  if (true)
  {
    // For debugging what's hanging the teensy
    doc["response"] = torques_(0);
    doc["seq"] = msg_seq;
    msg_seq += 1;
  }
  if (control_mode_ == DriveControlMode::kError)
  {
    doc["err"][0] = viol_pos_mask_first_;
    doc["err"][1] = viol_vel_mask_first_;
  }
  uint16_t num_bytes = measureMsgPack(doc);
  // Serial.println(num_bytes);
  Serial.write(69);
  Serial.write(69);
  Serial.write(num_bytes >> 8 & 0xff);
  Serial.write(num_bytes & 0xff);
  serializeMsgPack(doc, Serial);
  Serial.println();
}
