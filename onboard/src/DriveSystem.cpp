#include "DriveSystem.h"

#include <ArduinoJson.h>
#include <Streaming.h>

#include "Utils.h"

DriveSystem::DriveSystem() : front_bus_(), rear_bus_() 
{
  control_mode_ = DriveControlMode::kTorqueControl; // - mathew
  fault_current_ = 10.0; // Violation sets control mode to kError
  fault_position_array_[0] = PI/4; // PI/4 fault_positions for each joint (hip, shoulder, elbow, for all legs) - mathew
  fault_position_array_[1] = 5.0; //5.0;
  fault_position_array_[2] = 5.0; //5.0;

  fault_velocity_ =
      10.0;  // TODO: Determine if this is reasonable
  max_current_ = 4.0; // Saturates current command
  velocity_reference_.fill(0.0);
  current_reference_.fill(
      0.0);  // TODO: log the commanded current even when in position PID mode
  active_mask_.fill(false);

  // quaternion of sensor frame relative to global frame
  q0_ = 1;
  q1_ = 0;
  q2_ = 0;
  q3_ = 0;

                             // FR0, FR1, FR2                   FL0, FL1, FL2                  BR0,BR1,BR2                    BL0,BL1,BL2     
  //zero_position_offset_ = {.06539, 1.19682, 2.71176,  -.06539, -1.19682, -2.71176,   .06539, 1.19682, 2.71176,    -.06539, -1.19682, -2.71176}; // used to zero pupper laying down
  // zero_position_offset_ = {.1,.2,.3,.4,.5,.6,.7,.8,.9,.10,.11,.12}; // Testing to see mapping

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
  // for (size_t i = 0; i < kNumActuators; i++) {
  //   // check positions
  //   if (abs(GetActuatorPosition(i)) > fault_position_) {
  //     Serial << "actuator[" << i << "] hit fault position: " << fault_position_
  //            << endl;
  //     return DriveControlMode::kError;
  //   }
  //   // check velocities
  //   if (abs(GetActuatorVelocity(i)) > fault_velocity_) {
  //     Serial << "actuator[" << i << "] hit fault velocity: " << fault_velocity_
  //            << endl;
  //     return DriveControlMode::kError;
  //   }
  // }
  // check position of each motor
    for (size_t i = 0; i < 4; i++) 
    {
      if (abs(GetActuatorPosition(i*3)) > fault_position_array_[0] || 
          abs(GetActuatorPosition(i*3 + 1)) > fault_position_array_[1] ||
          abs(GetActuatorPosition(i*3 + 2)) > fault_position_array_[2]){
        // Serial << "Leg[" << i << "] hit fault position: " << fault_position_ << endl;
        return DriveControlMode::kError;
      }
    }
  //Serial << "CheckError() was called and that sets motors idle" << endl; // - mathew
  //return DriveControlMode::kIdle; // This shouldn't be called just for checking errors, right?
  return control_mode_; // - mathew
}

void DriveSystem::SetIdle() 
{ 
  // Serial << "SetIdle() was called - mathew" << endl; 
  //control_mode_ = DriveControlMode::kIdle; 
}

void DriveSystem::ZeroCurrentPosition() 
{
  zero_position_ = GetRawActuatorPositions();
}

void DriveSystem::SetQuaternions(float q0, float q1, float q2, float q3)
{
  q0_ = q0;
  q1_ = q1;
  q2_ = q2;
  q3_ = q3;
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
  control_mode_ = DriveControlMode::kTorqueControl; // Remove this so we stay in other control modes despite commands being sent
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
    motor_currents(i) = 1.0 * torques_(i); // TODO : Find out if we need to convert torque to current or does C610 do that
  }
  // Note: motor_currents are constrained in command_current to max_current_

  return motor_currents;
}

void DriveSystem::Update() {
  // If there are errors, put the system in the error state.
  if (CheckErrors() == DriveControlMode::kError) {
    control_mode_ = DriveControlMode::kError;
  }

  switch (control_mode_) 
  {
    case DriveControlMode::kError: 
    {
      Serial << "\nDriveControlMode is ERROR\n" << endl;
      CommandIdle();
      break;
    }
    case DriveControlMode::kTorqueControl: 
    {
      //Serial << "Control Mode: " << "kTorqueControl" << endl;
      CommandCurrents(Utils::VectorToArray<12, 12>(TorqueControl()));
      break;
    }
  }
}

void DriveSystem::SetActivations(ActuatorActivations acts) {
  active_mask_ = acts;  // Is this a copy?
}

void DriveSystem::CommandIdle() {
  ActuatorCurrentVector currents;
  currents.fill(0.0);
  CommandCurrents(currents);
  // Serial << "CommandIdle() was called - Mathew" << endl;
}

void DriveSystem::CommandCurrents(ActuatorCurrentVector currents) {
  ActuatorCurrentVector current_command =
      Utils::Constrain(currents, -max_current_, max_current_);
  if (Utils::Maximum(current_command) > fault_current_ ||
      Utils::Minimum(current_command) < -fault_current_) {
    Serial << "\nRequested current too large. Erroring out.\n" << endl;
    control_mode_ = DriveControlMode::kError;
    return;
  }
  // Set disabled motors to zero current
  current_command = Utils::MaskArray(current_command, active_mask_);

  // Convert the currents into the motors' local frames
  current_command =
      Utils::ElemMultiply(current_command, direction_multipliers_);

  // Convert from float array to int32 array in units milli amps.
  std::array<int32_t, kNumActuators> currents_mA =
      Utils::ConvertToFixedPoint(current_command, 1000);

  //Serial << "Current 0 (mA): " << currents_mA[0] << " Current 1 (mA): " << currents_mA[1] << " Current 2 (mA): " << currents_mA[2] << endl; // - mathew
  // Send current commands down the CAN buses
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
  doc["ts"] = millis();
  for (uint8_t i = 0; i < kNumActuators; i++) {
    if (options.positions) {
      doc["pos"][i] = GetActuatorPosition(i);
    }
    if (options.velocities) {
      doc["vel"][i] = GetActuatorVelocity(i);
    }
    if (options.currents) {
      doc["cur"][i] = GetActuatorCurrent(i);
    }
    if (options.quaternion){ 
      doc["quat"][0] = q0_; // w
      doc["quat"][1] = q1_; // x
      doc["quat"][2] = q2_; // y
      doc["quat"][3] = q3_; // z
    }
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
