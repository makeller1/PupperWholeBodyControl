#include "DriveSystem.h"

#include <ArduinoJson.h>
#include <Streaming.h>

#include "Utils.h"

// CHANGE THIS BIG NO NO
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

DriveSystem::DriveSystem() : front_bus_(), rear_bus_() {
  // Serial << "DriveSystem::DriveSystem() initializer is setting control mode to idle" << endl;
  //control_mode_ = DriveControlMode::kIdle; // - mathew
  control_mode_ = DriveControlMode::kTorqueControl; // - mathew
  fault_current_ = 10.0;  // N  TODO: don't make this so high at default
  // fault_position_ = 3.5; // 3.5// rad // TODO: make this a param - mathew
  fault_position_array_[0] = PI/4; // fault_positions for each joint (hip, shoulder, elbow, for all legs) - mathew
  fault_position_array_[1] = 5.0; //3.5;
  fault_position_array_[2] = 5.0; //3.5;
  // fault_position_array_[0] = PI*2; // fault_positions for each joint (hip, shoulder, elbow, for all legs) - mathew
  // fault_position_array_[1] = PI*2;
  // fault_position_array_[2] = PI*2;
  fault_velocity_ =
      10.0;  // TODO: make this a param, set to something more reasonable.
  max_current_ = 4.0;
  position_reference_.fill(0.0);
  velocity_reference_.fill(0.0);
  current_reference_.fill(
      0.0);  // TODO: log the commanded current even when in position PID mode
  active_mask_.fill(false);

                             // FR0, FR1, FR2                   FL0, FL1, FL2                  BR0,BR1,BR2                    BL0,BL1,BL2     
  //zero_position_offset_ = {.06539, 1.19682, 2.71176,  -.06539, -1.19682, -2.71176,   .06539, 1.19682, 2.71176,    -.06539, -1.19682, -2.71176}; // used to zero pupper laying down
  // zero_position_offset_ = {.1,.2,.3,.4,.5,.6,.7,.8,.9,.10,.11,.12}; // Testing to see mapping

  zero_position_.fill(0.0);

  cartesian_position_gains_.kp.Fill(0.0);
  cartesian_position_gains_.kd.Fill(0.0);

  std::array<float, 12> direction_multipliers = {-1, -1, 1, -1, 1, -1,
                                                 -1, -1, 1, -1, 1, -1};
  direction_multipliers_ = direction_multipliers;

  SetDefaultCartesianPositions();
}

void DriveSystem::CheckForCANMessages() {
  front_bus_.PollCAN();
  rear_bus_.PollCAN();
}

DriveControlMode DriveSystem::CheckErrors() {
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
    for (size_t i = 0; i < 4; i++) {
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

void DriveSystem::SetIdle() { 
  // Serial << "SetIdle() was called - mathew" << endl; 
  //control_mode_ = DriveControlMode::kIdle; 
  }

void DriveSystem::ZeroCurrentPosition() {
  SetZeroPositions(GetRawActuatorPositions());
}

void DriveSystem::SetZeroPositions(ActuatorPositionVector zero) {
  zero_position_ = zero;
  // Serial << "Joint angles now: " << zero_position_ <<endl;
  // Zero in laying down position (Done only in python)
  // for (int i=0; i<12; i++){
  //   zero_position_[i] += zero_position_offset_[i];
  // }
  // Serial << "Joint angles after offset: " << zero_position_ <<endl;
}

ActuatorPositionVector DriveSystem::DefaultCartesianPositions() {
  ActuatorPositionVector pos;
  for (int i = 0; i < 4; i++) {
    BLA::Matrix<3> p = ForwardKinematics({0, 0, 0}, leg_parameters_, i) +
                       HipPosition(hip_layout_parameters_, i);
    pos[3 * i] = p(0);
    pos[3 * i + 1] = p(1);
    pos[3 * i + 2] = p(2);
  }
  return pos;
}

void DriveSystem::SetDefaultCartesianPositions() {
  cartesian_position_reference_ = DefaultCartesianPositions();
}

void DriveSystem::SetJointPositions(ActuatorPositionVector pos) {
  Serial << "SetJointPositions Called and control_mode_ set to kPositionControl (DriveSystem.cpp) - mathew " << endl;
  control_mode_ = DriveControlMode::kPositionControl;
  position_reference_ = pos;
}

void DriveSystem::SetPositionKp(float kp) { position_gains_.kp = kp; }

void DriveSystem::SetPositionKd(float kd) { position_gains_.kd = kd; }

void DriveSystem::SetCartesianKp3x3(BLA::Matrix<3, 3> kp) {
  cartesian_position_gains_.kp = kp;
}

void DriveSystem::SetCartesianKd3x3(BLA::Matrix<3, 3> kd) {
  cartesian_position_gains_.kd = kd;
}

void DriveSystem::SetCartesianPositions(ActuatorPositionVector pos) {
  control_mode_ = DriveControlMode::kCartesianPositionControl;
  cartesian_position_reference_ = pos;
}

void DriveSystem::SetCartesianVelocities(ActuatorVelocityVector vel) {
  control_mode_ = DriveControlMode::kCartesianPositionControl;
  cartesian_velocity_reference_ = vel;
}

void DriveSystem::SetFeedForwardForce(BLA::Matrix<12> force) {
  ff_force_ = force;
}

void DriveSystem::SetCurrent(uint8_t i, float current_reference) {
  // control_mode_ = DriveControlMode::kCurrentControl; // Why would we change control mode here - mathew
  current_reference_[i] = current_reference;
}

void DriveSystem::SetFaultCurrent(float fault_current) {
  fault_current_ = fault_current;
}

void DriveSystem::SetMaxCurrent(float max_current) {
  max_current_ = max_current;
}

void DriveSystem::SetTorques(BLA::Matrix<12> torques) { // mathew
  control_mode_ = DriveControlMode::kTorqueControl;
  torques_ = torques; // commanded torques
}

BLA::Matrix<12> DriveSystem::CartesianPositionControl() {
  BLA::Matrix<12> actuator_torques;
  for (int leg_index = 0; leg_index < 4; leg_index++) {
    auto joint_angles = LegJointAngles(leg_index);
    auto joint_velocities = LegJointVelocities(leg_index);
    BLA::Matrix<3, 3> jac =
        LegJacobian(joint_angles, leg_parameters_, leg_index);

    auto measured_hip_relative_positions =
        ForwardKinematics(joint_angles, leg_parameters_, leg_index);
    auto measured_velocities = jac * joint_velocities;
    auto reference_hip_relative_positions =
        LegCartesianPositionReference(leg_index) -
        HipPosition(hip_layout_parameters_, leg_index);
    auto reference_velocities = LegCartesianVelocityReference(leg_index);

    auto cartesian_forces =
        PDControl3(measured_hip_relative_positions, measured_velocities,
                   reference_hip_relative_positions, reference_velocities,
                   cartesian_position_gains_) +
        LegFeedForwardForce(leg_index);
    auto joint_torques = ~jac * cartesian_forces;

    // Ensures that the direction of the force is preserved when motors
    // saturate
    float norm = Utils::InfinityNorm3(joint_torques);
    if (norm > max_current_) {
      joint_torques = joint_torques * max_current_ / norm;
    }

    actuator_torques(3 * leg_index) = joint_torques(0);
    actuator_torques(3 * leg_index + 1) = joint_torques(1);
    actuator_torques(3 * leg_index + 2) = joint_torques(2);
  }
  return actuator_torques;
}

BLA::Matrix<12> DriveSystem::TorqueControl() { // - mathew
  // Converts torques_ to currents, obeying current limit
  // using actuator_torques

  // Convert torques to currents
  BLA::Matrix<12> motor_currents;
  motor_currents.Fill(0);

  for (int i=0; i<12; i++){
    motor_currents(i) = 1.0 * torques_(i); // TODO : Find out if we need to convert torque to current or does C610 do that
  }
  // Note: motor_currents are constrained in command_current to +-max_current_

  return motor_currents;
}


void DriveSystem::Update() {
  // If there are errors, put the system in the error state.
  if (CheckErrors() == DriveControlMode::kError) {
    control_mode_ = DriveControlMode::kError;
  }

  switch (control_mode_) {
    case DriveControlMode::kError: {
      Serial << "\nDriveControlMode is ERROR\n" << endl;
      CommandIdle();
      break;
    }
    // case DriveControlMode::kIdle: {
    //   Serial << "DriveControlMode is in idle case" << endl;
    //   CommandIdle();
    //   break;
    // }
    case DriveControlMode::kPositionControl: {
      ActuatorCurrentVector pd_current;

      for (size_t i = 0; i < kNumActuators; i++) {
        PD(pd_current[i], GetActuatorPosition(i), GetActuatorVelocity(i),
           position_reference_[i], velocity_reference_[i], position_gains_);
      }
      Serial << "pd_current: " << pd_current << endl;
      Serial << "position_gains:" << position_gains_ << endl;
      CommandCurrents(pd_current);
      break;
    }
    case DriveControlMode::kCartesianPositionControl: {
      CommandCurrents(Utils::VectorToArray<12, 12>(CartesianPositionControl()));
      break;
    }
    case DriveControlMode::kTorqueControl: {
      //Serial << "Control Mode: " << "kTorqueControl" << endl;
      CommandCurrents(Utils::VectorToArray<12, 12>(TorqueControl()));
      break;
    }
    case DriveControlMode::kCurrentControl: {
      CommandCurrents(current_reference_);
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

  // Update record of last commanded current
  last_commanded_current_ = current_command;

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

float DriveSystem::GetTotalMechanicalPower() {
  float power = 0.0;
  for (uint8_t i = 0; i < kNumActuators; i++) {
    power += GetController(i).MechanicalPower();
  }
  return power;
}

BLA::Matrix<3> DriveSystem::LegJointAngles(uint8_t i) {
  return {GetActuatorPosition(3 * i), GetActuatorPosition(3 * i + 1),
          GetActuatorPosition(3 * i + 2)};
}

BLA::Matrix<3> DriveSystem::LegJointVelocities(uint8_t i) {
  return {GetActuatorVelocity(3 * i), GetActuatorVelocity(3 * i + 1),
          GetActuatorVelocity(3 * i + 2)};
}

// Get the cartesian reference position for leg i.
BLA::Matrix<3> DriveSystem::LegCartesianPositionReference(uint8_t i) {
  return {cartesian_position_reference_[3 * i],
          cartesian_position_reference_[3 * i + 1],
          cartesian_position_reference_[3 * i + 2]};
}

// Return the cartesian reference velocity for leg i.
BLA::Matrix<3> DriveSystem::LegCartesianVelocityReference(uint8_t i) {
  return {cartesian_velocity_reference_[3 * i],
          cartesian_velocity_reference_[3 * i + 1],
          cartesian_velocity_reference_[3 * i + 2]};
}

BLA::Matrix<3> DriveSystem::LegFeedForwardForce(uint8_t i) {
  return {ff_force_(3 * i), ff_force_(3 * i + 1), ff_force_(3 * i + 2)};
}

void DriveSystem::PrintHeader(DrivePrintOptions options) {
  if (options.time) {
    Serial << "T" << options.delimiter;
  }
  for (size_t i = 0; i < kNumActuators; i++) {
    if (!active_mask_[i]) continue;
    if (options.positions) {
      Serial << "p[" << i << "]" << options.delimiter;
    }
    if (options.velocities) {
      Serial << "v[" << i << "]" << options.delimiter;
    }
    if (options.currents) {
      Serial << "I[" << i << "]" << options.delimiter;
    }
    if (options.position_references) {
      Serial << "pr[" << i << "]" << options.delimiter;
    }
    if (options.velocity_references) {
      Serial << "vr[" << i << "]" << options.delimiter;
    }
    if (options.current_references) {
      Serial << "Ir[" << i << "]" << options.delimiter;
    }
    if (options.last_current) {
      Serial << "Il[" << i << "]" << options.delimiter;
    }
  }
  Serial << endl;
}

// Sends data to the pc - mathew
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
    if (options.position_references) {
      doc["pref"][i] = position_reference_[i];
    }
    if (options.velocity_references) {
      doc["vref"][i] = velocity_reference_[i];
    }
    if (options.current_references) {
      doc["cref"][i] = current_reference_[i];
    }
    if (options.last_current) {
      doc["lcur"][i] = last_commanded_current_[i];
    }
  if (true){ // add option for quaternion 
    doc["quat"][0] = q0; // w
    doc["quat"][1] = q1; // x
    doc["quat"][2] = q2; // y
    doc["quat"][3] = q3; // z
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

void DriveSystem::PrintStatus(DrivePrintOptions options) {
  char delimiter = options.delimiter;
  if (options.time) {
    Serial << millis() << delimiter;
  }
  for (uint8_t i = 0; i < kNumActuators; i++) {
    if (!active_mask_[i]) continue;
    if (options.positions) {
      Serial.print(GetActuatorPosition(i), 2);
      Serial << delimiter;
    }
    if (options.velocities) {
      Serial.print(GetActuatorVelocity(i), 2);
      Serial << delimiter;
    }
    if (options.currents) {
      Serial.print(GetActuatorCurrent(i), 2);
      Serial << delimiter;
    }
    if (options.position_references) {
      Serial.print(position_reference_[i], 2);
      Serial << delimiter;
    }
    if (options.velocity_references) {
      Serial.print(velocity_reference_[i], 2);
      Serial << delimiter;
    }
    if (options.current_references) {
      Serial.print(current_reference_[i], 2);
      Serial << delimiter;
    }
    if (options.last_current) {
      Serial.print(last_commanded_current_[i], 2);
      Serial << delimiter;
    }
  }
  Serial << endl;
  Serial.flush();
}

BLA::Matrix<kNumDriveSystemDebugValues> DriveSystem::DebugData() {
  uint32_t write_index = 0;
  BLA::Matrix<kNumDriveSystemDebugValues> output;
  output(write_index++) = millis();
  for (uint8_t i = 0; i < kNumActuators; i++) {
    output(write_index++) = GetActuatorPosition(i);
    output(write_index++) = GetActuatorVelocity(i);
    output(write_index++) = GetActuatorCurrent(i);
    output(write_index++) = position_reference_[i];
    output(write_index++) = velocity_reference_[i];
    output(write_index++) = current_reference_[i];
    output(write_index++) = last_commanded_current_[i];
  }
  return output;
}