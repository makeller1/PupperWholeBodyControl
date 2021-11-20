#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <NonBlockingSerialBuffer.h>
#include <Streaming.h>

#include <array>

#include "RobotTypes.h"

enum class CheckResultFlag { kNothing, kNewCommand, kError };
struct CheckResult {
  bool new_torque = false; 
  bool trq_mode_set = false;
  bool new_max_current = false;
  bool new_activation = false;
  bool do_zero = false;
  bool new_debug = false;
  CheckResultFlag flag = CheckResultFlag::kNothing;
};

class CommandInterpreter {
 private:
  ActuatorTorqueVector torque_command_;
  ActuatorPositionVector position_command_;
  float max_current_;
  bool trq_mode_set_;
  bool print_debug_info_;

  StaticJsonDocument<512> doc_;
  NonBlockingSerialBuffer<512> reader_;

  const bool use_msgpack_;

 public:
  // Default to using msgpack, 0x00 as the message start indicator, and Serial
  // as the input stream
  CommandInterpreter(bool use_msgpack = true, uint8_t start_byte = 0x00,
                     Stream &stream = Serial);

  // Checks the serial input buffer for bytes. Returns an enum indicating the
  // result of the read: kNothing, kNewCommand, or kError. Should be called as
  // fast as possible.
  CheckResult CheckForMessages();

  ActuatorPositionVector LatestCartesianPositionCommand();

  // Returns an ActuatorTorqueVector with the latest torque commands.
  ActuatorTorqueVector LatestTorqueCommand(); // - mathew

  bool LatestDebug();

  // Empty the input buffer
  void Flush();

  float LatestMaxCurrent();
};

// Start character: 0x00
// Stream: Serial
// Add '\0' to treat input as a string? : true
CommandInterpreter::CommandInterpreter(bool use_msgpack, uint8_t start_byte,
                                       Stream &stream)
    : reader_(start_byte, stream, true), use_msgpack_(use_msgpack) {}

template <class T, unsigned int SIZE>
CheckResultFlag CopyJsonArray(JsonArray json, std::array<T, SIZE> &arr) {
  if (json.size() != arr.size()) {
    Serial << "Error: Invalid number of parameters in position command."
           << endl;
    return CheckResultFlag::kError;
  }
  uint8_t i = 0;
  for (auto v : json) {
    arr[i++] = v.as<T>();
  }
  return CheckResultFlag::kNewCommand;
}

CheckResult CommandInterpreter::CheckForMessages() {
  CheckResult result;
  BufferResult buffer_result = reader_.Read();
  if (buffer_result == BufferResult::kDone) 
  {
    doc_.clear();
    auto err = use_msgpack_ ? deserializeMsgPack(doc_, reader_.buffer_)
                            : deserializeJson(doc_, reader_.buffer_);
    if (err) 
    {
      Serial << "Deserialize failed: " << err.c_str() << endl;
      result.flag = CheckResultFlag::kError;
      return result;
    }
    auto obj = doc_.as<JsonObject>();
    if (obj.containsKey("trq")) 
    {
      auto json_array = obj["trq"].as<JsonArray>();
      result.flag = CopyJsonArray(json_array, torque_command_);
      result.new_torque = result.flag == CheckResultFlag::kNewCommand;
    }
    if (obj.containsKey("trq_mode")) 
    { // Set trq control mode
      result.flag = CheckResultFlag::kNewCommand;
      result.trq_mode_set = true;
      trq_mode_set_ = obj["trq_mode"].as<bool>();
    }
    if (obj.containsKey("max_current")) 
    {
      max_current_ = obj["max_current"].as<float>();
      result.new_max_current = true;
      result.flag = CheckResultFlag::kNewCommand;
    }
    if (obj.containsKey("zero")) 
    {
      if (obj["zero"].as<bool>()) 
      {
        result.flag = CheckResultFlag::kNewCommand;
        result.do_zero = true;
      }
    }
    if (obj.containsKey("debug")) 
    {
      result.flag = CheckResultFlag::kNewCommand;
      result.new_debug = true;
      print_debug_info_ = obj["debug"].as<bool>();
    }
  }
  return result;
}

bool CommandInterpreter::LatestDebug() { return print_debug_info_; }

ActuatorTorqueVector CommandInterpreter::LatestTorqueCommand(){
  return torque_command_;
}

float CommandInterpreter::LatestMaxCurrent() { return max_current_; }

void CommandInterpreter::Flush() { reader_.FlushStream(); }