#pragma once

#include <Arduino.h>
#include <string.h>

#include "OrientationFusion.h"

class SerialProtocol {
 public:
  void poll(Stream &serial, OrientationFusion &fusion) {
    while (serial.available() > 0) {
      const char received = static_cast<char>(serial.read());
      if (received == '\r') {
        continue;
      }
      if (received == '\n') {
        command_[commandLength_] = '\0';
        executeCommand(fusion);
        commandLength_ = 0;
        continue;
      }

      if (commandLength_ < sizeof(command_) - 1) {
        command_[commandLength_++] = received;
      } else {
        commandLength_ = 0;
      }
    }
  }

  static void writeOrientation(Print &serial, FilterMode mode,
                               const OrientationAngles &angles) {
    serial.print("ORI,");
    serial.print(static_cast<uint8_t>(mode));
    serial.print(',');
    serial.print(angles.rollDeg, 2);
    serial.print(',');
    serial.print(angles.pitchDeg, 2);
    serial.print(',');
    serial.println(angles.yawDeg, 2);
  }

 private:
  void executeCommand(OrientationFusion &fusion) {
    if (strcmp(command_, "FILTER,0") == 0) {
      fusion.selectMode(FilterMode::GyroOnly);
    } else if (strcmp(command_, "FILTER,1") == 0) {
      fusion.selectMode(FilterMode::Madgwick);
    } else if (strcmp(command_, "FILTER,2") == 0) {
      fusion.selectMode(FilterMode::Kalman);
    }
  }

  char command_[16] = {};
  uint8_t commandLength_ = 0;
};
