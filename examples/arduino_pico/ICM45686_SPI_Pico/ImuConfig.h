#pragma once

#include <Arduino.h>

namespace ImuConfig {

constexpr uint8_t PIN_MISO = 16;
constexpr uint8_t PIN_CS = 17;
constexpr uint8_t PIN_SCK = 18;
constexpr uint8_t PIN_MOSI = 19;

constexpr uint32_t SPI_HZ = 1000000;
constexpr uint16_t ODR_HZ = 100;
constexpr uint16_t ACCEL_FSR_G = 16;
constexpr uint16_t GYRO_FSR_DPS = 2000;
constexpr float RAW_FULL_SCALE = 32768.0f;
constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / ODR_HZ;
constexpr float SAMPLE_PERIOD_SEC = 1.0f / ODR_HZ;

constexpr float RADIANS_TO_DEGREES = 57.2957795131f;
constexpr float KALMAN_ACCEL_MIN_G = 0.75f;
constexpr float KALMAN_ACCEL_MAX_G = 1.25f;

}  // namespace ImuConfig

enum class FilterMode : uint8_t {
  GyroOnly = 0,
  Madgwick = 1,
  Kalman = 2,
};

struct ImuSample {
  float accelXG;
  float accelYG;
  float accelZG;
  float gyroXDps;
  float gyroYDps;
  float gyroZDps;
};

struct OrientationAngles {
  float rollDeg;
  float pitchDeg;
  float yawDeg;
};
