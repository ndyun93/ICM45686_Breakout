#pragma once

#include <math.h>
#include <MadgwickAHRS.h>

#include "AngleKalman.h"
#include "ImuConfig.h"

class OrientationFusion {
 public:
  void begin(float sampleFrequencyHz) {
    madgwick_.begin(sampleFrequencyHz);
  }

  void selectMode(FilterMode newMode) {
    if (newMode == mode_) {
      return;
    }

    // Start the newly selected estimator from the current displayed attitude
    // so Processing can switch modes without a large discontinuity.
    if (newMode == FilterMode::GyroOnly) {
      gyroOnly_ = output_;
    } else if (newMode == FilterMode::Kalman) {
      kalmanRoll_.reset(output_.rollDeg);
      kalmanPitch_.reset(output_.pitchDeg);
      kalmanYawDeg_ = output_.yawDeg;
      kalmanInitialized_ = true;
    }

    mode_ = newMode;
  }

  FilterMode mode() const {
    return mode_;
  }

  OrientationAngles update(const ImuSample &sample, float dtSec) {
    madgwick_.updateIMU(sample.gyroXDps, sample.gyroYDps, sample.gyroZDps,
                        sample.accelXG, sample.accelYG, sample.accelZG);

    OrientationAngles kalmanAngles = updateKalman(sample, dtSec);

    if (mode_ == FilterMode::GyroOnly) {
      gyroOnly_.rollDeg = wrapDegrees(gyroOnly_.rollDeg + sample.gyroXDps * dtSec);
      gyroOnly_.pitchDeg = wrapDegrees(gyroOnly_.pitchDeg + sample.gyroYDps * dtSec);
      gyroOnly_.yawDeg = wrapDegrees(gyroOnly_.yawDeg + sample.gyroZDps * dtSec);
      output_ = gyroOnly_;
    } else if (mode_ == FilterMode::Kalman) {
      output_ = kalmanAngles;
    } else {
      output_.rollDeg = madgwick_.getRoll();
      output_.pitchDeg = madgwick_.getPitch();
      output_.yawDeg = madgwick_.getYaw();
    }

    return output_;
  }

 private:
  OrientationAngles updateKalman(const ImuSample &sample, float dtSec) {
    const float accelMagnitude =
        sqrtf(sample.accelXG * sample.accelXG +
              sample.accelYG * sample.accelYG +
              sample.accelZG * sample.accelZG);
    const bool accelIsGravity =
        accelMagnitude >= ImuConfig::KALMAN_ACCEL_MIN_G &&
        accelMagnitude <= ImuConfig::KALMAN_ACCEL_MAX_G;

    const float accelRollDeg =
        atan2f(sample.accelYG, sample.accelZG) * ImuConfig::RADIANS_TO_DEGREES;
    const float accelPitchDeg =
        atan2f(-sample.accelXG,
               sqrtf(sample.accelYG * sample.accelYG +
                     sample.accelZG * sample.accelZG)) *
        ImuConfig::RADIANS_TO_DEGREES;

    OrientationAngles angles;
    if (!kalmanInitialized_) {
      kalmanRoll_.reset(accelRollDeg);
      kalmanPitch_.reset(accelPitchDeg);
      kalmanYawDeg_ = madgwick_.getYaw();
      kalmanInitialized_ = true;
      angles.rollDeg = accelRollDeg;
      angles.pitchDeg = accelPitchDeg;
    } else {
      angles.rollDeg = kalmanRoll_.update(
          accelRollDeg, sample.gyroXDps, dtSec, accelIsGravity);
      angles.pitchDeg = kalmanPitch_.update(
          accelPitchDeg, sample.gyroYDps, dtSec, accelIsGravity);
    }

    kalmanYawDeg_ = wrapDegrees(kalmanYawDeg_ + sample.gyroZDps * dtSec);
    angles.yawDeg = kalmanYawDeg_;
    return angles;
  }

  Madgwick madgwick_;
  AngleKalman kalmanRoll_;
  AngleKalman kalmanPitch_;

  FilterMode mode_ = FilterMode::Madgwick;
  bool kalmanInitialized_ = false;
  float kalmanYawDeg_ = 0.0f;
  OrientationAngles gyroOnly_ = {0.0f, 0.0f, 0.0f};
  OrientationAngles output_ = {0.0f, 0.0f, 0.0f};
};
