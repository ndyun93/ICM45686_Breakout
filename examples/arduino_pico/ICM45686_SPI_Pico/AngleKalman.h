#pragma once

inline float wrapDegrees(float angleDeg) {
  while (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }
  while (angleDeg < -180.0f) {
    angleDeg += 360.0f;
  }
  return angleDeg;
}

// Estimates one angle and its gyro bias. OrientationFusion owns one instance
// for Roll and another for Pitch.
class AngleKalman {
 public:
  void reset(float angleDeg) {
    angleDeg_ = wrapDegrees(angleDeg);
    biasDps_ = 0.0f;
    p00_ = 0.0f;
    p01_ = 0.0f;
    p10_ = 0.0f;
    p11_ = 0.0f;
  }

  float update(float measuredAngleDeg, float gyroRateDps, float dtSec,
               bool useAccelMeasurement) {
    const float unbiasedRateDps = gyroRateDps - biasDps_;
    angleDeg_ = wrapDegrees(angleDeg_ + dtSec * unbiasedRateDps);

    p00_ += dtSec * (dtSec * p11_ - p01_ - p10_ + Q_ANGLE);
    p01_ -= dtSec * p11_;
    p10_ -= dtSec * p11_;
    p11_ += Q_BIAS * dtSec;

    if (useAccelMeasurement) {
      correctWithAccel(measuredAngleDeg);
    }

    return angleDeg_;
  }

 private:
  void correctWithAccel(float measuredAngleDeg) {
    const float innovationDeg = wrapDegrees(measuredAngleDeg - angleDeg_);
    const float innovationVariance = p00_ + R_MEASURE;
    const float k0 = p00_ / innovationVariance;
    const float k1 = p10_ / innovationVariance;

    angleDeg_ = wrapDegrees(angleDeg_ + k0 * innovationDeg);
    biasDps_ += k1 * innovationDeg;

    const float p00Before = p00_;
    const float p01Before = p01_;
    p00_ -= k0 * p00Before;
    p01_ -= k0 * p01Before;
    p10_ -= k1 * p00Before;
    p11_ -= k1 * p01Before;
  }

  static constexpr float Q_ANGLE = 0.001f;
  static constexpr float Q_BIAS = 0.003f;
  static constexpr float R_MEASURE = 0.03f;

  float angleDeg_ = 0.0f;
  float biasDps_ = 0.0f;
  float p00_ = 0.0f;
  float p01_ = 0.0f;
  float p10_ = 0.0f;
  float p11_ = 0.0f;
};
