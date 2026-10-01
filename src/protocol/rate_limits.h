// Copyright (c) 2026 Jean Elsner
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

/**
 * @file rate_limits.h
 * The limits libfranka's rate limiter holds motion commands to, per robot.
 *
 * franka/rate_limiting.h has them as constants, which since libfranka 0.10
 * are the Franka Research 3's. The Franka Emika Robot's differ in nearly every
 * entry, so a build that drives both selects them at run time.
 */

#include <array>

#include <franka/rate_limiting.h>

namespace franka::protocol {

struct RateLimits {
  std::array<double, 7> max_joint_jerk;
  std::array<double, 7> max_joint_acceleration;
  /// Subtracted from the joint velocity limits the robot description gives.
  std::array<double, 7> joint_velocity_tolerance;
  double max_translational_jerk;
  double max_translational_acceleration;
  double max_translational_velocity;
  double max_rotational_jerk;
  double max_rotational_acceleration;
  double max_rotational_velocity;
  double max_elbow_jerk;
  double max_elbow_acceleration;
  double max_elbow_velocity;
  double min_elbow_velocity;
};

/// Franka Research 3: libfranka 0.21.3's constants, unchanged.
inline const RateLimits kFr3RateLimits{
    kMaxJointJerk,         kMaxJointAcceleration,         kJointVelocityLimitsTolerance,
    kMaxTranslationalJerk, kMaxTranslationalAcceleration, kMaxTranslationalVelocity,
    kMaxRotationalJerk,    kMaxRotationalAcceleration,    kMaxRotationalVelocity,
    kMaxElbowJerk,         kMaxElbowAcceleration,         kMaxElbowVelocity,
    kMinElbowVelocity,
};

namespace fer {

// libfranka 0.9.2's franka/rate_limiting.h, the last release for the Franka Emika Robot,
// including its margin for three lost packets, which later releases dropped.
constexpr double kTolNumberPacketsLost = 3.0;
constexpr std::array<double, 7> kMaxJointJerk{
    {7500.0 - kLimitEps, 3750.0 - kLimitEps, 5000.0 - kLimitEps, 6250.0 - kLimitEps,
     7500.0 - kLimitEps, 10000.0 - kLimitEps, 10000.0 - kLimitEps}};
constexpr std::array<double, 7> kMaxJointAcceleration{
    {15.0 - kLimitEps, 7.5 - kLimitEps, 10.0 - kLimitEps, 12.5 - kLimitEps, 15.0 - kLimitEps,
     20.0 - kLimitEps, 20.0 - kLimitEps}};
/// 0.9.2 limited joint velocity to v_max - kLimitEps - 3 kDeltaT a_max.
constexpr std::array<double, 7> joint_velocity_tolerance() {
  std::array<double, 7> tolerance{};
  for (size_t i = 0; i < tolerance.size(); i++) {
    tolerance[i] = kLimitEps + kTolNumberPacketsLost * kDeltaT * kMaxJointAcceleration[i];
  }
  return tolerance;
}
constexpr double kMaxTranslationalJerk = 6500.0 - kLimitEps;
constexpr double kMaxTranslationalAcceleration = 13.0 - kLimitEps;
constexpr double kMaxTranslationalVelocity =
    2.0 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxTranslationalAcceleration;
constexpr double kMaxRotationalJerk = 12500.0 - kLimitEps;
constexpr double kMaxRotationalAcceleration = 25.0 - kLimitEps;
constexpr double kMaxRotationalVelocity =
    2.5 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxRotationalAcceleration;
constexpr double kMaxElbowJerk = 5000 - kLimitEps;
constexpr double kMaxElbowAcceleration = 10.0 - kLimitEps;
constexpr double kMaxElbowVelocity =
    2.175 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxElbowAcceleration;

}  // namespace fer

/// Franka Emika Robot: libfranka 0.9.2's limits. Its elbow limit was symmetric.
inline const RateLimits kFerRateLimits{
    fer::kMaxJointJerk,         fer::kMaxJointAcceleration,         fer::joint_velocity_tolerance(),
    fer::kMaxTranslationalJerk, fer::kMaxTranslationalAcceleration, fer::kMaxTranslationalVelocity,
    fer::kMaxRotationalJerk,    fer::kMaxRotationalAcceleration,    fer::kMaxRotationalVelocity,
    fer::kMaxElbowJerk,         fer::kMaxElbowAcceleration,         fer::kMaxElbowVelocity,
    -fer::kMaxElbowVelocity,
};

}  // namespace franka::protocol
