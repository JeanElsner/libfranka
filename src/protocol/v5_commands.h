// Copyright (c) 2026 Jean Elsner
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

/**
 * @file v5_commands.h
 * Translation between the current protocol's commands, which the rest of
 * libfranka is written against, and their protocol version 5 equivalents.
 *
 * Outgoing messages are translated to v5 and incoming responses back, so code
 * above Robot::Impl's send and receive points only ever sees current types.
 * The differences that matter:
 *
 *  - Command identifiers: v5 still has GetCartesianLimit and SetFilters, so
 *    every command after StopMove has a different number. Using the v5 types
 *    gets this right without numbers appearing anywhere.
 *  - Response statuses: later protocols insert safety function statuses in the
 *    middle of the Move, StopMove and AutomaticErrorRecovery lists, so those
 *    are translated by name, never cast.
 *  - Pure torque control: the current protocol runs it as a Move without a
 *    motion generator (kNone) and ends it with ControllerCommand's
 *    torque_command_finished. v5 has neither; libfranka 0.9.2 ran it as a joint
 *    velocity motion generator that commands zero and finished that.
 */

#include <optional>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <vector>

#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>

#include "v5.h"

namespace franka::protocol::v5 {

namespace current = ::research_interface::robot;
namespace wire = ::franka::protocol::v5::research_interface::robot;

/// The v5 command for a current one; undefined for commands v5 does not have.
template <typename T>
struct Counterpart;

#define FRANKA_V5_COUNTERPART(name)   \
  template <>                         \
  struct Counterpart<current::name> { \
    using type = wire::name;          \
  }

FRANKA_V5_COUNTERPART(Move);
FRANKA_V5_COUNTERPART(StopMove);
FRANKA_V5_COUNTERPART(AutomaticErrorRecovery);
FRANKA_V5_COUNTERPART(SetCollisionBehavior);
FRANKA_V5_COUNTERPART(SetJointImpedance);
FRANKA_V5_COUNTERPART(SetCartesianImpedance);
FRANKA_V5_COUNTERPART(SetGuidingMode);
FRANKA_V5_COUNTERPART(SetEEToK);
FRANKA_V5_COUNTERPART(SetNEToEE);
FRANKA_V5_COUNTERPART(SetLoad);

#undef FRANKA_V5_COUNTERPART

template <typename T, typename = void>
struct HasCounterpart : std::false_type {};
template <typename T>
struct HasCounterpart<T, std::void_t<typename Counterpart<T>::type>> : std::true_type {};

// ---------------------------------------------------------------- statuses

#define FRANKA_V5_STATUS(name) \
  case From::name:             \
    return To::name

inline current::Move::Status translate(wire::Move::Status status) {
  using From = wire::Move::Status;
  using To = current::Move::Status;
  switch (status) {
    FRANKA_V5_STATUS(kSuccess);
    FRANKA_V5_STATUS(kMotionStarted);
    FRANKA_V5_STATUS(kPreempted);
    FRANKA_V5_STATUS(kCommandNotPossibleRejected);
    FRANKA_V5_STATUS(kStartAtSingularPoseRejected);
    FRANKA_V5_STATUS(kInvalidArgumentRejected);
    FRANKA_V5_STATUS(kReflexAborted);
    FRANKA_V5_STATUS(kEmergencyAborted);
    FRANKA_V5_STATUS(kInputErrorAborted);
    FRANKA_V5_STATUS(kAborted);
  }
  return To::kAborted;
}

inline current::StopMove::Status translate(wire::StopMove::Status status) {
  using From = wire::StopMove::Status;
  using To = current::StopMove::Status;
  switch (status) {
    FRANKA_V5_STATUS(kSuccess);
    FRANKA_V5_STATUS(kCommandNotPossibleRejected);
    FRANKA_V5_STATUS(kEmergencyAborted);
    FRANKA_V5_STATUS(kReflexAborted);
    FRANKA_V5_STATUS(kAborted);
  }
  return To::kAborted;
}

inline current::AutomaticErrorRecovery::Status translate(
    wire::AutomaticErrorRecovery::Status status) {
  using From = wire::AutomaticErrorRecovery::Status;
  using To = current::AutomaticErrorRecovery::Status;
  switch (status) {
    FRANKA_V5_STATUS(kSuccess);
    FRANKA_V5_STATUS(kCommandNotPossibleRejected);
    FRANKA_V5_STATUS(kManualErrorRecoveryRequiredRejected);
    FRANKA_V5_STATUS(kReflexAborted);
    FRANKA_V5_STATUS(kEmergencyAborted);
    FRANKA_V5_STATUS(kAborted);
  }
  return To::kAborted;
}

#undef FRANKA_V5_STATUS

/// The setters share CommandBase's status list, which later protocols only append to.
template <typename T5, typename T>
typename T::Status translateStatus(typename T5::Status status) {
  if constexpr (std::is_same_v<T, current::Move> || std::is_same_v<T, current::StopMove> ||
                std::is_same_v<T, current::AutomaticErrorRecovery>) {
    return translate(status);
  } else {
    static_assert(static_cast<int>(T5::Status::kCommandNotPossibleRejected) ==
                      static_cast<int>(T::Status::kCommandNotPossibleRejected),
                  "setter statuses must share their numbering");
    return static_cast<typename T::Status>(status);
  }
}

/// A v5 response as the current protocol's response type.
template <typename T>
typename T::Response translateResponse(const typename Counterpart<T>::type::Response& response) {
  return typename T::Response(translateStatus<typename Counterpart<T>::type, T>(response.status));
}

// ---------------------------------------------------------------- requests

/// Move as v5 sends it. Pure torque control becomes a joint velocity motion that commands zero.
inline wire::Move::Request moveRequest(
    current::Move::ControllerMode controller_mode,
    current::Move::MotionGeneratorMode motion_generator_mode,
    const current::Move::Deviation& maximum_path_deviation,
    const current::Move::Deviation& maximum_goal_pose_deviation,
    bool use_async_motion_generator = false,
    const std::optional<std::vector<double>>& /* maximum_velocity */ = std::nullopt) {
  if (use_async_motion_generator) {
    throw std::invalid_argument(
        "libfranka: the asynchronous motion generator needs protocol version 9 or later; this "
        "robot speaks version 5.");
  }
  auto mode = motion_generator_mode == current::Move::MotionGeneratorMode::kNone
                  ? wire::Move::MotionGeneratorMode::kJointVelocity
                  : static_cast<wire::Move::MotionGeneratorMode>(motion_generator_mode);
  auto deviation = [](const current::Move::Deviation& d) {
    return wire::Move::Deviation(d.translation, d.rotation, d.elbow);
  };
  return wire::Move::Request(static_cast<wire::Move::ControllerMode>(controller_mode), mode,
                             deviation(maximum_path_deviation),
                             deviation(maximum_goal_pose_deviation));
}

inline wire::Move::Request moveRequest(const current::Move::Request& request) {
  return moveRequest(request.controller_mode, request.motion_generator_mode,
                     request.maximum_path_deviation, request.maximum_goal_pose_deviation,
                     request.use_async_motion_generator);
}

/**
 * The v5 request for a current command, from either the current request or the arguments to
 * construct it. Apart from Move, the requests panda-py sends are byte-identical in both versions.
 */
template <typename T, typename... TArgs>
typename Counterpart<T>::type::Request request(TArgs&&... args) {
  using T5 = typename Counterpart<T>::type;
  constexpr bool kIsRequest =
      sizeof...(TArgs) == 1 && (std::is_same_v<std::decay_t<TArgs>, typename T::Request> && ...);
  if constexpr (std::is_same_v<T, current::Move>) {
    return moveRequest(std::forward<TArgs>(args)...);
  } else if constexpr (kIsRequest) {
    static_assert(sizeof(typename T5::Request) == sizeof(typename T::Request),
                  "only requests with the same layout can be passed through");
    const auto& current_request = std::get<0>(std::forward_as_tuple(args...));
    return *reinterpret_cast<const typename T5::Request*>(&current_request);
  } else {
    return typename T5::Request(std::forward<TArgs>(args)...);
  }
}

// ---------------------------------------------------------------- 1 kHz

/**
 * The robot command as v5 sends it. For pure torque control, the motion
 * generator that v5 runs commands zero velocity and finishes when the torque
 * command does.
 */
inline wire::RobotCommand robotCommand(const current::RobotCommand& command, bool torque_only) {
  wire::RobotCommand converted{};
  converted.message_id = command.message_id;
  converted.motion.q_c = command.motion.q_c;
  converted.motion.dq_c = command.motion.dq_c;
  converted.motion.O_T_EE_c = command.motion.O_T_EE_c;
  converted.motion.O_dP_EE_c = command.motion.O_dP_EE_c;
  converted.motion.elbow_c = command.motion.elbow_c;
  converted.motion.valid_elbow = command.motion.valid_elbow;
  converted.motion.motion_generation_finished = command.motion.motion_generation_finished;
  converted.control.tau_J_d = command.control.tau_J_d;
  if (torque_only) {
    converted.motion.dq_c = {};
    converted.motion.motion_generation_finished = command.control.torque_command_finished;
  }
  return converted;
}

}  // namespace franka::protocol::v5
