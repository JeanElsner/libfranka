#pragma once

#include <array>
#include <initializer_list>

/**
 * @file control_types.h
 * Contains helper types for returning values for motion generation
 * and torque control.
 */

namespace franka {

/**
 * Controller mode for FRANKA.
 */
enum ControllerMode { kMotorPD, kJointPosition, kJointImpedance, kCartesianImpedance };

/**
 * Used to decide whether to enforce realtime mode for a control loop thread.
 *
 * @see Robot::Robot
 */
enum RealtimeConfig { kEnforce, kIgnore };

/**
 * Helper type for control and motion generation loops.
 *
 * Used to determine whether to terminate a loop after the control callback
 * has returned.
 * @see @em franka::Stop variable in control_types.h
 */
class IsStop {
 public:
  /**
   * Determines whether to stop the control and motion generation loops.
   *
   * @return True if the control loop should be stopped.
   */
  bool stop() const noexcept;

 protected:
  /**
   * Creates a new instance with stop() = false;
   */
  IsStop() noexcept;
  /**
   * Creates a new instance with stop() = is_stop;
   */
  IsStop(bool is_stop) noexcept;

 private:
  bool is_stop_;
};

/**
 * Stores values for torque control.
 */
class Torques : public IsStop {
 public:
  /**
   * Creates a new Torques instance.
   *
   * @param[in] torques Desired torques in [Nm].
   */
  Torques(const std::array<double, 7>& torques) noexcept;

  /**
   * Creates a new Torques instance.
   *
   * @param[in] torques Desired torques in [Nm].
   *
   * @throw ControlException Invalid number of elements in torques.
   */
  Torques(std::initializer_list<double> torques);

  /**
   * Desired torques in [Nm].
   */
  std::array<double, 7> tau_J{};  // NOLINT (readability-identifier-naming)

 protected:
  Torques() noexcept;
};

/**
 * Stores values for joint position motion generation.
 */
class JointPositions : public IsStop {
 public:
  /**
   * Creates a new JointPositions instance.
   *
   * @param[in] joint_positions Desired joint angles in [rad].
   */
  JointPositions(const std::array<double, 7>& joint_positions) noexcept;

  /**
   * Creates a new JointPositions instance.
   *
   * @param[in] joint_positions Desired joint angles in [rad].
   *
   * @throw ControlException Invalid number of elements in joint_positions.
   */
  JointPositions(std::initializer_list<double> joint_positions);

  /**
   * Desired joint angles in [rad].
   */
  std::array<double, 7> q{};

 protected:
  JointPositions() noexcept;
};

/**
 * Stores values for joint velocity motion generation.
 */
class JointVelocities : public IsStop {
 public:
  /**
   * Creates a new JointVelocities instance.
   *
   * @param[in] joint_velocities Desired joint velocities in [rad/s].
   */
  JointVelocities(const std::array<double, 7>& joint_velocities) noexcept;

  /**
   * Creates a new JointVelocities instance.
   *
   * @param[in] joint_velocities Desired joint velocities in [rad/s].
   *
   * @throw ControlException Invalid number of elements in joint_velocities.
   */
  JointVelocities(std::initializer_list<double> joint_velocities);

  /**
   * Desired joint velocities in [rad/s].
   */
  std::array<double, 7> dq{};

 protected:
  JointVelocities() noexcept;
};

/**
 * Stores values for Cartesian pose motion generation.
 */
class CartesianPose : public IsStop {
 public:
  /**
   * Creates a new CartesianPose instance.
   *
   * @throw ControlException if cartesian_pose is not a valid vectorized
   *                         homogeneous transformation matrix (column-major).
   *
   * @param[in] cartesian_pose Desired vectorized homogeneous transformation
   * matrix \f${}_O \mathbf{T}_{EE,d}\f$, column major, that transforms from the
   * end-effector frame \f$EE\f$ to base frame \f$O\f$.
   */
  CartesianPose(const std::array<double, 16>& cartesian_pose);

  /**
   * Creates a new CartesianPose instance.
   *
   * @param[in] cartesian_pose Desired vectorized homogeneous transformation
   * matrix \f${}_O \mathbf{T}_{EE,d}\f$, column major, that transforms from the
   * end-effector frame \f$EE\f$ to base frame \f$O\f$.
   *
   * @throw ControlException if cartesian_pose is not a valid vectorized
   *                         homogeneous transformation matrix (column-major).
   */
  CartesianPose(std::initializer_list<double> cartesian_pose);

  /**
   * Homogeneous transformation \f${}_O \mathbf{T}_{EE,d}\f$, column major, that
   * transforms from the end-effector frame \f$EE\f$ to base frame \f$O\f$
   */
  std::array<double, 16> O_T_EE{};  // NOLINT (readability-identifier-naming)

 protected:
  CartesianPose() noexcept;

 private:
  void checkHomogeneousTransformation();

  /**
   * Checks a a homogeneous transformation for validity.
   *
   * @param[in] transform Homogeneous transformation to be checked,
   * passed as column major array.
   *
   * @return True if transformation has ortho-normal rotation matrix,
   * the last row is [0 0 0 1] and the array defines a column major matrix.
   */
  static bool isHomogeneousTransformation(const std::array<double, 16>& transform) noexcept;
};

/**
 * Stores values for Cartesian velocity motion generation.
 */
class CartesianVelocities : public IsStop {
 public:
  /**
   * Creates a new CartesianVelocities instance.
   *
   * @param[in] cartesian_velocities Desired Cartesian velocity w.r.t. O-frame
   * {dx in [m/s], dx in [m/s], dz in [m/s], omegax in [rad/s], omegay in
   * [rad/s], omegaz in [rad/s]}.
   */
  CartesianVelocities(const std::array<double, 6>& cartesian_velocities);

  /**
   * Creates a new CartesianVelocities instance.
   *
   * @param[in] cartesian_velocities Desired Cartesian velocity w.r.t. O-frame
   * {dx in [m/s], dx in [m/s], dz in [m/s], omegax in [rad/s], omegay in
   * [rad/s], omegaz in [rad/s]}.
   *
   * @throw ControlException Invalid number of elements in cartesian_pose.
   */
  CartesianVelocities(std::initializer_list<double> cartesian_velocities);

  /**
   * Desired Cartesian velocity w.r.t. O-frame {dx in [m/s], dx in [m/s], dz in
   * [m/s], omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}
   */
  std::array<double, 6> O_dP_EE{};  // NOLINT (readability-identifier-naming)

 protected:
  CartesianVelocities() noexcept;
};

/**
 * A static instance of this class @em franka::Stop is used to signal the
 * termination of motion generation and control loops.
 *
 * @see Robot::control
 */
struct StopT final : Torques, JointPositions, JointVelocities, CartesianVelocities, CartesianPose {
  StopT() noexcept = default;
};

/**
 * Used to signal the termination of motion generation and control loops.
 *
 * @see Robot::control
 */
static const StopT Stop{};  // NOLINT (readability-identifier-naming)

}  // namespace franka
