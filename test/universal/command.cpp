// Copyright (c) 2026 Jean Elsner
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Exercises every command panda-py sends: all setters, error recovery, a pure
// torque control loop and a joint velocity motion. Used against fake_robot.py,
// which checks that each reaches it in the robot's protocol. It moves a real
// robot: do not point it at one.
//
//   command <robot-hostname>

#include <franka/exception.h>
#include <franka/robot.h>

#include <array>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <robot-hostname>\n";
    return 2;
  }
  try {
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
    std::cout << "server version " << robot.serverVersion() << '\n';

    robot.setCollisionBehavior({{20, 20, 18, 18, 16, 14, 12}}, {{20, 20, 18, 18, 16, 14, 12}},
                               {{20, 20, 18, 18, 16, 14, 12}}, {{20, 20, 18, 18, 16, 14, 12}},
                               {{20, 20, 20, 25, 25, 25}}, {{20, 20, 20, 25, 25, 25}},
                               {{20, 20, 20, 25, 25, 25}}, {{20, 20, 20, 25, 25, 25}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    robot.setGuidingMode({{true, true, true, false, false, false}}, false);
    robot.setEE({{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0.1034, 1}});
    robot.setK({{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}});
    robot.setLoad(0.5, {{0.01, -0.02, 0.05}}, {{1e-3, 0, 0, 0, 2e-3, 0, 0, 0, 1.5e-3}});
    robot.automaticErrorRecovery();
    std::cout << "setters and error recovery done\n";

    int ticks = 0;
    robot.control([&ticks](const franka::RobotState&, franka::Duration) -> franka::Torques {
      franka::Torques torques(std::array<double, 7>{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7});
      return ++ticks >= 100 ? franka::MotionFinished(torques) : torques;
    });
    std::cout << "torque control: " << ticks << " ticks\n";

    ticks = 0;
    robot.control([&ticks](const franka::RobotState&, franka::Duration) -> franka::JointVelocities {
      franka::JointVelocities zero(std::array<double, 7>{});
      return ++ticks >= 50 ? franka::MotionFinished(zero) : zero;
    });
    std::cout << "joint velocity motion: " << ticks << " ticks\n";

    // A step to 0.5 rad/s, which the rate limiter turns into a ramp at the robot's limits.
    ticks = 0;
    robot.control(
        [&ticks](const franka::RobotState&, franka::Duration) -> franka::JointVelocities {
          franka::JointVelocities step(std::array<double, 7>{0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
          return ++ticks >= 200 ? franka::MotionFinished(step) : step;
        },
        franka::ControllerMode::kJointImpedance, /*limit_rate=*/true);
    std::cout << "rate limited joint velocity step: " << ticks << " ticks\n";
  } catch (const franka::Exception& e) {
    std::cerr << "failed: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
