// Copyright (c) 2026 Jean Elsner
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Connects to a robot, reads one state and loads the model, printing both as
// JSON. Read-only: nothing is commanded. Used against fake_robot.py, and on
// hardware to check a protocol the build falls back to.
//
//   read_state <robot-hostname>

#include <franka/model.h>
#include <franka/robot.h>

#include <array>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

template <size_t N>
void print(const char* name, const std::array<double, N>& values, bool last = false) {
  std::cout << "  \"" << name << "\": [";
  for (size_t i = 0; i < N; i++) {
    std::cout << (i ? ", " : "") << values[i];
  }
  std::cout << "]" << (last ? "\n" : ",\n");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <robot-hostname>\n";
    return 2;
  }
  franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
  franka::RobotState state = robot.readOnce();
  franka::Model model = robot.loadModel();

  std::cout << std::setprecision(17) << "{\n";
  std::cout << "  \"server_version\": " << robot.serverVersion() << ",\n";
  std::cout << "  \"urdf_bytes\": " << robot.getRobotModel().size() << ",\n";
  std::cout << "  \"time_ms\": " << state.time.toMSec() << ",\n";
  std::cout << "  \"robot_mode\": " << static_cast<int>(state.robot_mode) << ",\n";
  std::cout << "  \"control_command_success_rate\": " << state.control_command_success_rate
            << ",\n";
  std::cout << "  \"errors\": " << std::string(state.current_errors) << ",\n";
  std::cout << "  \"m_ee\": " << state.m_ee << ",\n";
  print("q", state.q);
  print("dq", state.dq);
  print("tau_J", state.tau_J);
  print("tau_ext_hat_filtered", state.tau_ext_hat_filtered);
  print("O_T_EE", state.O_T_EE);
  print("O_ddP_O", state.O_ddP_O);
  print("elbow", state.elbow);
  print("theta", state.theta);
  print("dtheta", state.dtheta);
  print("gravity", model.gravity(state), true);
  std::cout << "}\n";
  return 0;
}
