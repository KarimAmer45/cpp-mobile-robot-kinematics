#include <cpp_mobile_robot_kinematics/ackermann_drive.hpp>

#include <iostream>

namespace cmrk = cpp_mobile_robot_kinematics;

int main() {
  cmrk::AckermannDriveKinematics drive({0.85, 0.65});

  const cmrk::AckermannCommand command{1.2, 0.25};
  const cmrk::Twist2D body_twist = drive.forward(command);
  const cmrk::AckermannCommand recovered_command = drive.inverse(body_twist);
  const cmrk::Pose2D next_pose =
      drive.integrate({0.0, 0.0, 0.1}, command, 0.05);

  std::cout << "command speed:   " << command.speed << '\n';
  std::cout << "command steer:   " << command.steering_angle << '\n';
  std::cout << "body twist:      " << body_twist << '\n';
  std::cout << "recovered speed: " << recovered_command.speed << '\n';
  std::cout << "recovered steer: " << recovered_command.steering_angle << '\n';
  std::cout << "next pose:       " << next_pose << '\n';
}

