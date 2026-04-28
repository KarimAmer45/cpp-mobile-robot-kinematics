#include <cpp_mobile_robot_kinematics/differential_drive.hpp>

#include <iostream>

namespace cmrk = cpp_mobile_robot_kinematics;

int main() {
  cmrk::DifferentialDriveKinematics drive({0.075, 0.42});

  const cmrk::Twist2D command{0.6, 0.0, 0.8};
  const cmrk::WheelSpeeds wheel_speeds = drive.inverse(command);
  const cmrk::Twist2D recovered_twist = drive.forward(wheel_speeds);
  const cmrk::Pose2D next_pose =
      drive.integrate({0.0, 0.0, 0.0}, wheel_speeds, 0.02);

  std::cout << "command:         " << command << '\n';
  std::cout << "wheel speeds:    " << wheel_speeds << '\n';
  std::cout << "recovered twist: " << recovered_twist << '\n';
  std::cout << "next pose:       " << next_pose << '\n';
}

