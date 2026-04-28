#include <cpp_mobile_robot_kinematics/mecanum_drive.hpp>

#include <iostream>

namespace cmrk = cpp_mobile_robot_kinematics;

int main() {
  cmrk::MecanumDriveKinematics drive({0.05, 0.36, 0.30});

  const cmrk::Twist2D command{0.4, 0.2, -0.5};
  const cmrk::FourWheelSpeeds wheel_speeds = drive.inverse(command);
  const cmrk::Twist2D recovered_twist = drive.forward(wheel_speeds);
  const cmrk::Pose2D next_pose =
      drive.integrate({1.0, 2.0, 0.3}, wheel_speeds, 0.1);

  std::cout << "command:         " << command << '\n';
  std::cout << "wheel speeds:    " << wheel_speeds << '\n';
  std::cout << "recovered twist: " << recovered_twist << '\n';
  std::cout << "next pose:       " << next_pose << '\n';
}

