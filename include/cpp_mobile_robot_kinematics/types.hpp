#pragma once

#include <ostream>

namespace cpp_mobile_robot_kinematics {

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Twist2D {
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

struct WheelSpeeds {
  double left{0.0};
  double right{0.0};
};

struct FourWheelSpeeds {
  double front_left{0.0};
  double front_right{0.0};
  double rear_left{0.0};
  double rear_right{0.0};
};

std::ostream &operator<<(std::ostream &stream, const Pose2D &pose);
std::ostream &operator<<(std::ostream &stream, const Twist2D &twist);
std::ostream &operator<<(std::ostream &stream, const WheelSpeeds &speeds);
std::ostream &operator<<(std::ostream &stream, const FourWheelSpeeds &speeds);

} // namespace cpp_mobile_robot_kinematics

