#pragma once

#include <cpp_mobile_robot_kinematics/types.hpp>

namespace cpp_mobile_robot_kinematics {

constexpr double pi() { return 3.141592653589793238462643383279502884; }

double normalize_angle(double angle);
Pose2D integrate_body_twist(const Pose2D &pose, const Twist2D &body_twist,
                            double dt);

} // namespace cpp_mobile_robot_kinematics

