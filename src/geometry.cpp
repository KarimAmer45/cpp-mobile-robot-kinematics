#include <cpp_mobile_robot_kinematics/geometry.hpp>

#include <cmath>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>

namespace cpp_mobile_robot_kinematics {
namespace {

constexpr double kEpsilon = 1e-12;

void require_finite(double value, const char *name) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string{name} + " must be finite");
  }
}

} // namespace

double normalize_angle(double angle) {
  require_finite(angle, "angle");

  const double two_pi = 2.0 * pi();
  double normalized = std::fmod(angle + pi(), two_pi);
  if (normalized < 0.0) {
    normalized += two_pi;
  }
  return normalized - pi();
}

Pose2D integrate_body_twist(const Pose2D &pose, const Twist2D &body_twist,
                            double dt) {
  require_finite(pose.x, "pose.x");
  require_finite(pose.y, "pose.y");
  require_finite(pose.yaw, "pose.yaw");
  require_finite(body_twist.linear_x, "body_twist.linear_x");
  require_finite(body_twist.linear_y, "body_twist.linear_y");
  require_finite(body_twist.angular_z, "body_twist.angular_z");
  require_finite(dt, "dt");

  const double theta = body_twist.angular_z * dt;
  double body_dx = body_twist.linear_x * dt;
  double body_dy = body_twist.linear_y * dt;

  if (std::abs(body_twist.angular_z) > kEpsilon) {
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    body_dx = (body_twist.linear_x * sin_theta +
               body_twist.linear_y * (cos_theta - 1.0)) /
              body_twist.angular_z;
    body_dy = (body_twist.linear_x * (1.0 - cos_theta) +
               body_twist.linear_y * sin_theta) /
              body_twist.angular_z;
  }

  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);

  return {pose.x + cos_yaw * body_dx - sin_yaw * body_dy,
          pose.y + sin_yaw * body_dx + cos_yaw * body_dy,
          normalize_angle(pose.yaw + theta)};
}

std::ostream &operator<<(std::ostream &stream, const Pose2D &pose) {
  return stream << "Pose2D{x=" << pose.x << ", y=" << pose.y
                << ", yaw=" << pose.yaw << "}";
}

std::ostream &operator<<(std::ostream &stream, const Twist2D &twist) {
  return stream << "Twist2D{linear_x=" << twist.linear_x
                << ", linear_y=" << twist.linear_y
                << ", angular_z=" << twist.angular_z << "}";
}

std::ostream &operator<<(std::ostream &stream, const WheelSpeeds &speeds) {
  return stream << "WheelSpeeds{left=" << speeds.left
                << ", right=" << speeds.right << "}";
}

std::ostream &operator<<(std::ostream &stream,
                         const FourWheelSpeeds &speeds) {
  return stream << "FourWheelSpeeds{front_left=" << speeds.front_left
                << ", front_right=" << speeds.front_right
                << ", rear_left=" << speeds.rear_left
                << ", rear_right=" << speeds.rear_right << "}";
}

} // namespace cpp_mobile_robot_kinematics
