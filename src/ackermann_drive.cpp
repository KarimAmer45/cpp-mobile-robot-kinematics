#include <cpp_mobile_robot_kinematics/ackermann_drive.hpp>

#include <cpp_mobile_robot_kinematics/geometry.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace cpp_mobile_robot_kinematics {
namespace {

constexpr double kLinearTolerance = 1e-9;
constexpr double kLateralTolerance = 1e-9;

void require_positive(double value, const char *name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string{name} + " must be positive");
  }
}

void require_non_negative(double value, const char *name) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string{name} + " must be non-negative");
  }
}

void require_finite(double value, const char *name) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string{name} + " must be finite");
  }
}

void require_steering_limit(double steering_angle, double max_steering_angle) {
  if (std::abs(steering_angle) > max_steering_angle) {
    throw std::invalid_argument("steering_angle exceeds max_steering_angle");
  }
}

} // namespace

AckermannDriveKinematics::AckermannDriveKinematics(
    AckermannDriveParameters parameters)
    : parameters_{parameters} {
  require_positive(parameters_.wheel_base, "wheel_base");
  require_non_negative(parameters_.max_steering_angle, "max_steering_angle");
  if (parameters_.max_steering_angle >= pi() * 0.5) {
    throw std::invalid_argument("max_steering_angle must be less than pi / 2");
  }
}

AckermannCommand AckermannDriveKinematics::inverse(
    const Twist2D &body_twist) const {
  require_finite(body_twist.linear_x, "body_twist.linear_x");
  require_finite(body_twist.linear_y, "body_twist.linear_y");
  require_finite(body_twist.angular_z, "body_twist.angular_z");

  if (std::abs(body_twist.linear_y) > kLateralTolerance) {
    throw std::invalid_argument(
        "ackermann drive cannot command lateral velocity");
  }

  if (std::abs(body_twist.linear_x) <= kLinearTolerance) {
    if (std::abs(body_twist.angular_z) <= kLinearTolerance) {
      return {0.0, 0.0};
    }
    throw std::invalid_argument(
        "ackermann drive cannot rotate in place with zero speed");
  }

  const double steering_angle =
      std::atan(parameters_.wheel_base * body_twist.angular_z /
                body_twist.linear_x);
  require_steering_limit(steering_angle, parameters_.max_steering_angle);
  return {body_twist.linear_x, steering_angle};
}

Twist2D AckermannDriveKinematics::forward(
    const AckermannCommand &command) const {
  require_finite(command.speed, "command.speed");
  require_finite(command.steering_angle, "command.steering_angle");
  require_steering_limit(command.steering_angle,
                         parameters_.max_steering_angle);

  return {command.speed, 0.0,
          command.speed * std::tan(command.steering_angle) /
              parameters_.wheel_base};
}

Pose2D AckermannDriveKinematics::integrate(
    const Pose2D &pose, const AckermannCommand &command, double dt) const {
  return integrate_body_twist(pose, forward(command), dt);
}

} // namespace cpp_mobile_robot_kinematics

