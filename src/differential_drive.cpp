#include <cpp_mobile_robot_kinematics/differential_drive.hpp>

#include <cpp_mobile_robot_kinematics/geometry.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace cpp_mobile_robot_kinematics {
namespace {

constexpr double kLateralTolerance = 1e-9;

void require_positive(double value, const char *name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string{name} + " must be positive");
  }
}

void require_finite(double value, const char *name) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string{name} + " must be finite");
  }
}

} // namespace

DifferentialDriveKinematics::DifferentialDriveKinematics(
    DifferentialDriveParameters parameters)
    : parameters_{parameters} {
  require_positive(parameters_.wheel_radius, "wheel_radius");
  require_positive(parameters_.track_width, "track_width");
}

WheelSpeeds DifferentialDriveKinematics::inverse(
    const Twist2D &body_twist) const {
  require_finite(body_twist.linear_x, "body_twist.linear_x");
  require_finite(body_twist.linear_y, "body_twist.linear_y");
  require_finite(body_twist.angular_z, "body_twist.angular_z");

  if (std::abs(body_twist.linear_y) > kLateralTolerance) {
    throw std::invalid_argument(
        "differential drive cannot command lateral velocity");
  }

  const double half_track = parameters_.track_width * 0.5;
  return {(body_twist.linear_x - body_twist.angular_z * half_track) /
              parameters_.wheel_radius,
          (body_twist.linear_x + body_twist.angular_z * half_track) /
              parameters_.wheel_radius};
}

Twist2D DifferentialDriveKinematics::forward(
    const WheelSpeeds &wheel_speeds) const {
  require_finite(wheel_speeds.left, "wheel_speeds.left");
  require_finite(wheel_speeds.right, "wheel_speeds.right");

  return {parameters_.wheel_radius * (wheel_speeds.right + wheel_speeds.left) *
              0.5,
          0.0,
          parameters_.wheel_radius * (wheel_speeds.right - wheel_speeds.left) /
              parameters_.track_width};
}

Pose2D DifferentialDriveKinematics::integrate(
    const Pose2D &pose, const WheelSpeeds &wheel_speeds, double dt) const {
  return integrate_body_twist(pose, forward(wheel_speeds), dt);
}

} // namespace cpp_mobile_robot_kinematics

