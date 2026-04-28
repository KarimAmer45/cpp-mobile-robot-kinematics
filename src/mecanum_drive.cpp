#include <cpp_mobile_robot_kinematics/mecanum_drive.hpp>

#include <cpp_mobile_robot_kinematics/geometry.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace cpp_mobile_robot_kinematics {
namespace {

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

MecanumDriveKinematics::MecanumDriveKinematics(
    MecanumDriveParameters parameters)
    : parameters_{parameters} {
  require_positive(parameters_.wheel_radius, "wheel_radius");
  require_positive(parameters_.wheel_base, "wheel_base");
  require_positive(parameters_.track_width, "track_width");
}

FourWheelSpeeds MecanumDriveKinematics::inverse(
    const Twist2D &body_twist) const {
  require_finite(body_twist.linear_x, "body_twist.linear_x");
  require_finite(body_twist.linear_y, "body_twist.linear_y");
  require_finite(body_twist.angular_z, "body_twist.angular_z");

  const double k = rotation_coupling();
  const double r = parameters_.wheel_radius;

  return {(body_twist.linear_x - body_twist.linear_y -
           k * body_twist.angular_z) /
              r,
          (body_twist.linear_x + body_twist.linear_y +
           k * body_twist.angular_z) /
              r,
          (body_twist.linear_x + body_twist.linear_y -
           k * body_twist.angular_z) /
              r,
          (body_twist.linear_x - body_twist.linear_y +
           k * body_twist.angular_z) /
              r};
}

Twist2D MecanumDriveKinematics::forward(
    const FourWheelSpeeds &wheel_speeds) const {
  require_finite(wheel_speeds.front_left, "wheel_speeds.front_left");
  require_finite(wheel_speeds.front_right, "wheel_speeds.front_right");
  require_finite(wheel_speeds.rear_left, "wheel_speeds.rear_left");
  require_finite(wheel_speeds.rear_right, "wheel_speeds.rear_right");

  const double r = parameters_.wheel_radius;
  const double k = rotation_coupling();

  return {r * (wheel_speeds.front_left + wheel_speeds.front_right +
               wheel_speeds.rear_left + wheel_speeds.rear_right) *
              0.25,
          r * (-wheel_speeds.front_left + wheel_speeds.front_right +
               wheel_speeds.rear_left - wheel_speeds.rear_right) *
              0.25,
          r * (-wheel_speeds.front_left + wheel_speeds.front_right -
               wheel_speeds.rear_left + wheel_speeds.rear_right) /
              (4.0 * k)};
}

Pose2D MecanumDriveKinematics::integrate(
    const Pose2D &pose, const FourWheelSpeeds &wheel_speeds, double dt) const {
  return integrate_body_twist(pose, forward(wheel_speeds), dt);
}

double MecanumDriveKinematics::rotation_coupling() const {
  return 0.5 * (parameters_.wheel_base + parameters_.track_width);
}

} // namespace cpp_mobile_robot_kinematics

