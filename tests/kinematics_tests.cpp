#include <cpp_mobile_robot_kinematics/ackermann_drive.hpp>
#include <cpp_mobile_robot_kinematics/differential_drive.hpp>
#include <cpp_mobile_robot_kinematics/geometry.hpp>
#include <cpp_mobile_robot_kinematics/mecanum_drive.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace cmrk = cpp_mobile_robot_kinematics;

namespace {

int failures = 0;

void expect_near(double actual, double expected, double tolerance,
                 const std::string &name) {
  if (std::abs(actual - expected) > tolerance) {
    std::cerr << "FAIL " << name << ": expected " << expected << ", got "
              << actual << '\n';
    ++failures;
  }
}

template <typename Callable>
void expect_throws(Callable callable, const std::string &name) {
  try {
    callable();
    std::cerr << "FAIL " << name << ": expected exception\n";
    ++failures;
  } catch (const std::invalid_argument &) {
  }
}

void differential_drive_round_trip() {
  cmrk::DifferentialDriveKinematics drive({0.1, 0.5});
  const cmrk::Twist2D command{1.0, 0.0, 0.8};
  const cmrk::WheelSpeeds wheels = drive.inverse(command);
  const cmrk::Twist2D recovered = drive.forward(wheels);

  expect_near(wheels.left, 8.0, 1e-12, "diff left wheel");
  expect_near(wheels.right, 12.0, 1e-12, "diff right wheel");
  expect_near(recovered.linear_x, command.linear_x, 1e-12, "diff vx");
  expect_near(recovered.linear_y, 0.0, 1e-12, "diff vy");
  expect_near(recovered.angular_z, command.angular_z, 1e-12, "diff wz");
}

void differential_drive_rejects_lateral_velocity() {
  cmrk::DifferentialDriveKinematics drive({0.1, 0.5});
  expect_throws([&drive] { drive.inverse({0.0, 0.1, 0.0}); },
                "diff lateral rejection");
}

void mecanum_drive_round_trip() {
  cmrk::MecanumDriveKinematics drive({0.05, 0.4, 0.3});
  const cmrk::Twist2D command{0.7, -0.2, 0.4};
  const cmrk::FourWheelSpeeds wheels = drive.inverse(command);
  const cmrk::Twist2D recovered = drive.forward(wheels);

  expect_near(recovered.linear_x, command.linear_x, 1e-12, "mecanum vx");
  expect_near(recovered.linear_y, command.linear_y, 1e-12, "mecanum vy");
  expect_near(recovered.angular_z, command.angular_z, 1e-12, "mecanum wz");
}

void ackermann_drive_round_trip() {
  cmrk::AckermannDriveKinematics drive({1.2, 0.7});
  const cmrk::AckermannCommand command{2.0, 0.3};
  const cmrk::Twist2D twist = drive.forward(command);
  const cmrk::AckermannCommand recovered = drive.inverse(twist);

  expect_near(twist.linear_x, command.speed, 1e-12, "ackermann vx");
  expect_near(twist.linear_y, 0.0, 1e-12, "ackermann vy");
  expect_near(recovered.speed, command.speed, 1e-12, "ackermann speed");
  expect_near(recovered.steering_angle, command.steering_angle, 1e-12,
              "ackermann steering");
}

void ackermann_rejects_in_place_rotation() {
  cmrk::AckermannDriveKinematics drive({1.2, 0.7});
  expect_throws([&drive] { drive.inverse({0.0, 0.0, 1.0}); },
                "ackermann in-place rejection");
}

void integrates_exact_arc() {
  const cmrk::Pose2D pose =
      cmrk::integrate_body_twist({0.0, 0.0, 0.0}, {1.0, 0.0, 1.0}, 1.0);

  expect_near(pose.x, std::sin(1.0), 1e-12, "arc x");
  expect_near(pose.y, 1.0 - std::cos(1.0), 1e-12, "arc y");
  expect_near(pose.yaw, 1.0, 1e-12, "arc yaw");
}

void normalizes_angle() {
  expect_near(cmrk::normalize_angle(3.0 * cmrk::pi()), -cmrk::pi(), 1e-12,
              "normalize 3pi");
}

} // namespace

int main() {
  differential_drive_round_trip();
  differential_drive_rejects_lateral_velocity();
  mecanum_drive_round_trip();
  ackermann_drive_round_trip();
  ackermann_rejects_in_place_rotation();
  integrates_exact_arc();
  normalizes_angle();

  if (failures != 0) {
    std::cerr << failures << " test failure(s)\n";
    return 1;
  }

  std::cout << "All kinematics tests passed\n";
  return 0;
}

