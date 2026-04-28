#pragma once

#include <cpp_mobile_robot_kinematics/types.hpp>

namespace cpp_mobile_robot_kinematics {

struct MecanumDriveParameters {
  double wheel_radius{0.0};
  double wheel_base{0.0};
  double track_width{0.0};
};

class MecanumDriveKinematics {
public:
  explicit MecanumDriveKinematics(MecanumDriveParameters parameters);

  FourWheelSpeeds inverse(const Twist2D &body_twist) const;
  Twist2D forward(const FourWheelSpeeds &wheel_speeds) const;
  Pose2D integrate(const Pose2D &pose, const FourWheelSpeeds &wheel_speeds,
                   double dt) const;

  const MecanumDriveParameters &parameters() const { return parameters_; }

private:
  double rotation_coupling() const;

  MecanumDriveParameters parameters_;
};

} // namespace cpp_mobile_robot_kinematics

