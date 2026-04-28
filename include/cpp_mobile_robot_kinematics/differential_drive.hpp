#pragma once

#include <cpp_mobile_robot_kinematics/types.hpp>

namespace cpp_mobile_robot_kinematics {

struct DifferentialDriveParameters {
  double wheel_radius{0.0};
  double track_width{0.0};
};

class DifferentialDriveKinematics {
public:
  explicit DifferentialDriveKinematics(DifferentialDriveParameters parameters);

  WheelSpeeds inverse(const Twist2D &body_twist) const;
  Twist2D forward(const WheelSpeeds &wheel_speeds) const;
  Pose2D integrate(const Pose2D &pose, const WheelSpeeds &wheel_speeds,
                   double dt) const;

  const DifferentialDriveParameters &parameters() const { return parameters_; }

private:
  DifferentialDriveParameters parameters_;
};

} // namespace cpp_mobile_robot_kinematics

