#pragma once

#include <cpp_mobile_robot_kinematics/types.hpp>

namespace cpp_mobile_robot_kinematics {

struct AckermannDriveParameters {
  double wheel_base{0.0};
  double max_steering_angle{0.7853981633974483};
};

struct AckermannCommand {
  double speed{0.0};
  double steering_angle{0.0};
};

class AckermannDriveKinematics {
public:
  explicit AckermannDriveKinematics(AckermannDriveParameters parameters);

  AckermannCommand inverse(const Twist2D &body_twist) const;
  Twist2D forward(const AckermannCommand &command) const;
  Pose2D integrate(const Pose2D &pose, const AckermannCommand &command,
                   double dt) const;

  const AckermannDriveParameters &parameters() const { return parameters_; }

private:
  AckermannDriveParameters parameters_;
};

} // namespace cpp_mobile_robot_kinematics

