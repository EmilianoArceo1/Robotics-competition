#ifndef EXPLORATION_ROBOT_CONTROL__ACCELERATION_CONTROLLER_HPP_
#define EXPLORATION_ROBOT_CONTROL__ACCELERATION_CONTROLLER_HPP_

#include <utility>

namespace exploration_robot_control
{

struct ControlLimits
{
  double max_linear_acceleration{2.0};
  double max_angular_acceleration{4.0};
  double max_linear_speed{3.0};
  double max_angular_speed{6.0};

  void validate() const;
};

struct AccelerationCommand
{
  double linear{0.0};
  double angular{0.0};
};

struct VelocityCommand
{
  double linear{0.0};
  double angular{0.0};
};

class AccelerationController
{
public:
  explicit AccelerationController(ControlLimits limits = {});

  void apply_control(double linear_acceleration, double angular_acceleration);
  VelocityCommand update(double dt_seconds);
  void reset(double linear_velocity = 0.0, double angular_velocity = 0.0);

  [[nodiscard]] const ControlLimits & limits() const noexcept;
  [[nodiscard]] const AccelerationCommand & acceleration() const noexcept;
  [[nodiscard]] const VelocityCommand & velocity() const noexcept;

private:
  static double clamp(double value, double lower, double upper) noexcept;

  ControlLimits limits_;
  AccelerationCommand acceleration_;
  VelocityCommand velocity_;
};

}  // namespace exploration_robot_control

#endif  // EXPLORATION_ROBOT_CONTROL__ACCELERATION_CONTROLLER_HPP_
