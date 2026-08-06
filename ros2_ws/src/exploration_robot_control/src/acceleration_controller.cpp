#include "exploration_robot_control/acceleration_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace exploration_robot_control
{

void ControlLimits::validate() const
{
  const bool finite =
    std::isfinite(max_linear_acceleration) &&
    std::isfinite(max_angular_acceleration) &&
    std::isfinite(max_linear_speed) &&
    std::isfinite(max_angular_speed);
  const bool positive =
    max_linear_acceleration > 0.0 &&
    max_angular_acceleration > 0.0 &&
    max_linear_speed > 0.0 &&
    max_angular_speed > 0.0;
  if (!finite || !positive) {
    throw std::invalid_argument("Control limits must be finite and positive");
  }
}

AccelerationController::AccelerationController(ControlLimits limits)
: limits_(limits)
{
  limits_.validate();
}

void AccelerationController::apply_control(
  const double linear_acceleration,
  const double angular_acceleration)
{
  if (!std::isfinite(linear_acceleration) || !std::isfinite(angular_acceleration)) {
    throw std::invalid_argument("Acceleration commands must be finite");
  }
  acceleration_.linear = clamp(
    linear_acceleration,
    -limits_.max_linear_acceleration,
    limits_.max_linear_acceleration);
  acceleration_.angular = clamp(
    angular_acceleration,
    -limits_.max_angular_acceleration,
    limits_.max_angular_acceleration);
}

VelocityCommand AccelerationController::update(const double dt_seconds)
{
  if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
    throw std::invalid_argument("dt_seconds must be finite and greater than zero");
  }
  velocity_.linear = clamp(
    velocity_.linear + acceleration_.linear * dt_seconds,
    -limits_.max_linear_speed,
    limits_.max_linear_speed);
  velocity_.angular = clamp(
    velocity_.angular + acceleration_.angular * dt_seconds,
    -limits_.max_angular_speed,
    limits_.max_angular_speed);
  return velocity_;
}

void AccelerationController::reset(
  const double linear_velocity,
  const double angular_velocity)
{
  if (!std::isfinite(linear_velocity) || !std::isfinite(angular_velocity)) {
    throw std::invalid_argument("Velocity values must be finite");
  }
  velocity_.linear = clamp(
    linear_velocity, -limits_.max_linear_speed, limits_.max_linear_speed);
  velocity_.angular = clamp(
    angular_velocity, -limits_.max_angular_speed, limits_.max_angular_speed);
  acceleration_ = {};
}

const ControlLimits & AccelerationController::limits() const noexcept
{
  return limits_;
}

const AccelerationCommand & AccelerationController::acceleration() const noexcept
{
  return acceleration_;
}

const VelocityCommand & AccelerationController::velocity() const noexcept
{
  return velocity_;
}

double AccelerationController::clamp(
  const double value, const double lower, const double upper) noexcept
{
  return std::max(lower, std::min(value, upper));
}

}  // namespace exploration_robot_control
