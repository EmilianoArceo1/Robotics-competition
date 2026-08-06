#include "exploration_robot_exploration/sensing/sensor.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace exploration_robot_exploration
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double radians_to_degrees(const double radians)
{
  return radians * 180.0 / kPi;
}

double angular_difference(const double first, const double second)
{
  return std::remainder(first - second, 360.0);
}
}  // namespace

Sensor::Sensor(
  std::string sensor_type,
  const double detection_radius,
  const double field_of_view_degrees)
: sensor_type_(std::move(sensor_type)),
  detection_radius_(detection_radius),
  field_of_view_degrees_(field_of_view_degrees)
{
  if (sensor_type_.empty()) {
    throw std::invalid_argument("sensor_type cannot be empty");
  }
  std::transform(
    sensor_type_.begin(), sensor_type_.end(), sensor_type_.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  if (!std::isfinite(detection_radius_) || detection_radius_ <= 0.0) {
    throw std::invalid_argument("detection_radius must be finite and positive");
  }
  if (!std::isfinite(field_of_view_degrees_) ||
    field_of_view_degrees_ <= 0.0 || field_of_view_degrees_ > 360.0)
  {
    throw std::invalid_argument("field_of_view_degrees must be in (0, 360]");
  }
}

SensorMatrix Sensor::detect(
  const exploration_robot_control::RobotState & robot_state,
  const SensorMatrix & environment) const
{
  SensorMatrix detected;
  const double robot_heading = radians_to_degrees(robot_state.theta);
  for (const auto & cell : environment) {
    const double dx = cell.coordinate[0] - robot_state.x;
    const double dy = cell.coordinate[1] - robot_state.y;
    if (std::hypot(dx, dy) > detection_radius_) {
      continue;
    }
    if (field_of_view_degrees_ < 360.0) {
      const double point_heading = radians_to_degrees(std::atan2(dy, dx));
      if (std::abs(angular_difference(point_heading, robot_heading)) >
        field_of_view_degrees_ / 2.0)
      {
        continue;
      }
    }
    detected.push_back(cell);
  }
  return detected;
}

const std::string & Sensor::sensor_type() const noexcept {return sensor_type_;}
double Sensor::detection_radius() const noexcept {return detection_radius_;}
double Sensor::field_of_view_degrees() const noexcept {return field_of_view_degrees_;}

}  // namespace exploration_robot_exploration
