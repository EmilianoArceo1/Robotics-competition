#ifndef EXPLORATION_ROBOT_EXPLORATION__SENSING__SENSOR_HPP_
#define EXPLORATION_ROBOT_EXPLORATION__SENSING__SENSOR_HPP_

#include <string>

#include "exploration_robot_control/robot_state.hpp"
#include "exploration_robot_exploration/types.hpp"

namespace exploration_robot_exploration
{

class Sensor
{
public:
  Sensor(
    std::string sensor_type = "lidar",
    double detection_radius = 10.0,
    double field_of_view_degrees = 360.0);

  [[nodiscard]] SensorMatrix detect(
    const exploration_robot_control::RobotState & robot_state,
    const SensorMatrix & environment) const;

  [[nodiscard]] const std::string & sensor_type() const noexcept;
  [[nodiscard]] double detection_radius() const noexcept;
  [[nodiscard]] double field_of_view_degrees() const noexcept;

private:
  std::string sensor_type_;
  double detection_radius_;
  double field_of_view_degrees_;
};

}  // namespace exploration_robot_exploration

#endif  // EXPLORATION_ROBOT_EXPLORATION__SENSING__SENSOR_HPP_
