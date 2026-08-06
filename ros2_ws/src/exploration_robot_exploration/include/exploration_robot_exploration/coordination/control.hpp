#ifndef EXPLORATION_ROBOT_EXPLORATION__COORDINATION__CONTROL_HPP_
#define EXPLORATION_ROBOT_EXPLORATION__COORDINATION__CONTROL_HPP_

#include "exploration_robot_control/acceleration_controller.hpp"
#include "exploration_robot_control/robot_state.hpp"
#include "exploration_robot_exploration/mapping/belief_map.hpp"
#include "exploration_robot_exploration/planning/track.hpp"
#include "exploration_robot_exploration/sensing/sensor.hpp"
#include "exploration_robot_exploration/types.hpp"

namespace exploration_robot_exploration
{

class Control
{
public:
  Control(
    exploration_robot_control::AccelerationController & motion_controller,
    Sensor sensor = {});
  virtual ~Control() = default;

  SensorMatrix detect(
    const exploration_robot_control::RobotState & robot_state,
    const SensorMatrix & environment);

  virtual Coordinate assign_goal(
    const BeliefMap & belief_map,
    const exploration_robot_control::RobotState & robot_state) = 0;

  const Waypoints & create_route_to_assigned_goal(
    Track & track,
    const exploration_robot_control::RobotState & robot_state);

  [[nodiscard]] const BeliefMap & belief_map() const noexcept;
  [[nodiscard]] BeliefMap & belief_map() noexcept;
  [[nodiscard]] const Sensor & sensor() const noexcept;

private:
  exploration_robot_control::AccelerationController & motion_controller_;
  Sensor sensor_;
  BeliefMap belief_map_;
};

}  // namespace exploration_robot_exploration

#endif  // EXPLORATION_ROBOT_EXPLORATION__COORDINATION__CONTROL_HPP_
