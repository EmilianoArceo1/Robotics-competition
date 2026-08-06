#include "exploration_robot_exploration/coordination/control.hpp"

#include <stdexcept>
#include <utility>

namespace exploration_robot_exploration
{

Control::Control(
  exploration_robot_control::AccelerationController & motion_controller,
  Sensor sensor)
: motion_controller_(motion_controller), sensor_(std::move(sensor))
{
}

SensorMatrix Control::detect(
  const exploration_robot_control::RobotState & robot_state,
  const SensorMatrix & environment)
{
  SensorMatrix observations = sensor_.detect(robot_state, environment);
  belief_map_.update(observations);
  return observations;
}

const Waypoints & Control::create_route_to_assigned_goal(
  Track & track,
  const exploration_robot_control::RobotState & robot_state)
{
  if (&track.controller() != &motion_controller_) {
    throw std::invalid_argument("Track and Control must share the motion controller");
  }
  const Coordinate start{robot_state.x, robot_state.y};
  return track.create_route(start, assign_goal(belief_map_, robot_state));
}

const BeliefMap & Control::belief_map() const noexcept {return belief_map_;}
BeliefMap & Control::belief_map() noexcept {return belief_map_;}
const Sensor & Control::sensor() const noexcept {return sensor_;}

}  // namespace exploration_robot_exploration
