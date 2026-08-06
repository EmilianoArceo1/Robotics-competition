#ifndef EXPLORATION_ROBOT_EXPLORATION__PLANNING__TRACK_HPP_
#define EXPLORATION_ROBOT_EXPLORATION__PLANNING__TRACK_HPP_

#include "exploration_robot_control/acceleration_controller.hpp"
#include "exploration_robot_control/robot_state.hpp"
#include "exploration_robot_exploration/planning/waypoints.hpp"
#include "exploration_robot_exploration/types.hpp"

namespace exploration_robot_exploration
{

class Track
{
public:
  explicit Track(
    exploration_robot_control::AccelerationController & controller,
    double waypoint_tolerance = 0.10,
    double target_speed = 1.0,
    double linear_gain = 1.5,
    double angular_gain = 4.0,
    double angular_damping = 1.5);
  virtual ~Track() = default;

  virtual CoordinateMatrix plan_route(
    const Coordinate & start, const Coordinate & goal) = 0;

  const Waypoints & create_route(const Coordinate & start, const Coordinate & goal);
  bool follow_waypoint(const exploration_robot_control::RobotState & state);
  [[nodiscard]] bool route_complete() const noexcept;
  [[nodiscard]] const Waypoints & waypoints() const noexcept;
  [[nodiscard]] exploration_robot_control::AccelerationController & controller() noexcept;

private:
  exploration_robot_control::AccelerationController & controller_;
  Waypoints waypoints_;
  double waypoint_tolerance_;
  double target_speed_;
  double linear_gain_;
  double angular_gain_;
  double angular_damping_;
};

}  // namespace exploration_robot_exploration

#endif  // EXPLORATION_ROBOT_EXPLORATION__PLANNING__TRACK_HPP_
