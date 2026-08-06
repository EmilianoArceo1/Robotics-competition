#include "exploration_robot_exploration/planning/track.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace exploration_robot_exploration
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
double wrap_angle(const double angle) {return std::remainder(angle, 2.0 * kPi);}
}  // namespace

Track::Track(
  exploration_robot_control::AccelerationController & controller,
  const double waypoint_tolerance,
  const double target_speed,
  const double linear_gain,
  const double angular_gain,
  const double angular_damping)
: controller_(controller),
  waypoint_tolerance_(waypoint_tolerance),
  target_speed_(target_speed),
  linear_gain_(linear_gain),
  angular_gain_(angular_gain),
  angular_damping_(angular_damping)
{
  if (waypoint_tolerance_ <= 0.0 || target_speed_ <= 0.0) {
    throw std::invalid_argument("Tolerance and target speed must be positive");
  }
}

const Waypoints & Track::create_route(const Coordinate & start, const Coordinate & goal)
{
  waypoints_.replace(plan_route(start, goal));
  return waypoints_;
}

bool Track::follow_waypoint(const exploration_robot_control::RobotState & state)
{
  auto target = waypoints_.current();
  if (!target) {
    controller_.reset();
    return true;
  }

  double dx = (*target)[0] - state.x;
  double dy = (*target)[1] - state.y;
  double distance = std::hypot(dx, dy);
  if (distance <= waypoint_tolerance_) {
    waypoints_.advance();
    target = waypoints_.current();
    if (!target) {
      controller_.reset();
      return true;
    }
    dx = (*target)[0] - state.x;
    dy = (*target)[1] - state.y;
    distance = std::hypot(dx, dy);
  }

  const double heading_error = wrap_angle(std::atan2(dy, dx) - state.theta);
  const double alignment = std::max(0.0, 1.0 - std::abs(heading_error) / (kPi / 2.0));
  const double desired_speed = std::min(target_speed_, distance) * alignment;
  const double linear_acceleration = linear_gain_ * (desired_speed - state.linear_velocity);
  const double angular_acceleration =
    angular_gain_ * heading_error - angular_damping_ * state.angular_velocity;
  controller_.apply_control(linear_acceleration, angular_acceleration);
  return false;
}

bool Track::route_complete() const noexcept {return waypoints_.complete();}
const Waypoints & Track::waypoints() const noexcept {return waypoints_;}
exploration_robot_control::AccelerationController & Track::controller() noexcept
{
  return controller_;
}

}  // namespace exploration_robot_exploration
