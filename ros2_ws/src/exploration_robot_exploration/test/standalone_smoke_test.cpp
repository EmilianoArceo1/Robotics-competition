#include <cassert>
#include <cmath>

#include "exploration_robot_control/acceleration_controller.hpp"
#include "exploration_robot_control/robot_state.hpp"
#include "exploration_robot_exploration/mapping/belief_map.hpp"
#include "exploration_robot_exploration/objective_assignment/frontiers/nearest_frontier.hpp"
#include "exploration_robot_exploration/planning/track.hpp"
#include "exploration_robot_exploration/sensing/sensor.hpp"

using namespace exploration_robot_exploration;

class StraightTrack final : public Track
{
public:
  using Track::Track;
  CoordinateMatrix plan_route(const Coordinate & start, const Coordinate & goal) override
  {
    return {start, goal};
  }
};

int main()
{
  exploration_robot_control::AccelerationController motion;
  exploration_robot_control::RobotState state;
  Sensor camera("camera", 5.0, 120.0);
  const SensorMatrix environment{
    {{3.0, 0.0}, Occupancy::Occupied},
    {{0.0, 3.0}, Occupancy::Free},
    {{-3.0, 0.0}, Occupancy::Unknown}};
  const auto detected = camera.detect(state, environment);
  assert(detected.size() == 1U);
  assert(detected.front().occupancy == Occupancy::Occupied);

  BeliefMap map({
    {{1.0, 1.0}, Occupancy::Free},
    {{2.0, 1.0}, Occupancy::Unknown},
    {{5.0, 5.0}, Occupancy::Free}});
  NearestFrontier frontier_detector;
  assert(frontier_detector.assign_goal(map, state) == (Coordinate{1.0, 1.0}));
  assert(frontier_detector.get_frontiers(map) == frontier_detector.raw_frontiers());

  StraightTrack track(motion);
  track.create_route({0.0, 0.0}, {1.0, 0.0});
  for (int iteration = 0; iteration < 1000 && !track.route_complete(); ++iteration) {
    if (track.follow_waypoint(state)) {
      break;
    }
    const auto velocity = motion.update(0.02);
    state.linear_velocity = velocity.linear;
    state.angular_velocity = velocity.angular;
    state.x += state.linear_velocity * std::cos(state.theta) * 0.02;
    state.y += state.linear_velocity * std::sin(state.theta) * 0.02;
    state.theta += state.angular_velocity * 0.02;
  }
  assert(track.route_complete());
  assert(std::abs(state.x - 1.0) < 0.11);
  return 0;
}
