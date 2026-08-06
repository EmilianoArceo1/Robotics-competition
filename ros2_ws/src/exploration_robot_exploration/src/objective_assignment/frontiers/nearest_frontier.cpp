#include "exploration_robot_exploration/objective_assignment/frontiers/nearest_frontier.hpp"

#include <algorithm>
#include <stdexcept>

namespace exploration_robot_exploration
{

CoordinateMatrix NearestFrontier::cluster_frontiers(const CoordinateMatrix & frontiers)
{
  return frontiers;
}

Coordinate NearestFrontier::assign_goal(
  const BeliefMap & belief_map,
  const exploration_robot_control::RobotState & robot_state)
{
  const CoordinateMatrix candidates = get_frontiers(belief_map);
  if (candidates.empty()) {
    throw std::runtime_error("No frontiers are available");
  }
  return *std::min_element(
    candidates.begin(), candidates.end(),
    [&robot_state](const Coordinate & first, const Coordinate & second) {
      const double first_dx = first[0] - robot_state.x;
      const double first_dy = first[1] - robot_state.y;
      const double second_dx = second[0] - robot_state.x;
      const double second_dy = second[1] - robot_state.y;
      return first_dx * first_dx + first_dy * first_dy <
             second_dx * second_dx + second_dy * second_dy;
    });
}

}  // namespace exploration_robot_exploration
