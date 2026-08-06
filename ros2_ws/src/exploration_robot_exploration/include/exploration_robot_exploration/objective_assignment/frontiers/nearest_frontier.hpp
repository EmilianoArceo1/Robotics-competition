#ifndef EXPLORATION_ROBOT_EXPLORATION__OBJECTIVE_ASSIGNMENT__FRONTIERS__NEAREST_FRONTIER_HPP_
#define EXPLORATION_ROBOT_EXPLORATION__OBJECTIVE_ASSIGNMENT__FRONTIERS__NEAREST_FRONTIER_HPP_

#include "exploration_robot_control/robot_state.hpp"
#include "exploration_robot_exploration/objective_assignment/frontiers/frontiers.hpp"

namespace exploration_robot_exploration
{

class NearestFrontier final : public Frontiers
{
public:
  using Frontiers::Frontiers;

  CoordinateMatrix cluster_frontiers(const CoordinateMatrix & frontiers) override;
  Coordinate assign_goal(
    const BeliefMap & belief_map,
    const exploration_robot_control::RobotState & robot_state);
};

}  // namespace exploration_robot_exploration

#endif  // EXPLORATION_ROBOT_EXPLORATION__OBJECTIVE_ASSIGNMENT__FRONTIERS__NEAREST_FRONTIER_HPP_
