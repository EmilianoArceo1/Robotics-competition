#ifndef EXPLORATION_ROBOT_EXPLORATION__OBJECTIVE_ASSIGNMENT__FRONTIERS__FRONTIERS_HPP_
#define EXPLORATION_ROBOT_EXPLORATION__OBJECTIVE_ASSIGNMENT__FRONTIERS__FRONTIERS_HPP_

#include "exploration_robot_exploration/mapping/belief_map.hpp"
#include "exploration_robot_exploration/types.hpp"

namespace exploration_robot_exploration
{

class Frontiers
{
public:
  explicit Frontiers(double cell_size = 1.0);
  virtual ~Frontiers() = default;

  CoordinateMatrix detect_frontiers(const BeliefMap & belief_map);
  virtual CoordinateMatrix cluster_frontiers(const CoordinateMatrix & frontiers) = 0;
  CoordinateMatrix get_frontiers(const BeliefMap & belief_map);
  [[nodiscard]] const CoordinateMatrix & raw_frontiers() const noexcept;

private:
  double cell_size_;
  CoordinateMatrix raw_frontiers_;
};

}  // namespace exploration_robot_exploration

#endif  // EXPLORATION_ROBOT_EXPLORATION__OBJECTIVE_ASSIGNMENT__FRONTIERS__FRONTIERS_HPP_
