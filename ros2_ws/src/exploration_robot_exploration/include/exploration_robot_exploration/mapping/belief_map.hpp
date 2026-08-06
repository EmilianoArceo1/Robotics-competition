#ifndef EXPLORATION_ROBOT_EXPLORATION__MAPPING__BELIEF_MAP_HPP_
#define EXPLORATION_ROBOT_EXPLORATION__MAPPING__BELIEF_MAP_HPP_

#include <map>
#include <optional>
#include <utility>

#include "exploration_robot_exploration/types.hpp"

namespace exploration_robot_exploration
{

class BeliefMap
{
public:
  BeliefMap() = default;
  explicit BeliefMap(const SensorMatrix & observations);

  void update(const SensorMatrix & observations);
  [[nodiscard]] SensorMatrix matrix() const;
  [[nodiscard]] std::optional<Occupancy> value_at(const Coordinate & coordinate) const;
  void clear() noexcept;
  [[nodiscard]] std::size_t size() const noexcept;

private:
  std::map<std::pair<double, double>, Occupancy> cells_;
};

}  // namespace exploration_robot_exploration

#endif  // EXPLORATION_ROBOT_EXPLORATION__MAPPING__BELIEF_MAP_HPP_
