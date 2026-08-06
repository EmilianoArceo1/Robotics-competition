#include "exploration_robot_exploration/mapping/belief_map.hpp"

#include <cmath>
#include <stdexcept>

namespace exploration_robot_exploration
{

namespace
{
void validate_coordinate(const Coordinate & coordinate)
{
  if (!std::isfinite(coordinate[0]) || !std::isfinite(coordinate[1])) {
    throw std::invalid_argument("Coordinates must be finite");
  }
}
}  // namespace

BeliefMap::BeliefMap(const SensorMatrix & observations)
{
  update(observations);
}

void BeliefMap::update(const SensorMatrix & observations)
{
  for (const auto & cell : observations) {
    validate_coordinate(cell.coordinate);
  }
  for (const auto & cell : observations) {
    cells_[{cell.coordinate[0], cell.coordinate[1]}] = cell.occupancy;
  }
}

SensorMatrix BeliefMap::matrix() const
{
  SensorMatrix result;
  result.reserve(cells_.size());
  for (const auto & [coordinate, occupancy] : cells_) {
    result.push_back({{coordinate.first, coordinate.second}, occupancy});
  }
  return result;
}

std::optional<Occupancy> BeliefMap::value_at(const Coordinate & coordinate) const
{
  validate_coordinate(coordinate);
  const auto found = cells_.find({coordinate[0], coordinate[1]});
  if (found == cells_.end()) {
    return std::nullopt;
  }
  return found->second;
}

void BeliefMap::clear() noexcept
{
  cells_.clear();
}

std::size_t BeliefMap::size() const noexcept
{
  return cells_.size();
}

}  // namespace exploration_robot_exploration
