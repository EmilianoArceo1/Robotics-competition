#include "exploration_robot_exploration/objective_assignment/frontiers/frontiers.hpp"

#include <array>
#include <map>
#include <stdexcept>
#include <utility>

namespace exploration_robot_exploration
{

Frontiers::Frontiers(const double cell_size) : cell_size_(cell_size)
{
  if (cell_size_ <= 0.0) {
    throw std::invalid_argument("cell_size must be positive");
  }
}

CoordinateMatrix Frontiers::detect_frontiers(const BeliefMap & belief_map)
{
  std::map<std::pair<double, double>, Occupancy> cells;
  for (const auto & cell : belief_map.matrix()) {
    cells[{cell.coordinate[0], cell.coordinate[1]}] = cell.occupancy;
  }

  const std::array<Coordinate, 4> offsets{{
    {cell_size_, 0.0}, {-cell_size_, 0.0}, {0.0, cell_size_}, {0.0, -cell_size_}}};
  raw_frontiers_.clear();
  for (const auto & [position, occupancy] : cells) {
    if (occupancy != Occupancy::Free) {
      continue;
    }
    bool borders_unknown = false;
    for (const auto & offset : offsets) {
      const auto neighbor = cells.find({position.first + offset[0], position.second + offset[1]});
      if (neighbor == cells.end() || neighbor->second == Occupancy::Unknown) {
        borders_unknown = true;
        break;
      }
    }
    if (borders_unknown) {
      raw_frontiers_.push_back({position.first, position.second});
    }
  }
  return raw_frontiers_;
}

CoordinateMatrix Frontiers::get_frontiers(const BeliefMap & belief_map)
{
  return cluster_frontiers(detect_frontiers(belief_map));
}

const CoordinateMatrix & Frontiers::raw_frontiers() const noexcept
{
  return raw_frontiers_;
}

}  // namespace exploration_robot_exploration
