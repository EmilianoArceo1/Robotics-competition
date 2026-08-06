#ifndef EXPLORATION_ROBOT_EXPLORATION__TYPES_HPP_
#define EXPLORATION_ROBOT_EXPLORATION__TYPES_HPP_

#include <array>
#include <cstdint>
#include <vector>

namespace exploration_robot_exploration
{

using Coordinate = std::array<double, 2>;
using CoordinateMatrix = std::vector<Coordinate>;

enum class Occupancy : std::int8_t
{
  Unknown = -1,
  Free = 0,
  Occupied = 1
};

struct SensorCell
{
  Coordinate coordinate{};
  Occupancy occupancy{Occupancy::Unknown};
};

using SensorMatrix = std::vector<SensorCell>;

}  // namespace exploration_robot_exploration

#endif  // EXPLORATION_ROBOT_EXPLORATION__TYPES_HPP_
