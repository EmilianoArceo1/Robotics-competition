#include "exploration_robot_exploration/planning/waypoints.hpp"

#include <cmath>
#include <stdexcept>

namespace exploration_robot_exploration
{

Waypoints::Waypoints(const CoordinateMatrix & coordinates)
{
  replace(coordinates);
}

void Waypoints::replace(const CoordinateMatrix & coordinates)
{
  for (const auto & coordinate : coordinates) {
    if (!std::isfinite(coordinate[0]) || !std::isfinite(coordinate[1])) {
      throw std::invalid_argument("Waypoint coordinates must be finite");
    }
  }
  coordinates_ = coordinates;
  current_index_ = 0;
}

const CoordinateMatrix & Waypoints::matrix() const noexcept {return coordinates_;}

std::optional<Coordinate> Waypoints::current() const
{
  if (complete()) {
    return std::nullopt;
  }
  return coordinates_[current_index_];
}

std::size_t Waypoints::current_index() const noexcept {return current_index_;}
bool Waypoints::complete() const noexcept {return current_index_ >= coordinates_.size();}

bool Waypoints::advance() noexcept
{
  if (!complete()) {
    ++current_index_;
  }
  return complete();
}

void Waypoints::reset() noexcept {current_index_ = 0;}
std::size_t Waypoints::size() const noexcept {return coordinates_.size();}

}  // namespace exploration_robot_exploration
