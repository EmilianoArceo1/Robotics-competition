#ifndef EXPLORATION_ROBOT_EXPLORATION__PLANNING__WAYPOINTS_HPP_
#define EXPLORATION_ROBOT_EXPLORATION__PLANNING__WAYPOINTS_HPP_

#include <cstddef>
#include <optional>

#include "exploration_robot_exploration/types.hpp"

namespace exploration_robot_exploration
{

class Waypoints
{
public:
  Waypoints() = default;
  explicit Waypoints(const CoordinateMatrix & coordinates);

  void replace(const CoordinateMatrix & coordinates);
  [[nodiscard]] const CoordinateMatrix & matrix() const noexcept;
  [[nodiscard]] std::optional<Coordinate> current() const;
  [[nodiscard]] std::size_t current_index() const noexcept;
  [[nodiscard]] bool complete() const noexcept;
  bool advance() noexcept;
  void reset() noexcept;
  [[nodiscard]] std::size_t size() const noexcept;

private:
  CoordinateMatrix coordinates_;
  std::size_t current_index_{0};
};

}  // namespace exploration_robot_exploration

#endif  // EXPLORATION_ROBOT_EXPLORATION__PLANNING__WAYPOINTS_HPP_
