#include <gtest/gtest.h>

#include "exploration_robot_control/robot_state.hpp"
#include "exploration_robot_exploration/mapping/belief_map.hpp"
#include "exploration_robot_exploration/objective_assignment/frontiers/nearest_frontier.hpp"
#include "exploration_robot_exploration/sensing/sensor.hpp"

using namespace exploration_robot_exploration;

TEST(Sensor, preserves_occupancy_values)
{
  Sensor camera("camera", 5.0, 120.0);
  exploration_robot_control::RobotState state;
  const SensorMatrix environment{
    {{3.0, 0.0}, Occupancy::Occupied},
    {{0.0, 3.0}, Occupancy::Free},
    {{-3.0, 0.0}, Occupancy::Unknown}};
  const SensorMatrix detected = camera.detect(state, environment);
  ASSERT_EQ(detected.size(), 1U);
  EXPECT_EQ(detected.front().occupancy, Occupancy::Occupied);
}

TEST(BeliefMap, latest_observation_wins)
{
  BeliefMap map({{{1.0, 2.0}, Occupancy::Unknown}});
  map.update({{{1.0, 2.0}, Occupancy::Free}});
  ASSERT_TRUE(map.value_at({1.0, 2.0}).has_value());
  EXPECT_EQ(*map.value_at({1.0, 2.0}), Occupancy::Free);
}

TEST(NearestFrontier, returns_raw_nearest_frontier)
{
  BeliefMap map({
    {{1.0, 1.0}, Occupancy::Free},
    {{2.0, 1.0}, Occupancy::Unknown},
    {{5.0, 5.0}, Occupancy::Free}});
  NearestFrontier detector;
  const auto goal = detector.assign_goal(map, {});
  EXPECT_EQ(goal, (Coordinate{1.0, 1.0}));
  EXPECT_EQ(detector.get_frontiers(map), detector.raw_frontiers());
}
