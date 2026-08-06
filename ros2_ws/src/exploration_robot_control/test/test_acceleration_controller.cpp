#include "exploration_robot_control/acceleration_controller.hpp"

#include <gtest/gtest.h>

using exploration_robot_control::AccelerationController;

TEST(AccelerationController, integrates_and_limits_commands)
{
  AccelerationController controller;
  controller.apply_control(100.0, -100.0);
  const auto velocity = controller.update(0.5);
  EXPECT_DOUBLE_EQ(controller.acceleration().linear, 2.0);
  EXPECT_DOUBLE_EQ(controller.acceleration().angular, -4.0);
  EXPECT_DOUBLE_EQ(velocity.linear, 1.0);
  EXPECT_DOUBLE_EQ(velocity.angular, -2.0);

  const auto saturated = controller.update(10.0);
  EXPECT_DOUBLE_EQ(saturated.linear, 3.0);
  EXPECT_DOUBLE_EQ(saturated.angular, -6.0);
}

TEST(AccelerationController, rejects_invalid_time_step)
{
  AccelerationController controller;
  EXPECT_THROW(controller.update(0.0), std::invalid_argument);
}

TEST(AccelerationController, reset_stops_the_robot)
{
  AccelerationController controller;
  controller.apply_control(1.0, 1.0);
  controller.update(1.0);
  controller.reset();
  EXPECT_DOUBLE_EQ(controller.velocity().linear, 0.0);
  EXPECT_DOUBLE_EQ(controller.velocity().angular, 0.0);
  EXPECT_DOUBLE_EQ(controller.acceleration().linear, 0.0);
  EXPECT_DOUBLE_EQ(controller.acceleration().angular, 0.0);
}
