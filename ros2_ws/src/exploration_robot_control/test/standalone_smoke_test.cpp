#include "exploration_robot_control/acceleration_controller.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>

int main()
{
  using exploration_robot_control::AccelerationController;

  AccelerationController controller;
  controller.apply_control(100.0, -100.0);
  const auto first = controller.update(0.5);
  assert(std::abs(first.linear - 1.0) < 1e-12);
  assert(std::abs(first.angular + 2.0) < 1e-12);

  const auto saturated = controller.update(10.0);
  assert(std::abs(saturated.linear - 3.0) < 1e-12);
  assert(std::abs(saturated.angular + 6.0) < 1e-12);

  controller.reset();
  assert(controller.velocity().linear == 0.0);
  assert(controller.velocity().angular == 0.0);

  bool rejected_invalid_dt = false;
  try {
    controller.update(0.0);
  } catch (const std::invalid_argument &) {
    rejected_invalid_dt = true;
  }
  assert(rejected_invalid_dt);
  return 0;
}
