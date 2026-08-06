#ifndef EXPLORATION_ROBOT_CONTROL__ROBOT_STATE_HPP_
#define EXPLORATION_ROBOT_CONTROL__ROBOT_STATE_HPP_

namespace exploration_robot_control
{

struct RobotState
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double linear_velocity{0.0};
  double angular_velocity{0.0};
};

}  // namespace exploration_robot_control

#endif  // EXPLORATION_ROBOT_CONTROL__ROBOT_STATE_HPP_
