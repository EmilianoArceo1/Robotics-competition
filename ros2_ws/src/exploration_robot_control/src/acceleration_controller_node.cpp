#include "exploration_robot_control/acceleration_controller.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>

#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace exploration_robot_control
{

class AccelerationControllerNode final : public rclcpp::Node
{
public:
  AccelerationControllerNode()
  : Node("acceleration_controller"),
    controller_(load_limits())
  {
    const double update_rate = declare_parameter("update_rate", 50.0);
    command_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter("command_timeout", 0.5));
    if (update_rate <= 0.0) {
      throw std::invalid_argument("update_rate must be greater than zero");
    }

    command_subscription_ = create_subscription<geometry_msgs::msg::AccelStamped>(
      "cmd_accel", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::AccelStamped::SharedPtr message) {
        controller_.apply_control(message->accel.linear.x, message->accel.angular.z);
        last_command_time_ = now();
        command_received_ = true;
      });

    velocity_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", rclcpp::SystemDefaultsQoS());

    const auto period = std::chrono::duration<double>(1.0 / update_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AccelerationControllerNode::update, this));
    previous_update_time_ = now();
  }

private:
  ControlLimits load_limits()
  {
    return {
      declare_parameter("max_linear_acceleration", 2.0),
      declare_parameter("max_angular_acceleration", 4.0),
      declare_parameter("max_linear_speed", 3.0),
      declare_parameter("max_angular_speed", 6.0)};
  }

  void update()
  {
    const rclcpp::Time current_time = now();
    const double dt = (current_time - previous_update_time_).seconds();
    previous_update_time_ = current_time;
    if (dt <= 0.0) {
      return;
    }

    if (!command_received_ || current_time - last_command_time_ > command_timeout_) {
      controller_.reset();
      command_received_ = false;
    }

    const VelocityCommand velocity = controller_.update(dt);
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = current_time;
    output.header.frame_id = "base_link";
    output.twist.linear.x = velocity.linear;
    output.twist.angular.z = velocity.angular;
    velocity_publisher_->publish(output);
  }

  AccelerationController controller_;
  rclcpp::Duration command_timeout_{0, 0};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_update_time_{0, 0, RCL_ROS_TIME};
  bool command_received_{false};
  rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr command_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace exploration_robot_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<exploration_robot_control::AccelerationControllerNode>());
  rclcpp::shutdown();
  return 0;
}
