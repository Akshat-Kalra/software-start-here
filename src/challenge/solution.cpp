#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class TurtleSineMotion : public rclcpp::Node
{
public:
  TurtleSineMotion() : Node("solution")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TurtleSineMotion::move_turtle, this));
  }

private:
  void move_turtle()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = std::sin(time_);  // Sine wave for linear velocity
    message.angular.z = std::cos(time_); // Cosine wave for angular velocity
    publisher_->publish(message);
    time_ += 0.1;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double time_ = 0.0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleSineMotion>());
  rclcpp::shutdown();
  return 0;
}
