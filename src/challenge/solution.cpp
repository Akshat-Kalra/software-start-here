#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class SineTurtle : public rclcpp::Node
{
public:
    SineTurtle() : Node("solution"), time_(0.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SineTurtle::move_turtle, this));
    }

private:
    void move_turtle()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 2.0; // Constant forward velocity
        message.angular.z = std::sin(time_); // Sine wave angular velocity

        publisher_->publish(message);
        time_ += 0.1; // Increment time for the sine function
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SineTurtle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
