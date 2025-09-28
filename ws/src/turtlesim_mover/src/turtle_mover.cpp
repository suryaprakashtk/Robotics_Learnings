#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TurtleMover : public rclcpp::Node
{
public:
    TurtleMover() : Node("turtle_mover")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&TurtleMover::move_turtle, this));
    }

private:
    void move_turtle()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 1.0;      // forward speed
        msg.angular.z = 0.5;     // rotate
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleMover>());
    rclcpp::shutdown();
    return 0;
}
