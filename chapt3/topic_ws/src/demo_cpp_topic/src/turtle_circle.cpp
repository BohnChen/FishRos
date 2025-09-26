#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>


// 使用时间单位的字面量， 可以在代码中使用s 和 ms 表示时间
using namespace std::chrono_literals;

class TurtleCircle : public rclcpp::Node
{
private:
    // 定时器智能指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 发布者，智能指针
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

public:
    explicit TurtleCircle(const std::string& node_name) : Node(node_name)
    {
        // 调用继承而来的父类函数创建发布者
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        // 调用继承而来的父类函数创建发布者
        timer_ = this->create_wall_timer(1000ms, std::bind(&TurtleCircle::timer_callback, this));
    }


private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 1.0;
        msg.angular.z = 0.5;
        publisher_->publish(msg);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleCircle>("turtle_circle");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
