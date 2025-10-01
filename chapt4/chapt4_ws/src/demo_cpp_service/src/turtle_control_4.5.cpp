#include "chapt4_interfaces/srv/patrol.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;
using Patrol = chapt4_interfaces::srv::Patrol;

class TurtleController : public rclcpp::Node {
public:
  TurtleController() : Node("turtle_controller") {

    // 声明和获取参数初始值
    this->declare_parameter("k", 1.0);
    this->declare_parameter("max_speed", 1.0);
    this->get_parameter("k", k_);
    this->get_parameter("max_speed", max_speed_);

    // 添加参数设置回调
    parameters_callback_handle_ = this->add_on_set_parameters_callback(
        [&](const std::vector<rclcpp::Parameter> &params)
            -> SetParametersResult {
          // 遍历参数
          for (auto param : params) {
            RCLCPP_INFO(this->get_logger(), "更新参数 %s 值为：%f",
                        param.get_name().c_str(), param.as_double());
            if (param.get_name() == "k") {
              k_ = param.as_double();
            } else if (param.get_name() == "max_speed") {
              max_speed_ = param.as_double();
            }
          }
          auto result = SetParametersResult();
          result.successful = true;
          return result;
        });
  }

private:
  void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose) {
    // TODO:收到位置计算误差，发布速度指令

    auto message = geometry_msgs::msg::Twist();
    // 1. 记录当前位置
    double current_x = pose->x;
    double current_y = pose->y;
    RCLCPP_INFO(this->get_logger(), "当前位置： （x=%f, y=%f）", current_x,
                current_y);

    // 2. 计算与目标之间的距离，以及与当前海龟朝向的角度差
    double distance =
        std::sqrt((target_x_ - current_x) * (target_x_ - current_x) +
                  (target_y_ - current_y) * (target_y_ - current_y));
    double angle =
        std::atan2(target_y_ - current_y, target_x_ - current_x) - pose->theta;

    // 3. 控制策略：距离大于0.1继续运动，角度差大于0.2则原地旋转，否则直行
    if (distance > 0.1) {
      if (fabs(angle) > 0.2) {
        message.angular.z = fabs(angle);
      } else {
        // 通过比例控制器计算输出速度
        message.linear.x = k_ * distance;
      }
    }

    // 4. 限制最大值并发布消息
    if (message.linear.x > max_speed_) {
      message.linear.x = max_speed_;
    }
    velocity_publisher_->publish(message);
  }

private:
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Service<Patrol>::SharedPtr patrol_server_;
  // 目标位置x， 设置默认值1.0
  double target_x_{1.0};
  // 目标位置y, 设置默认值1.0
  double target_y_{1.0};
  // 比例系数，控制输出=误差*比例系数
  double k_{1.0};
  // 最大线速度， 设置默认值3.0
  double max_speed_{3.0};
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
