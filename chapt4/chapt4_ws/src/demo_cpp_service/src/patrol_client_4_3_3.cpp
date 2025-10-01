#include "chapt4_interfaces/srv/patrol.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdlib>
// 引入时间相关头文件
#include <ctime>
// 使用时间单位的字面量
using namespace std::chrono_literals;
using Patrol = chapt4_interfaces::srv::Patrol;

class PatrolClient : public rclcpp::Node {
public:
  PatrolClient() : Node("patrol_client") {
    patrol_client_ = this->create_client<Patrol>("patrol");
    timer_ = this->create_wall_timer(
        10s, std::bind(&PatrolClient::timer_callback, this));
    // 初始化随机数种子，使用当前时间作为种子
    srand(time(NULL));
  }
  void timer_callback() {
    // 1. 等待服务端上线
    while (!patrol_client_->wait_for_service(std::chrono::seconds(1))) {
      // 等待时检测rclcpp的状态
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }
    // 2. 构造请求的对象
    auto request = std::make_shared<Patrol::Request>();
    request->target_x = rand() % 15;
    request->target_y = rand() % 15;
    RCLCPP_INFO(this->get_logger(), "请求巡逻： (%f, %f)", request->target_x,
                request->target_y);
    // 3. 发送异步请求，然后等待返回，返回时调用回调函数
    patrol_client_->async_send_request(
        request,
        [&](rclcpp::Client<Patrol>::SharedFuture result_future) -> void {
          auto response = result_future.get();
          if (response->result == Patrol::Response::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "目标点处理成功");
          } else if (response->result == Patrol::Response::FAIL)
            RCLCPP_INFO(this->get_logger(), "目标点处理失败");
        });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<Patrol>::SharedPtr patrol_client_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
