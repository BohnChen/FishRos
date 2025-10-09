#include <memory>
// 提供消息接口
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
// 提供tf2::Quaternion 类
#include "tf2/LinearMath/Quaternion.h"
// 提供 tf2::getEulerYPR 函数
#include "tf2/utils.h"
// 提供消息类型转换函数
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// 提供 TF 缓冲类 Buffer 
#include "tf2_ros/buffer.h"
// 提供坐标监听类
#include "tf2_ros/transform_listener.h"
// 引入时间相关头文件
#include <chrono>
using namespace std::chrono_literals;


class TFListener : public rclcpp::Node
{
public:
    TFListener() : Node("tf_listener") {
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        timer_ = this->create_wall_timer(5s, std::bind(&TFListener::getTransform, this));
    }

    void getTransform() {
        try {
            // 等待变换可用
            const auto transform = buffer_->lookupTransform(
                "base_link", "target_point", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0f));
            // 转换结果及输出
            const auto &translation = transform.transform.translation;
            const auto &rotation = transform.transform.rotation;
            double yaw, pitch, roll;
            // 四元数转欧拉角
            tf2::getEulerYPR(rotation, yaw, pitch, roll);
            RCLCPP_INFO(get_logger(), "平移分量:(%f, %f, %f)", translation.x, translation.y, translation.z);
            RCLCPP_INFO(get_logger(), "旋转分量:(%f, %f, %f)", yaw, pitch, roll);
        }catch (const std::exception & ex) {
            // 处理异常
            RCLCPP_WARN(get_logger(), "异常: %s", ex.what());
        }
    }
private:
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}   