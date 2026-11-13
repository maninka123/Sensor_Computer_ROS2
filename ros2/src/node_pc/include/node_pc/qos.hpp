#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <type_traits>
#include <cmath>

namespace node_pc {
inline rclcpp::QoS state_qos() {
  return rclcpp::QoS(1).reliable().transient_local();
}
template<class Message>
rclcpp::QoS subscription_qos(const std::string & topic, size_t depth) {
  if constexpr (std::is_same_v<Message, sensor_msgs::msg::Image> ||
                std::is_same_v<Message, sensor_msgs::msg::PointCloud2>) {
    return rclcpp::SensorDataQoS().keep_last(depth);
  }
  if (topic.size() >= 7 && topic.compare(topic.size()-7, 7, "/status") == 0) {
    return state_qos();
  }
  return rclcpp::QoS(depth).reliable();
}
inline builtin_interfaces::msg::Time from_seconds(double value) {
  return rclcpp::Time(static_cast<int64_t>(std::llround(value * 1e9)), RCL_ROS_TIME);
}
} // namespace node_pc
