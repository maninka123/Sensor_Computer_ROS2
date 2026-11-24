// ROS 2 port of src/node_pc/src/lidar_timestamp_shift.cpp; numerical processing preserved.
/*
 * ROS node that shifts lidar PointCloud2 timestamps by a configurable offset.
 * Defaults: input /livox/lidar, output /livox/lidar_shifted, capture offset 32.43 ms.
 */

#include <rclcpp/rclcpp.hpp>
#include <node_pc/qos.hpp>
#include <chrono>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cmath>

class LidarTimestampShift : public rclcpp::Node
{
public:
  LidarTimestampShift() : rclcpp::Node("lidar_timestamp_shift")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/livox/lidar_shifted");
    // timestamp_offset: converts LiDAR device time to Unix/sim time
    timestamp_offset_ = this->declare_parameter<double>("timestamp_offset", 1739510486.944657);
    // Live Livox timestamps restart with the sensor. Estimate that changing
    // clock-domain conversion from message arrival and the known scan span.
    auto_timestamp_offset_ = this->declare_parameter<bool>("auto_timestamp_offset", false);
    scan_duration_ = this->declare_parameter<double>("scan_duration", 0.1);
    auto_offset_alpha_ = this->declare_parameter<double>("auto_offset_alpha", 0.01);
    // capture_offset: compensates for sensor capture delay difference
    capture_offset_ = this->declare_parameter<double>("capture_offset", 0.03243);
    verbose_ = this->declare_parameter<bool>("verbose", false);
    const bool normalize_imu = this->declare_parameter<bool>("normalize_livox_imu", false);
    if (normalize_imu)
    {
      imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu", rclcpp::QoS(10).reliable());
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/livox/imu_raw", rclcpp::QoS(50).reliable(),
          std::bind(&LidarTimestampShift::imuCallback, this, std::placeholders::_1));
    }

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS(10).reliable());
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, node_pc::subscription_qos<sensor_msgs::msg::PointCloud2>(input_topic_, 10), std::bind(&LidarTimestampShift::callback, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(this->get_logger(), "Shifting timestamps on " << input_topic_
                    << " | timestamp_offset=" << timestamp_offset_ << "s"
                    << " | auto_timestamp_offset=" << (auto_timestamp_offset_ ? "true" : "false")
                    << " | scan_duration=" << scan_duration_ << "s"
                    << " | capture_offset=" << capture_offset_ << "s"
                    << " | verbose=" << (verbose_ ? "true" : "false")
                    << " | publishing to " << output_topic_);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
  {
    // The original Avia SDK publishes uptime and acceleration in g. Share the
    // LiDAR clock conversion, without the camera capture-delay compensation.
    // Wait for a fresh cloud after reconnect rather than reuse a stale epoch.
    const double device_time = rclcpp::Time(msg->header.stamp).seconds();
    if (!auto_offset_initialized_ || std::abs(device_time - last_lidar_time_) > 1.0)
      return;
    auto imu = *msg;
    imu.header.stamp = node_pc::from_seconds(device_time + auto_offset_);
    constexpr double gravity = 9.80665;
    imu.linear_acceleration.x *= gravity;
    imu.linear_acceleration.y *= gravity;
    imu.linear_acceleration.z *= gravity;
    for (auto& covariance : imu.linear_acceleration_covariance)
      covariance *= gravity * gravity;
    imu.orientation_covariance[0] = -1.0;  // Avia supplies no orientation.
    imu_pub_->publish(imu);
  }

  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    sensor_msgs::msg::PointCloud2 shifted = *msg;
    double effective_offset = timestamp_offset_;
    const double lidar_time = rclcpp::Time(msg->header.stamp).seconds();
    if (auto_timestamp_offset_)
    {
      const double measured_offset = this->now().seconds() - lidar_time - scan_duration_;
      const bool lidar_clock_restarted = auto_offset_initialized_ && lidar_time + 0.5 < last_lidar_time_;
      const bool clock_domain_changed = auto_offset_initialized_ &&
        std::abs(measured_offset - auto_offset_) > 1.0;
      if (!auto_offset_initialized_ || lidar_clock_restarted || clock_domain_changed)
      {
        auto_offset_ = measured_offset;
        auto_offset_initialized_ = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "Initialized live LiDAR clock conversion offset=" << auto_offset_ << "s");
      }
      else
      {
        auto_offset_ += auto_offset_alpha_ * (measured_offset - auto_offset_);
      }
      last_lidar_time_ = lidar_time;
      effective_offset = auto_offset_;
    }
    // Convert LiDAR device time to Unix/sim time using timestamp_offset,
    // then add capture_offset because the header refers to the first point and
    // the represented environmental event occurs later.
    // new_stamp = header.stamp + timestamp_offset + capture_offset
    double new_time = lidar_time + effective_offset + capture_offset_;
    if (new_time < 0.0)
    {
      new_time = 0.0;
    }
    shifted.header.stamp = node_pc::from_seconds(new_time);
    pub_->publish(shifted);

    if (verbose_)
    {
      RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2.0 * 1000, "Shifted timestamp: " << rclcpp::Time(msg->header.stamp).seconds()
                               << " -> " << new_time);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  // timestamp_offset: converts LiDAR device-relative time to Unix/sim time (from rosbag analysis)
  double timestamp_offset_{1739510486.944657};
  bool auto_timestamp_offset_{false};
  double scan_duration_{0.1};
  double auto_offset_alpha_{0.01};
  bool auto_offset_initialized_{false};
  double auto_offset_{0.0};
  double last_lidar_time_{0.0};
  // capture_offset: sensor capture delay difference to sync environmental events
  double capture_offset_{0.03243};
  bool verbose_{false};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarTimestampShift>());
  rclcpp::shutdown();
  return 0;
}
