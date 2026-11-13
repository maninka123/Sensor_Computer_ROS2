// ROS 2 port of src/node_pc/src/pointcloud_merge.cpp; numerical processing preserved.
/*
 * Accumulate PointCloud2 frames and attach exact per-point acquisition
 * provenance. The output remains sensor_msgs/PointCloud2 and preserves all
 * original fields while adding source_frame_index and the source timestamp.
 */

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <node_pc/qos.hpp>
#include <chrono>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>

class PointCloudMerger : public rclcpp::Node
{
public:
  PointCloudMerger() : rclcpp::Node("pointcloud_merger")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/livox/lidar_shifted");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "");
    if (output_topic_.empty())
    {
      output_topic_ = this->declare_parameter<std::string>("merged_topic", "/livox/lidar_merged");
    }
    reset_topic_ = this->declare_parameter<std::string>("reset_topic", "/image_enhancement");
    queue_size_ = this->declare_parameter<int>("queue_size", 10);
    consecutive_count_ = this->declare_parameter<int>("consecutive_count", 10);
    overlapping_merge_ = this->declare_parameter<bool>("overlapping_merge", false);
    enhancement_mode_ = this->declare_parameter<bool>("image_enhancement", false);
    verbose_ = this->declare_parameter<bool>("verbose", false);
    consecutive_count_ = std::max(1, consecutive_count_);

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS(1).reliable());
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, node_pc::subscription_qos<sensor_msgs::msg::PointCloud2>(input_topic_, queue_size_), std::bind(&PointCloudMerger::cloudCallback, this, std::placeholders::_1));
    if (!reset_topic_.empty())
    {
      reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(reset_topic_, node_pc::subscription_qos<std_msgs::msg::Bool>(reset_topic_, 1), std::bind(&PointCloudMerger::modeCallback, this, std::placeholders::_1));
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Frame-indexing merger: " << consecutive_count_ << " clouds "
                    << input_topic_ << " -> " << output_topic_
                    << " overlapping=" << (overlapping_merge_ ? "true" : "false")
                    << " reset_topic=" << reset_topic_);
  }

private:
  static bool hasReservedField(const sensor_msgs::msg::PointCloud2& cloud)
  {
    for (const auto& field : cloud.fields)
    {
      if (field.name == "source_frame_index" ||
          field.name == "source_frame_stamp_sec" ||
          field.name == "source_frame_stamp_nsec" ||
          field.name == "image_enhancement_mode")
      {
        return true;
      }
    }
    return false;
  }

  static std::size_t pointCount(const sensor_msgs::msg::PointCloud2& cloud)
  {
    return static_cast<std::size_t>(cloud.width) * cloud.height;
  }

  static bool compatibleLayout(const sensor_msgs::msg::PointCloud2& lhs,
                               const sensor_msgs::msg::PointCloud2& rhs)
  {
    if (lhs.point_step != rhs.point_step || lhs.is_bigendian != rhs.is_bigendian ||
        lhs.fields.size() != rhs.fields.size())
    {
      return false;
    }
    for (std::size_t i = 0; i < lhs.fields.size(); ++i)
    {
      const auto& a = lhs.fields[i];
      const auto& b = rhs.fields[i];
      if (a.name != b.name || a.offset != b.offset ||
          a.datatype != b.datatype || a.count != b.count)
      {
        return false;
      }
    }
    return true;
  }

  template <typename T>
  static void writeValue(std::uint8_t* destination, const T& value)
  {
    std::memcpy(destination, &value, sizeof(T));
  }

  void modeCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
  {
    if (msg->data == enhancement_mode_)
    {
      return;
    }
    enhancement_mode_ = msg->data;
    const std::size_t discarded = buffer_.size();
    buffer_.clear();
    RCLCPP_INFO_STREAM(this->get_logger(), "Image mode changed to " << (enhancement_mode_ ? "enhanced" : "raw")
                    << "; reset LiDAR accumulation (discarded " << discarded << " frames)");
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    const std::size_t row_bytes = static_cast<std::size_t>(msg->width) * msg->point_step;
    if (msg->width == 0 || msg->height == 0 || msg->point_step == 0 ||
        msg->row_step < row_bytes || msg->data.size() < row_bytes ||
        (msg->height > 1 && static_cast<std::size_t>(msg->height - 1) >
            (msg->data.size() - row_bytes) / msg->row_step))
    {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2.0 * 1000,
          "Dropping malformed/empty LiDAR cloud: width=" << msg->width
          << " height=" << msg->height << " point_step=" << msg->point_step
          << " row_step=" << msg->row_step << " data=" << msg->data.size());
      return;
    }
    if (hasReservedField(*msg))
    {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2.0 * 1000, "Input cloud already contains reserved source-frame fields; dropping it");
      return;
    }
    if (!buffer_.empty() && !compatibleLayout(*buffer_.front(), *msg))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "PointCloud2 layout changed while accumulating; resetting the current batch");
      buffer_.clear();
    }
    if (!buffer_.empty() && rclcpp::Time(msg->header.stamp) < rclcpp::Time(buffer_.back()->header.stamp))
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "LiDAR timestamp jumped backward; clearing accumulated fusion frames");
      buffer_.clear();
    }

    // Retain ROS's immutable shared message instead of deep-copying every
    // incoming cloud. A ten-frame Livox batch can otherwise briefly consume
    // roughly twice its final payload size on memory-constrained SBCs.
    buffer_.push_back(msg);
    while (static_cast<int>(buffer_.size()) > consecutive_count_)
    {
      buffer_.pop_front();
    }
    if (static_cast<int>(buffer_.size()) < consecutive_count_)
    {
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    if (buildIndexedCloud(output))
    {
      pub_->publish(output);
    }

    if (overlapping_merge_)
    {
      buffer_.pop_front();
    }
    else
    {
      buffer_.clear();
    }
  }

  bool buildIndexedCloud(sensor_msgs::msg::PointCloud2& output) const
  {
    if (buffer_.empty())
    {
      return false;
    }

    const sensor_msgs::msg::PointCloud2& first = *buffer_.front();
    const std::uint32_t original_step = first.point_step;
    const std::uint32_t index_offset = original_step;
    const std::uint32_t sec_offset = index_offset + sizeof(std::uint32_t);
    const std::uint32_t nsec_offset = sec_offset + sizeof(std::uint32_t);
    const std::uint32_t mode_offset = nsec_offset + sizeof(std::uint32_t);
    const std::uint32_t indexed_step = mode_offset + sizeof(std::uint32_t);

    std::size_t total_points = 0;
    for (const auto& cloud : buffer_)
    {
      total_points += pointCount(*cloud);
    }
    if (total_points > std::numeric_limits<std::uint32_t>::max() / indexed_step)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Accumulated cloud is too large for PointCloud2 row_step");
      return false;
    }

    output.header = buffer_.back()->header;
    output.height = 1;
    output.width = static_cast<std::uint32_t>(total_points);
    output.fields = first.fields;
    output.fields.push_back(makeField("source_frame_index", index_offset));
    output.fields.push_back(makeField("source_frame_stamp_sec", sec_offset));
    output.fields.push_back(makeField("source_frame_stamp_nsec", nsec_offset));
    output.fields.push_back(makeField("image_enhancement_mode", mode_offset));
    output.is_bigendian = first.is_bigendian;
    output.point_step = indexed_step;
    output.row_step = indexed_step * output.width;
    output.is_dense = true;
    output.data.resize(output.row_step);

    std::size_t destination_index = 0;
    for (std::size_t frame_index = 0; frame_index < buffer_.size(); ++frame_index)
    {
      const sensor_msgs::msg::PointCloud2& cloud = *buffer_[frame_index];
      output.is_dense = output.is_dense && cloud.is_dense;
      for (std::uint32_t row = 0; row < cloud.height; ++row)
      {
        for (std::uint32_t column = 0; column < cloud.width; ++column)
        {
          const std::size_t source_offset = static_cast<std::size_t>(row) * cloud.row_step +
                                            static_cast<std::size_t>(column) * cloud.point_step;
          std::uint8_t* destination = output.data.data() + destination_index * indexed_step;
          std::memcpy(destination, cloud.data.data() + source_offset, original_step);
          writeValue(destination + index_offset, static_cast<std::uint32_t>(frame_index));
          writeValue(destination + sec_offset, cloud.header.stamp.sec);
          writeValue(destination + nsec_offset, cloud.header.stamp.nanosec);
          writeValue(destination + mode_offset, static_cast<std::uint32_t>(enhancement_mode_ ? 1 : 0));
          ++destination_index;
        }
      }
    }

    if (verbose_)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Published " << total_points << " indexed points from "
                      << buffer_.size() << " source frames");
    }
    return true;
  }

  static sensor_msgs::msg::PointField makeField(const std::string& name, std::uint32_t offset)
  {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::UINT32;
    field.count = 1;
    return field;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string reset_topic_;
  int queue_size_{10};
  int consecutive_count_{10};
  bool overlapping_merge_{false};
  bool enhancement_mode_{false};
  bool verbose_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> buffer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMerger>());
  rclcpp::shutdown();
  return 0;
}
