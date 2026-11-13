// ROS 2 port of src/node_pc/src/pointcloud_voxel_filter.cpp; numerical processing preserved.
/*
 * Frame-aware radial-outlier removal followed by voxel-grid sampling.
 * Source frames are filtered independently so a voxel can never mix points
 * acquired at different times or corrupt their provenance identifier.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <node_pc/qos.hpp>
#include <chrono>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudVoxelFilter : public rclcpp::Node
{
public:
  PointCloudVoxelFilter() : rclcpp::Node("pointcloud_voxel_filter")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/livox/lidar_merged");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/livox/lidar_filtered");
    enabled_topic_ = this->declare_parameter<std::string>("enabled_topic", "/voxel_downsampling");
    leaf_size_topic_ = this->declare_parameter<std::string>("leaf_size_topic", "/voxel_leaf_size");
    enabled_status_topic_ = this->declare_parameter<std::string>("enabled_status_topic", enabled_topic_ + "/status");
    leaf_size_status_topic_ = this->declare_parameter<std::string>("leaf_size_status_topic", leaf_size_topic_ + "/status");
    enabled_ = this->declare_parameter<bool>("enabled", true);
    leaf_size_ = this->declare_parameter<double>("leaf_size", kDefaultLeafSize);
    radial_filter_enabled_ = this->declare_parameter<bool>("radial_filter_enabled", true);
    voxel_before_radial_ = this->declare_parameter<bool>("voxel_before_radial", true);
    radius_search_ = this->declare_parameter<double>("radius_search", 0.05);
    min_neighbors_ = this->declare_parameter<int>("min_neighbors", 2);
    queue_size_ = this->declare_parameter<int>("queue_size", 2);
    verbose_ = this->declare_parameter<bool>("verbose", false);

    validateConfiguration();

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS(1).reliable());
    enabled_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(enabled_status_topic_, node_pc::state_qos());
    leaf_size_status_pub_ = this->create_publisher<std_msgs::msg::Float64>(leaf_size_status_topic_, node_pc::state_qos());
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, node_pc::subscription_qos<sensor_msgs::msg::PointCloud2>(input_topic_, queue_size_), std::bind(&PointCloudVoxelFilter::cloudCallback, this, std::placeholders::_1));
    enabled_sub_ = this->create_subscription<std_msgs::msg::Bool>(enabled_topic_, node_pc::subscription_qos<std_msgs::msg::Bool>(enabled_topic_, 1), std::bind(&PointCloudVoxelFilter::enabledCallback, this, std::placeholders::_1));
    leaf_size_sub_ = this->create_subscription<std_msgs::msg::Float64>(leaf_size_topic_, node_pc::subscription_qos<std_msgs::msg::Float64>(leaf_size_topic_, 1), std::bind(&PointCloudVoxelFilter::leafSizeCallback, this, std::placeholders::_1));

    publishStatus();
    RCLCPP_INFO_STREAM(this->get_logger(), "Frame-aware filters: " << input_topic_ << " -> " << output_topic_
                    << " radial=" << (radial_filter_enabled_ ? "on" : "off")
                    << " voxel_first=" << (voxel_before_radial_ ? "true" : "false")
                    << " radius=" << radius_search_ << " m min_neighbors=" << min_neighbors_
                    << " voxel=" << (enabled_ ? "on" : "off")
                    << " leaf_size=" << leaf_size_ << " m");
  }

private:
  struct FrameGroup
  {
    std::uint32_t index{0};
    std::uint32_t stamp_sec{0};
    std::uint32_t stamp_nsec{0};
    std::uint32_t enhancement_mode{0};
    pcl::PointCloud<pcl::PointXYZ>::Ptr points{new pcl::PointCloud<pcl::PointXYZ>};
  };

  static constexpr double kDefaultLeafSize = 0.02;

  static bool validPositive(double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  void validateConfiguration()
  {
    if (!validPositive(leaf_size_))
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Invalid configured voxel leaf_size=" << leaf_size_
                      << "; using default " << kDefaultLeafSize << " m");
      leaf_size_ = kDefaultLeafSize;
      this->set_parameter(rclcpp::Parameter("leaf_size", leaf_size_));
    }
    if (!validPositive(radius_search_))
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Invalid radius_search=" << radius_search_ << "; using 0.05 m");
      radius_search_ = 0.05;
      this->set_parameter(rclcpp::Parameter("radius_search", radius_search_));
    }
    if (min_neighbors_ < 1)
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Invalid min_neighbors=" << min_neighbors_ << "; using 1");
      min_neighbors_ = 1;
      this->set_parameter(rclcpp::Parameter("min_neighbors", min_neighbors_));
    }
  }

  void enabledCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
  {
    enabled_ = msg->data;
    this->set_parameter(rclcpp::Parameter("enabled", enabled_));
    publishStatus();
    RCLCPP_INFO_STREAM(this->get_logger(), "Voxel downsampling toggle received: " << (enabled_ ? "ON" : "OFF"));
  }

  void leafSizeCallback(const std_msgs::msg::Float64::ConstSharedPtr& msg)
  {
    if (!validPositive(msg->data))
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Ignoring invalid voxel leaf size " << msg->data
                      << " m; current value remains " << leaf_size_ << " m");
      publishStatus();
      return;
    }
    leaf_size_ = msg->data;
    this->set_parameter(rclcpp::Parameter("leaf_size", leaf_size_));
    publishStatus();
    RCLCPP_INFO_STREAM(this->get_logger(), "Voxel leaf size updated to " << leaf_size_ << " m");
  }

  void publishStatus()
  {
    std_msgs::msg::Bool enabled_msg;
    enabled_msg.data = enabled_;
    enabled_status_pub_->publish(enabled_msg);
    std_msgs::msg::Float64 leaf_size_msg;
    leaf_size_msg.data = leaf_size_;
    leaf_size_status_pub_->publish(leaf_size_msg);
  }

  bool readGroups(const sensor_msgs::msg::PointCloud2& msg, std::vector<FrameGroup>& groups) const
  {
    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(msg, "z");
      sensor_msgs::PointCloud2ConstIterator<std::uint32_t> source(msg, "source_frame_index");
      sensor_msgs::PointCloud2ConstIterator<std::uint32_t> sec(msg, "source_frame_stamp_sec");
      sensor_msgs::PointCloud2ConstIterator<std::uint32_t> nsec(msg, "source_frame_stamp_nsec");
      sensor_msgs::PointCloud2ConstIterator<std::uint32_t> mode(msg, "image_enhancement_mode");

      std::unordered_map<std::uint32_t, std::size_t> positions;
      positions.reserve(16);
      groups.reserve(16);
      const std::size_t count = static_cast<std::size_t>(msg.width) * msg.height;
      for (std::size_t i = 0; i < count; ++i, ++x, ++y, ++z, ++source, ++sec, ++nsec, ++mode)
      {
        std::size_t group_position;
        if (!groups.empty() && groups.back().index == *source)
        {
          group_position = groups.size() - 1;
        }
        else
        {
          auto position = positions.find(*source);
          if (position == positions.end())
          {
            FrameGroup group;
            group.index = *source;
            group.stamp_sec = *sec;
            group.stamp_nsec = *nsec;
            group.enhancement_mode = *mode;
            groups.push_back(group);
            group_position = groups.size() - 1;
            positions.emplace(*source, group_position);
          }
          else
          {
            group_position = position->second;
          }
        }
        FrameGroup& group = groups[group_position];
        if (group.stamp_sec != *sec || group.stamp_nsec != *nsec ||
            group.enhancement_mode != *mode)
        {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Source-frame index " << *source << " has inconsistent timestamps");
          return false;
        }
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z))
        {
          group.points->push_back(pcl::PointXYZ(*x, *y, *z));
        }
      }
    }
    catch (const std::runtime_error& error)
    {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2.0 * 1000, "Indexed PointCloud2 is missing a required field: " << error.what());
      return false;
    }
    std::sort(groups.begin(), groups.end(), [](const FrameGroup& lhs, const FrameGroup& rhs) {
      return lhs.index < rhs.index;
    });
    return !groups.empty();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelFilter(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    const float leaf = static_cast<float>(leaf_size_);
    filter.setInputCloud(input);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.filter(*output);
    return output;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr radialFilter(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(input);
    filter.setRadiusSearch(radius_search_);
    filter.setMinNeighborsInRadius(min_neighbors_);
    filter.filter(*output);
    return output;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filterGroup(const FrameGroup& group) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr current = group.points;
    // Voxel reduction first gives the much more expensive radius search a
    // smaller spatial index. The legacy order remains selectable for exact
    // comparison with older captures.
    if (enabled_ && voxel_before_radial_ && !current->empty())
    {
      current = voxelFilter(current);
    }
    if (radial_filter_enabled_ && !current->empty())
    {
      current = radialFilter(current);
    }
    if (enabled_ && !voxel_before_radial_ && !current->empty())
    {
      current = voxelFilter(current);
    }
    return current;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    std::vector<FrameGroup> groups;
    if (!readGroups(*msg, groups))
    {
      return;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered;
    filtered.reserve(groups.size());
    std::size_t output_count = 0;
    for (const auto& group : groups)
    {
      filtered.push_back(filterGroup(group));
      output_count += filtered.back()->size();
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = msg->header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(
        7,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "source_frame_index", 1, sensor_msgs::msg::PointField::UINT32,
        "source_frame_stamp_sec", 1, sensor_msgs::msg::PointField::UINT32,
        "source_frame_stamp_nsec", 1, sensor_msgs::msg::PointField::UINT32,
        "image_enhancement_mode", 1, sensor_msgs::msg::PointField::UINT32);
    modifier.resize(output_count);

    sensor_msgs::PointCloud2Iterator<float> x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> z(output, "z");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> source(output, "source_frame_index");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> sec(output, "source_frame_stamp_sec");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> nsec(output, "source_frame_stamp_nsec");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> mode(output, "image_enhancement_mode");

    for (std::size_t group_index = 0; group_index < groups.size(); ++group_index)
    {
      const FrameGroup& group = groups[group_index];
      for (const auto& point : filtered[group_index]->points)
      {
        *x = point.x;
        *y = point.y;
        *z = point.z;
        *source = group.index;
        *sec = group.stamp_sec;
        *nsec = group.stamp_nsec;
        *mode = group.enhancement_mode;
        ++x; ++y; ++z; ++source; ++sec; ++nsec; ++mode;
      }
    }
    output.is_dense = true;
    cloud_pub_->publish(output);

    if (verbose_)
    {
      const std::size_t input_count = static_cast<std::size_t>(msg->width) * msg->height;
      RCLCPP_INFO_STREAM(this->get_logger(), "Frame-aware radial/voxel filtering " << input_count << " -> "
                      << output_count << " points across " << groups.size() << " source frames");
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string enabled_topic_;
  std::string leaf_size_topic_;
  std::string enabled_status_topic_;
  std::string leaf_size_status_topic_;
  bool enabled_{true};
  double leaf_size_{kDefaultLeafSize};
  bool radial_filter_enabled_{true};
  bool voxel_before_radial_{true};
  double radius_search_{0.05};
  int min_neighbors_{2};
  int queue_size_{2};
  bool verbose_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leaf_size_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leaf_size_status_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudVoxelFilter>());
  rclcpp::shutdown();
  return 0;
}
