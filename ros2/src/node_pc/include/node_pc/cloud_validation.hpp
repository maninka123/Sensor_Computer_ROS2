#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstdint>
#include <initializer_list>
#include <string>

namespace node_pc {
// Iterators do not validate payload length, field type, endian or row padding.
// Indexed internal clouds are flattened by the merger before these consumers.
inline bool indexed_cloud_valid(const sensor_msgs::msg::PointCloud2& cloud) {
  if (cloud.height != 1 || cloud.width == 0 || cloud.point_step == 0 || cloud.is_bigendian)
    return false;
  const std::uint64_t size = std::uint64_t(cloud.width) * cloud.point_step;
  if (size != cloud.row_step || size != cloud.data.size()) return false;
  for (const auto& name : {"x", "y", "z", "source_frame_index", "source_frame_stamp_sec",
                           "source_frame_stamp_nsec", "image_enhancement_mode"}) {
    bool found = false;
    const auto expected = std::string(name).size() == 1 ?
      sensor_msgs::msg::PointField::FLOAT32 : sensor_msgs::msg::PointField::UINT32;
    for (const auto& field : cloud.fields) {
      if (field.name != name) continue;
      if (found || field.count != 1 || field.datatype != expected ||
          field.offset > cloud.point_step || cloud.point_step - field.offset < 4)
        return false;
      found = true;
    }
    if (!found) return false;
  }
  return true;
}
} // namespace node_pc
