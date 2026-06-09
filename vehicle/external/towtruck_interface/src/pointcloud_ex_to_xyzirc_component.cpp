// Copyright 2026 Autoware Towtruck Project
//
// Licensed under the Apache License, Version 2.0.
//
// Strip PointXYZIRCAEDT (32 B) -> PointXYZIRC (16 B). Stand-in for
// ring_outlier_filter, which is meaningless on the MID-360's non-repetitive
// scan but is the stage that normally produces the PointXYZIRC layout the
// concatenator expects.
//
// The first 16 bytes of PointXYZIRCAEDT are bit-identical to PointXYZIRC
// (x/y/z/intensity/return_type/channel at the same offsets), so the
// conversion is a per-point memcpy of the leading 16 bytes.
//
// Downstream of this node, PointCloudConcatenateDataSynchronizerComponent
// merges the per-lidar streams into /sensing/lidar/concatenated/pointcloud.

#include "towtruck_interface/pointcloud_ex_to_xyzirc_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <cstdint>
#include <cstring>
#include <utility>

namespace towtruck_interface
{

namespace
{
constexpr uint32_t kAedtPointStep   = 32;
constexpr uint32_t kXyzircPointStep = 16;

void fill_xyzirc_fields(sensor_msgs::msg::PointCloud2 & cloud)
{
  using sensor_msgs::msg::PointField;
  cloud.fields.resize(6);
  auto set = [&](size_t i, const char * name, uint32_t offset, uint8_t dtype) {
      cloud.fields[i].name     = name;
      cloud.fields[i].offset   = offset;
      cloud.fields[i].datatype = dtype;
      cloud.fields[i].count    = 1;
  };
  set(0, "x",           0,  PointField::FLOAT32);
  set(1, "y",           4,  PointField::FLOAT32);
  set(2, "z",           8,  PointField::FLOAT32);
  set(3, "intensity",   12, PointField::UINT8);
  set(4, "return_type", 13, PointField::UINT8);
  set(5, "channel",     14, PointField::UINT16);
}
}  // namespace

PointCloudExToXyzirc::PointCloudExToXyzirc(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_ex_to_xyzirc", options)
{
  const auto in_topic = this->declare_parameter<std::string>(
      "input_topic", "/sensing/lidar/left/rectified/pointcloud_ex");
  const auto out_topic = this->declare_parameter<std::string>(
      "output_topic", "/sensing/lidar/left/pointcloud_before_sync");

  // Pipeline-internal topic — concatenator subscribes BEST_EFFORT on
  // sensor_data QoS, matching the distortion_corrector publisher upstream.
  const auto sensor_qos = rclcpp::SensorDataQoS();

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic, sensor_qos,
      std::bind(&PointCloudExToXyzirc::on_cloud, this, std::placeholders::_1));
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, sensor_qos);
}

void PointCloudExToXyzirc::on_cloud(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  if (msg->point_step != kAedtPointStep) {
    if (!warned_unexpected_step_) {
      RCLCPP_WARN(
          get_logger(),
          "Unexpected point_step=%u (expected %u for PointXYZIRCAEDT).",
          msg->point_step, kAedtPointStep);
      warned_unexpected_step_ = true;
    }
    return;
  }

  const size_t n = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  if (n == 0) {
    return;
  }

  auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
  out->header = msg->header;
  out->height = 1;
  out->width = static_cast<uint32_t>(n);
  out->is_bigendian = false;
  out->is_dense = msg->is_dense;
  out->point_step = kXyzircPointStep;
  out->row_step = kXyzircPointStep * static_cast<uint32_t>(n);
  fill_xyzirc_fields(*out);
  out->data.resize(out->row_step);

  const uint8_t * src = msg->data.data();
  uint8_t * dst = out->data.data();
  for (size_t i = 0; i < n; ++i) {
    std::memcpy(dst + i * kXyzircPointStep,
                src + i * kAedtPointStep,
                kXyzircPointStep);
  }

  pub_->publish(std::move(out));
}

}  // namespace towtruck_interface

RCLCPP_COMPONENTS_REGISTER_NODE(towtruck_interface::PointCloudExToXyzirc)
