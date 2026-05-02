// Copyright 2026 Autoware Towtruck Project
//
// Licensed under the Apache License, Version 2.0.
//
// Strip PointXYZIRCAEDT (32 B) -> PointXYZIRC (16 B), publish on both
// the per-lidar pipeline topic and the concatenated topic NDT consumes.
//
// Stand-in for ring_outlier_filter (meaningless on MID-360's non-repetitive
// scan) AND for the multi-lidar concatenator (which refuses N=1).
//
// The first 16 bytes of PointXYZIRCAEDT are bit-identical to PointXYZIRC
// (x/y/z/intensity/return_type/channel at the same offsets), so the
// conversion is a per-point memcpy of the leading 16 bytes.
//
// When a second lidar is added, drop the concatenated_topic publisher here
// and bring back PointCloudConcatenateDataSynchronizerComponent.

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
  const auto concat_topic = this->declare_parameter<std::string>(
      "concatenated_topic", "/sensing/lidar/concatenated/pointcloud");

  // Pipeline-internal topics stay on sensor_data BEST_EFFORT (matches the
  // distortion_corrector publisher). The concatenated topic crosses out
  // of the filter chain into NDT / pose_initializer / etc., which subscribe
  // with RELIABLE — log77:753 showed this mismatch killing the dataflow.
  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic, sensor_qos,
      std::bind(&PointCloudExToXyzirc::on_cloud, this, std::placeholders::_1));
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, sensor_qos);
  concat_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      concat_topic, reliable_qos);
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

  // We need to publish twice (intermediate + concatenated). Build the
  // payload once, then duplicate the message for the second publisher
  // since intra-process publish takes ownership of the unique_ptr.
  auto out_a = std::make_unique<sensor_msgs::msg::PointCloud2>();
  out_a->header = msg->header;
  out_a->height = 1;
  out_a->width = static_cast<uint32_t>(n);
  out_a->is_bigendian = false;
  out_a->is_dense = msg->is_dense;
  out_a->point_step = kXyzircPointStep;
  out_a->row_step = kXyzircPointStep * static_cast<uint32_t>(n);
  fill_xyzirc_fields(*out_a);
  out_a->data.resize(out_a->row_step);

  const uint8_t * src = msg->data.data();
  uint8_t * dst = out_a->data.data();
  for (size_t i = 0; i < n; ++i) {
    std::memcpy(dst + i * kXyzircPointStep,
                src + i * kAedtPointStep,
                kXyzircPointStep);
  }

  // Copy for the second publisher *before* we move out_a.
  auto out_b = std::make_unique<sensor_msgs::msg::PointCloud2>(*out_a);

  pub_->publish(std::move(out_a));
  concat_pub_->publish(std::move(out_b));
}

}  // namespace towtruck_interface

RCLCPP_COMPONENTS_REGISTER_NODE(towtruck_interface::PointCloudExToXyzirc)
