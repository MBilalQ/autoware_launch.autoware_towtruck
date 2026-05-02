// Copyright 2026 Autoware Towtruck Project
//
// Licensed under the Apache License, Version 2.0.
//
// Livox MID-360 -> Autoware sensing pipeline bridge.
//
// Input  (driver xfer_format=0, LivoxPointXyzrtlt, point_step=26):
//     x:f32 @0, y:f32 @4, z:f32 @8,
//     intensity:f32 @12 (named 'intensity' in the message; carries reflectivity),
//     tag:u8 @16, line:u8 @17,
//     timestamp:f64 @18 (per-point ns offset from frame base time, cast to double).
//
// Output (Autoware PointXYZIRCAEDT, point_step=32) on
//        /sensing/lidar/left/pointcloud_raw_ex, frame_id=velodyne_left:
//     x:f32 @0, y:f32 @4, z:f32 @8,
//     intensity:u8 @12, return_type:u8 @13, channel:u16 @14,
//     azimuth:f32 @16, elevation:f32 @20, distance:f32 @24,
//     time_stamp:u32 @28 (ns offset from header.stamp).
//
// The header stamp is rewritten to wall clock — the MID-360 onboard clock
// lags ROS time enough that the EKF rejects every NDT pose. Per-point
// time_stamp values stay valid as relative offsets within the frame.
//
// The MID-360's onboard IMU is bridged to /sensing/imu/tamagawa/imu_raw with
// the same wall-clock restamp + frame_id rewrite so imu_corrector runs
// natively (matches the prior Python bridge behaviour).

#include "towtruck_interface/livox_to_autoware_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>

namespace towtruck_interface
{

namespace
{

#pragma pack(push, 1)
struct LivoxPoint
{
  float    x;
  float    y;
  float    z;
  float    intensity;  // reflectivity, despite the field name
  uint8_t  tag;
  uint8_t  line;
  double   timestamp;  // ns offset from frame base, cast to double
};
#pragma pack(pop)
static_assert(sizeof(LivoxPoint) == 26, "LivoxPoint must be 26 bytes");

struct OutPoint
{
  float    x;             // 0
  float    y;             // 4
  float    z;             // 8
  uint8_t  intensity;     // 12
  uint8_t  return_type;   // 13
  uint16_t channel;       // 14
  float    azimuth;       // 16
  float    elevation;     // 20
  float    distance;      // 24
  uint32_t time_stamp;    // 28
};
static_assert(sizeof(OutPoint) == 32, "OutPoint (PointXYZIRCAEDT) must be 32 bytes");
static_assert(offsetof(OutPoint, intensity)   == 12, "");
static_assert(offsetof(OutPoint, return_type) == 13, "");
static_assert(offsetof(OutPoint, channel)     == 14, "");
static_assert(offsetof(OutPoint, azimuth)     == 16, "");
static_assert(offsetof(OutPoint, elevation)   == 20, "");
static_assert(offsetof(OutPoint, distance)    == 24, "");
static_assert(offsetof(OutPoint, time_stamp)  == 28, "");

constexpr uint32_t kLivoxPointStep = sizeof(LivoxPoint);
constexpr uint32_t kOutPointStep   = sizeof(OutPoint);

void fill_aedt_fields(sensor_msgs::msg::PointCloud2 & cloud)
{
  using sensor_msgs::msg::PointField;
  cloud.fields.resize(10);
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
  set(6, "azimuth",     16, PointField::FLOAT32);
  set(7, "elevation",   20, PointField::FLOAT32);
  set(8, "distance",    24, PointField::FLOAT32);
  set(9, "time_stamp",  28, PointField::UINT32);
}

}  // namespace

LivoxToAutoware::LivoxToAutoware(const rclcpp::NodeOptions & options)
: rclcpp::Node("livox_to_autoware", options)
{
  lidar_frame_id_ = this->declare_parameter<std::string>("lidar_frame_id", "velodyne_left");
  imu_frame_id_   = this->declare_parameter<std::string>("imu_frame_id",   "tamagawa/imu_link");

  // Cloud chain stays on sensor_data QoS — the Autoware pointcloud
  // filters (cropbox, distortion, ...) advertise BEST_EFFORT and a
  // BEST_EFFORT subscriber is still compatible with the Livox driver's
  // RELIABLE publisher.
  const auto sensor_qos = rclcpp::SensorDataQoS();
  // imu_corrector subscribes with default (RELIABLE) QoS, so we must
  // publish IMU as RELIABLE — log77:223 showed it rejecting a
  // BEST_EFFORT publisher with RELIABILITY_QOS_POLICY.
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", sensor_qos,
      std::bind(&LivoxToAutoware::on_cloud, this, std::placeholders::_1));
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/left/pointcloud_raw_ex", sensor_qos);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", sensor_qos,
      std::bind(&LivoxToAutoware::on_imu, this, std::placeholders::_1));
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/sensing/imu/tamagawa/imu_raw", reliable_qos);
}

void LivoxToAutoware::on_imu(sensor_msgs::msg::Imu::UniquePtr msg)
{
  msg->header.stamp = this->now();
  msg->header.frame_id = imu_frame_id_;
  imu_pub_->publish(std::move(msg));
}

void LivoxToAutoware::on_cloud(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  if (msg->point_step != kLivoxPointStep) {
    if (!warned_unexpected_step_) {
      RCLCPP_WARN(
          get_logger(),
          "Unexpected Livox point_step=%u (expected %u). Check driver xfer_format=0.",
          msg->point_step, kLivoxPointStep);
      warned_unexpected_step_ = true;
    }
    return;
  }

  const size_t n = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  if (n == 0) {
    return;
  }

  auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
  out->header.stamp = this->now();
  out->header.frame_id = lidar_frame_id_;
  out->height = 1;
  out->width = static_cast<uint32_t>(n);
  out->is_bigendian = false;
  out->is_dense = true;
  out->point_step = kOutPointStep;
  out->row_step = kOutPointStep * static_cast<uint32_t>(n);
  fill_aedt_fields(*out);
  out->data.resize(out->row_step);

  const auto * in_pts = reinterpret_cast<const LivoxPoint *>(msg->data.data());
  auto * out_pts = reinterpret_cast<OutPoint *>(out->data.data());

  for (size_t i = 0; i < n; ++i) {
    const LivoxPoint & p = in_pts[i];
    OutPoint & q = out_pts[i];

    q.x = p.x;
    q.y = p.y;
    q.z = p.z;

    const float clipped = std::clamp(p.intensity, 0.0f, 255.0f);
    q.intensity   = static_cast<uint8_t>(clipped);
    q.return_type = 1;  // SINGLE_STRONGEST
    q.channel     = static_cast<uint16_t>(p.line);

    const float xy_sq = p.x * p.x + p.y * p.y;
    q.azimuth   = std::atan2(p.y, p.x);
    q.elevation = std::atan2(p.z, std::sqrt(xy_sq));
    q.distance  = std::sqrt(xy_sq + p.z * p.z);

    // Livox per-point timestamp is offset_time in ns relative to the frame's
    // base time, stored as a double in the message. Cast back to uint32 — at
    // 100 ms per frame the value is well under 4.29 s (uint32 max).
    q.time_stamp = static_cast<uint32_t>(p.timestamp);
  }

  cloud_pub_->publish(std::move(out));
}

}  // namespace towtruck_interface

RCLCPP_COMPONENTS_REGISTER_NODE(towtruck_interface::LivoxToAutoware)
