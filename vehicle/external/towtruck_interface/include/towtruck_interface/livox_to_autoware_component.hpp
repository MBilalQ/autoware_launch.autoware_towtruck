// Copyright 2026 Autoware Towtruck Project
//
// Licensed under the Apache License, Version 2.0.

#ifndef TOWTRUCK_INTERFACE__LIVOX_TO_AUTOWARE_COMPONENT_HPP_
#define TOWTRUCK_INTERFACE__LIVOX_TO_AUTOWARE_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

namespace towtruck_interface
{

class LivoxToAutoware : public rclcpp::Node
{
public:
  explicit LivoxToAutoware(const rclcpp::NodeOptions & options);

private:
  void on_cloud(sensor_msgs::msg::PointCloud2::UniquePtr msg);
  void on_imu(sensor_msgs::msg::Imu::UniquePtr msg);

  std::string lidar_frame_id_;
  std::string imu_frame_id_;
  bool warned_unexpected_step_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    cloud_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    imu_pub_;
};

}  // namespace towtruck_interface

#endif  // TOWTRUCK_INTERFACE__LIVOX_TO_AUTOWARE_COMPONENT_HPP_
