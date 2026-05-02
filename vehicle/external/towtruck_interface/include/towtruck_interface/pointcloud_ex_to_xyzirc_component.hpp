// Copyright 2026 Autoware Towtruck Project
//
// Licensed under the Apache License, Version 2.0.

#ifndef TOWTRUCK_INTERFACE__POINTCLOUD_EX_TO_XYZIRC_COMPONENT_HPP_
#define TOWTRUCK_INTERFACE__POINTCLOUD_EX_TO_XYZIRC_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

namespace towtruck_interface
{

class PointCloudExToXyzirc : public rclcpp::Node
{
public:
  explicit PointCloudExToXyzirc(const rclcpp::NodeOptions & options);

private:
  void on_cloud(sensor_msgs::msg::PointCloud2::UniquePtr msg);

  bool warned_unexpected_step_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    concat_pub_;
};

}  // namespace towtruck_interface

#endif  // TOWTRUCK_INTERFACE__POINTCLOUD_EX_TO_XYZIRC_COMPONENT_HPP_
