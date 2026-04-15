#!/usr/bin/env python3
"""
Dummy Perception Publisher for Autoware Towtruck

Publishes empty perception data to satisfy planning modules when
the real perception stack is disabled (launch_perception=false).

What this publishes and why:
  1. PredictedObjects on /perception/object_recognition/objects
     → behavior_path_planner blocks on "waiting for dynamic_object" without this
  2. PointCloud2 on /perception/obstacle_segmentation/pointcloud
     → topic_state_monitor reports ERROR without this
  3. OccupancyGrid on /perception/occupancy_grid_map/map
     → behavior_path_planner blocks on "waiting for occupancy_grid" without this

All messages are empty (no obstacles), meaning the planner will assume
the path is clear. This is safe ONLY in a controlled test environment.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from autoware_perception_msgs.msg import PredictedObjects
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid


class DummyPerceptionPublisher(Node):

    def __init__(self):
        super().__init__("dummy_perception_publisher")

        # --- Publishers ---

        # 1) Empty predicted objects (no obstacles detected)
        self.objects_pub = self.create_publisher(
            PredictedObjects,
            "/perception/object_recognition/objects",
            10,
        )

        # 2) Empty obstacle segmentation pointcloud
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            "/perception/obstacle_segmentation/pointcloud",
            10,
        )

        # 3) Empty occupancy grid
        self.occupancy_pub = self.create_publisher(
            OccupancyGrid,
            "/perception/occupancy_grid_map/map",
            10,
        )

        # Publish at 10 Hz — fast enough to keep topic monitors happy
        self.create_timer(0.1, self.publish_all)
        self.get_logger().info("Dummy perception publisher started (10 Hz)")

    def publish_all(self):
        now = self.get_clock().now().to_msg()

        # ---- PredictedObjects (empty list = no obstacles) ----
        obj_msg = PredictedObjects()
        obj_msg.header.stamp = now
        obj_msg.header.frame_id = "map"
        obj_msg.objects = []  # empty — no detected obstacles
        self.objects_pub.publish(obj_msg)

        # ---- PointCloud2 (empty cloud = clear path) ----
        pc_msg = PointCloud2()
        pc_msg.header.stamp = now
        pc_msg.header.frame_id = "base_link"
        pc_msg.height = 1
        pc_msg.width = 0  # zero points
        pc_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.point_step = 12
        pc_msg.row_step = 0
        pc_msg.data = []
        pc_msg.is_dense = True
        self.pointcloud_pub.publish(pc_msg)

        # ---- OccupancyGrid (all cells free = 0) ----
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = now
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = 0.5          # 0.5m per cell
        grid_msg.info.width = 200               # 100m x 100m grid
        grid_msg.info.height = 200
        grid_msg.info.origin.position.x = -50.0  # centered around origin
        grid_msg.info.origin.position.y = -50.0
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = [0] * (200 * 200)       # 0 = free space everywhere
        self.occupancy_pub.publish(grid_msg)


def main():
    rclpy.init()
    node = DummyPerceptionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
