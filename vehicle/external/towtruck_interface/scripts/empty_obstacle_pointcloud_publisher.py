#!/usr/bin/env python3

import importlib.util
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class EmptyObstaclePointCloudPublisher(Node):
    def __init__(self) -> None:
        super().__init__("empty_obstacle_pointcloud_publisher")

        self.topic_name = str(
            self.declare_parameter(
                "topic_name", "/perception/obstacle_segmentation/pointcloud"
            ).value
        )
        self.frame_id = str(self.declare_parameter("frame_id", "base_link").value)
        self.publish_rate = float(self.declare_parameter("publish_rate", 10.0).value)

        self.publisher = self.create_publisher(PointCloud2, self.topic_name, 10)
        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.1
        self.create_timer(period, self.publish_cloud)

    def publish_cloud(self) -> None:
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = 0
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 0
        msg.data = b""
        msg.is_dense = False
        self.publisher.publish(msg)


def main(args=None) -> None:
    argv = list(sys.argv if args is None else args)
    if "--path-corridor-detector" in argv:
        argv.remove("--path-corridor-detector")
        detector_path = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "path_corridor_obstacle_detector.py"
        )
        spec = importlib.util.spec_from_file_location("path_corridor_obstacle_detector", detector_path)
        if spec is None or spec.loader is None:
            raise RuntimeError("Unable to load path corridor obstacle detector")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.main(argv)
        return

    rclpy.init(args=args)
    node = EmptyObstaclePointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
