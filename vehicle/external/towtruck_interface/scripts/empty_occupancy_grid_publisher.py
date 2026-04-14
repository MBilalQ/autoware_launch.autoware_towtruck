#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class EmptyOccupancyGridPublisher(Node):
    def __init__(self) -> None:
        super().__init__("empty_occupancy_grid_publisher")

        self.frame_id = str(self.declare_parameter("frame_id", "map").value)
        self.topic_name = str(
            self.declare_parameter("topic_name", "/perception/occupancy_grid_map/map").value
        )
        self.publish_rate = float(self.declare_parameter("publish_rate", 1.0).value)
        self.resolution = float(self.declare_parameter("resolution", 5.0).value)
        self.width = int(self.declare_parameter("width", 400).value)
        self.height = int(self.declare_parameter("height", 400).value)
        self.origin_x = float(self.declare_parameter("origin_x", -1000.0).value)
        self.origin_y = float(self.declare_parameter("origin_y", -1000.0).value)
        self.origin_z = float(self.declare_parameter("origin_z", 0.0).value)

        self.publisher = self.create_publisher(OccupancyGrid, self.topic_name, 10)
        self.empty_data = [0] * (self.width * self.height)

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 1.0
        self.create_timer(period, self.publish_grid)

    def publish_grid(self) -> None:
        msg = OccupancyGrid()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.info.map_load_time = now
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = self.origin_z
        msg.info.origin.orientation.w = 1.0
        msg.data = self.empty_data
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmptyOccupancyGridPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
