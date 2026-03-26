#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class LivoxDownsampler(Node):
    def __init__(self):
        super().__init__('livox_downsampler')
        # Listen to raw Livox
        self.sub = self.create_subscription(PointCloud2, '/livox/lidar', self.cb, 10)
        # Publish lightweight cloud to NDT
        self.pub = self.create_publisher(PointCloud2, '/sensing/lidar/concatenated/pointcloud', 10)
        
        # Standard PointXYZI
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
        ]

    def cb(self, msg):
        # Read the raw points
        raw_points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        
        # DOWNSAMPLE: Take every 10th point (reduces CPU load by 90%)
        downsampled_points = raw_points[::10]
        
        # Package and publish
        new_msg = pc2.create_cloud(msg.header, self.fields, downsampled_points)
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LivoxDownsampler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()