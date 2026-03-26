#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class LivoxFormatConverter(Node):
    def __init__(self):
        super().__init__('livox_format_converter')
        
        self.sub = self.create_subscription(PointCloud2, '/livox/lidar', self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/sensing/lidar/concatenated/pointcloud', 10)

        # Perfect 32-byte alignment matching Autoware's C++ PointXYZIRC struct
        self.autoware_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            # 4 bytes of silent padding exist here (bytes 12-15) for PCL_ADD_POINT4D
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='return_type', offset=20, datatype=PointField.UINT8, count=1),
            # 1 byte of silent padding exists here (byte 21)
            PointField(name='channel', offset=22, datatype=PointField.UINT16, count=1),
            # Dummy field to force the overall struct to perfectly pad out to 32 bytes
            PointField(name='dummy_pad', offset=31, datatype=PointField.UINT8, count=1),
        ]

    def cb(self, msg):
        raw_points = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        
        # We now append 3 items: return_type=1, channel=0, dummy_pad=0
        converted_points = [[p[0], p[1], p[2], p[3], 1, 0, 0] for p in raw_points]
        
        new_msg = pc2.create_cloud(msg.header, self.autoware_fields, converted_points)
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LivoxFormatConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()