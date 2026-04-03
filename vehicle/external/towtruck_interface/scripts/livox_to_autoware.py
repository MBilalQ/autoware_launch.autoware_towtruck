#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, PointField
# import sensor_msgs_py.point_cloud2 as pc2

# class LivoxFormatConverter(Node):
#     def __init__(self):
#         super().__init__('livox_format_converter')
        
#         self.sub = self.create_subscription(PointCloud2, '/livox/lidar', self.cb, 10)
#         self.pub = self.create_publisher(PointCloud2, '/sensing/lidar/concatenated/pointcloud', 10)

#         # Perfect 32-byte alignment matching Autoware's C++ PointXYZIRC struct
#         self.autoware_fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#             # 4 bytes of silent padding exist here (bytes 12-15) for PCL_ADD_POINT4D
#             PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
#             PointField(name='return_type', offset=20, datatype=PointField.UINT8, count=1),
#             # 1 byte of silent padding exists here (byte 21)
#             PointField(name='channel', offset=22, datatype=PointField.UINT16, count=1),
#             # Dummy field to force the overall struct to perfectly pad out to 32 bytes
#             PointField(name='dummy_pad', offset=31, datatype=PointField.UINT8, count=1),
#         ]

#     def cb(self, msg):
#         raw_points = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        
#         # We now append 3 items: return_type=1, channel=0, dummy_pad=0
#         converted_points = [[p[0], p[1], p[2], p[3], 1, 0, 0] for p in raw_points]
        
#         new_msg = pc2.create_cloud(msg.header, self.autoware_fields, converted_points)
#         self.pub.publish(new_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LivoxFormatConverter()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

class LivoxToAutowareFast(Node):
    def __init__(self):
        super().__init__('livox_to_autoware_fast')
        self.sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.callback, 10)
        # Publish directly to the concatenated topic so Autoware's filters pick it up
        self.pub = self.create_publisher(
            PointCloud2, '/sensing/lidar/concatenated/pointcloud', 10)

    def callback(self, msg: PointCloud2):
        n_points = msg.width * msg.height
        
        # Parse incoming Livox data
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, msg.point_step)
        
        # Safely extract input bytes based on their original positions
        # (Assuming standard Livox xfer_format=2: x, y, z, intensity)
        xs = raw[:, 0:4]
        ys = raw[:, 4:8]
        zs = raw[:, 8:12]
        # intensity = raw[:, 12:16]

        # ---------------------------------------------------------
        # Build strict Autoware C++ Layout (PointXYZIRC - 32 Bytes)
        # ---------------------------------------------------------
        float_intensity = np.frombuffer(raw[:, 12:16].tobytes(), dtype=np.float32)
        intensity_uint8 = np.clip(float_intensity, 0, 255).astype(np.uint8).reshape(-1, 1)
        
        out_step = 32
        out_buf = np.zeros((n_points, out_step), dtype=np.uint8)

        # 0-11: X, Y, Z
        out_buf[:, 0:4]   = xs
        out_buf[:, 4:8]   = ys
        out_buf[:, 8:12]  = zs

        out_buf[:, 12]    = intensity_uint8.flatten() # Autoware: OFFSET 12
        out_buf[:, 13]    = 1                         # Autoware: OFFSET 13 (return_type)
        out_buf[:, 14:16] = 0                         # Autoware: OFFSET 14 (channel requires 2 bytes -> UINT16)
        
        # 12-15: SILENT PADDING (Required by PCL Point4D)
        
        # 16-19: Intensity
        # out_buf[:, 16:20] = intensity
        
        # 20: Return Type (Hardcode to 1 for Autoware)
        # out_buf[:, 20] = 1
        
        # 21: SILENT PADDING
        
        # 22-23: Channel (Hardcode to 0)
        # out_buf[:, 22:24] = 0
        
        # 24-31: SILENT PADDING to reach exactly 32 bytes

        # Build output message
        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.height = 1
        out_msg.width  = n_points
        out_msg.is_dense = False
        out_msg.is_bigendian = False
        out_msg.point_step = out_step
        out_msg.row_step   = out_step * n_points

        out_msg.fields = [
            PointField(name='x',           offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',           offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',           offset=8,  datatype=PointField.FLOAT32, count=1),
    
            PointField(name='intensity',   offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='return_type', offset=13, datatype=PointField.UINT8,   count=1),
            PointField(name='channel',    offset=14, datatype=PointField.UINT16, count=1),
        ]

        out_msg.data = out_buf.tobytes()
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    rclpy.spin(LivoxToAutowareFast())

if __name__ == '__main__':
    main()