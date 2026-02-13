#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from autoware_vehicle_msgs.msg import VelocityReport, SteeringReport
from std_msgs.msg import Float32 
import math

class AutowareOdometryPublisher(Node):

    def __init__(self):
        super().__init__("autoware_odometry_publisher")

        self.v = 0.0
        self.steering = 0.0

        # We ONLY listen for raw data from your kinematics/serial node
        # We DO NOT calculate x, y, theta anymore. Autoware does that.
        self.create_subscription(Float32, "v", self.cb_v, 10)
        self.create_subscription(Float32, "steering_angle_data", self.cb_steering, 10)

        # Publishers for Autoware Vehicle Feedback
        # EKF Localizer listens to these to calculate position!
        self.vel_pub = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 10)

        self.steer_pub = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 10)

        # Publish at 50Hz to ensure EKF is happy
        self.create_timer(0.02, self.publish_status)

    def cb_v(self, msg): 
        self.v = msg.data

    def cb_steering(self, msg): 
        # Convert degrees to radians for Autoware
        self.steering = msg.data * (math.pi / 180.0)

    def publish_status(self):
        now = self.get_clock().now().to_msg()

        # ---------------- Velocity Report --------------
        # Autoware uses this to predict movement (x += v * dt)
        vel = VelocityReport()
        vel.header.stamp = now
        vel.header.frame_id = "base_link" 
        vel.longitudinal_velocity = float(self.v)
        vel.lateral_velocity = 0.0 # Assuming non-holonomic
        vel.heading_rate = 0.0     # Optional, but EKF can calculate it from steering
        
        self.vel_pub.publish(vel)

        # ---------------- Steering Report --------------
        steer = SteeringReport()
        steer.steering_tire_angle = float(self.steering)

        self.steer_pub.publish(steer)

def main ():
    rclpy.init()
    node = AutowareOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
