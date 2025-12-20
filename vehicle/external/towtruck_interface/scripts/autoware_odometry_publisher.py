#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_vehicle_msgs.msg import VelocityReport, SteeringReport
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32 
import tf2_ros
import math

class AutowareOdometryPublisher(Node):

    def __init__(self):
        super().__init__("autoware_odometry_publisher")

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.steering = 0.0

        # Subscribers from your kinematic node
        self.create_subscription(Float32, "x", self.cb_x, 10)
        self.create_subscription(Float32, "y", self.cb_y, 10)
        self.create_subscription(Float32, "theta", self.cb_theta, 10)
        self.create_subscription(Float32, "v", self.cb_v, 10)
        self.create_subscription(Float32, "steering_angle_data", self.cb_steering, 10)

        # Autoware-required publishers
        self.odom_pub = self.create_publisher(
            Odometry, "/localization/kinematic_state", 10)

        self.vel_pub = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 10)

        self.steer_pub = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def cb_x(self, msg): self.x = msg.data; self.publish_all()
    def cb_y(self, msg): self.y = msg.data; self.publish_all()
    def cb_theta(self, msg): self.theta = msg.data; self.publish_all()
    def cb_v(self, msg): self.v = msg.data; self.publish_all()
    def cb_steering(self, msg): self.steering = msg.data * math.pi/180

    def publish_all(self):
        now = self.get_clock().now().to_msg()

        # ------------------ ODOMETRY ------------------
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta/2)
        odom.pose.pose.orientation.w = math.cos(self.theta/2)

        odom.twist.twist.linear.x = self.v

        self.odom_pub.publish(odom)

        # ---------------- TF --------------------------
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = math.sin(self.theta/2)
        tf.transform.rotation.w = math.cos(self.theta/2)
        self.tf_broadcaster.sendTransform(tf)

        # ---------------- Velocity Report --------------
        vel = VelocityReport()
        vel.header.stamp = now
        vel.longitudinal_velocity = float(self.v)

        self.vel_pub.publish(vel)

        # ---------------- Steering Report --------------
        steer = SteeringReport()
        steer.steering_tire_angle = float(self.steering)

        self.steer_pub.publish(steer)
