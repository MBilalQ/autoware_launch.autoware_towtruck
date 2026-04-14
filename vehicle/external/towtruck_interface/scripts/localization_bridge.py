#!/usr/bin/env python3

import math
from typing import Tuple

import rclpy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from geometry_msgs.msg import AccelWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def compose_pose(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    ax, ay, ayaw = a
    bx, by, byaw = b
    cos_yaw = math.cos(ayaw)
    sin_yaw = math.sin(ayaw)
    return (
        ax + cos_yaw * bx - sin_yaw * by,
        ay + sin_yaw * bx + cos_yaw * by,
        normalize_angle(ayaw + byaw),
    )


def inverse_pose(pose: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, yaw = pose
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        -cos_yaw * x - sin_yaw * y,
        sin_yaw * x - cos_yaw * y,
        normalize_angle(-yaw),
    )


class LocalizationBridge(Node):
    def __init__(self) -> None:
        super().__init__("localization_bridge")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.map_z_override = float(self.declare_parameter("map_z_override", float("nan")).value)
        publish_rate = float(self.declare_parameter("publish_rate", 50.0).value)
        self.ignore_initialpose_before_startup = bool(
            self.declare_parameter("ignore_initialpose_before_startup", True).value
        )
        self.startup_time = self.get_clock().now()

        self.local_x = 0.0
        self.local_y = 0.0
        self.local_theta = 0.0
        self.local_v = 0.0
        self.local_theta_time = None
        self.yaw_rate = 0.0

        self.anchor_map_pose = None
        self.anchor_map_z = 0.0
        self.anchor_local_pose = None
        self.pose_covariance = [0.0] * 36
        self.reported_waiting_for_initialpose = False

        initialpose_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        initialpose_qos.durability = DurabilityPolicy.VOLATILE
        latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initialpose_callback, initialpose_qos
        )
        self.create_subscription(Float32, "x", self.x_callback, 10)
        self.create_subscription(Float32, "y", self.y_callback, 10)
        self.create_subscription(Float32, "theta", self.theta_callback, 10)
        self.create_subscription(Float32, "v", self.v_callback, 10)

        self.kinematic_state_pub = self.create_publisher(Odometry, "/localization/kinematic_state", 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, "/localization/pose_twist_fusion_filter/pose", 10
        )
        self.accel_pub = self.create_publisher(AccelWithCovarianceStamped, "/localization/acceleration", 10)
        self.initialpose3d_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose3d", latched_qos
        )
        self.localization_initialization_state_pub = self.create_publisher(
            LocalizationInitializationState, "/localization/initialization_state", latched_qos
        )
        self.api_localization_initialization_state_pub = self.create_publisher(
            LocalizationInitializationState, "/api/localization/initialization_state", latched_qos
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publish_initialization_state(LocalizationInitializationState.UNINITIALIZED)
        self.create_timer(1.0 / publish_rate, self.publish_state)

    def x_callback(self, msg: Float32) -> None:
        self.local_x = msg.data

    def y_callback(self, msg: Float32) -> None:
        self.local_y = msg.data

    def theta_callback(self, msg: Float32) -> None:
        now = self.get_clock().now()
        if self.local_theta_time is not None:
            dt = (now - self.local_theta_time).nanoseconds * 1.0e-9
            if dt > 0.0:
                theta_delta = normalize_angle(msg.data - self.local_theta)
                self.yaw_rate = theta_delta / dt

        self.local_theta = msg.data
        self.local_theta_time = now

    def v_callback(self, msg: Float32) -> None:
        self.local_v = msg.data

    def publish_initialization_state(self, state: int) -> None:
        msg = LocalizationInitializationState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = state
        self.localization_initialization_state_pub.publish(msg)
        self.api_localization_initialization_state_pub.publish(msg)

    def initialpose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        stamp = Time.from_msg(msg.header.stamp)
        if (
            self.ignore_initialpose_before_startup
            and stamp.nanoseconds != 0
            and stamp < self.startup_time
        ):
            self.get_logger().info(
                "Ignoring stale /initialpose from before localization_bridge startup"
            )
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"Received /initialpose in frame '{msg.header.frame_id}', expected '{self.map_frame}'. "
                "Using the pose values as-is."
            )

        orientation = msg.pose.pose.orientation
        self.anchor_map_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
        )
        if math.isfinite(self.map_z_override):
            self.anchor_map_z = self.map_z_override
        else:
            self.anchor_map_z = msg.pose.pose.position.z
        self.anchor_local_pose = (self.local_x, self.local_y, self.local_theta)
        self.pose_covariance = list(msg.pose.covariance)
        self.reported_waiting_for_initialpose = True

        initialpose3d_msg = PoseWithCovarianceStamped()
        initialpose3d_msg.header = msg.header
        if not initialpose3d_msg.header.frame_id:
            initialpose3d_msg.header.frame_id = self.map_frame
        initialpose3d_msg.pose = msg.pose
        initialpose3d_msg.pose.pose.position.z = self.anchor_map_z
        self.initialpose3d_pub.publish(initialpose3d_msg)
        self.publish_initialization_state(LocalizationInitializationState.INITIALIZED)

        self.get_logger().info(
            "Stored /initialpose anchor at "
            f"x={self.anchor_map_pose[0]:.3f}, y={self.anchor_map_pose[1]:.3f}, "
            f"yaw={self.anchor_map_pose[2]:.3f} rad, z={self.anchor_map_z:.3f}"
        )

    def publish_state(self) -> None:
        if self.anchor_map_pose is None or self.anchor_local_pose is None:
            if not self.reported_waiting_for_initialpose:
                self.get_logger().info("Waiting for /initialpose before publishing /localization/kinematic_state")
                self.reported_waiting_for_initialpose = True
            return

        local_pose = (self.local_x, self.local_y, self.local_theta)
        relative_pose = compose_pose(inverse_pose(self.anchor_local_pose), local_pose)
        map_pose = compose_pose(self.anchor_map_pose, relative_pose)
        quat_x, quat_y, quat_z, quat_w = quaternion_from_yaw(map_pose[2])
        stamp = self.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = map_pose[0]
        odom_msg.pose.pose.position.y = map_pose[1]
        odom_msg.pose.pose.position.z = self.anchor_map_z
        odom_msg.pose.pose.orientation.x = quat_x
        odom_msg.pose.pose.orientation.y = quat_y
        odom_msg.pose.pose.orientation.z = quat_z
        odom_msg.pose.pose.orientation.w = quat_w
        odom_msg.pose.covariance = self.pose_covariance
        odom_msg.twist.twist.linear.x = self.local_v
        odom_msg.twist.twist.angular.z = self.yaw_rate

        self.kinematic_state_pub.publish(odom_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose = odom_msg.pose.pose
        self.pose_pub.publish(pose_msg)

        accel_msg = AccelWithCovarianceStamped()
        accel_msg.header.stamp = stamp
        accel_msg.header.frame_id = self.base_frame
        accel_msg.accel.accel.linear.x = 0.0
        accel_msg.accel.accel.linear.y = 0.0
        accel_msg.accel.accel.linear.z = 0.0
        accel_msg.accel.accel.angular.x = 0.0
        accel_msg.accel.accel.angular.y = 0.0
        accel_msg.accel.accel.angular.z = 0.0
        self.accel_pub.publish(accel_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = map_pose[0]
        tf_msg.transform.translation.y = map_pose[1]
        tf_msg.transform.translation.z = self.anchor_map_z
        tf_msg.transform.rotation.x = quat_x
        tf_msg.transform.rotation.y = quat_y
        tf_msg.transform.rotation.z = quat_z
        tf_msg.transform.rotation.w = quat_w

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
