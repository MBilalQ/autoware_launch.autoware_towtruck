#!/usr/bin/env python3

import math

import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def gear_command_allows_motion(command: int) -> bool:
    return command not in (GearCommand.NONE, GearCommand.NEUTRAL, GearCommand.PARK)


def gear_command_is_reverse(command: int) -> bool:
    return command in (GearCommand.REVERSE, GearCommand.REVERSE_2)


class ControlKinematicsSimulator(Node):
    def __init__(self) -> None:
        super().__init__("control_kinematics_simulator")

        self.wheel_base = float(self.declare_parameter("wheel_base", 1.1).value)
        self.publish_rate = float(self.declare_parameter("publish_rate", 50.0).value)
        self.command_timeout = float(self.declare_parameter("command_timeout", 0.5).value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.velocity = 0.0
        self.steering_angle = 0.0
        self.latest_gear_cmd = GearCommand.DRIVE
        self.last_command_time = None

        self.pub_x = self.create_publisher(Float32, "x", 10)
        self.pub_y = self.create_publisher(Float32, "y", 10)
        self.pub_theta = self.create_publisher(Float32, "theta", 10)
        self.pub_v = self.create_publisher(Float32, "v", 10)
        self.pub_steering = self.create_publisher(Float32, "steering_angle_data", 10)

        self.create_subscription(Control, "/control/command/control_cmd", self.on_control_cmd, 10)

        gear_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            GearCommand, "/control/command/gear_cmd", self.on_gear_cmd, gear_qos
        )

        period = 1.0 / self.publish_rate if self.publish_rate > 1.0e-6 else 0.02
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            "Simulating vehicle kinematics from /control/command/* with hardware output disabled"
        )

    def on_control_cmd(self, msg: Control) -> None:
        self.velocity = float(msg.longitudinal.velocity)
        self.steering_angle = float(msg.lateral.steering_tire_angle)
        self.last_command_time = self.get_clock().now()

    def on_gear_cmd(self, msg: GearCommand) -> None:
        self.latest_gear_cmd = msg.command

    def on_timer(self) -> None:
        now = self.get_clock().now()
        dt = 1.0 / self.publish_rate if self.publish_rate > 1.0e-6 else 0.02

        target_velocity = self.velocity
        if self.last_command_time is None:
            target_velocity = 0.0
        else:
            age_sec = (now - self.last_command_time).nanoseconds * 1.0e-9
            if age_sec > self.command_timeout:
                target_velocity = 0.0

        if not gear_command_allows_motion(self.latest_gear_cmd):
            target_velocity = 0.0
        elif gear_command_is_reverse(self.latest_gear_cmd):
            target_velocity = -abs(target_velocity)
        else:
            target_velocity = abs(target_velocity)

        yaw_rate = 0.0
        if abs(self.wheel_base) > 1.0e-6:
            yaw_rate = target_velocity * math.tan(self.steering_angle) / self.wheel_base

        self.theta = normalize_angle(self.theta + yaw_rate * dt)
        self.x += target_velocity * math.cos(self.theta) * dt
        self.y += target_velocity * math.sin(self.theta) * dt

        self.publish_float(self.pub_x, self.x)
        self.publish_float(self.pub_y, self.y)
        self.publish_float(self.pub_theta, self.theta)
        self.publish_float(self.pub_v, target_velocity)
        self.publish_float(self.pub_steering, math.degrees(self.steering_angle))

    def publish_float(self, publisher, value: float) -> None:
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlKinematicsSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
