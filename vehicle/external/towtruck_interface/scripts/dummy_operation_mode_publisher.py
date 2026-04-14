#!/usr/bin/env python3

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class DummyOperationModePublisher(Node):
    def __init__(self) -> None:
        super().__init__("dummy_operation_mode_publisher")

        self.publish_rate = float(self.declare_parameter("publish_rate", 2.0).value)
        self.publish_system_topic = bool(self.declare_parameter("publish_system_topic", True).value)
        self.publish_api_topic = bool(self.declare_parameter("publish_api_topic", True).value)
        self.mode = int(self.declare_parameter("mode", int(OperationModeState.AUTONOMOUS)).value)
        self.is_autoware_control_enabled = bool(
            self.declare_parameter("is_autoware_control_enabled", True).value
        )
        self.is_in_transition = bool(self.declare_parameter("is_in_transition", False).value)
        self.is_stop_mode_available = bool(
            self.declare_parameter("is_stop_mode_available", True).value
        )
        self.is_autonomous_mode_available = bool(
            self.declare_parameter("is_autonomous_mode_available", True).value
        )
        self.is_local_mode_available = bool(
            self.declare_parameter("is_local_mode_available", False).value
        )
        self.is_remote_mode_available = bool(
            self.declare_parameter("is_remote_mode_available", False).value
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.mode_publishers = []
        if self.publish_system_topic:
            self.mode_publishers.append(
                self.create_publisher(OperationModeState, "/system/operation_mode/state", qos)
            )
        if self.publish_api_topic:
            self.mode_publishers.append(
                self.create_publisher(OperationModeState, "/api/operation_mode/state", qos)
            )

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.5
        self.create_timer(period, self.publish_state)

    def publish_state(self) -> None:
        msg = OperationModeState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = self.mode
        msg.is_autoware_control_enabled = self.is_autoware_control_enabled
        msg.is_in_transition = self.is_in_transition
        msg.is_stop_mode_available = self.is_stop_mode_available
        msg.is_autonomous_mode_available = self.is_autonomous_mode_available
        msg.is_local_mode_available = self.is_local_mode_available
        msg.is_remote_mode_available = self.is_remote_mode_available

        for publisher in self.mode_publishers:
            publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummyOperationModePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
