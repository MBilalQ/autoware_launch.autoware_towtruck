#!/usr/bin/env python3

import rclpy
from autoware_adapi_v1_msgs.msg import MrmState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class DummyMrmStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("dummy_mrm_state_publisher")

        self.publish_rate = float(self.declare_parameter("publish_rate", 10.0).value)
        self.topic_name = str(
            self.declare_parameter("topic_name", "/system/fail_safe/mrm_state").value
        )
        self.state = int(self.declare_parameter("state", int(MrmState.NORMAL)).value)
        self.behavior = int(self.declare_parameter("behavior", int(MrmState.NONE)).value)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(MrmState, self.topic_name, qos)

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.1
        self.create_timer(period, self.publish_state)

    def publish_state(self) -> None:
        msg = MrmState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.behavior = self.behavior
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummyMrmStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
