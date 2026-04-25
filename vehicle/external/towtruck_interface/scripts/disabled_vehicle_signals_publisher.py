#!/usr/bin/env python3

import rclpy
from autoware_vehicle_msgs.msg import HazardLightsCommand, TurnIndicatorsCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class DisabledVehicleSignalsPublisher(Node):
    def __init__(self) -> None:
        super().__init__("disabled_vehicle_signals_publisher")

        self.publish_rate = float(self.declare_parameter("publish_rate", 2.0).value)
        self.turn_topic = str(
            self.declare_parameter("turn_topic", "/planning/turn_indicators_cmd").value
        )
        self.hazard_topic = str(
            self.declare_parameter("hazard_topic", "/planning/hazard_lights_cmd").value
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.turn_pub = self.create_publisher(TurnIndicatorsCommand, self.turn_topic, qos)
        self.hazard_pub = self.create_publisher(HazardLightsCommand, self.hazard_topic, qos)

        # Publish once immediately so late-starting control nodes have a latched OFF command.
        self.publish_commands()

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.5
        self.create_timer(period, self.publish_commands)

    def publish_commands(self) -> None:
        stamp = self.get_clock().now().to_msg()

        turn = TurnIndicatorsCommand()
        turn.stamp = stamp
        turn.command = TurnIndicatorsCommand.DISABLE
        self.turn_pub.publish(turn)

        hazard = HazardLightsCommand()
        hazard.stamp = stamp
        hazard.command = HazardLightsCommand.DISABLE
        self.hazard_pub.publish(hazard)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DisabledVehicleSignalsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
