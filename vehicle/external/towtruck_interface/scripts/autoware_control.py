#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time
from pySerialTransfer import pySerialTransfer as txfer

from autoware_control_msgs.msg import ControlCommandStamped

class AutowareControlToArduino(Node):
    def __init__(self):
        super().__init__('autoware_control_to_arduino')

        # ---- Vehicle parameters (MUST match other nodes) ----
        self.wheel_radius = 0.2032      # meters
        self.pulses_per_rev = 740 * 1.1

        # ---- State ----
        self.speed_pulses = 0
        self.steering_deg = 0

        # ---- Subscriber (Autoware) ----
        self.sub = self.create_subscription(
            ControlCommandStamped,
            '/control/command/control',
            self.control_callback,
            10
        )

        # ---- Serial links (MATCH serial_reader.py) ----
        self.link_wheels = txfer.SerialTransfer('arduino_wheels', baud=115200)
        self.link_steering = txfer.SerialTransfer('arduino_steering', baud=115200)

        self.link_wheels.open()
        self.link_steering.open()

        self.get_logger().info("Connected to arduino_wheels and arduino_steering")

        self.timer = self.create_timer(0.05, self.send_to_arduinos)

    # -----------------------------------------------------
    def control_callback(self, msg: ControlCommandStamped):
        """
        Autoware → Arduino conversion
        """

        v = max(0.0, msg.control.velocity)  # m/s
        steer_rad = msg.control.steering_angle

        # Steering: rad → deg
        self.steering_deg = int(math.degrees(steer_rad))

        # Speed: m/s → pulses/sec
        self.speed_pulses = int(
            v * self.pulses_per_rev / (2 * math.pi * self.wheel_radius)
        )

        self.get_logger().info(f"v={v:.2f} m/s | steering={math.degrees(steer_rad):.2f} deg")

    # -----------------------------------------------------
    def send_to_arduinos(self):
        try:
            # ---- Wheels Arduino ----
            send_size = 0
            send_size += self.link_wheels.tx_obj(self.speed_pulses, send_size)

            if not self.link_wheels.send(send_size):
                self.reconnect(self.link_wheels, 'arduino_wheels')

            # ---- Steering Arduino ----
            send_size = 0
            send_size += self.link_steering.tx_obj(self.steering_deg, send_size)

            if not self.link_steering.send(send_size):
                self.reconnect(self.link_steering, 'arduino_steering')

        except Exception as e:
            self.get_logger().error(str(e))

    # -----------------------------------------------------
    def reconnect(self, link, name):
        self.get_logger().warn(f"{name} disconnected, reconnecting...")
        try:
            link.close()
            time.sleep(0.1)
            link.open()
        except:
            self.get_logger().error(f"Failed to reconnect {name}")


def main(args=None):
    rclpy.init(args=args)
    node = AutowareControlToArduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
