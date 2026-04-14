#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# CORRECT IMPORT for newer Autoware Universe
from autoware_control_msgs.msg import Control
import math

try:
    from pySerialTransfer import pySerialTransfer as txfer
except ModuleNotFoundError:
    txfer = None

class AutowareArduinoControl(Node):
    def __init__(self):
        super().__init__('autoware_arduino_control')

        # --- Configuration ---
        self.control_update_period = 0.06 # 20Hz
        self.wheel_radius = float(self.declare_parameter('wheel_radius', 0.2032).value)
        self.pulses_per_rev = float(
            self.declare_parameter('pulses_per_revolution', 740 * 1.1).value
        )

        # --- SUBSCRIBER ---
        # Updated to subscribe to 'Control' message on the Universe topic
        self.create_subscription(
            Control,
            '/control/command/control_cmd', 
            self.control_callback,
            10
        )

        # --- SERIAL CONNECTIONS ---
        self.link_steering = None
        self.link_wheels = None
        try:
            self.link_steering = txfer.SerialTransfer('arduino_steering', baud=115200)
            self.link_steering.open()
            self.get_logger().info("Connected to arduino_steering")
            
            self.link_wheels = txfer.SerialTransfer('arduino_wheels', baud=115200)
            self.link_wheels.open()
            self.get_logger().info("Connected to arduino_wheels")

        except Exception as e:
            self.get_logger().error(f"Failed to open Serial: {e}")

        # --- STATE VARIABLES ---
        self.reinitialize = False
        self.manual_mode = False
        self.brake_active = False
        self.reverse_mode = False
        self.speed = 0
        self.steering_angle = 0
        self.debug_mode = False
        self.received_first_cmd = False

        self.create_timer(self.control_update_period, self.send_commands)

    def control_callback(self, msg: Control):
        """
        Converts autoware_control_msgs/Control -> Arduino Variables
        """
        self.get_logger().info(
            f"RECEIVED control_cmd: v={msg.longitudinal.velocity}, "
            f"steer={msg.lateral.steering_tire_angle}"
        )
        self.received_first_cmd = True

        # 1. VELOCITY (in msg.longitudinal.velocity)
        target_v = msg.longitudinal.velocity
        
        if target_v < 0:
            self.reverse_mode = True
            v_abs = abs(target_v)
        else:
            self.reverse_mode = False
            v_abs = target_v

        # Brake Logic
        if v_abs < 0.01:
            self.brake_active = True
            self.speed = 0
        else:
            self.brake_active = False
            self.speed = int((v_abs * self.pulses_per_rev) / (2 * math.pi * self.wheel_radius))

        # 2. STEERING (in msg.lateral.steering_tire_angle)
        # Note: This is in radians.
        steer_rad = msg.lateral.steering_tire_angle
        self.steering_angle = int(math.degrees(steer_rad))

    def send_commands(self):
        if not self.received_first_cmd:
            return

        if self.link_wheels and self.link_wheels.connection.is_open:
            self.send_packet(self.link_wheels, "wheels")
        if self.link_steering and self.link_steering.connection.is_open:
            self.send_packet(self.link_steering, "steering")

    def send_packet(self, link, dev_name):
        try:
            send_size = 0
            # CORRECTED: send_size = link.tx_obj(...) instead of +=
            send_size = link.tx_obj(chr(1 if self.reinitialize else 0), send_size)
            send_size = link.tx_obj(chr(1 if self.manual_mode else 0), send_size)
            send_size = link.tx_obj(chr(1 if self.brake_active else 0), send_size)
            send_size = link.tx_obj(chr(1 if self.reverse_mode else 0), send_size)
            send_size = link.tx_obj(int(self.speed), send_size)
            send_size = link.tx_obj(int(self.steering_angle), send_size)
            send_size = link.tx_obj(chr(1 if self.debug_mode else 0), send_size)
            
            self.get_logger().info(
                f"{dev_name}: manual={self.manual_mode}, brake={self.brake_active}, "
                f"reverse={self.reverse_mode}, speed={self.speed}, steer={self.steering_angle}"
            )
            link.send(send_size)
        except Exception as e:
            self.get_logger().error(f"Error sending to {dev_name}: {e}")

def main(args=None):
    rclpy.init(args=args)

    if txfer is None:
        node = rclpy.create_node('autoware_arduino_control')
        node.get_logger().error(
            "Python module 'pySerialTransfer' is missing. Install it in the Autoware runtime "
            "before launching towtruck_interface, for example with 'pip install pySerialTransfer'."
        )
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    node = AutowareArduinoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
