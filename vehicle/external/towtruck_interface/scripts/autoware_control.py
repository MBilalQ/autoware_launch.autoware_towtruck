#!/usr/bin/env python3

import math

import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from pySerialTransfer import pySerialTransfer as txfer
except ModuleNotFoundError:
    txfer = None


def gear_command_allows_motion(command: int) -> bool:
    return command not in (GearCommand.NONE, GearCommand.NEUTRAL, GearCommand.PARK)


def gear_command_is_reverse(command: int) -> bool:
    return command in (GearCommand.REVERSE, GearCommand.REVERSE_2)


def describe_gear_command(command: int) -> str:
    reverse_gears = {
        GearCommand.REVERSE: "REVERSE",
        GearCommand.REVERSE_2: "REVERSE_2",
    }
    stopped_gears = {
        GearCommand.NONE: "NONE",
        GearCommand.NEUTRAL: "NEUTRAL",
        GearCommand.PARK: "PARK",
    }
    if command in reverse_gears:
        return reverse_gears[command]
    if command in stopped_gears:
        return stopped_gears[command]
    return f"DRIVE_LIKE({command})"


class AutowareArduinoControl(Node):
    def __init__(self):
        super().__init__('autoware_arduino_control')

        # --- Configuration ---
        self.control_update_period = 0.06  # 20Hz
        self.wheel_radius = float(self.declare_parameter('wheel_radius', 0.2032).value)
        self.pulses_per_rev = float(
            self.declare_parameter('pulses_per_revolution', 740 * 1.1).value
        )
        self.standstill_velocity_threshold = float(
            self.declare_parameter('standstill_velocity_threshold', 0.01).value
        )
        self.startup_acceleration_threshold = float(
            self.declare_parameter('startup_acceleration_threshold', 0.1).value
        )
        self.minimum_startup_velocity = float(
            self.declare_parameter('minimum_startup_velocity', 0.25).value
        )

        # --- SUBSCRIBERS ---
        self.create_subscription(
            Control,
            '/control/command/control_cmd',
            self.control_callback,
            10
        )
        gear_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            GearCommand,
            '/control/command/gear_cmd',
            self.gear_callback,
            gear_qos,
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
        self.received_gear_cmd = False
        self.latest_gear_cmd = GearCommand.DRIVE
        self.last_logged_gear_cmd = None

        self.create_timer(self.control_update_period, self.send_commands)

    def meters_per_second_to_pulses(self, velocity_mps: float) -> int:
        return int((velocity_mps * self.pulses_per_rev) / (2.0 * math.pi * self.wheel_radius))

    def gear_callback(self, msg: GearCommand):
        self.received_gear_cmd = True
        self.latest_gear_cmd = msg.command
        if self.last_logged_gear_cmd != msg.command:
            self.get_logger().info(
                f"RECEIVED gear_cmd: gear={describe_gear_command(msg.command)}"
            )
            self.last_logged_gear_cmd = msg.command

    def control_callback(self, msg: Control):
        """
        Converts autoware_control_msgs/Control -> Arduino Variables
        """
        self.received_first_cmd = True

        target_v = float(msg.longitudinal.velocity)
        target_acc = (
            float(msg.longitudinal.acceleration)
            if msg.longitudinal.is_defined_acceleration
            else 0.0
        )

        if self.received_gear_cmd:
            self.reverse_mode = gear_command_is_reverse(self.latest_gear_cmd)
            gear_allows_motion = gear_command_allows_motion(self.latest_gear_cmd)
        else:
            self.reverse_mode = target_v < 0.0
            gear_allows_motion = True

        requested_speed = abs(target_v)
        direction_sign = -1.0 if self.reverse_mode else 1.0
        startup_requested = (
            gear_allows_motion and
            msg.longitudinal.is_defined_acceleration and
            direction_sign * target_acc > self.startup_acceleration_threshold
        )

        if not gear_allows_motion:
            self.brake_active = True
            self.speed = 0
        elif requested_speed < self.standstill_velocity_threshold and startup_requested:
            self.brake_active = False
            self.speed = self.meters_per_second_to_pulses(self.minimum_startup_velocity)
        elif requested_speed < self.standstill_velocity_threshold:
            self.brake_active = True
            self.speed = 0
        else:
            self.brake_active = False
            self.speed = self.meters_per_second_to_pulses(requested_speed)

        steer_rad = msg.lateral.steering_tire_angle
        self.steering_angle = int(math.degrees(steer_rad))
        gear_text = (
            describe_gear_command(self.latest_gear_cmd)
            if self.received_gear_cmd
            else 'UNSET'
        )
        self.get_logger().info(
            "RECEIVED control_cmd: "
            f"v={target_v}, a={target_acc}, gear={gear_text}, "
            f"brake={self.brake_active}, reverse={self.reverse_mode}, "
            f"speed={self.speed}, steer={msg.lateral.steering_tire_angle}"
        )

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
