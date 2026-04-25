#!/usr/bin/env python3

import contextlib
import io
import math
import threading

import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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
        self.control_update_period = 0.05  # 20Hz
        self.wheel_radius = float(self.declare_parameter('wheel_radius', 0.2).value)
        self.pulses_per_rev = float(
            self.declare_parameter('pulses_per_revolution', 740 * 1.1).value
        )
        self.max_steer_angle_rad = float(
            self.declare_parameter('max_steer_angle_rad', 0.9773843811).value
        )
        self.wheels_serial_port = str(
            self.declare_parameter('wheels_serial_port', '/dev/arduino_wheels').value
        )
        self.steering_serial_port = str(
            self.declare_parameter('steering_serial_port', '/dev/arduino_steering').value
        )
        self.serial_baud_rate = int(self.declare_parameter('serial_baud_rate', 115200).value)
        self.serial_timeout_sec = float(
            self.declare_parameter('serial_timeout_sec', 0.05).value
        )
        self.serial_write_timeout_sec = float(
            self.declare_parameter('serial_write_timeout_sec', 0.05).value
        )
        self.reconnect_retry_interval_sec = float(
            self.declare_parameter('reconnect_retry_interval_sec', 1.0).value
        )
        self.unchanged_command_resend_interval_sec = float(
            self.declare_parameter('unchanged_command_resend_interval_sec', 0.5).value
        )
        self.clear_output_buffer_before_send = bool(
            self.declare_parameter('clear_output_buffer_before_send', True).value
        )
        self.standstill_velocity_threshold = float(
            self.declare_parameter('standstill_velocity_threshold', 0.01).value
        )
        self.startup_acceleration_threshold = float(
            self.declare_parameter('startup_acceleration_threshold', 0.1).value
        )
        self.minimum_startup_velocity = float(
            self.declare_parameter('minimum_startup_velocity', 0.35).value
        )
        self.control_log_interval_sec = 1.0
        self.packet_log_interval_sec = 1.0

        self.command_callback_group = MutuallyExclusiveCallbackGroup()
        self.io_callback_group = MutuallyExclusiveCallbackGroup()

        # --- SUBSCRIBERS ---
        self.create_subscription(
            Control,
            '/control/command/control_cmd',
            self.control_callback,
            10,
            callback_group=self.command_callback_group,
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
            callback_group=self.command_callback_group,
        )

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
        self.last_logged_control_signature = None
        self.last_logged_control_time_sec = 0.0
        self.last_logged_packet_by_device = {}
        self.last_logged_packet_time_by_device = {}
        self.last_sent_packet_by_device = {}
        self.last_sent_time_by_device = {}
        self.next_reconnect_time_by_device = {}
        self.last_send_error_by_device = {}
        self.send_lock = threading.Lock()

        # --- SERIAL CONNECTIONS ---
        self.link_steering = None
        self.link_wheels = None
        self.try_connect_serial_links()

        self.create_timer(
            self.control_update_period,
            self.send_commands,
            callback_group=self.io_callback_group,
        )

    def meters_per_second_to_pulses(self, velocity_mps: float) -> int:
        return int((velocity_mps * self.pulses_per_rev) / (2.0 * math.pi * self.wheel_radius))

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def open_serial_link(self, device_path: str, label: str):
        try:
            link = txfer.SerialTransfer(device_path, baud=self.serial_baud_rate)
            link.open()
            connection = getattr(link, 'connection', None)
            if connection is not None:
                if hasattr(connection, 'timeout'):
                    connection.timeout = self.serial_timeout_sec
                if hasattr(connection, 'write_timeout'):
                    connection.write_timeout = self.serial_write_timeout_sec
                if hasattr(connection, 'reset_input_buffer'):
                    connection.reset_input_buffer()
                if hasattr(connection, 'reset_output_buffer'):
                    connection.reset_output_buffer()
            self.last_sent_packet_by_device.pop(label, None)
            self.last_sent_time_by_device.pop(label, None)
            self.last_send_error_by_device.pop(label, None)
            self.get_logger().info(f"Connected to {label} on {device_path}")
            return link
        except Exception as e:
            self.get_logger().error(f"Failed to open {label} on {device_path}: {e}")
            return None

    def link_is_open(self, link) -> bool:
        return bool(link and getattr(link, 'connection', None) and link.connection.is_open)

    def try_connect_serial_link(self, link_attr: str, device_path: str, label: str):
        if self.link_is_open(getattr(self, link_attr)):
            return
        if self.now_sec() < self.next_reconnect_time_by_device.get(label, 0.0):
            return
        setattr(self, link_attr, self.open_serial_link(device_path, label))

    def try_connect_serial_links(self):
        self.try_connect_serial_link('link_steering', self.steering_serial_port, "steering")
        self.try_connect_serial_link('link_wheels', self.wheels_serial_port, "wheels")

    def reset_serial_link(self, link_attr: str, label: str, error_reason: str | None = None):
        link = getattr(self, link_attr)
        if link is None:
            self.next_reconnect_time_by_device[label] = (
                self.now_sec() + self.reconnect_retry_interval_sec
            )
        else:
            try:
                connection = getattr(link, 'connection', None)
                if connection and connection.is_open:
                    connection.close()
            except Exception as e:
                self.get_logger().warn(f"Failed to close {label} link cleanly: {e}")
        setattr(self, link_attr, None)
        self.last_sent_packet_by_device.pop(label, None)
        self.last_sent_time_by_device.pop(label, None)
        self.next_reconnect_time_by_device[label] = (
            self.now_sec() + self.reconnect_retry_interval_sec
        )
        message = "serial write failed"
        if error_reason:
            message = f"{message}: {error_reason}"
        if self.last_send_error_by_device.get(label) != message:
            self.get_logger().warn(
                f"Resetting {label} serial link after {message}. "
                f"Reconnect will be retried in {self.reconnect_retry_interval_sec:.2f}s."
            )
            self.last_send_error_by_device[label] = message

    def current_packet_signature(self):
        return (
            self.manual_mode,
            self.brake_active,
            self.reverse_mode,
            self.speed,
            self.steering_angle,
            self.debug_mode,
        )

    def should_send_packet(self, dev_name: str, packet_signature) -> bool:
        last_packet = self.last_sent_packet_by_device.get(dev_name)
        if last_packet is None or last_packet != packet_signature:
            return True
        last_sent_time = self.last_sent_time_by_device.get(dev_name, 0.0)
        return (
            self.now_sec() - last_sent_time
            >= self.unchanged_command_resend_interval_sec
        )

    def summarize_suppressed_serial_output(self, text: str) -> str:
        if "Write timeout" in text:
            return "write timeout"
        if "SerialTimeoutException" in text:
            return "serial timeout"
        cleaned_lines = [line.strip() for line in text.splitlines() if line.strip()]
        if not cleaned_lines:
            return "serial library reported an error"
        return cleaned_lines[-1]

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

        steer_rad = max(
            -self.max_steer_angle_rad,
            min(self.max_steer_angle_rad, float(msg.lateral.steering_tire_angle)),
        )
        self.steering_angle = int(math.degrees(steer_rad))
        gear_text = (
            describe_gear_command(self.latest_gear_cmd)
            if self.received_gear_cmd
            else 'UNSET'
        )
        control_signature = (
            round(target_v, 4),
            round(target_acc, 4),
            gear_text,
            self.brake_active,
            self.reverse_mode,
            self.speed,
            self.steering_angle,
        )
        now_sec = self.now_sec()
        if (
            control_signature != self.last_logged_control_signature
            or now_sec - self.last_logged_control_time_sec >= self.control_log_interval_sec
        ):
            self.get_logger().info(
                "RECEIVED control_cmd: "
                f"v={target_v}, a={target_acc}, gear={gear_text}, "
                f"brake={self.brake_active}, reverse={self.reverse_mode}, "
                f"speed={self.speed}, steer={msg.lateral.steering_tire_angle}"
            )
            self.last_logged_control_signature = control_signature
            self.last_logged_control_time_sec = now_sec

    def send_commands(self):
        if not self.received_first_cmd:
            return
        if not self.send_lock.acquire(blocking=False):
            return

        try:
            self.try_connect_serial_links()
            packet_signature = self.current_packet_signature()

            if self.link_is_open(self.link_wheels) and self.should_send_packet("wheels", packet_signature):
                if not self.send_packet(self.link_wheels, "wheels", packet_signature):
                    self.reset_serial_link('link_wheels', 'wheels', "write timeout")
            if self.link_is_open(self.link_steering) and self.should_send_packet("steering", packet_signature):
                if not self.send_packet(self.link_steering, "steering", packet_signature):
                    self.reset_serial_link('link_steering', 'steering', "write timeout")
        finally:
            self.send_lock.release()

    def send_packet(self, link, dev_name, packet_signature):
        try:
            connection = getattr(link, 'connection', None)
            if (
                self.clear_output_buffer_before_send and
                connection is not None and
                hasattr(connection, 'reset_output_buffer')
            ):
                connection.reset_output_buffer()

            send_size = 0
            send_size = link.tx_obj(chr(1 if self.reinitialize else 0), send_size)
            send_size = link.tx_obj(chr(1 if self.manual_mode else 0), send_size)
            send_size = link.tx_obj(chr(1 if self.brake_active else 0), send_size)
            send_size = link.tx_obj(chr(1 if self.reverse_mode else 0), send_size)
            send_size = link.tx_obj(int(self.speed), send_size)
            send_size = link.tx_obj(int(self.steering_angle), send_size)
            send_size = link.tx_obj(chr(1 if self.debug_mode else 0), send_size)
            stdout_capture = io.StringIO()
            stderr_capture = io.StringIO()
            with contextlib.redirect_stdout(stdout_capture), contextlib.redirect_stderr(stderr_capture):
                link.send(send_size)

            suppressed_output = stdout_capture.getvalue() + stderr_capture.getvalue()
            if suppressed_output.strip():
                raise RuntimeError(self.summarize_suppressed_serial_output(suppressed_output))

            now_sec = self.now_sec()
            if (
                packet_signature != self.last_logged_packet_by_device.get(dev_name)
                or now_sec - self.last_logged_packet_time_by_device.get(dev_name, 0.0)
                >= self.packet_log_interval_sec
            ):
                self.get_logger().info(
                    f"{dev_name}: manual={self.manual_mode}, brake={self.brake_active}, "
                    f"reverse={self.reverse_mode}, speed={self.speed}, steer={self.steering_angle}"
                )
                self.last_logged_packet_by_device[dev_name] = packet_signature
                self.last_logged_packet_time_by_device[dev_name] = now_sec
            self.last_sent_packet_by_device[dev_name] = packet_signature
            self.last_sent_time_by_device[dev_name] = now_sec
            self.last_send_error_by_device.pop(dev_name, None)
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending to {dev_name}: {e}")
            return False

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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
