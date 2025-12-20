#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import serial
import glob
import time
import os


class SerialArduinoReader(Node):

    def __init__(self):
        super().__init__('serial_reader')

        # ---------------- Publishers ----------------
        self.pub_frequency = self.create_publisher(Float32, 'frequency_data', 10)
        self.pub_steering = self.create_publisher(Float32, 'steering_angle_data', 10)

        # ---------------- Serial Handles ----------------
        self.arduino_frequency = None
        self.arduino_steering = None

        # Preferred (stable) device names
        self.freq_preferred = "/dev/arduino_wheels"
        self.steer_preferred = "/dev/arduino_steering"

        # Baudrate
        self.baud = 115200

        # Timer to manage reconnect + read
        self.timer = self.create_timer(0.05, self.loop)

    # ==========================================================
    # SERIAL DISCOVERY (ROBUST AGAINST DOCKER)
    # ==========================================================
    def connect_serial(self, preferred_path):
        """
        Try preferred path first, then fallback to /dev/ttyACM*
        """
        # 1️⃣ Try preferred device
        if preferred_path and os.path.exists(preferred_path):
            try:
                ser = serial.Serial(preferred_path, self.baud, timeout=0.1)
                self.get_logger().info(f"Connected to {preferred_path}")
                return ser
            except Exception as e:
                self.get_logger().warn(f"Failed opening {preferred_path}: {e}")

        # 2️⃣ Fallback: any ttyACM*
        for dev in sorted(glob.glob("/dev/ttyACM*")):
            try:
                ser = serial.Serial(dev, self.baud, timeout=0.1)
                self.get_logger().info(f"Connected to fallback device {dev}")
                return ser
            except Exception:
                pass

        return None

    # ==========================================================
    # MAIN LOOP
    # ==========================================================
    def loop(self):

        # ---------- Ensure frequency Arduino ----------
        if self.arduino_frequency is None or not self.arduino_frequency.is_open:
            self.arduino_frequency = self.connect_serial(self.freq_preferred)
            return

        # ---------- Ensure steering Arduino ----------
        if self.arduino_steering is None or not self.arduino_steering.is_open:
            self.arduino_steering = self.connect_serial(self.steer_preferred)
            return

        # ---------- Read frequency ----------
        try:
            if self.arduino_frequency.in_waiting:
                line = self.arduino_frequency.readline().decode(errors="ignore").strip()
                frq = float(line)
                msg = Float32()
                msg.data = frq
                self.pub_frequency.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Frequency serial error: {e}")
            self.safe_close(self.arduino_frequency)
            self.arduino_frequency = None

        # ---------- Read steering ----------
        try:
            if self.arduino_steering.in_waiting:
                line = self.arduino_steering.readline().decode(errors="ignore").strip()
                steering = float(line)
                msg = Float32()
                msg.data = steering
                self.pub_steering.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Steering serial error: {e}")
            self.safe_close(self.arduino_steering)
            self.arduino_steering = None

    def safe_close(self, ser):
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = SerialArduinoReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
