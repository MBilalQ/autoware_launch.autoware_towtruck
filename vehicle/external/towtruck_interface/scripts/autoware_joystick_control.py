#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from autoware_control_msgs.msg import Control
import math
from enum import Enum
from pySerialTransfer import pySerialTransfer as txfer
from numpy import interp
from collections import deque

# --- XBOX Controller Mappings ---
class Button(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
    FRONT_LEFT_TOP = 4
    FRONT_RIGHT_TOP = 5
    CENTER_LEFT = 6    # Teleop
    CENTER_RIGHT = 7   # Manual
    LEFT_THUMB = 9
    RIGHT_THUMB = 10
    CENTER_MIDDLE = 11 # Autonomous

class Axis(Enum):
    LEFT_THUMB_HORIZONTAL = 0  # Steering
    LEFT_THUMB_VERTICAL = 1
    FRONT_LEFT_BOTTOM = 2
    RIGHT_THUMB_HORIZONTAL = 3
    RIGHT_THUMB_VERTICAL = 4
    FRONT_RIGHT_BOTTOM = 5     # Speed
    ARROW_LEFT_RIGHT = 6
    ARROW_UP_DOWN = 7

class AutowareArduinoInterface(Node):
    def __init__(self):
        super().__init__('autoware_arduino_interface')

        # --- Configuration ---
        self.control_update_period = 0.06 # 20Hz
        self.wheel_radius = 0.2032
        self.pulses_per_rev = 740 * 1.1

        # --- SUBSCRIBERS ---
        self.create_subscription(Joy, '/joy', self.joy_callback, 1)
        self.create_subscription(Control, '/control/command/control_cmd', self.control_callback, 10)

        # --- SERIAL CONNECTIONS ---
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
        self.autonom_mode = True # Safest to start in autonomous (or False if you prefer manual startup)
        self.manual_mode = False
        self.teleop_mode = False
        self.reinitialize = False
        self.brake_active = False
        self.reverse_mode = False
        self.debug_mode = False

        self.prev_reverse_button = 0
        self.prev_debug_mode_button = 0

        # Output Variables (Sent to Arduino)
        self.speed = 0
        self.steering_angle = 0
        
        # Source Variables (Updated by callbacks)
        self.auto_speed = 0
        self.auto_steering_angle = 0
        self.auto_reverse = False
        
        self.manual_speed = 0
        self.manual_steering = 0
        self.manual_reverse = False

        # Steering smoothing
        steering_angle_time_constant = 0.06
        steering_angle_window_size = int(max(1, steering_angle_time_constant / self.control_update_period))
        self.steering_angle_window = deque(maxlen=steering_angle_window_size)

        self.create_timer(self.control_update_period, self.send_commands)

    def control_callback(self, msg: Control):
        """ Updates Autonomous variables continuously """
        target_v = msg.longitudinal.velocity
        
        # Autoware reverse/forward logic
        if target_v < 0:
            self.auto_reverse = True
            v_abs = abs(target_v)
        else:
            self.auto_reverse = False
            v_abs = target_v

        # Calculate Auto Speed
        if v_abs < 0.01:
            self.auto_speed = 0
        else:
            self.auto_speed = int((v_abs * self.pulses_per_rev) / (2 * math.pi * self.wheel_radius))

        # Calculate Auto Steering (msg is in radians)
        steer_rad = msg.lateral.steering_tire_angle
        self.auto_steering_angle = int(math.degrees(steer_rad))

    def joy_callback(self, msg: Joy):
        """ Updates Manual variables and handles Mode Switching """
        try:
            # Buttons
            reinit_button = msg.buttons[Button.A.value]
            manual_button = msg.buttons[Button.CENTER_RIGHT.value]
            teleop_button = msg.buttons[Button.CENTER_LEFT.value]
            autonom_button = msg.buttons[Button.CENTER_MIDDLE.value]
            brake_button = msg.buttons[Button.B.value]
            reverse_button = msg.buttons[Button.FRONT_LEFT_TOP.value]
            debug_mode_button = msg.buttons[Button.Y.value] 

            self.reinitialize = reinit_button == 1
            self.brake_active = brake_button == 1

            # Mode Switching
            if manual_button == 1:
                self.manual_mode, self.teleop_mode, self.autonom_mode = True, False, False
                self.get_logger().info("Manual Mode ON")
            elif teleop_button == 1:
                self.manual_mode, self.teleop_mode, self.autonom_mode = False, True, False
                self.get_logger().info("Teleop Mode ON")
            elif autonom_button == 1:
                self.manual_mode, self.teleop_mode, self.autonom_mode = False, False, True
                self.get_logger().info("Autonomous Mode ON")

            # Manual Reverse Toggle
            if reverse_button == 1 and self.prev_reverse_button == 0:
                self.manual_reverse = not self.manual_reverse
                self.get_logger().info(f"Manual Direction: {'Reverse' if self.manual_reverse else 'Forward'}")

            # Debug Mode Toggle
            if debug_mode_button == 1 and self.prev_debug_mode_button == 0:
                self.debug_mode = not self.debug_mode
                self.get_logger().info(f"Debug Mode: {self.debug_mode}")

            # Calculate Manual Speed and Steering
            raw_speed = msg.axes[Axis.FRONT_RIGHT_BOTTOM.value]
            self.manual_speed = int(interp(4.4 - (raw_speed + 1) * ((4.4 - 2.5) / 2), [2.5, 4.4], [0, 600]))
            
            raw_steering = msg.axes[Axis.LEFT_THUMB_HORIZONTAL.value]
            self.manual_steering = int(53 - (raw_steering + 1) * ((53 + 53) / 2)) * -1

            self.prev_reverse_button = reverse_button
            self.prev_debug_mode_button = debug_mode_button

        except Exception as e:
            self.get_logger().error(f"Joystick parsing error: {e}")

    def send_commands(self):
        """ Multiplexer: Decides which variables to use and sends them """
        
        # 1. Select Data Source based on Mode
        if self.autonom_mode:
            current_target_speed = self.auto_speed
            current_target_steer = self.auto_steering_angle
            self.reverse_mode = self.auto_reverse
            # Auto-brake if speed is very low
            if self.auto_speed == 0:
                self.brake_active = True
        else:
            current_target_speed = self.manual_speed
            current_target_steer = self.manual_steering
            self.reverse_mode = self.manual_reverse

        # 2. Apply Brake Override
        if self.brake_active:
            self.speed = 0
        else:
            self.speed = int(current_target_speed)

        # 3. Apply Steering Smoothing
        self.steering_angle_window.append(current_target_steer)
        if len(self.steering_angle_window) > 0:
            self.steering_angle = int(sum(self.steering_angle_window) / len(self.steering_angle_window))

        # 4. Transmit over Serial
        if hasattr(self, 'link_wheels') and self.link_wheels.connection.is_open:
            self.send_packet(self.link_wheels, "wheels")
        if hasattr(self, 'link_steering') and self.link_steering.connection.is_open:
            self.send_packet(self.link_steering, "steering")

    def send_packet(self, link, dev_name):
        try:
            send_size = 0
            send_size = link.tx_obj(chr(1 if self.reinitialize else 0), send_size)
            send_size = link.tx_obj(chr(1 if self.manual_mode else 0), send_size)
            send_size = link.tx_obj(chr(1 if self.brake_active else 0), send_size)
            send_size = link.tx_obj(chr(1 if self.reverse_mode else 0), send_size)
            send_size = link.tx_obj(int(self.speed), send_size)
            send_size = link.tx_obj(int(self.steering_angle), send_size)
            send_size = link.tx_obj(chr(1 if self.debug_mode else 0), send_size)
            
            link.send(send_size)
        except Exception as e:
            self.get_logger().error(f"Error sending to {dev_name}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AutowareArduinoInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()