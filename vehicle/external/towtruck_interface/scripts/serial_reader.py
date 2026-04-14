#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Float32
import time  # Required for the micro-sleep

try:
    import serial
except ModuleNotFoundError:
    serial = None


def decode_float(raw_line):
    text = raw_line.decode('utf-8', errors='ignore').strip()
    if not text:
        return None

    try:
        return float(text)
    except ValueError:
        return None

def restart_serial(node, serial_dev_name, baud_rate):
    device_connected = False
    while (device_connected == False):
        try:
            serial_handler = serial.Serial(serial_dev_name, baud_rate)
            node.get_logger().info(f'Connected to {serial_dev_name}')
            device_connected = True
        except serial.SerialException as e:
            node.get_logger().error(f'Failed to connect to {serial_dev_name}: {e}')
            time.sleep(1)
    return serial_handler

def main():
    rclpy.init()
    node = rclpy.create_node('serial_reader')

    if serial is None:
        node.get_logger().error(
            "Python module 'serial' is missing. Install 'python3-serial' in the Autoware runtime "
            "before launching towtruck_interface."
        )
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)
    
    pub_float_frequency = node.create_publisher(Float32, 'frequency_data', 10)
    pub_float_steering_angle = node.create_publisher(Float32, 'steering_angle_data', 10)

    arduino_frequency = restart_serial(node, '/dev/arduino_wheels', 115200) 
    arduino_steering = restart_serial(node, '/dev/arduino_steering', 115200)

    try:
        while rclpy.ok():
            try:
                if arduino_frequency.in_waiting > 0:
                    value = decode_float(arduino_frequency.readline())
                    if value is not None:
                        msg_float = Float32()
                        msg_float.data = value
                        pub_float_frequency.publish(msg_float)
            except Exception as e:
                node.get_logger().error(f'Unexpected error: {str(e)}')
                arduino_frequency = restart_serial(node, '/dev/arduino_wheels', 115200) 

            # --- Read steering angle data ---
            try:
                if arduino_steering.in_waiting > 0:
                    value = decode_float(arduino_steering.readline())
                    if value is not None:
                        msg_float = Float32()
                        msg_float.data = value
                        pub_float_steering_angle.publish(msg_float)
            except Exception as e:
                node.get_logger().error(f'Unexpected error: {str(e)}')
                arduino_steering = restart_serial(node, '/dev/arduino_steering', 115200)

            # --- THE MICRO-SLEEP ---
            # 1 millisecond sleep. Saves the laptop CPU, but keeps Arduino buffer empty!
            time.sleep(0.001)

    except KeyboardInterrupt:
        pass 
    finally:
        if arduino_frequency.is_open:
            arduino_frequency.close()
        if arduino_steering.is_open:
            arduino_steering.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
