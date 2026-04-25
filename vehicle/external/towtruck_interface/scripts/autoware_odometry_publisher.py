#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    GearCommand,
    GearReport,
    HazardLightsReport,
    SteeringReport,
    TurnIndicatorsReport,
    VelocityReport,
)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32 
import math


def gear_report_from_command(command: int):
    if command in (GearCommand.NONE, GearCommand.NEUTRAL, GearCommand.PARK):
        return None
    return command


class AutowareOdometryPublisher(Node):

    def __init__(self):
        super().__init__("autoware_odometry_publisher")
        self.wheel_base = float(self.declare_parameter("wheel_base", 1.1).value)

        self.v = 0.0
        self.steering = 0.0
        self.current_gear_report = GearReport.DRIVE

        # We ONLY listen for raw data from your kinematics/serial node
        # We DO NOT calculate x, y, theta anymore. Autoware does that.
        self.create_subscription(Float32, "v", self.cb_v, 10)
        self.create_subscription(Float32, "steering_angle_data", self.cb_steering, 10)

        # Publishers for Autoware Vehicle Feedback
        # EKF Localizer listens to these to calculate position!
        self.vel_pub = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 10)

        self.steer_pub = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 10)
        
        # NEW: Safety Heartbeat Publishers (Makes the Auto button appear)
        self.mode_pub = self.create_publisher(ControlModeReport, "/vehicle/status/control_mode", 10)
        self.gear_pub = self.create_publisher(GearReport, "/vehicle/status/gear_status", 10)
        self.turn_pub = self.create_publisher(TurnIndicatorsReport, "/vehicle/status/turn_indicators_status", 10)
        self.hazard_pub = self.create_publisher(HazardLightsReport, "/vehicle/status/hazard_lights_status", 10)
        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            GearCommand, "/control/command/gear_cmd", self.cb_gear_cmd, command_qos
        )

        # Publish at 50Hz to ensure EKF is happy
        self.create_timer(0.02, self.publish_status)

    def cb_v(self, msg): 
        self.v = msg.data

    def cb_steering(self, msg): 
        # Convert degrees to radians for Autoware
        self.steering = msg.data * (math.pi / 180.0)

    def cb_gear_cmd(self, msg: GearCommand):
        reported_gear = gear_report_from_command(msg.command)
        if reported_gear is not None:
            self.current_gear_report = reported_gear

    def publish_status(self):
        now = self.get_clock().now().to_msg()

        # ---------------- Velocity Report --------------
        # Autoware uses this to predict movement (x += v * dt)
        vel = VelocityReport()
        vel.header.stamp = now
        vel.header.frame_id = "base_link" 
        vel.longitudinal_velocity = float(self.v)
        vel.lateral_velocity = 0.0 # Assuming non-holonomic
        vel.heading_rate = 0.0 if self.wheel_base <= 1.0e-6 else float(
            (self.v / self.wheel_base) * math.tan(self.steering)
        )
        
        self.vel_pub.publish(vel)

        # ---------------- Steering Report --------------
        steer = SteeringReport()
        steer.steering_tire_angle = float(self.steering)

        self.steer_pub.publish(steer)


        # ---------------- Control Mode Report ----------
        # 1 = AUTONOMOUS mode. Tells RViz the car is ready.
        mode = ControlModeReport()
        mode.stamp = now
        mode.mode = 1 
        self.mode_pub.publish(mode)

        # ---------------- Gear Report ------------------
        gear = GearReport()
        gear.stamp = now
        gear.report = self.current_gear_report
        self.gear_pub.publish(gear)

        # ---------------- Lights (Dummy OFF) ---------------
        turn = TurnIndicatorsReport()
        turn.stamp = now
        turn.report = TurnIndicatorsReport.DISABLE # 1 = Disable/Off
        self.turn_pub.publish(turn)

        hazard = HazardLightsReport()
        hazard.stamp = now
        hazard.report = HazardLightsReport.DISABLE # 1 = Disable/Off
        self.hazard_pub.publish(hazard)

def main ():
    rclpy.init()
    node = AutowareOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
