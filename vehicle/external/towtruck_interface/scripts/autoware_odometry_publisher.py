#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    GearReport,
    HazardLightsCommand,
    HazardLightsReport,
    SteeringReport,
    TurnIndicatorsCommand,
    TurnIndicatorsReport,
    VelocityReport,
)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32 
import math

class AutowareOdometryPublisher(Node):

    def __init__(self):
        super().__init__("autoware_odometry_publisher")
        self.wheel_base = float(self.declare_parameter("wheel_base", 1.17).value)

        self.v = 0.0
        self.steering = 0.0

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
        self.turn_cmd_pub = self.create_publisher(
            TurnIndicatorsCommand, "/planning/turn_indicators_cmd", command_qos
        )
        self.hazard_cmd_pub = self.create_publisher(
            HazardLightsCommand, "/planning/hazard_lights_cmd", command_qos
        )

        # Publish at 50Hz to ensure EKF is happy
        self.create_timer(0.02, self.publish_status)

    def cb_v(self, msg): 
        self.v = msg.data

    def cb_steering(self, msg): 
        # Convert degrees to radians for Autoware
        self.steering = msg.data * (math.pi / 180.0)

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
        # 2 = DRIVE gear. Tells RViz the car is in gear.
        gear = GearReport()
        gear.stamp = now
        gear.report = GearReport.DRIVE 
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

        turn_cmd = TurnIndicatorsCommand()
        turn_cmd.stamp = now
        turn_cmd.command = TurnIndicatorsCommand.DISABLE
        self.turn_cmd_pub.publish(turn_cmd)

        hazard_cmd = HazardLightsCommand()
        hazard_cmd.stamp = now
        hazard_cmd.command = HazardLightsCommand.DISABLE
        self.hazard_cmd_pub.publish(hazard_cmd)

def main ():
    rclpy.init()
    node = AutowareOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
