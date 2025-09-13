#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
import numpy as np


class FsdController(Node):
    def __init__(self):
        super().__init__("fsd_controller")
        # Params
        self.L = self.declare_parameter("wheelbase", 2.6).value
        self.kp_v = self.declare_parameter("kp_v", 0.45).value
        self.ld_min = self.declare_parameter("ld_min", 4.5).value
        # self.k_ld = self.declare_parameter("k_ld", 0.8).value
        self.max_steer = self.declare_parameter("max_steer", 10).value
        self.max_speed = self.declare_parameter("max_speed", 3).value  # m/s
        # self.max_acc = self.declare_parameter("max_acc", 1).value  # m/s^2
        self.rate_hz = self.declare_parameter("rate_hz", 100).value

        self.sub_oms = self.create_subscription(
            TwistStamped, "/vehicle/ground_speed", self.cb_oms, 10
        )
        self.sub_path = self.create_subscription(
            Path, "/trajectory/path", self.cb_path, 10
        )
        
        self.timer = self.create_timer(1.0 / self.rate_hz, self.step)

        self.v = 0.0
        self.last_stamp = None
        self.dist = 0.0
        self.path = Path()

    def cb_oms(self, msg: TwistStamped):
        self.v = np.sqrt((msg.twist.linear.x**2 + msg.twist.linear.y**2))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        dt = stamp - self.last_stamp if self.last_stamp != None else 5e-3
        self.dist += dt * self.v

    def cb_path(self, msg: Path):
        self.path = msg

    def speed_controll(self):
        target_speed = self.max_speed if self.dist < 76.5 else 0.0
        return self.v + self.kp_v * (target_speed - self.v) if self.v >= 0.2 else 0.0

    def steering_controll(self, x, y):
        alpha = np.arctan2(y, x + 1e-6)
        delta = np.arctan2(2.0 * self.L * np.sin(alpha), self.ld_min)
        return float(np.clip(delta, -self.max_steer, self.max_steer))
    
    def find_point(self):
        for pose in self.path.poses: 

    def step(self):
        x_f, y_f = self.find_point()
        
def main():
    rclpy.init()
    node = FsdController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()