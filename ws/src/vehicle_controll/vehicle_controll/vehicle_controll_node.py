#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped, PointStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener
import numpy as np


class FsdController(Node):
    def __init__(self):
        super().__init__("fsd_controller")
        # Params
        self.L = self.declare_parameter("wheelbase", 2.6).value
        self.kp_v = self.declare_parameter("kp_v", 0.8).value
        self.ld_min = self.declare_parameter("ld_min", 4.5).value
        # self.k_ld = self.declare_parameter("k_ld", 0.8).value
        self.max_steer = self.declare_parameter("max_steer", np.radians(5)).value
        self.max_speed = self.declare_parameter("max_speed", 15).value  # m/s
        # self.max_acc = self.declare_parameter("max_acc", 1).value  # m/s^2
        self.rate_hz = self.declare_parameter("rate_hz", 100).value
        self.vehicle_frame_id = self.declare_parameter(
            "vehicle_frame_id", "base_link"
        ).value
        self.global_frame_id = self.declare_parameter("global_frame_id", "odom").value

        self.sub_oms = self.create_subscription(
            TwistStamped, "/vehicle/ground_speed", self.cb_oms, 10
        )
        self.sub_path = self.create_subscription(
            Path, "/trajectory/path", self.cb_path, 10
        )

        self.pub_v = self.create_publisher(Float32, "/target_speed", 10)
        self.pub_d = self.create_publisher(Float32, "/steering_angle", 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.step)

        self.v = 0.0
        self.last_stamp = None
        self.dist = 0.0
        self.path = Path()

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

    def cb_oms(self, msg: TwistStamped):
        self.v = np.sqrt((msg.twist.linear.x**2 + msg.twist.linear.y**2))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        dt = stamp - self.last_stamp if self.last_stamp != None else 5e-3
        self.dist += dt * self.v
        self.last_stamp = stamp

    def cb_path(self, msg: Path):
        self.path = msg

    def speed_controll(self):
        target_speed = self.max_speed if self.dist < 76.0 else 0.0
        return (
            self.v + self.kp_v * (target_speed - self.v)
            if self.v >= 1.0 or target_speed > 0.0
            else 0.0
        )

    def steering_controll(self, x, y):
        alpha = np.arctan2(y, x + 1e-6)
        delta = np.arctan2(2.0 * self.L * np.sin(alpha), self.ld_min)
        return float(np.clip(delta, -self.max_steer, self.max_steer))

    def find_point(self):
        dist = 0
        for pose in self.path.poses:
            ps = PointStamped()
            ps.point = pose.pose.position
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = self.global_frame_id
            tr = self.tf_buf.lookup_transform(
                self.vehicle_frame_id,
                self.global_frame_id,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            tp = tf2_geometry_msgs.do_transform_point(ps, tr)
            if tp.point.x <= 0:
                continue
            dist = np.sqrt(tp.point.x**2 + tp.point.y**2)
            if dist >= self.ld_min:
                return tp.point.x, tp.point.y

    def step(self):
        if (
            not self.tf_buf.can_transform(
                self.global_frame_id, self.vehicle_frame_id, Time()
            )
            or len(self.path.poses) == 0
        ):
            return
        x_f, y_f = self.find_point()
        steering_angel = self.steering_controll(x_f, y_f)
        target_speed = self.speed_controll()
        msg_v = Float32()
        msg_v.data = target_speed
        msg_d = Float32()
        msg_d.data = steering_angel
        self.pub_v.publish(msg_v)
        self.pub_d.publish(msg_d)


def main():
    rclpy.init()
    node = FsdController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
