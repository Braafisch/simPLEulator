#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped, TransformStamped
import tf2_ros
import math


def check_yaw(yaw):
    if yaw > math.pi:
        yaw -= 2 * math.pi
    if yaw <= -math.pi:
        yaw += 2 * math.pi
    return yaw


def body_to_world(vx_b, vy_b, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    vx_w = c * vx_b - s * vy_b
    vy_w = s * vx_b + c * vy_b
    return vx_w, vy_w


class Localization(Node):
    def __init__(self):
        super().__init__("localization")
        self.declare_parameter("sensor_frame", "/vehicle/ground_speed")
        self.declare_parameter("target_frame_tf", "base_link")
        self.declare_parameter("source_frame_tf", "odom")

        # Read parameters with typed helpers
        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.target_frame_tf = self.get_parameter("target_frame_tf").value
        self.source_frame_tf = self.get_parameter("source_frame_tf").value

        qos = QoSProfile(depth=10)

        self.sub_oms = self.create_subscription(
            TwistStamped,
            "/vehicle/ground_speed",
            self.cb,
            qos,
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.last_stamp = None

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.source_frame_tf
        t.child_frame_id = self.target_frame_tf
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw * 0.5)
        t.transform.rotation.w = math.cos(self.yaw * 0.5)
        self.tf_broadcaster.sendTransform(t)

    def cb(self, msg: TwistStamped):
        vx_oms = msg.twist.linear.x
        vy_oms = msg.twist.linear.y
        yaw_rate_oms = msg.twist.angular.z
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        dt = stamp - self.last_stamp if self.last_stamp != None else 5e-3
        self.last_stamp = stamp
        self.yaw += math.radians(yaw_rate_oms * dt)
        self.yaw = check_yaw(self.yaw)
        vx, vy = body_to_world(vx_oms, vy_oms, self.yaw)
        self.x += vx * dt
        self.y += vy * dt
        self.publish_tf()


def main():
    rclpy.init()
    node = Localization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
