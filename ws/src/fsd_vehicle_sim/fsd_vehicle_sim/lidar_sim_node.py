#!/usr/bin/env python3
# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
# ros2 run tf2_ros static_transform_publisher 0.9 0.0 0.35 0 0 0 base_link lidar_link

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, PointStamped
from vehicle_msgs.msg import ConeDetection
import tf2_ros
import tf2_geometry_msgs


class OS1LikeLidar(Node):
    def __init__(self):
        super().__init__("os1_like_lidar")

        # ---- Input / Output config
        self.declare_parameter("cones_topic", "/visualization_marker_array")
        self.declare_parameter("lidar_frame", "lidar_link")
        self.declare_parameter("rate_hz", 20.0)

        # ---- Ranging limits / masks
        self.declare_parameter("min_range", 0.2)
        self.declare_parameter("max_range", 90.0)
        self.declare_parameter("h_fov_deg", 170.0)
        self.declare_parameter("min_z", -1.0)
        self.declare_parameter("max_z", 2.0)

        # ---- Noise/model
        self.declare_parameter("pos_noise_xy_std", 0.03)
        self.declare_parameter("pos_noise_z_std", 0.02)
        self.declare_parameter("dropout_prob", 0.0)  # per-beam miss prob

        # ---- Cone geometry (for occlusion)
        self.declare_parameter("cone_radius", 0.15)  # m (≈ Fußradius)
        self.declare_parameter("cone_height", 0.5)  # m (Marker uses height; top ~0.5)

        # Params -> members
        self.cones_topic = self.get_parameter("cones_topic").value
        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.h_fov_rad = math.radians(self.get_parameter("h_fov_deg").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)

        self.noise_xy = float(self.get_parameter("pos_noise_xy_std").value)
        self.noise_z = float(self.get_parameter("pos_noise_z_std").value)
        self.dropout_p = float(self.get_parameter("dropout_prob").value)

        self.cone_r = float(self.get_parameter("cone_radius").value)
        self.cone_h = float(self.get_parameter("cone_height").value)

        # Subscribe cones
        # store (x,y,z,label) in source frame
        self.cones_src = []
        self.cones_src_frame = None
        self.create_subscription(MarkerArray, self.cones_topic, self.cb_cones, 10)

        # Publisher: ground-truth cones
        self.pub_gt = self.create_publisher(ConeDetection, "cones", 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer (simple fixed rate)
        self.dt = 1.0 / max(0.1, self.rate_hz)
        self.timer = self.create_timer(self.dt, self.step)

    def cb_cones(self, msg: MarkerArray):
        pts = []
        src_frame = msg.markers[0].header.frame_id if msg.markers else None
        for m in msg.markers:
            x, y, z = m.pose.position.x, m.pose.position.y, m.pose.position.z
            pts.append((x, y, z))
        self.cones_src_frame = src_frame
        self.cones_src = pts

    def transform_point(self, p_src):
        ps = PointStamped()
        ps.header.frame_id = self.cones_src_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.point.x, ps.point.y, ps.point.z = p_src

        try:
            tr = self.tf_buffer.lookup_transform(
                self.lidar_frame,
                self.cones_src_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            tp = tf2_geometry_msgs.do_transform_point(ps, tr)
            return (tp.point.x, tp.point.y, tp.point.z)
        except Exception as e:
            self.get_logger().warn_throttle(
                2000, f"TF {self.cones_src_frame}->{self.lidar_frame} fehlt: {e}"
            )
            return None

    def transform_cones_to_lidar(self):
        if not self.cones_src or not self.cones_src_frame:
            return []
        if not self.tf_buffer.can_transform(
            self.lidar_frame, self.cones_src_frame, Time()
        ):
            return []
        out = []
        for cone_src in self.cones_src:
            t = self.transform_point(cone_src)
            if t is not None:
                out.append(t)
        return out

    def check_if_visible(self, cones_lidar):
        visible = []
        for cx, cy, cz in cones_lidar:
            rng = math.sqrt(cx * cx + cy * cy + cz * cz)
            if rng < self.min_range or rng > self.max_range:
                continue
            if cz < self.min_z or cz > self.max_z:
                continue
            if self.h_fov_rad < 2.0 * math.pi:
                # azimuth: 0 along +x, CCW
                az = math.atan2(cy, cx)
                # keep centered around +x
                if (
                    abs(((az + math.pi) % (2 * math.pi)) - math.pi)
                    > self.h_fov_rad * 0.5
                ):
                    continue
            visible.append((cx, cy, cz))
        return visible

    def step(self):
        cones_lidar = self.transform_cones_to_lidar()
        if not cones_lidar:
            return
        visible_cones = self.check_if_visible(cones_lidar)
        msg = ConeDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.lidar_frame
        for cx, cy, cz in visible_cones:
            if self.dropout_p > 0.0 and random.random() < self.dropout_p:
                continue
            c = Point()
            c.x = cx + random.gauss(0.0, self.noise_xy)
            c.y = cy + random.gauss(0.0, self.noise_xy)
            c.z = cz + random.gauss(0.0, self.noise_z)
            msg.cones.append(c)
        msg.count = len(msg.cones)
        self.pub_gt.publish(msg)


def main():
    rclpy.init()
    node = OS1LikeLidar()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
