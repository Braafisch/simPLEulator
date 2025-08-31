#!/usr/bin/env python3

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs


class OS1LikeLidar(Node):
    def __init__(self):
        super().__init__("os1_like_lidar")

        # ---- Input / Output config
        self.declare_parameter("cones_topic", "/visualization_marker_array")
        self.declare_parameter("lidar_frame", "lidar_link")

        # ---- Scan & pattern (OS1-32-ish)
        self.declare_parameter("rings", 32)
        self.declare_parameter("vert_fov_deg", 42.4)  # ~ OS1-32 total vertical FoV
        self.declare_parameter("spin_hz", 10.0)  # 10 or 20 typical
        self.declare_parameter("columns_per_rev", 1024)  # 512..2048 typical

        # ---- Ranging limits / masks
        self.declare_parameter("min_range", 0.2)
        self.declare_parameter("max_range", 90.0)
        self.declare_parameter("h_fov_deg", 360.0)
        self.declare_parameter("min_z", -1.0)
        self.declare_parameter("max_z", 2.0)

        # ---- Noise/model
        self.declare_parameter("range_noise_std", 0.02)  # 2 cm
        self.declare_parameter("dropout_prob", 0.0)  # per-beam miss prob
        self.declare_parameter("intensity_base", 80.0)  # nominal
        self.declare_parameter("intensity_decay_per_m", 0.5)

        # ---- Cone geometry (for occlusion)
        self.declare_parameter("cone_radius", 0.15)  # m (≈ Fußradius)
        self.declare_parameter("cone_height", 0.5)  # m (Marker uses height; top ~0.5)

        # Params -> members
        self.cones_topic = self.get_parameter("cones_topic").value
        self.lidar_frame = self.get_parameter("lidar_frame").value

        # Lidar
        self.R = int(self.get_parameter("rings").value)
        self.vert_fov_rad = math.radians(self.get_parameter("vert_fov_deg").value)
        self.spin_hz = float(self.get_parameter("spin_hz").value)
        self.cols_per_rev = int(self.get_parameter("columns_per_rev").value)

        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.h_fov_rad = math.radians(self.get_parameter("h_fov_deg").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)

        self.range_noise = float(self.get_parameter("range_noise_std").value)
        self.dropout_p = float(self.get_parameter("dropout_prob").value)
        self.I0 = float(self.get_parameter("intensity_base").value)
        self.I_decay = float(self.get_parameter("intensity_decay_per_m").value)

        self.cone_r = float(self.get_parameter("cone_radius").value)
        self.cone_h = float(self.get_parameter("cone_height").value)

        vmin = -self.vert_fov_rad * 0.5
        vmax = self.vert_fov_rad * 0.5
        self.vert_angles = [
            vmin + i * (vmax - vmin) / (self.R - 1) for i in range(self.R)
        ]

        # Subscribe cones
        self.cones_xyz = []
        self.create_subscription(MarkerArray, self.cones_topic, self.cb_cones, 10)

        # Publisher
        self.pub_cloud = self.create_publisher(PointCloud2, "lidar/os1/points", 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Rotation timing
        self.rev_period = 1.0 / self.spin_hz
        self.col_dt = self.rev_period / self.cols_per_rev
        self.timer = self.create_timer(self.col_dt, self.step)

    def cb_cones(self, msg: MarkerArray):
        pts = []
        for m in msg.markers:
            x, y, z = m.pose.position.x, m.pose.position.y, m.pose.position.z
            # Use the marker's frame if different (robust)
            # We assume all markers share same frame (common case)
            pts.append((x, y, z))
        self.cones_src_frame = msg.markers[0].header.frame_id
        self.cones_xyz = pts

    def transform_point(self, p_src):
        ps = PointStamped()
        ps.header.frame_id = self.cones_src_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.point.x, ps.point.y, ps.point.z = p_src

        try:
            # erst sicherstellen, dass target/source existieren
            if not self.tf_buffer.can_transform(
                self.lidar_frame, self.cones_src_frame, Time()
            ):
                return None
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
        if not self.cones_xyz or not self.cones_src_frame:
            return []
        if not self.tf_buffer.can_transform(
            self.lidar_frame, self.cones_src_frame, Time()
        ):
            return []
        out = []
        for cone_xyz in self.cones_xyz:
            t_cone = self.transform_point(cone_xyz)
            out.append(t_cone)
        return out

    def step(self):
        cones_lidar = self.transform_cones_to_lidar()


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
