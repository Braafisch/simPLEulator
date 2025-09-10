#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped
from vehicle_msgs.msg import ConeDetection, ConeTrackedArray
from tf2_ros import Buffer, TransformListener
from filterpy.kalman import KalmanFilter
from scipy.spatial import cKDTree
from scipy.optimize import linear_sum_assignment


class TrackingNode(Node):
    def __init__(self):
        super().__init__("fsd_perception_tracking")

        # Params
        self.declare_parameter("input_topic", "/detection/cones")
        self.declare_parameter("output_topic", "/perception/tracked_cones")
        self.declare_parameter("source_frame", "lidar_link")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("gating_distance", 1.5)  # [m]
        self.declare_parameter("max_lives", 5)  # frames
        self.declare_parameter("min_lives", 3)
        self.declare_parameter("process_var", 2.0)
        self.declare_parameter("meas_var", 0.05)
        self.declare_parameter("min_conf_new", 0.3)
        self.declare_parameter("conf_inc", 0.1)
        self.declare_parameter("conf_dec", 0.1)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.source_frame = self.get_parameter("source_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.gating = float(self.get_parameter("gating_distance").value)
        self.max_lives = int(self.get_parameter("max_lives").value)
        self.min_lives = int(self.get_parameter("min_lives").value)
        self.proc_var = float(self.get_parameter("process_var").value)
        self.meas_var = float(self.get_parameter("meas_var").value)
        self.min_conf_new = float(self.get_parameter("min_conf_new").value)
        self.conf_inc = float(self.get_parameter("conf_inc").value)
        self.conf_dec = float(self.get_parameter("conf_dec").value)

        # TF
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        self.pub_cones = self.create_publisher(
            ConeTrackedArray, self.get_parameter("output_topic").value, 20
        )

        # QoS: Sensordaten
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.cones_src = []
        self.create_subscription(
            ConeDetection, self.input_topic, self.cb_detections, qos
        )

    def transform_cones(self, point: Point, stamp):
        ps = PointStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.source_frame
        ps.point = point
        tr = self.tf_buf.lookup_transform(
            self.target_frame,
            self.source_frame,
            Time(),
            timeout=Duration(seconds=0.2),
        )
        tp = tf2_geometry_msgs.do_transform_point(ps, tr)
        return tp.point

    def cb_detections(self, msg: ConeDetection):
        dets = []
        for cone in msg.cones:
            pt = self.transform_cones(cone, msg.header.stamp)
            if pt is None:
                continue
            dets.append((pt.x, pt.y))


def main():
    rclpy.init()
    node = TrackingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
