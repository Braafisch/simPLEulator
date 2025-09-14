#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped
from vehicle_msgs.msg import ConeDetection, ConeTrackedArray, ConeTracked
from tf2_ros import Buffer, TransformListener
from scipy.spatial import cKDTree

# from filterpy.kalman import KalmanFilter
# from scipy.optimize import linear_sum_assignment


class TrackingNode(Node):
    def __init__(self):
        super().__init__("fsd_perception_tracking")

        # Params
        self.declare_parameter("input_topic", "/detection/cones")
        self.declare_parameter("output_topic", "/perception/tracked_cones")
        self.declare_parameter("source_frame", "lidar_link")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("gating_distance", 0.15)  # [m]
        self.declare_parameter("max_misses", 5)  # frames
        self.declare_parameter("min_count", 3)
        self.declare_parameter("process_var", 2.0)
        self.declare_parameter("meas_var", 0.05)
        self.declare_parameter("min_conf_new", 0.3)
        self.declare_parameter("conf_inc", 0.1)
        self.declare_parameter("conf_dec", 0.1)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.source_frame = self.get_parameter("source_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.gating = float(self.get_parameter("gating_distance").value)
        self.max_misses = int(self.get_parameter("max_misses").value)
        self.min_count = int(self.get_parameter("min_count").value)
        self.proc_var = float(self.get_parameter("process_var").value)
        self.meas_var = float(self.get_parameter("meas_var").value)
        self.min_conf_new = float(self.get_parameter("min_conf_new").value)
        self.conf_inc = float(self.get_parameter("conf_inc").value)
        self.conf_dec = float(self.get_parameter("conf_dec").value)

        # TF
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        self.pub_tracked_cones = self.create_publisher(
            ConeTrackedArray, self.output_topic, 20
        )

        # QoS: Sensordaten
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            ConeDetection, self.input_topic, self.cb_detections, qos
        )

        self.timer = self.create_timer(1.0 / self.rate_hz, self.step)
        self.track = []
        self.id_counter = 1

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

    def associate_nn(self, track_xy, det_xy, gate=0.1):
        # Greedy NN per Track mit cKDTree und Radius-Gating.
        det_tree = cKDTree(det_xy)
        track_tree = cKDTree(track_xy)
        indexes_track = track_tree.query_ball_tree(det_tree, r=gate)
        indexes_det = det_tree.query_ball_tree(track_tree, r=gate)
        return indexes_track, indexes_det

    def mean_position(self, track, det, count):
        x = track[0] * (count - 1.0) / count + det[0] * (1.0 / count)
        y = track[1] * (count - 1.0) / count + det[1] * (1.0 / count)
        return (x, y)

    def cb_detections(self, msg: ConeDetection):
        if not self.tf_buf.can_transform(self.target_frame, self.source_frame, Time()):
            return
        dets = []
        for cone in msg.cones:
            pt = self.transform_cones(cone, msg.header.stamp)
            if pt is None:
                continue
            dets.append((pt.x, pt.y))
        if len(self.track) == 0:
            for det in dets:
                dig = {"id": self.id_counter, "p": det, "count": 1, "misses": 0}
                self.id_counter += 1
                self.track.append(dig)
        else:
            indexes_track, indexes_det = self.associate_nn(
                [tr["p"] for tr in self.track], dets, self.gating
            )
            for i in range(len(indexes_track)):
                if len(indexes_track[i]) == 1:
                    self.track[i]["count"] += 1
                    self.track[i]["misses"] = 0
                    self.track[i]["p"] = self.mean_position(
                        self.track[i]["p"],
                        dets[indexes_track[i][0]],
                        self.track[i]["count"],
                    )
                elif len(indexes_track[i]) == 0:
                    self.track[i]["misses"] += 1
                    # if self.track[i]["misses"] > self.max_misses:
                    #     self.track.pop(i)
                else:
                    continue

            for i in range(len(indexes_det)):
                if len(indexes_det[i]) == 0:
                    dig = {"id": self.id_counter, "p": dets[i], "count": 1, "misses": 0}
                    self.id_counter += 1
                    self.track.append(dig)

    def step(self):
        cone_tracked_array = ConeTrackedArray()
        cone_tracked_array.header.stamp = self.get_clock().now().to_msg()
        cone_tracked_array.header.frame_id = self.target_frame
        for cone in self.track:
            if cone["count"] > self.min_count:
                cone_tracked = ConeTracked()
                cone_tracked.id = cone["id"]
                p = Point()
                p.x = cone["p"][0]
                p.y = cone["p"][1]
                cone_tracked.position = p
                cone_tracked.color = "Someone like you"
                cone_tracked_array.cones.append(cone_tracked)

        if len(cone_tracked_array.cones) > 0:
            self.pub_tracked_cones.publish(cone_tracked_array)


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
