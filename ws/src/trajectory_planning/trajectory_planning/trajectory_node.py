#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from vehicle_msgs.msg import ConeTrackedArray, ConeTracked
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
from sklearn.linear_model import RANSACRegressor, LinearRegression


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__("fsd_trajectory_planning")
        self.declare_parameter("vehicle_frame_id", "base_link")
        self.declare_parameter("global_frame_id", "odom")
        self.declare_parameter("cones_topic", "/perception/tracked_cones")
        self.declare_parameter("trajectory_topic", "/trajectory/path")
        self.declare_parameter("ransac_thresh", 0.25)  # in m
        self.declare_parameter("ransac_iters", 100)
        self.declare_parameter("path_step", 0.5)
        self.declare_parameter("planning_forward", 100.0)
        self.declare_parameter("planning_backward", 2.0)
        self.declare_parameter("min_cones", 6)
        self.declare_parameter("cone_line_thresh", 0.5)  # in m
        self.declare_parameter("path_hz", 5)

        self.vehicle_frame_id = self.get_parameter("vehicle_frame_id").value
        self.global_frame_id = self.get_parameter("global_frame_id").value
        self.cones_topic = self.get_parameter("cones_topic").value
        self.trajectory_topic = self.get_parameter("trajectory_topic").value
        self.ransac_thresh = float(self.get_parameter("ransac_thresh").value)
        self.ransac_iters = self.get_parameter("ransac_iters").value
        self.path_step = self.get_parameter("path_step").value
        self.planning_forward = self.get_parameter("planning_forward").value
        self.planning_backward = self.get_parameter("planning_backward").value
        self.min_cones = self.get_parameter("min_cones").value
        self.cone_line_thresh = self.get_parameter("cone_line_thresh").value
        self.path_hz = self.get_parameter("path_hz").value

        self.path = []
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        self.create_subscription(ConeTrackedArray, self.cones_topic, self.cb, 5)
        self.pub_trajectory = self.create_publisher(Path, self.trajectory_topic, 10)
        self.timer = self.create_timer(1.0 / self.path_hz, self.step)

    def ransac_fit_line(
        self, points: np.ndarray, residual_threshold=0.25, max_trials=100
    ):
        """
        points: Nx2 array (x,y).
        Rückgabe: slope m, intercept b
        """
        if points.shape[0] < 2:
            return 0.0, 0.0

        X = points[:, 0][:, None]  # x-Koordinaten
        y = points[:, 1]  # y-Koordinaten

        ransac = RANSACRegressor(
            estimator=LinearRegression(),
            min_samples=2,
            residual_threshold=residual_threshold,
            max_trials=max_trials,
        )
        ransac.fit(X, y)

        m = ransac.estimator_.coef_[0]
        b = ransac.estimator_.intercept_
        inlier_mask = ransac.inlier_mask_
        return m, b, inlier_mask

    def centerline_path(self, m, b, x_start=0.0, x_end=100.0, step=0.5):
        xs = np.arange(x_start, x_end, step)
        ys = m * xs + b
        return np.stack([xs, ys], axis=1)

    def transform_path(self, path_vehicle, header):
        path_map = []
        for np_path_v in path_vehicle:
            p = Point()
            p.x = np_path_v[0]
            p.y = np_path_v[1]
            tp = self.transform_point(
                p, header, self.vehicle_frame_id, self.global_frame_id
            )
            path_map.append(tp)

        return path_map

    def transform_point(self, point: Point, header, source, target):
        ps = PointStamped()
        ps.header = header
        ps.point = point
        tr = self.tf_buf.lookup_transform(
            target,
            source,
            Time(),
            timeout=Duration(seconds=0.2),
        )
        tp = tf2_geometry_msgs.do_transform_point(ps, tr)
        return tp.point

    def cb(self, msg: ConeTrackedArray):
        if (
            not self.tf_buf.can_transform(
                self.global_frame_id, self.vehicle_frame_id, Time()
            )
            or len(msg.cones) < self.min_cones
        ):
            return
        p_cones_global = [c.position for c in msg.cones]
        p_cones_local = [
            self.transform_point(
                c, msg.header, self.global_frame_id, self.vehicle_frame_id
            )
            for c in p_cones_global
        ]
        cones_local_rigth = []
        cones_local_left = []
        for p_cone in p_cones_local:
            if abs(p_cone.y) <= (1.5 + self.cone_line_thresh) and abs(p_cone.y) >= (
                1.5 - self.cone_line_thresh
            ):
                if p_cone.y >= 0:
                    cones_local_rigth.append((p_cone.x, p_cone.y))
                else:
                    cones_local_left.append((p_cone.x, p_cone.y))
            else:
                continue

        m_L, b_L, inL = self.ransac_fit_line(
            np.array(cones_local_left),
            residual_threshold=self.ransac_thresh,
            max_trials=self.ransac_iters,
        )
        m_R, b_R, inR = self.ransac_fit_line(
            np.array(cones_local_rigth),
            residual_threshold=self.ransac_thresh,
            max_trials=self.ransac_iters,
        )
        # Centerline mitteln
        m_C = 0.5 * (m_L + m_R)
        b_C = 0.5 * (b_L + b_R)
        v_path = self.centerline_path(
            m_C,
            b_C,
            x_start=-self.planning_backward,
            x_end=self.planning_forward,
            step=self.path_step,
        )
        self.path = self.transform_path(v_path, msg.header)

    def step(self):
        if len(self.path) == 0:
            return
        stamp = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame_id
        path_msg.header.stamp = stamp
        for p in self.path:
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame_id
            pose.header.stamp = stamp
            pose.pose.position = p
            path_msg.poses.append(pose)
        self.pub_trajectory.publish(path_msg)


def main():
    rclpy.init()
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
