#!/usr/bin/env python3
import rclpy
import math
import random
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, TwistStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
from visualization_msgs.msg import Marker
import tf2_ros


class BicycleSim(Node):
    """
    Sehr simples kinematisches Bicycle-Model:
      x_dot = v * cos(yaw)
      y_dot = v * sin(yaw)
      yaw_dot = v/L * tan(delta)

    Inputs:
      /target_speed   (Float32)  [m/s]
      /steering_angle (Float32)  [rad] (Lenkwinkel Vorderrad)

    Outputs:
      /odom (nav_msgs/Odometry)  (frame_id: map, child_frame_id: base_link)
      TF: map -> base_link
    """

    def __init__(self):
        super().__init__("bicycle_sim")

        # --- Parameter ---
        # Fahrzeug / Modell
        self.declare_parameter("wheelbase", 1.6)  # [m]
        self.declare_parameter("rate_hz", 50.0)  # [Hz]
        self.declare_parameter("speed_limit", 15.0)  # [m/s]
        self.declare_parameter("steer_limit", math.radians(35.0))  # [rad]
        self.declare_parameter("tau_speed", 0.3)  # [s]  1st-order zu Sollgeschw.
        self.declare_parameter("tau_steer", 0.2)  # [s]  1st-order zu Soll-Lenkw.
        self.declare_parameter("x0", 0.0)
        self.declare_parameter("y0", 0.0)
        self.declare_parameter("yaw0", 0.0)  # [rad]
        self.declare_parameter("frame_map", "map")
        self.declare_parameter("frame_base", "base_link")

        # „Sensorik“
        self.declare_parameter("gps_lat0_deg", 48.999000)  # Referenz (z.B. Testgelände)
        self.declare_parameter("gps_lon0_deg", 8.999000)
        self.declare_parameter("gps_alt0_m", 300.0)
        self.declare_parameter("noise_gps_xy_std", 0.20)  # m (1σ)
        self.declare_parameter("noise_gps_alt_std", 0.5)  # m

        # Marker (Auto-Klotz)
        self.declare_parameter("marker_length", 1.8)  # [m]
        self.declare_parameter("marker_width", 1.2)  # [m]
        self.declare_parameter("marker_height", 0.5)  # [m]
        self.declare_parameter("marker_ns", "vehicle")

        # -------- Parameter lesen -------
        self.L = float(self.get_parameter("wheelbase").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.speed_lim = float(self.get_parameter("speed_limit").value)
        self.steer_lim = float(self.get_parameter("steer_limit").value)
        self.tau_v = float(self.get_parameter("tau_speed").value)
        self.tau_d = float(self.get_parameter("tau_steer").value)
        self.frame_map = (
            self.get_parameter("frame_map").get_parameter_value().string_value
        )
        self.frame_base = (
            self.get_parameter("frame_base").get_parameter_value().string_value
        )

        self.lat0 = math.radians(float(self.get_parameter("gps_lat0_deg").value))
        self.lon0 = math.radians(float(self.get_parameter("gps_lon0_deg").value))
        self.alt0 = float(self.get_parameter("gps_alt0_m").value)
        self.R_earth = 6378137.0

        self.noise_gps_xy = float(self.get_parameter("noise_gps_xy_std").value)
        self.noise_gps_alt = float(self.get_parameter("noise_gps_alt_std").value)

        # -------- State --------
        self.x = float(self.get_parameter("x0").value)
        self.y = float(self.get_parameter("y0").value)
        self.yaw = math.radians(float(self.get_parameter("yaw0").value))
        self.v = 0.0  # aktuelle Geschwindigkeit
        self.delta = 0.0  # aktueller Lenkwinkel

        self.v_cmd = 0.0
        self.delta_cmd = 0.0

        # -------- IO --------
        self.sub_v = self.create_subscription(
            Float32, "/target_speed", self.on_speed, 10
        )
        self.sub_d = self.create_subscription(
            Float32, "/steering_angle", self.on_steer, 10
        )
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_twist = self.create_publisher(
            TwistStamped, "/vehicle/ground_speed", 10
        )
        self.pub_gps = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.pub_marker = self.create_publisher(Marker, "/vehicle/marker", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Timing ---
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.step)
        self.last_time = self.get_clock().now()

        self.get_logger().info(
            "bicycle_sim ready: subs [/target_speed, /steering_angle] → pub [/odom, TF map→base_link]"
        )

        # --- Subscribers ---

    def on_speed(self, msg: Float32):
        self.v_cmd = max(-self.speed_lim, min(self.speed_lim, float(msg.data)))

    def on_steer(self, msg: Float32):
        self.delta_cmd = max(-self.steer_lim, min(self.steer_lim, float(msg.data)))

    # --- Main loop ---
    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.5:
            dt = self.dt  # falls erste Iteration oder Pause
        self.last_time = now

        # Einfache 1st-order Annäherung an Sollwerte (Aktuator-Trägheit)
        alpha_v = min(1.0, dt / max(self.tau_v, 1e-6))
        alpha_d = min(1.0, dt / max(self.tau_d, 1e-6))
        self.v += alpha_v * (self.v_cmd - self.v)
        self.delta += alpha_d * (self.delta_cmd - self.delta)

        # Bicycle-Kinematik
        yaw_rate = 0.0
        if abs(self.L) > 1e-6:
            yaw_rate = self.v / self.L * math.tan(self.delta)

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += yaw_rate * dt

        # Normiere yaw
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        # Publish Odom
        self.publish_odometry(now.to_msg(), self.v, yaw_rate)

        # Publish Ground Speed
        self.publish_ground_speed(now.to_msg(), self.v, yaw_rate)

        # Publish GPS
        self.publish_gps(now.to_msg())

        # Publish TF map -> base_link
        self.publish_tf(now.to_msg())

        # Publish Marker
        self.publish_marker(now.to_msg())

    def publish_odometry(self, stamp_msg, v_lin: float, yaw_rate: float):
        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = self.frame_map
        odom.child_frame_id = self.frame_base

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v_lin
        odom.twist.twist.angular.z = yaw_rate

        # # (Optionale) grobe Kovarianzen
        # odom.pose.covariance[0] = 0.05**2
        # odom.pose.covariance[7] = 0.05**2
        # odom.pose.covariance[35] = math.radians(1.0)**2

        self.pub_odom.publish(odom)

    def publish_tf(self, stamp_msg):
        t = TransformStamped()
        t.header.stamp = stamp_msg
        t.header.frame_id = self.frame_map
        t.child_frame_id = self.frame_base
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw * 0.5)
        t.transform.rotation.w = math.cos(self.yaw * 0.5)
        self.tf_broadcaster.sendTransform(t)

    def publish_ground_speed(self, stamp_msg, v_lin: float, yaw_rate: float):
        twist = TwistStamped()
        twist.header.stamp = stamp_msg
        twist.header.frame_id = self.frame_base
        # twist.child_frame_id = self.frame_base
        twist.twist.linear.x = v_lin
        twist.twist.angular.z = yaw_rate

        self.pub_twist.publish(twist)

    def publish_gps(self, stamp_msg):
        # ENU (map) -> LLA um lat0/lon0 herum (kleinräumige Approx.)
        dx = self.x + random.gauss(0.0, self.noise_gps_xy)
        dy = self.y + random.gauss(0.0, self.noise_gps_xy)
        dz = 0.0 + random.gauss(0.0, self.noise_gps_alt)

        dLat = dy / self.R_earth
        dLon = dx / (self.R_earth * math.cos(self.lat0))
        lat = self.lat0 + dLat
        lon = self.lon0 + dLon
        alt = self.alt0 + dz
        navsat = NavSatFix()
        navsat.header.stamp = stamp_msg
        navsat.header.frame_id = "gps"
        navsat.status.status = NavSatStatus.STATUS_FIX
        navsat.status.service = NavSatStatus.SERVICE_GPS
        navsat.latitude = math.degrees(lat)
        navsat.longitude = math.degrees(lon)
        navsat.altitude = alt

        pos_var_xy = self.noise_gps_xy**2
        pos_var_alt = self.noise_gps_alt**2
        navsat.position_covariance = [
            pos_var_xy,
            0.0,
            0.0,
            0.0,
            pos_var_xy,
            0.0,
            0.0,
            0.0,
            pos_var_alt,
        ]

        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub_gps.publish(navsat)

    def publish_marker(self, stamp_msg):
        marker = Marker()
        marker.header.stamp = stamp_msg
        marker.header.frame_id = self.frame_base  # also "base_link"
        marker.ns = "vehicle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Fahrzeug-Klotz (Länge x Breite x Höhe)
        marker.scale.x = float(self.get_parameter("marker_length").value)
        marker.scale.y = float(self.get_parameter("marker_width").value)
        marker.scale.z = float(self.get_parameter("marker_height").value)

        # Mittelpunkt mittig in base_link setzen
        marker.pose.position.x = marker.scale.x / 2.0  # vorne von base_link aus
        marker.pose.position.y = 0.0
        marker.pose.position.z = marker.scale.z / 2.0  # halb hoch
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Farbe
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.1
        marker.color.a = 0.8  # Alpha wichtig!

        self.pub_marker.publish(marker)


def main():
    rclpy.init()
    node = BicycleSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
