#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray


def color_rgba(r, g, b, a=1.0):
    from std_msgs.msg import ColorRGBA

    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


class AccelTrackMarkers(Node):
    def __init__(self):
        super().__init__("accel_track_markers")

        # --- Parameter mit sinnvollen Defaults
        self.declare_parameter("frame_id", "track")
        self.declare_parameter("spacing", 5.0)  # Abstand der Cones in Längsrichtung [m]
        self.declare_parameter("length", 90.0)  # Tracklänge [m]
        self.declare_parameter(
            "lane_half_width", 1.5
        )  # halber Spurabstand (links/rechts) [m]
        self.declare_parameter("radius", 0.15)  # "Fuß"-Durchmesser/2 (nur Optik)
        self.declare_parameter("height", 0.5)  # Cone-Höhe [m]
        self.declare_parameter("rate_hz", 2.0)  # Publish-Rate (ROS2 hat kein Latching)

        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.spacing = self.get_parameter("spacing").get_parameter_value().double_value
        self.length = self.get_parameter("length").get_parameter_value().double_value
        self.lane_half_width = (
            self.get_parameter("lane_half_width").get_parameter_value().double_value
        )
        self.radius = self.get_parameter("radius").get_parameter_value().double_value
        self.height = self.get_parameter("height").get_parameter_value().double_value
        self.rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(MarkerArray, "visualization_marker_array", qos)

        # Pre-build MarkerArray (IDs stabil halten)
        self.markers = self.build_markers()

        # Timer zum wiederholten Publizieren (ersetzt Latching)
        period = 1.0 / max(0.1, self.rate_hz)
        self.timer = self.create_timer(period, self.publish_once)
        self.get_logger().info(
            f"Accel track publishing at {self.rate_hz:.1f} Hz on /visualization_marker_array "
            f"(length={self.length} m, spacing={self.spacing} m, half_width={self.lane_half_width} m)"
        )

    def build_markers(self) -> MarkerArray:
        ma = MarkerArray()
        n = max(1, int(self.length / self.spacing) + 1)

        # Farben: gelb links (y<0), blau rechts (y>0)
        yellow = color_rgba(1.0, 1.0, 0.0, 1.0)
        blue = color_rgba(0.0, 0.0, 1.0, 1.0)

        # gemeinsame Header/Lifetime
        lifetime = Duration(
            sec=0, nanosec=0
        )  # 0 = bleibt bestehen, bis gelöscht/überschrieben

        def make_cone(mid: int, x: float, y: float, color, ns: str) -> Marker:
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = mid
            m.type = (
                Marker.CYLINDER
            )  # simple Geometrie; alternativ: MESH_RESOURCE mit Cone-Mesh
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = self.height * 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = self.radius * 2.0
            m.scale.y = self.radius * 2.0
            m.scale.z = self.height
            m.color = color
            m.lifetime = lifetime
            m.frame_locked = False
            return m

        # Links (gelb) bei -lane_half_width, rechts (blau) bei +lane_half_width
        for i in range(n):
            x = i * self.spacing
            ma.markers.append(
                make_cone(2 * i, x, -self.lane_half_width, yellow, "cones_left")
            )
            ma.markers.append(
                make_cone(2 * i + 1, x, self.lane_half_width, blue, "cones_right")
            )

        return ma

    def publish_once(self):
        # Zeitstempel aktualisieren (sonst meckert RViz je nach Fixed Frame)
        now = self.get_clock().now().to_msg()
        for m in self.markers.markers:
            m.header.stamp = now
        self.pub.publish(self.markers)


def main():
    rclpy.init()
    node = AccelTrackMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
