#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# OS1-32-like LiDAR simulator for cones provided via MarkerArray.
# - 32 rings, 360° sweep, columns_per_rev, spin_hz (10/20 Hz typical)
# - Per-beam time offset in 'time' field
# - Simple occlusion vs. cylindrical cones (nearest hit). Optional dual return.
# - Publishes sensor_msgs/PointCloud2 with fields: x,y,z,intensity,ring,time

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import tf2_ros
from std_msgs.msg import Header
import tf2_geometry_msgs

FLOAT32 = PointField.FLOAT32
UINT16  = PointField.UINT16
UINT32  = PointField.UINT32

class OS1LikeLidar(Node):
    def __init__(self):
        super().__init__('os1_like_lidar')

        # ---- Input / Output config
        self.declare_parameter('cones_topic', '/visualization_marker_array')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('accept_only_frame', '')   # e.g. 'map' or ''

        # ---- Scan & pattern (OS1-32-ish)
        self.declare_parameter('rings', 32)
        self.declare_parameter('vert_fov_deg', 42.4)      # ~ OS1-32 total vertical FoV
        self.declare_parameter('spin_hz', 10.0)           # 10 or 20 typical
        self.declare_parameter('columns_per_rev', 1024)   # 512..2048 typical
        self.declare_parameter('dual_return', False)      # if True, emit second hit when available

        # ---- Ranging limits / masks
        self.declare_parameter('min_range', 0.2)
        self.declare_parameter('max_range', 90.0)         # ~typical @10% target; set 200 for max reach
        self.declare_parameter('h_fov_deg', 360.0)        # keep 360 for OS1
        self.declare_parameter('min_z', -1.0)
        self.declare_parameter('max_z',  2.0)

        # ---- Noise/model
        self.declare_parameter('range_noise_std', 0.02)   # 2 cm
        self.declare_parameter('dropout_prob', 0.0)       # per-beam miss prob
        self.declare_parameter('intensity_base', 80.0)    # nominal
        self.declare_parameter('intensity_decay_per_m', 0.5)  # simple 1st-order decay with range

        # ---- Cone geometry (for occlusion)
        self.declare_parameter('cone_radius', 0.15)       # m (≈ Fußradius)
        self.declare_parameter('cone_height', 0.5)        # m (Marker uses height; top ~0.5)

        # Params -> members
        self.cones_topic   = self.get_parameter('cones_topic').value
        self.lidar_frame   = self.get_parameter('lidar_frame').value
        self.accept_only   = self.get_parameter('accept_only_frame').value

        self.R             = int(self.get_parameter('rings').value)
        self.vert_fov_rad  = math.radians(self.get_parameter('vert_fov_deg').value)
        self.spin_hz       = float(self.get_parameter('spin_hz').value)
        self.cols_per_rev  = int(self.get_parameter('columns_per_rev').value)
        self.dual_return   = bool(self.get_parameter('dual_return').value)

        self.min_range     = float(self.get_parameter('min_range').value)
        self.max_range     = float(self.get_parameter('max_range').value)
        self.h_fov_rad     = math.radians(self.get_parameter('h_fov_deg').value)
        self.min_z         = float(self.get_parameter('min_z').value)
        self.max_z         = float(self.get_parameter('max_z').value)

        self.range_noise   = float(self.get_parameter('range_noise_std').value)
        self.dropout_p     = float(self.get_parameter('dropout_prob').value)
        self.I0            = float(self.get_parameter('intensity_base').value)
        self.I_decay       = float(self.get_parameter('intensity_decay_per_m').value)

        self.cone_r        = float(self.get_parameter('cone_radius').value)
        self.cone_h        = float(self.get_parameter('cone_height').value)

        # Precompute vertical angles (OS1-32 has a known pattern; we approximate evenly spaced here)
        # Centered around 0: [-v/2, +v/2]
        if self.R < 1:
            self.R = 1
        if self.R == 1:
            self.vert_angles = [0.0]
        else:
            vmin = -self.vert_fov_rad * 0.5
            vmax =  self.vert_fov_rad * 0.5
            self.vert_angles = [vmin + i*(vmax-vmin)/(self.R-1) for i in range(self.R)]

        # Subscribe cones
        self._cones_src_frame = None
        self._cones_xyz = []  # list[(x,y,z), ...] in source frame
        self.create_subscription(MarkerArray, self.cones_topic, self._cb_cones, 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher
        self.pub_cloud = self.create_publisher(PointCloud2, 'lidar/os1/points', 10)

        # Rotation timing
        self.rev_period = 1.0 / max(0.1, self.spin_hz)
        self.col_dt = self.rev_period / self.cols_per_rev  # time per column
        self._col_idx = 0
        self._scan_points = []  # accumulate points of current revolution

        # Drive the scan with a high-rate timer stepping columns
        self.timer = self.create_timer(self.col_dt, self._on_timer)

        self.get_logger().info(
            f'OS1-like LiDAR: {self.R} rings, {self.cols_per_rev} cols @ {self.spin_hz} Hz → {self.cols_per_rev*self.R} pts/rev'
            f' — cones "{self.cones_topic}" → out "/lidar/os1/points"'
        )

    # ---------------- Cones input ----------------
    def _cb_cones(self, msg: MarkerArray):
        pts = []
        src_frame = msg.markers[0].header.frame_id if msg.markers else None
        if self.accept_only and src_frame and src_frame != self.accept_only:
            self.get_logger().warn_once(f'Ignoring cones in frame "{src_frame}" (accept_only="{self.accept_only}")')
            return
        for m in msg.markers:
            x, y, z = m.pose.position.x, m.pose.position.y, m.pose.position.z
            # Use the marker's frame if different (robust)
            # We assume all markers share same frame (common case)
            pts.append((x, y, z))
        self._cones_xyz = pts
        self._cones_src_frame = src_frame

    # ------------- Geometry helpers -------------
    def _transform_point(self, p_src, src_frame):
        ps = PointStamped()
        ps.header.frame_id = src_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.point.x, ps.point.y, ps.point.z = p_src
        try:
            tp = self.tf_buffer.transform(ps, self.lidar_frame, timeout=rclpy.duration.Duration(seconds=0.05))
            return (tp.point.x, tp.point.y, tp.point.z)
        except Exception:
            return None

    def _transform_cones_to_lidar(self):
        """Return list of (cx, cy, cz) cone centers in lidar frame."""
        if not self._cones_xyz or not self._cones_src_frame:
            return []
        try:
            _ = self.tf_buffer.lookup_transform(self.lidar_frame, self._cones_src_frame, Time())
        except Exception:
            return []
        out = []
        for p in self._cones_xyz:
            pl = self._transform_point(p, self._cones_src_frame)
            if pl is not None:
                out.append(pl)
        return out

    # Raycast cylinder (upright) intersection:
    # Cylinder axis along +z, radius r, height h, cylinder base centered at (cx,cy) at z=0 (assume cone base at z=0).
    # Ray from origin (0,0,0) in direction d=(dx,dy,dz), t>=0.
    # Returns smallest positive t where ray hits lateral surface and z(t) ∈ [0,h]; if none, returns None.
    def _raycast_cylinder(self, cx, cy, cz, r, h, dx, dy, dz):
        # Zylinderbasis in LiDAR-Koords:
        z_base = cz - 0.5*h

        # Ray-Ursprung in Zylinder-Lokalkoords (LiDAR bei 0,0,0)
        ox = -cx
        oy = -cy
        oz = -z_base

        # Seitenfläche: (ox+t*dx)^2 + (oy+t*dy)^2 = r^2
        a = dx*dx + dy*dy
        if a <= 1e-12:
            return None
        b = 2.0*(ox*dx + oy*dy)
        c = ox*ox + oy*oy - r*r
        disc = b*b - 4*a*c
        if disc < 0.0:
            return None
        sd = math.sqrt(disc)
        t1 = (-b - sd) / (2*a)
        t2 = (-b + sd) / (2*a)

        hits = []
        for t in (t1, t2):
            if t <= 0.0:
                continue
            zt = oz + t*dz
            if 0.0 <= zt <= h:
                hits.append(t)
        if not hits:
            return None
        return min(hits)

    # For dual return: second hit behind the first (if any)
    def _raycast_cylinder_second(self, cx, cy, r, h, dx, dy, dz, t_first):
        # Solve same quadratic but keep t > t_first + eps
        ox = -cx; oy = -cy
        a = dx*dx + dy*dy
        if a <= 1e-12:
            return None
        b = 2.0*(ox*dx + oy*dy)
        c = ox*ox + oy*oy - r*r
        disc = b*b - 4*a*c
        if disc < 0.0:
            return None
        sqrt_disc = math.sqrt(disc)
        t1 = (-b - sqrt_disc) / (2*a)
        t2 = (-b + sqrt_disc) / (2*a)
        cand = []
        for t in (t1, t2):
            if t > t_first + 1e-4:
                zt = t*dz
                if 0.0 <= zt <= h:
                    cand.append(t)
        if not cand:
            return None
        return min(cand)

    # ------------- Scan stepping -------------
    def _on_timer(self):
        # Need cones and TF ready
        cones_lidar = self._transform_cones_to_lidar()
        if not cones_lidar:
            self._col_idx = (self._col_idx + 1) % self.cols_per_rev
            return

        # Current column azimuth (0..2π)
        # Assume +x forward, +y left. Azimuth 0 along +x; increases CCW (towards +y)
        az = (self._col_idx / self.cols_per_rev) * 2.0 * math.pi
        # Limit by horizontal FoV if less than 360
        if self.h_fov_rad < 2.0*math.pi:
            # Shift FoV to center at +x; drop columns outside FoV
            if abs(((az + math.pi) % (2*math.pi)) - math.pi) > self.h_fov_rad * 0.5:
                # Outside FoV: skip but advance time
                self._advance_column_and_maybe_publish()
                return

        # For each ring, cast a ray
        for ring_idx, elev in enumerate(self.vert_angles):
            # Ray direction in lidar frame:
            # local spherical: yaw=az, pitch=elev
            ce = math.cos(elev); se = math.sin(elev)
            ca = math.cos(az);   sa = math.sin(az)
            dx, dy, dz = ce*ca, ce*sa, se

            # Occlusion-aware nearest hit:
            t_best = None
            cone_best = None
            for (cx, cy, cz) in cones_lidar:
                # Our cylinder model assumes base at z=0; markers’ z is around cone mid.
                # Shift by cz so base sits roughly on ground: assume ground at z=0 in lidar frame, cones centered at z=cone_h/2
                # If your markers already have z≈cone_h/2 in map, cz is ~cone_h/2 after TF; our raycast already assumes base at z=0
                # so we don't offset here—just rely on cylinder test with base at z=0. If your z-frame differs, adapt as needed.

                # Fast backface cull: if dot([cx,cy,?], [dx,dy]) < 0 and far behind, skip? Not necessary; quadratic handles it.
                t_hit = self._raycast_cylinder(cx, cy, cz, self.cone_r, self.cone_h, dx, dy, dz)
                if t_hit is None:
                    continue
                rng = t_hit  # since ray is unit-length (dx,dy,dz is normalized)
                # Range tests and z-window
                zt = rng*dz
                if rng < self.min_range or rng > self.max_range:
                    continue
                if zt < self.min_z or zt > self.max_z:
                    continue
                if t_best is None or rng < t_best:
                    t_best = rng
                    cone_best = (cx, cy, cz)

            if t_best is None:
                # dropout can also be simulated even if a cone is there; here we just skip when no hit
                continue

            if self.dropout_p > 0.0 and random.random() < self.dropout_p:
                continue

            # Add noise in range, recompute xyz along ray
            rng_noisy = max(self.min_range, t_best + random.gauss(0.0, self.range_noise))
            x = rng_noisy * dx
            y = rng_noisy * dy
            z = rng_noisy * dz

            # Intensity: simple distance decay; optionally color-aware later
            intensity = max(0.0, self.I0 - self.I_decay * rng_noisy)

            # Per-beam time offset within the revolution (column time + per-ring tiny skew)
            time_off = self._col_idx * self.col_dt + (ring_idx / self.R) * (self.col_dt * 0.2)

            # Append point (x,y,z,intensity,ring,time)
            self._scan_points.append((x, y, z, float(intensity), ring_idx, float(time_off)))

            # Dual return: look for a second intersection behind first (same ray, other cones)
            if self.dual_return:
                t_second = None
                for (cx2, cy2, cz2) in cones_lidar:
                    if cone_best and (cx2, cy2, cz2) == cone_best:
                        # same cylinder: second hit on backside possible; try dedicated second hit
                        t2 = self._raycast_cylinder_second(cx2, cy2, cz2, self.cone_r, self.cone_h, dx, dy, dz, t_best)
                        if t2 is not None:
                            rng2 = t2
                            zt2 = rng2*dz
                            if self.min_range <= rng2 <= self.max_range and self.min_z <= zt2 <= self.max_z:
                                if t_second is None or rng2 < t_second:
                                    t_second = rng2
                        continue
                    # other cones further out
                    t2 = self._raycast_cylinder(cx2, cy2, self.cone_r, self.cone_h, dx, dy, dz)
                    if t2 is None or t2 <= t_best + 1e-4:
                        continue
                    rng2 = t2
                    zt2 = rng2*dz
                    if self.min_range <= rng2 <= self.max_range and self.min_z <= zt2 <= self.max_z:
                        if t_second is None or rng2 < t_second:
                            t_second = rng2
                if t_second is not None:
                    rng2n = max(self.min_range, t_second + random.gauss(0.0, self.range_noise))
                    x2 = rng2n * dx
                    y2 = rng2n * dy
                    z2 = rng2n * dz
                    intensity2 = max(0.0, self.I0 - self.I_decay * rng2n) * 0.7  # weaker 2nd return
                    time_off2 = time_off + 1e-4
                    self._scan_points.append((x2, y2, z2, float(intensity2), ring_idx, float(time_off2)))

        # advance column; publish when a full revolution is done
        self._advance_column_and_maybe_publish()

    def _advance_column_and_maybe_publish(self):
        if self._col_idx == 0 and not self._scan_points:
            self.get_logger().warn_throttle(2000, 'Keine LiDAR-Treffer in letzter Umdrehung (prüfe TF/Z-Fenster)!')
        self._col_idx += 1
        if self._col_idx >= self.cols_per_rev:
            self._col_idx = 0
            if self._scan_points:
                self._publish_cloud(self._scan_points)
            self._scan_points = []

    def _publish_cloud(self, pts):
        # Build PointCloud2 with fields: x,y,z,float32; intensity float32; ring uint16; time float32
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.lidar_frame
        fields = [
            PointField(name='x', offset=0,  datatype=FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=FLOAT32, count=1),
            PointField(name='ring',      offset=16, datatype=UINT16,  count=1),
            PointField(name='time',      offset=20, datatype=FLOAT32, count=1),
        ]
        # pack into tuples matching fields
        cloud = point_cloud2.create_cloud(header, fields, pts)
        self.pub_cloud.publish(cloud)

def main():
    rclpy.init()
    node = OS1LikeLidar()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
