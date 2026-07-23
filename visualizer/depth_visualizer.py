#!/usr/bin/env python3
"""
laptop_visualizer_3d.py
Runs on your laptop. Listens for the SAME UDP JSON packets as
laptop_visualizer.py (sent by jetson_bridge.py) and renders a real-time,
mouse-navigable 3D scene using pyqtgraph's OpenGL view:

  - a fading trail of recent positions (dim = old, bright = recent)
  - a ground-projected "shadow" of the trail (depth cue)
  - a low-profile tracked-chassis robot mesh at the current position/pose
    (body + two tracks + a center stripe), instead of a plain marker dot
  - a full 3D orientation gizmo (robot-local X/Y/Z axes) driven by the
    IMU quaternion -- this uses full roll/pitch/yaw, not just yaw like
    the 2D top-down version
  - a heading arrow (the local +X / forward axis, extended)
  - a ground grid + world-origin axes for spatial reference
  - a live 2D obstacle scan ring from an Intel RealSense D455 (via the
    companion depth_obstacle_bridge.py on the Jetson), color-coded
    near (red) to far (green), flattened to the ground plane
  - an on-screen HUD: position, roll/pitch/yaw, speed, packet/fps stats,
    nearest obstacle distance

OBSTACLES: this listens on a SECOND UDP port (5006 by default) for
compact 2D scan-ring packets from depth_obstacle_bridge.py, which runs
separately on the Jetson alongside jetson_bridge.py. If that bridge
isn't running, the scene just shows no obstacle points -- everything
else still works. The scan is rotated into world space using yaw only
(not the full IMU tilt), so it stays flat/level like a real 2D lidar
scan regardless of chassis pitch or roll.

NOTE ON Z: your current jetson_bridge.py sends pos = [odom_x, odom_y]
(no Z), since position comes from 2D wheel odometry. This script is
tolerant of that -- it treats missing Z as 0 -- so the scene will look
flat until you add a Z source (barometer, lidar height, EKF estimate,
etc). Orientation is already full 3D today because it comes straight
from the fused IMU quaternion.

NOTE ON STARTING ORIENTATION: whatever quaternion arrives first is
treated as "level" -- the robot mesh always starts flat on the ground
with wheels down, regardless of the IMU's absolute mounting/calibration
offset at boot. Everything shown afterward (mesh tilt, gizmo, HUD
roll/pitch/yaw) is relative to that starting pose. Press 'R' in the
window at any time to re-zero using whatever the current pose is.

NOTE ON "FORWARD": the robot mesh, heading arrow, and gizmo all assume
the robot's local +X axis is "forward" in the Teensy/BNO055 mounting
frame. If the model or arrow points sideways/backwards relative to the
real robot, swap which unit vector is used for `forward` (e.g. [0,1,0]
or [-1,0,0]) to match your sensor's mounting orientation.

NOTE ON ROBOT DIMENSIONS: CHASSIS_L / CHASSIS_W / CHASSIS_H / TRACK_*
below are rough guesses based on your photos. Tweak them (in meters) to
match your real robot for a more accurate scale relative to the trail.

Install:  pip install pyqtgraph PyOpenGL numpy
          -- plus ONE Qt binding: PyQt5, PySide6, or PyQt6
          e.g.  pip install PySide6
Run:      python3 laptop_visualizer_3d.py
"""

import sys
import math
import json
import socket
import threading
from collections import deque

import numpy as np

# Go through pyqtgraph's Qt compatibility layer instead of importing a
# specific binding directly. This auto-detects whichever of PyQt5 /
# PySide2 / PySide6 / PyQt6 is actually installed, so you don't have to
# fight over exactly which one pip installed correctly.
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import pyqtgraph.opengl as gl

# ---------------- CONFIG ----------------
UDP_IP = "0.0.0.0"        # listen on all interfaces
UDP_PORT = 5005           # must match LAPTOP_PORT in jetson_bridge.py
OBSTACLE_UDP_PORT = 5006  # must match LAPTOP_PORT in depth_obstacle_bridge.py
MAX_OBSTACLE_RANGE = 6.0  # meters, just for the near->far color scale (should roughly match MAX_RANGE_M on the Jetson side)
OBSTACLE_POINT_SIZE = 9

TRAIL_MAXLEN = 4000
TRAIL_PLOT_POINTS = 1500   # max points drawn per frame (perf)
RECENTER_EVERY_MS = 600    # how often the camera re-centers on the trail

AXIS_LEN = 0.18            # meters, orientation gizmo arm length
UPDATE_MS = 20              # ~50 fps redraw

TRAIL_COLOR = (0.23, 0.63, 1.0)       # blue
FWD_COLOR = (0.36, 1.0, 0.56, 1.0)     # green -- forward/heading
AXIS_X_COLOR = (1.0, 0.25, 0.25, 1.0)
AXIS_Y_COLOR = (0.25, 1.0, 0.35, 1.0)
AXIS_Z_COLOR = (0.30, 0.55, 1.0, 1.0)

# Robot chassis dimensions in meters (rough guess from photos -- tune
# these to your real robot for correct scale relative to the trail).
CHASSIS_L = 0.30   # front-to-back length (local X)
CHASSIS_W = 0.17   # left-to-right width, body only, not tracks (local Y)
CHASSIS_H = 0.075  # height (local Z)
TRACK_RADIUS = 0.05
TRACK_GAP = 0.01           # gap between chassis body edge and track
STRIPE_W = 0.025
STRIPE_H = 0.006

CHASSIS_COLOR = (0.10, 0.10, 0.11, 1.0)   # dark body
TRACK_COLOR = (0.03, 0.03, 0.03, 1.0)      # near-black tracks
STRIPE_COLOR = (1.0, 0.42, 0.10, 1.0)      # ROSA orange accent
ARROW_LEN = CHASSIS_L * 0.9
# -----------------------------------------

latest = {"quat": [1.0, 0.0, 0.0, 0.0], "pos": [0.0, 0.0, 0.0],
          "v_lin": 0.0, "v_ang": 0.0, "n_pkts": 0}
trail = deque(maxlen=TRAIL_MAXLEN)
lock = threading.Lock()

obstacle_latest = {"angles_deg": [], "dist_m": [], "cam_forward_offset": 0.0,
                    "cam_height": 0.0, "t": 0.0}
obstacle_lock = threading.Lock()


def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening for IMU packets on UDP {UDP_PORT}...")
    while True:
        data, _ = sock.recvfrom(4096)
        try:
            pkt = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError:
            continue
        pos = pkt.get("pos", [0.0, 0.0])
        z = pos[2] if len(pos) > 2 else 0.0
        with lock:
            latest["quat"] = pkt.get("quat", [1.0, 0.0, 0.0, 0.0])
            latest["pos"] = [pos[0], pos[1], z]
            latest["v_lin"] = pkt.get("v_lin", 0.0)
            latest["v_ang"] = pkt.get("v_ang", 0.0)
            latest["n_pkts"] += 1
            trail.append((pos[0], pos[1], z))


def obstacle_listener():
    """Receives the compact 2D obstacle scan ring from depth_obstacle_bridge.py
    (D455 on the Jetson). Runs completely independently of the IMU/odometry
    listener above -- if this bridge isn't running, the scene simply shows
    no obstacle points."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, OBSTACLE_UDP_PORT))
    print(f"Listening for obstacle scans on UDP {OBSTACLE_UDP_PORT}...")
    while True:
        data, _ = sock.recvfrom(65536)
        try:
            pkt = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError:
            continue
        with obstacle_lock:
            obstacle_latest["angles_deg"] = pkt.get("angles_deg", [])
            obstacle_latest["dist_m"] = pkt.get("dist_m", [])
            obstacle_latest["cam_forward_offset"] = pkt.get("cam_forward_offset", 0.0)
            obstacle_latest["cam_height"] = pkt.get("cam_height", 0.0)
            obstacle_latest["t"] = pkt.get("t", 0.0)


def quat_to_rotmat(q):
    """3x3 rotation matrix from quaternion [w, x, y, z]."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n == 0:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def quat_to_euler_deg(q):
    """Returns (roll, pitch, yaw) in degrees from a [w,x,y,z] quaternion."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n == 0:
        return 0.0, 0.0, 0.0
    w, x, y, z = w / n, x / n, y / n, z / n

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def rotmat_to_euler_deg(R):
    """Returns (roll, pitch, yaw) in degrees from a 3x3 rotation matrix."""
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def obstacle_color(dist, max_range):
    """Near = red/orange, far = green -- simple distance heatmap."""
    t = max(0.0, min(1.0, dist / max_range))
    r = 1.0 - 0.7 * t
    g = 0.15 + 0.75 * t
    b = 0.15
    return (r, g, b, 0.9)


# ---------------- robot mesh builders ----------------

def box_mesh(sx, sy, sz, cx=0.0, cy=0.0, cz=0.0):
    """Solid box centered at (cx,cy,cz) with full extents sx,sy,sz."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    v = np.array([
        [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
        [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
    ], dtype=np.float32) + np.array([cx, cy, cz], dtype=np.float32)
    faces = np.array([
        [0, 1, 2], [0, 2, 3],   # bottom
        [4, 6, 5], [4, 7, 6],   # top
        [0, 4, 5], [0, 5, 1],   # -y side
        [3, 2, 6], [3, 6, 7],   # +y side
        [0, 3, 7], [0, 7, 4],   # -x side
        [1, 5, 6], [1, 6, 2],   # +x side
    ])
    return gl.MeshData(vertexes=v, faces=faces)


def cylinder_mesh_x(radius, length, cy=0.0, cz=0.0, segments=14):
    """Solid cylinder whose axis runs along local X, centered at
    (0, cy, cz), i.e. one straight roller/track shape per side."""
    half = length / 2
    angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    ring_y = radius * np.cos(angles)
    ring_z = radius * np.sin(angles)

    ring0 = np.stack([np.full(segments, -half), ring_y, ring_z], axis=1)
    ring1 = np.stack([np.full(segments, half), ring_y, ring_z], axis=1)
    center0 = np.array([[-half, 0.0, 0.0]])
    center1 = np.array([[half, 0.0, 0.0]])

    verts = np.vstack([ring0, ring1, center0, center1]).astype(np.float32)
    verts += np.array([0.0, cy, cz], dtype=np.float32)

    n = segments
    c0_idx, c1_idx = 2 * n, 2 * n + 1
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j])
        faces.append([i, n + j, n + i])
        faces.append([c0_idx, j, i])
        faces.append([c1_idx, n + i, n + j])
    return gl.MeshData(vertexes=verts, faces=np.array(faces))


class Visualizer3D(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live IMU + Odometry Tracking (3D)")
        self.resize(1100, 850)

        self.gl = gl.GLViewWidget()
        self.gl.setBackgroundColor((11, 12, 14, 255))
        self.gl.setCameraPosition(distance=6, elevation=28, azimuth=-55)
        self.setCentralWidget(self.gl)

        # Ground grid
        grid = gl.GLGridItem()
        grid.setSize(20, 20, 1)
        grid.setSpacing(0.5, 0.5, 1)
        grid.setColor((255, 255, 255, 30))
        self.gl.addItem(grid)

        # World-origin reference axes (small, fixed)
        origin_axis = gl.GLAxisItem()
        origin_axis.setSize(0.4, 0.4, 0.4)
        self.gl.addItem(origin_axis)

        # Trail (fading brightness) + ground-projected shadow
        self.trail_item = gl.GLLinePlotItem(mode="line_strip", antialias=True, width=2.2)
        self.gl.addItem(self.trail_item)

        self.shadow_item = gl.GLLinePlotItem(mode="line_strip", antialias=True, width=1.0,
                                              color=(1, 1, 1, 0.12))
        self.gl.addItem(self.shadow_item)

        # --- robot mesh: chassis + two tracks + center stripe ---
        track_y_offset = CHASSIS_W / 2 + TRACK_RADIUS + TRACK_GAP
        track_z = -CHASSIS_H / 2  # tracks sit level with the chassis underside

        chassis_md = box_mesh(CHASSIS_L, CHASSIS_W, CHASSIS_H)
        self.robot_chassis = gl.GLMeshItem(
            meshdata=chassis_md, smooth=False, shader="shaded",
            color=CHASSIS_COLOR, drawEdges=True, edgeColor=(0, 0, 0, 0.6),
        )

        track_L_md = cylinder_mesh_x(TRACK_RADIUS, CHASSIS_L, cy=track_y_offset, cz=track_z)
        track_R_md = cylinder_mesh_x(TRACK_RADIUS, CHASSIS_L, cy=-track_y_offset, cz=track_z)
        self.robot_track_L = gl.GLMeshItem(meshdata=track_L_md, smooth=True,
                                            shader="shaded", color=TRACK_COLOR)
        self.robot_track_R = gl.GLMeshItem(meshdata=track_R_md, smooth=True,
                                            shader="shaded", color=TRACK_COLOR)

        stripe_md = box_mesh(CHASSIS_L * 0.85, STRIPE_W, STRIPE_H,
                              cz=CHASSIS_H / 2 + STRIPE_H / 2)
        self.robot_stripe = gl.GLMeshItem(meshdata=stripe_md, smooth=False,
                                           shader="shaded", color=STRIPE_COLOR)

        self.robot_parts = [self.robot_chassis, self.robot_track_L,
                             self.robot_track_R, self.robot_stripe]
        for part in self.robot_parts:
            self.gl.addItem(part)

        # Orientation gizmo: local X / Y / Z axes at the robot, from the quaternion
        self.axis_x = gl.GLLinePlotItem(color=AXIS_X_COLOR, width=3, antialias=True)
        self.axis_y = gl.GLLinePlotItem(color=AXIS_Y_COLOR, width=3, antialias=True)
        self.axis_z = gl.GLLinePlotItem(color=AXIS_Z_COLOR, width=3, antialias=True)
        for a in (self.axis_x, self.axis_y, self.axis_z):
            self.gl.addItem(a)

        # Heading arrow (extended local +X, i.e. forward)
        self.heading_item = gl.GLLinePlotItem(color=FWD_COLOR, width=3.5, antialias=True)
        self.gl.addItem(self.heading_item)
        self.heading_tip = gl.GLScatterPlotItem(size=8, color=FWD_COLOR, pxMode=True)
        self.gl.addItem(self.heading_tip)

        # Obstacle scan ring (from depth_obstacle_bridge.py / D455), flattened
        # to the ground plane, colored near->far. Empty until scans arrive.
        self.obstacle_item = gl.GLScatterPlotItem(size=OBSTACLE_POINT_SIZE, pxMode=True)
        self.gl.addItem(self.obstacle_item)

        # HUD overlay
        self.hud = QtWidgets.QLabel(self.gl)
        self.hud.setStyleSheet(
            "color: #e8eaed; background-color: rgba(11,12,14,160);"
            "padding: 8px 10px; border-radius: 6px; font-family: monospace; font-size: 12px;"
        )
        self.hud.move(10, 10)
        self.hud.setText("waiting for packets...")
        self.hud.adjustSize()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_scene)
        self.timer.start(UPDATE_MS)

        self._last_recenter = 0
        self._frame_times = deque(maxlen=30)
        self._last_t = QtCore.QElapsedTimer()
        self._last_t.start()

        # Whatever quaternion arrives first is treated as "level" -- the
        # robot always starts flat on the ground with wheels down,
        # regardless of the IMU's absolute mounting/calibration offset.
        # Press 'R' at any time to re-zero using the current pose.
        self.R0T = None  # transpose of the reference rotation matrix

    def keyPressEvent(self, event):
        key_r = getattr(QtCore.Qt, "Key_R", None)
        if key_r is None:
            key_r = getattr(QtCore.Qt.Key, "Key_R", None)
        if event.key() == key_r:
            self.R0T = None  # forces recalibration on the next frame
        else:
            super().keyPressEvent(event)

    def update_scene(self):
        with lock:
            q = list(latest["quat"])
            p = list(latest["pos"])
            v_lin = latest["v_lin"]
            v_ang = latest["v_ang"]
            n_pkts = latest["n_pkts"]
            pts = list(trail)

        if not pts:
            return

        # --- trail (downsampled, fading alpha newest = brightest) ---
        step = max(1, len(pts) // TRAIL_PLOT_POINTS)
        shown = pts[::step]
        arr = np.array(shown, dtype=np.float32)
        n = len(arr)
        alphas = np.linspace(0.15, 0.95, n)
        colors = np.zeros((n, 4), dtype=np.float32)
        colors[:, 0] = TRAIL_COLOR[0]
        colors[:, 1] = TRAIL_COLOR[1]
        colors[:, 2] = TRAIL_COLOR[2]
        colors[:, 3] = alphas
        self.trail_item.setData(pos=arr, color=colors)

        shadow = arr.copy()
        shadow[:, 2] = 0.0
        self.shadow_item.setData(pos=shadow)

        # --- pose: shared rotation + translation for the robot mesh,
        #     the orientation gizmo, and the heading arrow ---
        R_raw = quat_to_rotmat(q)
        if self.R0T is None:
            self.R0T = R_raw.T  # first pose seen becomes "level"
        R = self.R0T @ R_raw    # rotation relative to the starting pose

        origin = np.array(p, dtype=np.float32)

        M = np.eye(4, dtype=np.float32)
        M[:3, :3] = R
        M[:3, 3] = origin
        transform = pg.Transform3D(*M.flatten().tolist())
        for part in self.robot_parts:
            part.setTransform(transform)

        x_axis = origin + AXIS_LEN * (R @ np.array([1, 0, 0]))
        y_axis = origin + AXIS_LEN * (R @ np.array([0, 1, 0]))
        z_axis = origin + AXIS_LEN * (R @ np.array([0, 0, 1]))
        self.axis_x.setData(pos=np.array([origin, x_axis]))
        self.axis_y.setData(pos=np.array([origin, y_axis]))
        self.axis_z.setData(pos=np.array([origin, z_axis]))

        forward_tip = origin + ARROW_LEN * (R @ np.array([1, 0, 0]))
        self.heading_item.setData(pos=np.array([origin, forward_tip]))
        self.heading_tip.setData(pos=np.array([forward_tip], dtype=np.float32))

        # --- obstacle scan ring (D455 via depth_obstacle_bridge.py, if any
        #     packets have arrived) -- flattened to the ground plane and
        #     rotated by yaw only, so it stays level even if the chassis
        #     pitches/rolls ---
        with obstacle_lock:
            obs_angles = list(obstacle_latest["angles_deg"])
            obs_dists = list(obstacle_latest["dist_m"])
            cam_fwd = obstacle_latest["cam_forward_offset"]

        nearest_obstacle = None
        if obs_angles:
            yaw_rad = math.atan2(R[1, 0], R[0, 0])
            cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
            world_pts = []
            obs_colors = []
            for ang_deg, dist in zip(obs_angles, obs_dists):
                if dist is None:
                    continue
                if nearest_obstacle is None or dist < nearest_obstacle:
                    nearest_obstacle = dist
                ang = math.radians(ang_deg)
                lx = cam_fwd + dist * math.cos(ang)
                ly = -dist * math.sin(ang)
                wx = origin[0] + cos_y * lx - sin_y * ly
                wy = origin[1] + sin_y * lx + cos_y * ly
                world_pts.append((wx, wy, 0.0))
                obs_colors.append(obstacle_color(dist, MAX_OBSTACLE_RANGE))
            if world_pts:
                self.obstacle_item.setData(pos=np.array(world_pts, dtype=np.float32),
                                            color=np.array(obs_colors, dtype=np.float32))
            else:
                self.obstacle_item.setData(pos=np.zeros((0, 3), dtype=np.float32))

        # --- camera auto-recenter (gentle, not every frame) ---
        now = QtCore.QDateTime.currentMSecsSinceEpoch()
        if now - self._last_recenter > RECENTER_EVERY_MS:
            self._last_recenter = now
            all_pts = np.array(pts, dtype=np.float32)
            mins = all_pts.min(axis=0)
            maxs = all_pts.max(axis=0)
            center = (mins + maxs) / 2
            self.gl.opts['center'] = pg.Vector(center[0], center[1], center[2])

        # --- HUD --- (roll/pitch/yaw shown relative to the zeroed start pose)
        roll, pitch, yaw = rotmat_to_euler_deg(R)
        fps = self.compute_fps()
        obstacle_line = (f"nearest obstacle: {nearest_obstacle:.2f} m"
                          if nearest_obstacle is not None else "nearest obstacle: --")
        self.hud.setText(
            f"packets: {n_pkts}   trail: {len(trail)} pts   fps: {fps:4.0f}\n"
            f"pos    x={p[0]:+.3f}  y={p[1]:+.3f}  z={p[2]:+.3f}  m\n"
            f"euler  roll={roll:+6.1f}  pitch={pitch:+6.1f}  yaw={yaw:+6.1f} deg\n"
            f"v_lin={v_lin:+.2f} m/s   v_ang={v_ang:+.2f} rad/s\n"
            f"{obstacle_line}"
        )
        self.hud.adjustSize()

    def compute_fps(self):
        dt = self._last_t.restart()
        if dt <= 0:
            return 0.0
        self._frame_times.append(dt)
        avg = sum(self._frame_times) / len(self._frame_times)
        return 1000.0 / avg if avg else 0.0


def main():
    threading.Thread(target=udp_listener, daemon=True).start()
    threading.Thread(target=obstacle_listener, daemon=True).start()

    # Request multisampling for smoother lines/edges. Must be set before
    # the QApplication is created to take effect.
    fmt = QtGui.QSurfaceFormat()
    fmt.setSamples(4)
    QtGui.QSurfaceFormat.setDefaultFormat(fmt)

    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    win = Visualizer3D()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()