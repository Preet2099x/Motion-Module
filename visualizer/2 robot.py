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
import json
import os
from pathlib import Path
import time

# Theme definitions
class Theme:
    palettes = {
        "midnight": {
            "background": (11, 12, 14, 255),
            "grid_color": (255, 255, 255, 30),
            "button_bg": "#2c2f38",
            "button_hover": "#3e424d",
            "button_active": "#0078d7",
            "hud_bg": "rgba(11,12,14,160)",
            "hud_text": "#e8eaed"
        },
        "solarized": {
            "background": (30, 30, 40, 255),
            "grid_color": (200, 200, 200, 30),
            "button_bg": "#586e75",
            "button_hover": "#657b83",
            "button_active": "#268bd2",
            "hud_bg": "rgba(30,30,40,160)",
            "hud_text": "#93a1a1"
        }
    }
    @staticmethod
    def get(name):
        return Theme.palettes.get(name, Theme.palettes["midnight"])
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
# Load user configuration
def load_user_config():
    cfg_path = Path(__file__).with_name("config.json")
    default_cfg = {
        "theme": "midnight",
        "trail_maxlen": 4000,
        "trail_plot_points": 1500,
        "obstacle_point_size": 9,
        "msaa_samples": 8,
        "hud": {"show_fps": True, "show_latency": True, "show_packet_loss": False}
    }
    if cfg_path.is_file():
        try:
            with open(cfg_path, "r") as f:
                user_cfg = json.load(f)
                default_cfg.update(user_cfg)
        except Exception:
            pass
    return default_cfg

_user_cfg = load_user_config()
TRAIL_MAXLEN = _user_cfg.get("trail_maxlen", 4000)
TRAIL_PLOT_POINTS = _user_cfg.get("trail_plot_points", 1500)
OBSTACLE_POINT_SIZE = _user_cfg.get("obstacle_point_size", 9)
MSAA_SAMPLES = _user_cfg.get("msaa_samples", 8)
THEME = Theme.get(_user_cfg.get("theme", "midnight"))
HUD_SETTINGS = _user_cfg.get("hud", {})
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

CHASSIS_COLOR = (0.95, 0.95, 0.98, 1.0)    # bright white body
TRACK_COLOR = (0.5, 0.5, 0.5, 1.0)         # grey tracks
STRIPE_COLOR = (1.0, 0.42, 0.10, 1.0)      # ROSA orange accent
ARROW_LEN = CHASSIS_L * 0.9

# --- trailing "ghost" robot mesh -----------------------------------
# A second copy of the robot mesh, tinted blue, that follows a fixed
# distance behind the main robot along the recorded trail path (not a
# fixed number of samples or a fixed time -- it walks back along the
# trail until this much path length has been covered). Gives a visual
# sense of "a few values behind" the current pose. Hidden until the
# robot has actually travelled at least this far.
GHOST_DISTANCE_BEHIND = 0.8  # meters, arc-length behind the current position

GHOST_CHASSIS_COLOR = CHASSIS_COLOR
GHOST_TRACK_COLOR = TRACK_COLOR
GHOST_STRIPE_COLOR = STRIPE_COLOR

# Starting guess (degrees) for the mesh/heading offset, used until you
# calibrate with the "Set Forward"/"Set Backward" buttons in the UI (see
# MANUAL CALIBRATION below). Not auto-corrected in the background anymore
# -- a background auto-corrector can't tell "driving forward" and
# "driving backward" apart from position alone (a straight segment looks
# identical either direction), so it kept flip-flopping the offset as you
# drove back and forth. Calibration is now an explicit, one-shot action
# you trigger, so nothing changes unless you ask it to.
MESH_YAW_OFFSET_DEG = -90.0

# ---------------- MANUAL CALIBRATION ----------------
# Press "Set Forward" then drive straight forward for a moment (or "Set
# Backward" + drive straight backward, or "Set Left"/"Set Right" then
# turn that way in place) and it takes ONE clean sample and locks it in.
# Forward/Backward calibrate yaw_align_offset (where the mesh's "front"
# points). Left/Right calibrate rotation_sign (whether the mesh visually
# turns the same way the real robot turns) in case the yaw math ends up
# inverted for your mounting.
MOTION_ALIGN_MIN_DIST = 0.12        # meters the straight segment must cover before sampling
MOTION_ALIGN_MIN_SPEED = 0.05       # m/s, below = treat as stopped, reset the segment
MOTION_ALIGN_MAX_ANG_VEL = 0.3      # rad/s, above = treat as turning, reset the fwd/back segment
ROTATION_CALIB_MIN_YAW_DEG = 20.0   # degrees of turn required before sampling Left/Right

# ---------------- MOTION LOCK ----------------
# The chassis is non-holonomic (tracks/diff-drive) -- it physically CANNOT
# strafe sideways, only drive along its own forward/back axis or turn in
# place. So whenever odometry shows real net translation, the mesh's
# heading is forced to exactly match that direction of travel (flipped
# 180 if v_lin says it's reversing) instead of trusting the calibrated
# IMU yaw, which can be off by a few degrees and make the mesh look like
# it's drawing sideways/diagonal lines relative to the trail. The IMU-
# based heading (yaw_align_offset / rotation_sign) is only used as a
# fallback while stopped or spinning in place, where there's no reliable
# direction of travel to lock onto.
MOTION_LOCK_MIN_SPEED = 0.05        # m/s (smoothed), above = lock heading to travel direction
MOTION_VEC_SMOOTHING = 0.25         # EMA smoothing on the odometry velocity vector (0-1)

# Which axis of the IMU points FORWARD?
# If your IMU is mounted on a wall (vertically), the X axis might be pointing UP,
# which causes the turning math to lock up. Change this to [0, 1, 0] or [0, 0, 1]
# to use the axis that is actually horizontal!
IMU_FORWARD_AXIS = [1.0, 0.0, 0.0]

# Extra yaw offset (degrees) applied ONLY to the D455 obstacle scan, on
# top of the mesh's forward direction (yaw_rad, auto-aligned to actual
# direction of travel -- see MOTION_ALIGN_* above).
#
# The D455 is physically mounted facing FORWARD -- its angle-0 ("straight
# ahead" in the scan) points the SAME way the robot is actually driving.
# So the scan needs zero extra rotation on top of the mesh's forward
# direction: wherever the camera reports obstacles is, by definition,
# forward.
OBSTACLE_YAW_OFFSET_DEG = 0.0

# The mesh's on-screen rotation is locked to YAW ONLY (turning left/right).
# Full IMU roll/pitch is intentionally NOT applied to the visual mesh:
# IMUs commonly show apparent pitch swings during linear acceleration /
# braking (not real chassis tilt), which made the model look like it was
# nosing forward/backward instead of just driving straight. Position
# (forward/backward movement) always comes purely from the odometry
# trail, never from orientation, so this can't affect that mapping --
# only the model's spin direction. The orientation gizmo (colored X/Y/Z
# axes) still shows the full raw roll/pitch/yaw for diagnostics.
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
            latest["_recv_time"] = time.perf_counter()
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


def wrap_angle(a):
    """Wrap an angle in radians to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


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


def compute_ghost_pose(pts, target_dist):
    """Walk backward along the trail (list of (x,y,z)) from the newest
    point until `target_dist` of arc-length has been covered, and return
    ((gx, gy, gz), heading_rad) at that point -- heading_rad is the
    direction of travel through that point (None if that segment had no
    net movement). Returns None if the trail is too short / the robot
    hasn't travelled target_dist yet."""
    if len(pts) < 2:
        return None
    cum = 0.0
    idx = len(pts) - 1
    while idx > 0:
        x0, y0, z0 = pts[idx]
        x1, y1, z1 = pts[idx - 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        if cum + seg >= target_dist:
            remain = target_dist - cum
            t = remain / seg if seg > 1e-9 else 0.0
            gx = x0 + (x1 - x0) * t
            gy = y0 + (y1 - y0) * t
            gz = z0 + (z1 - z0) * t
            dx, dy = x0 - x1, y0 - y1
            heading = math.atan2(dy, dx) if (abs(dx) > 1e-9 or abs(dy) > 1e-9) else None
            return (gx, gy, gz), heading
        cum += seg
        idx -= 1
    return None


class Visualizer3D(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live IMU + Odometry Tracking (3D)")
        self.resize(1100, 850)

        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QtWidgets.QVBoxLayout(main_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        self.gl = gl.GLViewWidget()
        self.gl.setBackgroundColor(THEME["background"])
        self.gl.setCameraPosition(distance=6, elevation=28, azimuth=-55)
        main_layout.addWidget(self.gl, stretch=1)

        # Control panel
        control_panel = QtWidgets.QWidget()
        control_panel.setStyleSheet("background-color: #1a1c23; color: white;")
        control_layout = QtWidgets.QHBoxLayout(control_panel)
        control_layout.setContentsMargins(15, 10, 15, 10)
        
        btn_style = """
            QPushButton {
                background-color: #2c2f38; border: 1px solid #3e424d;
                border-radius: 4px; padding: 6px 12px;
                font-weight: bold; font-size: 14px; color: white;
            }
            QPushButton:hover { background-color: #3e424d; }
            QPushButton:pressed { background-color: #1e2026; }
            QPushButton:checked { background-color: #0078d7; border: 1px solid #005a9e; }
        """

        self.btn_zoom_in = QtWidgets.QPushButton("🔍 Zoom In")
        self.btn_zoom_out = QtWidgets.QPushButton("🔍 Zoom Out")
        self.btn_drive_mode = QtWidgets.QPushButton("🚘 Drive Mode")
        self.btn_drive_mode.setCheckable(True)

        self.btn_calib_fwd = QtWidgets.QPushButton("⬆ Set Forward")
        self.btn_calib_back = QtWidgets.QPushButton("⬇ Set Backward")
        self.btn_calib_left = QtWidgets.QPushButton("⬅ Set Left")
        self.btn_calib_right = QtWidgets.QPushButton("➡ Set Right")

        self.btn_zoom_in.setStyleSheet(btn_style)
        self.btn_zoom_out.setStyleSheet(btn_style)
        self.btn_drive_mode.setStyleSheet(btn_style)
        self.btn_calib_fwd.setStyleSheet(btn_style)
        self.btn_calib_back.setStyleSheet(btn_style)
        self.btn_calib_left.setStyleSheet(btn_style)
        self.btn_calib_right.setStyleSheet(btn_style)

        self.btn_zoom_in.clicked.connect(self.zoom_in)
        self.btn_zoom_out.clicked.connect(self.zoom_out)
        self.btn_calib_fwd.clicked.connect(lambda: self.start_calibration("forward"))
        self.btn_calib_back.clicked.connect(lambda: self.start_calibration("backward"))
        self.btn_calib_left.clicked.connect(lambda: self.start_calibration("left"))
        self.btn_calib_right.clicked.connect(lambda: self.start_calibration("right"))

        control_layout.addWidget(self.btn_zoom_in)
        control_layout.addWidget(self.btn_zoom_out)
        control_layout.addWidget(self.btn_drive_mode)
        control_layout.addSpacing(20)
        control_layout.addWidget(self.btn_calib_fwd)
        control_layout.addWidget(self.btn_calib_back)
        control_layout.addWidget(self.btn_calib_left)
        control_layout.addWidget(self.btn_calib_right)
        control_layout.addStretch()

        main_layout.addWidget(control_panel)

        # Ground grid (infinite effect)
        self.grid = gl.GLGridItem()
        self.grid.setSize(150, 150, 1)
        self.grid.setSpacing(0.5, 0.5, 1)
        self.grid.setColor(THEME["grid_color"])
        self.gl.addItem(self.grid)

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
            color=CHASSIS_COLOR,
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

        # --- ghost robot mesh: same geometry, blue tint, trails behind
        #     the main robot along the trail path (see GHOST_DISTANCE_BEHIND) ---
        self.ghost_chassis = gl.GLMeshItem(
            meshdata=box_mesh(CHASSIS_L, CHASSIS_W, CHASSIS_H), smooth=False,
            shader="shaded", color=GHOST_CHASSIS_COLOR,
        )
        self.ghost_track_L = gl.GLMeshItem(meshdata=track_L_md, smooth=True,
                                            shader="shaded", color=GHOST_TRACK_COLOR)
        self.ghost_track_R = gl.GLMeshItem(meshdata=track_R_md, smooth=True,
                                            shader="shaded", color=GHOST_TRACK_COLOR)
        self.ghost_stripe = gl.GLMeshItem(meshdata=stripe_md, smooth=False,
                                           shader="shaded", color=GHOST_STRIPE_COLOR)
        self.ghost_parts = [self.ghost_chassis, self.ghost_track_L,
                             self.ghost_track_R, self.ghost_stripe]
        for part in self.ghost_parts:
            part.setVisible(False)  # shown once the robot has moved far enough
            self.gl.addItem(part)
        self._ghost_last_heading = None

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
        self.yaw0 = 0.0  # reference yaw in world frame

        # Manual calibration state (see MANUAL CALIBRATION constants + the
        # "Set Forward/Backward/Left/Right" buttons). yaw_align_offset is
        # seeded from the guess until you calibrate; rotation_sign flips
        # the turn direction if the mesh spins the wrong way.
        self.yaw_align_offset = math.radians(MESH_YAW_OFFSET_DEG)
        self.rotation_sign = 1.0
        self.calib_mode = None          # None | "forward" | "backward" | "left" | "right"
        self.calib_anchor_xy = None     # start position of the fwd/back sample segment
        self.calib_anchor_yaw = None    # start imu_yaw_rel of the left/right sample turn
        self.calib_status_msg = ""      # last calibration result, shown in the HUD

        # Motion-lock state: smoothed odometry velocity vector, used to
        # force the mesh heading to exactly match the direction of travel
        # while translating (see MOTION LOCK constants + update_scene).
        self.motion_prev_xy = None
        self.motion_vx = 0.0
        self.motion_vy = 0.0

    def zoom_in(self):
        dist = self.gl.opts.get('distance', 6)
        self.gl.setCameraPosition(distance=dist * 0.8)

    def zoom_out(self):
        dist = self.gl.opts.get('distance', 6)
        self.gl.setCameraPosition(distance=dist * 1.25)

    def start_calibration(self, mode):
        """Arm a manual calibration sample. Drive/turn the requested way
        right after clicking -- the next clean segment locks it in."""
        self.calib_mode = mode
        self.calib_anchor_xy = None
        self.calib_anchor_yaw = None
        self.calib_status_msg = f"CALIBRATING {mode}: drive/turn {mode} now..."

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
        
        # Calculate yaw robustly in the WORLD frame to prevent gimbal lock
        # if the IMU is mounted pitched or vertically.
        v_world = R_raw @ np.array(IMU_FORWARD_AXIS, dtype=np.float32)
        yaw_world = math.atan2(v_world[1], v_world[0])
        
        if self.R0T is None:
            self.R0T = R_raw.T  # first pose seen becomes "level"
            self.yaw0 = yaw_world
            
        R = self.R0T @ R_raw    # full rotation (roll+pitch+yaw) relative to start -- used for the gizmo only

        imu_yaw_rel = yaw_world - self.yaw0

        # --- manual calibration (armed by the Set Forward/Backward/Left/Right
        #     buttons) -- takes exactly ONE clean sample, then disarms. ---
        if self.calib_mode in ("forward", "backward"):
            if abs(v_lin) <= MOTION_ALIGN_MIN_SPEED or abs(v_ang) >= MOTION_ALIGN_MAX_ANG_VEL:
                self.calib_anchor_xy = None  # stopped or turning -- wait for clean straight motion
            elif self.calib_anchor_xy is None:
                self.calib_anchor_xy = (p[0], p[1])  # start timing the sample segment
            else:
                dx = p[0] - self.calib_anchor_xy[0]
                dy = p[1] - self.calib_anchor_xy[1]
                if math.hypot(dx, dy) > MOTION_ALIGN_MIN_DIST:
                    motion_heading = math.atan2(dy, dx)
                    if self.calib_mode == "backward":
                        # moving backward means the chassis front points the
                        # OPPOSITE way from the direction of travel
                        motion_heading = wrap_angle(motion_heading + math.pi)
                    self.yaw_align_offset = wrap_angle(motion_heading - self.rotation_sign * imu_yaw_rel)
                    self.calib_status_msg = f"'{self.calib_mode}' calibrated!"
                    self.calib_mode = None
                    self.calib_anchor_xy = None
        elif self.calib_mode in ("left", "right"):
            if self.calib_anchor_yaw is None:
                self.calib_anchor_yaw = imu_yaw_rel  # start timing the turn
            else:
                d_yaw = wrap_angle(imu_yaw_rel - self.calib_anchor_yaw)
                if abs(d_yaw) > math.radians(ROTATION_CALIB_MIN_YAW_DEG):
                    turned_left_physically = (self.calib_mode == "left")
                    turned_left_visually = d_yaw > 0  # matches the CCW-positive rotation used below
                    self.rotation_sign = 1.0 if (turned_left_physically == turned_left_visually) else -1.0
                    self.calib_status_msg = f"'{self.calib_mode}' calibrated!"
                    self.calib_mode = None
                    self.calib_anchor_yaw = None

        # Yaw-only rotation for the VISUAL robot mesh + heading arrow: this
        # locks the model so it only ever turns left/right (about vertical),
        # never nose-dips/rears up from noisy/acceleration-coupled IMU
        # pitch. Position/forward-backward movement is driven purely by the
        # odometry trail (origin, below), completely independent of this.
        calibrated_yaw_rad = self.rotation_sign * imu_yaw_rel + self.yaw_align_offset

        # --- motion lock: while actually translating, force the heading to
        #     exactly match the direction of travel (the chassis physically
        #     can't move any other way), instead of the calibrated IMU yaw
        #     which can be off by a few degrees and draw sideways-looking
        #     lines relative to the trail. Falls back to the calibrated yaw
        #     when stopped or turning in place (no reliable travel direction). ---
        if self.motion_prev_xy is not None:
            dt = UPDATE_MS / 1000.0
            raw_vx = (p[0] - self.motion_prev_xy[0]) / dt
            raw_vy = (p[1] - self.motion_prev_xy[1]) / dt
            self.motion_vx += MOTION_VEC_SMOOTHING * (raw_vx - self.motion_vx)
            self.motion_vy += MOTION_VEC_SMOOTHING * (raw_vy - self.motion_vy)
        self.motion_prev_xy = (p[0], p[1])

        speed = math.hypot(self.motion_vx, self.motion_vy)
        if speed > MOTION_LOCK_MIN_SPEED:
            motion_heading = math.atan2(self.motion_vy, self.motion_vx)
            if v_lin < 0:
                motion_heading = wrap_angle(motion_heading + math.pi)
            yaw_rad = motion_heading
        else:
            yaw_rad = calibrated_yaw_rad
        cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
        R_yaw = np.array([
            [cos_y, -sin_y, 0.0],
            [sin_y,  cos_y, 0.0],
            [0.0,    0.0,   1.0],
        ], dtype=np.float32)

        origin = np.array(p, dtype=np.float32)

        transform = pg.Transform3D()
        transform.translate(p[0], p[1], p[2])
        transform.rotate(math.degrees(yaw_rad), 0, 0, 1)
        for part in self.robot_parts:
            part.setTransform(transform)

        # --- ghost robot: trails GHOST_DISTANCE_BEHIND meters behind the
        #     main robot along the recorded trail path ---
        ghost_result = compute_ghost_pose(pts, GHOST_DISTANCE_BEHIND)
        if ghost_result is not None:
            (gx, gy, gz), ghost_heading = ghost_result
            if ghost_heading is not None:
                self._ghost_last_heading = ghost_heading
            show_heading = self._ghost_last_heading if self._ghost_last_heading is not None else yaw_rad
            ghost_transform = pg.Transform3D()
            ghost_transform.translate(gx, gy, gz)
            ghost_transform.rotate(math.degrees(show_heading), 0, 0, 1)
            for part in self.ghost_parts:
                part.setTransform(ghost_transform)
                part.setVisible(True)
        else:
            for part in self.ghost_parts:
                part.setVisible(False)

        # Gizmo still shows the FULL raw orientation (roll/pitch/yaw) for
        # diagnostics, so you can see what the IMU is actually reporting.
        x_axis = origin + AXIS_LEN * (R @ np.array([1, 0, 0]))
        y_axis = origin + AXIS_LEN * (R @ np.array([0, 1, 0]))
        z_axis = origin + AXIS_LEN * (R @ np.array([0, 0, 1]))
        self.axis_x.setData(pos=np.array([origin, x_axis]))
        self.axis_y.setData(pos=np.array([origin, y_axis]))
        self.axis_z.setData(pos=np.array([origin, z_axis]))

        forward_tip = origin + ARROW_LEN * (R_yaw @ np.array([1, 0, 0]))
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
            yaw_rad_obs = yaw_rad + math.radians(OBSTACLE_YAW_OFFSET_DEG)
            cos_y, sin_y = math.cos(yaw_rad_obs), math.sin(yaw_rad_obs)
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

        # --- camera auto-recenter or drive mode ---
        now = QtCore.QDateTime.currentMSecsSinceEpoch()
        if self.btn_drive_mode.isChecked():
            # Drive mode: lock camera center to robot position, and rotate to look forward
            self.gl.setCameraPosition(
                pos=pg.Vector(p[0], p[1], p[2]),
                azimuth=math.degrees(yaw_rad) + 180
            )
        else:
            if now - self._last_recenter > RECENTER_EVERY_MS:
                self._last_recenter = now
                all_pts = np.array(pts, dtype=np.float32)
                mins = all_pts.min(axis=0)
                maxs = all_pts.max(axis=0)
                center = (mins + maxs) / 2
                self.gl.opts['center'] = pg.Vector(center[0], center[1], center[2])

        # --- Infinite grid effect ---
        cam_center = self.gl.opts.get('center')
        if cam_center is not None:
            spacing = 0.5
            snap_x = round(cam_center[0] / spacing) * spacing
            snap_y = round(cam_center[1] / spacing) * spacing
            self.grid.resetTransform()
            self.grid.translate(snap_x, snap_y, 0)

        # --- HUD --- (roll/pitch/yaw shown relative to the zeroed start pose)
        roll, pitch, yaw = rotmat_to_euler_deg(R)
        fps = self.compute_fps()
        obstacle_line = (f"nearest obstacle: {nearest_obstacle:.2f} m"
                          if nearest_obstacle is not None else "nearest obstacle: --")
        hud_parts = []
        hud_parts.append(f"packets: {n_pkts}   trail: {len(trail)} pts")
        if HUD_SETTINGS.get("show_fps", True):
            hud_parts.append(f"fps: {fps:4.0f}")
        hud_parts.append(f"pos x={p[0]:+.3f} y={p[1]:+.3f} z={p[2]:+.3f}")
        hud_parts.append(f"euler roll={roll:+6.1f} pitch={pitch:+6.1f} yaw={yaw:+6.1f} deg")
        hud_parts.append(f"v_lin={v_lin:+.2f} m/s v_ang={v_ang:+.2f} rad/s")
        if HUD_SETTINGS.get("show_latency", True):
            latency_ms = (time.perf_counter() - latest.get("_recv_time", 0)) * 1000
            hud_parts.append(f"latency: {latency_ms:.1f} ms")
        hud_parts.append(obstacle_line)
        if self.calib_status_msg:
            hud_parts.append(self.calib_status_msg)
        self.hud.setText("   ".join(hud_parts))
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
    fmt.setSamples(MSAA_SAMPLES)
    QtGui.QSurfaceFormat.setDefaultFormat(fmt)

    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    win = Visualizer3D()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()