#!/usr/bin/env python3
"""
laptop_visualizer_3d.py
Runs on your laptop. Listens for the SAME UDP JSON packets as
laptop_visualizer.py (sent by jetson_bridge.py) and renders a real-time,
mouse-navigable 3D scene using pyqtgraph's OpenGL view:

  - a fading trail of recent positions (dim = old, bright = recent)
  - a ground-projected "shadow" of the trail (depth cue)
  - a marker at the current position
  - a full 3D orientation gizmo (robot-local X/Y/Z axes) driven by the
    IMU quaternion -- this uses full roll/pitch/yaw, not just yaw like
    the 2D top-down version
  - a heading arrow (the local +X / forward axis, extended)
  - a ground grid + world-origin axes for spatial reference
  - an on-screen HUD: position, roll/pitch/yaw, speed, packet/fps stats

NOTE ON Z: your current jetson_bridge.py sends pos = [odom_x, odom_y]
(no Z), since position comes from 2D wheel odometry. This script is
tolerant of that -- it treats missing Z as 0 -- so the scene will look
flat until you add a Z source (barometer, lidar height, EKF estimate,
etc). Orientation is already full 3D today because it comes straight
from the fused IMU quaternion.

NOTE ON "FORWARD": the heading arrow / gizmo assumes the robot's local
+X axis is "forward" in the Teensy/BNO055 mounting frame. If the arrow
points sideways or backwards relative to the real robot, swap which
unit vector is rotated for `forward` below (e.g. [0,1,0] or [-1,0,0])
to match your sensor's mounting orientation.

Install:  pip install pyqtgraph PyQt5 PyOpenGL numpy
Run:      python3 laptop_visualizer_3d.py
"""

import sys
import math
import json
import socket
import threading
from collections import deque

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl

# ---------------- CONFIG ----------------
UDP_IP = "0.0.0.0"        # listen on all interfaces
UDP_PORT = 5005           # must match LAPTOP_PORT in jetson_bridge.py

TRAIL_MAXLEN = 4000
TRAIL_PLOT_POINTS = 1500   # max points drawn per frame (perf)
PLOT_PADDING = 1.6
RECENTER_EVERY_MS = 600    # how often the camera re-centers on the trail

AXIS_LEN = 0.18            # meters, orientation gizmo arm length
ARROW_LEN = 0.32           # meters, heading-arrow length
UPDATE_MS = 20             # ~50 fps redraw

TRAIL_COLOR = (0.23, 0.63, 1.0)       # blue
MARKER_COLOR = (1.0, 0.30, 0.30, 1.0)  # red
FWD_COLOR = (0.36, 1.0, 0.56, 1.0)     # green -- forward/heading
AXIS_X_COLOR = (1.0, 0.25, 0.25, 1.0)
AXIS_Y_COLOR = (0.25, 1.0, 0.35, 1.0)
AXIS_Z_COLOR = (0.30, 0.55, 1.0, 1.0)
# -----------------------------------------

latest = {"quat": [1.0, 0.0, 0.0, 0.0], "pos": [0.0, 0.0, 0.0],
          "v_lin": 0.0, "v_ang": 0.0, "n_pkts": 0}
trail = deque(maxlen=TRAIL_MAXLEN)
lock = threading.Lock()


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

        # Current position marker
        self.marker_item = gl.GLScatterPlotItem(size=14, color=MARKER_COLOR, pxMode=True)
        self.gl.addItem(self.marker_item)

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

        # --- current position marker ---
        pos_arr = np.array([p], dtype=np.float32)
        self.marker_item.setData(pos=pos_arr)

        # --- orientation gizmo from full quaternion (roll+pitch+yaw) ---
        R = quat_to_rotmat(q)
        origin = np.array(p, dtype=np.float32)
        x_axis = origin + AXIS_LEN * (R @ np.array([1, 0, 0]))
        y_axis = origin + AXIS_LEN * (R @ np.array([0, 1, 0]))
        z_axis = origin + AXIS_LEN * (R @ np.array([0, 0, 1]))
        self.axis_x.setData(pos=np.array([origin, x_axis]))
        self.axis_y.setData(pos=np.array([origin, y_axis]))
        self.axis_z.setData(pos=np.array([origin, z_axis]))

        forward_tip = origin + ARROW_LEN * (R @ np.array([1, 0, 0]))
        self.heading_item.setData(pos=np.array([origin, forward_tip]))
        self.heading_tip.setData(pos=np.array([forward_tip], dtype=np.float32))

        # --- camera auto-recenter (gentle, not every frame) ---
        now = QtCore.QDateTime.currentMSecsSinceEpoch()
        if now - self._last_recenter > RECENTER_EVERY_MS:
            self._last_recenter = now
            all_pts = np.array(pts, dtype=np.float32)
            mins = all_pts.min(axis=0)
            maxs = all_pts.max(axis=0)
            center = (mins + maxs) / 2
            self.gl.opts['center'] = pg.Vector(center[0], center[1], center[2])

        # --- HUD ---
        roll, pitch, yaw = quat_to_euler_deg(q)
        fps = self.compute_fps()
        self.hud.setText(
            f"packets: {n_pkts}   trail: {len(trail)} pts   fps: {fps:4.0f}\n"
            f"pos    x={p[0]:+.3f}  y={p[1]:+.3f}  z={p[2]:+.3f}  m\n"
            f"euler  roll={roll:+6.1f}  pitch={pitch:+6.1f}  yaw={yaw:+6.1f} deg\n"
            f"v_lin={v_lin:+.2f} m/s   v_ang={v_ang:+.2f} rad/s"
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