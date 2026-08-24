#!/usr/bin/env python3
"""
imu_graph_visualizer.py
Runs on your laptop. Listens for UDP JSON packets from imu_bridge.py
and plots live rolling time-series graphs of:
  - gyro (gx, gy, gz), deg/s
  - yaw, deg
  - linear acceleration (ax, ay, az), m/s^2

Also renders a top-down UGV mesh panel that spins live to match the
incoming yaw. NOTE: imu_bridge.py's packets carry orientation (gyro/yaw)
and accel only -- no XY position -- so the mesh rotates in place but
does not drive across the panel; there's no position stream in this
pipeline to move it with (see jetson_bridge.py / laptop_visualizer_3d.py
for the pose+trail version).

Install:  pip install matplotlib numpy
Run:      python3 imu_graph_visualizer.py
"""

import math
import socket
import json
import threading
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

UDP_IP = "0.0.0.0"       # listen on all interfaces
UDP_PORT = 5007          # must match LAPTOP_PORT in imu_bridge.py

WINDOW_SECONDS = 15.0     # how much history to show
MAX_SAMPLES = 3000        # ring buffer cap, regardless of time span

lock = threading.Lock()
t_buf = deque(maxlen=MAX_SAMPLES)
gyro_buf = deque(maxlen=MAX_SAMPLES)   # each item: (gx, gy, gz)
yaw_buf = deque(maxlen=MAX_SAMPLES)
accel_buf = deque(maxlen=MAX_SAMPLES)  # each item: (ax, ay, az)

t0 = None


def udp_listener():
    global t0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening for IMU packets on UDP {UDP_PORT}...")
    while True:
        data, _ = sock.recvfrom(4096)
        try:
            pkt = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError:
            continue

        with lock:
            if t0 is None:
                t0 = pkt["t"]
            t_buf.append(pkt["t"] - t0)
            gyro_buf.append(tuple(pkt["gyro"]))
            yaw_buf.append(pkt["yaw"])
            accel_buf.append(tuple(pkt["accel"]))


# ---- UGV mesh (top-down outline, forward = +Y) ----
UGV_BODY = np.array([
    [-0.28, -0.40],  # rear left
    [ 0.28, -0.40],  # rear right
    [ 0.28,  0.10],  # front-right shoulder
    [ 0.00,  0.48],  # nose tip
    [-0.28,  0.10],  # front-left shoulder
])


def rotate_points(pts, yaw_deg):
    """Rotate 2D points by yaw_deg (compass-style: +yaw turns right)."""
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, s], [-s, c]])
    return pts @ r.T


def main():
    threading.Thread(target=udp_listener, daemon=True).start()

    fig = plt.figure(figsize=(13, 9))
    fig.patch.set_facecolor("#0b0c0e")
    fig.suptitle("Live IMU data", color="#e8eaed", fontsize=13)

    gs = fig.add_gridspec(3, 2, width_ratios=[2.2, 1.0], wspace=0.28, hspace=0.35)
    ax_gyro = fig.add_subplot(gs[0, 0])
    ax_yaw = fig.add_subplot(gs[1, 0], sharex=ax_gyro)
    ax_accel = fig.add_subplot(gs[2, 0], sharex=ax_gyro)
    ax_robot = fig.add_subplot(gs[:, 1])

    # --- UGV mesh panel ---
    ax_robot.set_facecolor("#0b0c0e")
    ax_robot.set_title("UGV heading (top-down)", color="#e8eaed", fontsize=10)
    ax_robot.set_xlim(-0.9, 0.9)
    ax_robot.set_ylim(-0.9, 0.9)
    ax_robot.set_aspect("equal")
    ax_robot.set_xticks([])
    ax_robot.set_yticks([])
    for spine in ax_robot.spines.values():
        spine.set_color((1, 1, 1, 0.15))
    compass = plt.Circle((0, 0), 0.85, fill=False, color=(1, 1, 1, 0.12), lw=1.0)
    ax_robot.add_patch(compass)

    ugv_patch = Polygon(UGV_BODY, closed=True, facecolor="#3aa0ff",
                         edgecolor="#e8eaed", linewidth=1.2)
    ax_robot.add_patch(ugv_patch)

    (heading_line,) = ax_robot.plot([], [], color="#5cff8f", lw=2.0)
    robot_hud = ax_robot.text(0, -0.98, "", ha="center", va="top",
                               color="#9aa0a6", fontsize=8.5, family="monospace")

    gyro_colors = ("#ff4d4d", "#5cff8f", "#3aa0ff")
    accel_colors = ("#ff4d4d", "#5cff8f", "#3aa0ff")

    lines_gyro = []
    for label, color in zip(("gx", "gy", "gz"), gyro_colors):
        (line,) = ax_gyro.plot([], [], color=color, lw=1.4, label=label)
        lines_gyro.append(line)
    ax_gyro.set_ylabel("gyro (deg/s)", color="#9aa0a6")
    ax_gyro.legend(loc="upper left", facecolor="#0b0c0e", labelcolor="#e8eaed", fontsize=8)

    (line_yaw,) = ax_yaw.plot([], [], color="#ffb84d", lw=1.6, label="yaw")
    ax_yaw.set_ylabel("yaw (deg)", color="#9aa0a6")
    ax_yaw.legend(loc="upper left", facecolor="#0b0c0e", labelcolor="#e8eaed", fontsize=8)

    lines_accel = []
    for label, color in zip(("ax", "ay", "az"), accel_colors):
        (line,) = ax_accel.plot([], [], color=color, lw=1.4, label=label)
        lines_accel.append(line)
    ax_accel.set_ylabel("accel (m/s^2)", color="#9aa0a6")
    ax_accel.set_xlabel("time (s)", color="#9aa0a6")
    ax_accel.legend(loc="upper left", facecolor="#0b0c0e", labelcolor="#e8eaed", fontsize=8)

    for ax in (ax_gyro, ax_yaw, ax_accel):
        ax.set_facecolor("#0b0c0e")
        ax.grid(True, color=(1, 1, 1, 0.08))
        for spine in ax.spines.values():
            spine.set_color((1, 1, 1, 0.15))
        ax.tick_params(colors="#6b7075", labelsize=8)

    while plt.fignum_exists(fig.number):
        with lock:
            ts = list(t_buf)
            gyro = list(gyro_buf)
            yaw = list(yaw_buf)
            accel = list(accel_buf)

        if ts:
            t_max = ts[-1]
            t_min = max(0.0, t_max - WINDOW_SECONDS)
            # trim to the visible window for plotting
            start = 0
            for i, t in enumerate(ts):
                if t >= t_min:
                    start = i
                    break
            ts_v = ts[start:]
            gyro_v = gyro[start:]
            yaw_v = yaw[start:]
            accel_v = accel[start:]

            for i, line in enumerate(lines_gyro):
                line.set_data(ts_v, [g[i] for g in gyro_v])
            line_yaw.set_data(ts_v, yaw_v)
            for i, line in enumerate(lines_accel):
                line.set_data(ts_v, [a[i] for a in accel_v])

            ax_gyro.set_xlim(t_min, max(t_max, t_min + 1.0))
            for ax, series in (
                (ax_gyro, [v for g in gyro_v for v in g]),
                (ax_yaw, yaw_v),
                (ax_accel, [v for a in accel_v for v in a]),
            ):
                if series:
                    lo, hi = min(series), max(series)
                    pad = max((hi - lo) * 0.15, 1.0)
                    ax.set_ylim(lo - pad, hi + pad)

            # --- UGV mesh: spins live to match the current yaw ---
            current_yaw = yaw_v[-1]
            current_gyro = gyro_v[-1]
            current_accel = accel_v[-1]

            ugv_patch.set_xy(rotate_points(UGV_BODY, current_yaw))

            nose = rotate_points(np.array([[0.0, 0.48]]), current_yaw)[0]
            heading_line.set_data([0, nose[0]], [0, nose[1]])

            robot_hud.set_text(
                f"yaw={current_yaw:+6.1f} deg\n"
                f"gz (turn)={current_gyro[2]:+6.1f} deg/s\n"
                f"|accel|={math.sqrt(sum(a * a for a in current_accel)):5.2f} m/s^2"
            )

        plt.pause(0.03)  # ~30 fps cap


if __name__ == "__main__":
    main()
