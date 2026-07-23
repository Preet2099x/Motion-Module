#!/usr/bin/env python3
"""
laptop_visualizer.py
Runs on your laptop. Listens for UDP JSON packets from jetson_bridge.py
and draws a live 2D top-down view (X-Y plane):
  - a trail showing recent position
  - a marker at the current position
  - an arrow showing heading (yaw, extracted from the quaternion)

The view auto-expands to fit the whole visible trail (with padding).

Install:  pip install numpy matplotlib
Run:      python3 laptop_visualizer.py
"""

import socket
import json
import threading
import math
from collections import deque

import matplotlib.pyplot as plt

UDP_IP = "0.0.0.0"       # listen on all interfaces
UDP_PORT = 5005          # must match LAPTOP_PORT in jetson_bridge.py

TRAIL_MAXLEN = 4000
TRAIL_PLOT_POINTS = 800   # max points drawn per frame (perf)
PLOT_MIN_HALF_RANGE = 0.4 # meters, never zoom in tighter than this
PLOT_PADDING = 1.4
RECENTER_EVERY = 4         # frames between axis-limit recalculations
ARROW_LEN = 0.2            # meters, heading-arrow length

latest = {"quat": [1.0, 0.0, 0.0, 0.0], "pos": [0.0, 0.0, 0.0]}
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
        with lock:
            latest["quat"] = pkt["quat"]
            latest["pos"] = pkt["pos"]
            trail.append((pkt["pos"][0], pkt["pos"][1]))


def quat_to_yaw(q):
    """Yaw (rotation about Z, i.e. heading in the X-Y plane) from a
    quaternion [w, x, y, z]."""
    w, x, y, z = q
    n = (w * w + x * x + y * y + z * z) ** 0.5
    if n == 0:
        return 0.0
    w, x, y, z = w / n, x / n, y / n, z / n
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    threading.Thread(target=udp_listener, daemon=True).start()

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.patch.set_facecolor("#0b0c0e")
    ax.set_facecolor("#0b0c0e")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color=(1, 1, 1, 0.08))
    for spine in ax.spines.values():
        spine.set_color((1, 1, 1, 0.15))
    ax.set_xlabel("X (m)", color="#9aa0a6")
    ax.set_ylabel("Y (m)", color="#9aa0a6")
    ax.tick_params(colors="#6b7075", labelsize=9)
    title = ax.set_title("Live IMU tracking (top-down)", color="#e8eaed", fontsize=13, pad=12)

    trail_line, = ax.plot([], [], color="#3aa0ff", lw=1.6, alpha=0.85)
    point_marker, = ax.plot([], [], "o", color="#ff4d4d", markersize=10)
    heading_arrow = ax.annotate(
        "", xy=(0, 0), xytext=(0, 0),
        arrowprops=dict(arrowstyle="-|>", color="#5cff8f", lw=2.5),
    )

    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)

    frame = 0
    while plt.fignum_exists(fig.number):
        with lock:
            q = latest["quat"]
            p = latest["pos"]
            pts = list(trail)

        if pts:
            step = max(1, len(pts) // TRAIL_PLOT_POINTS)
            shown = pts[::step]
            xs, ys = zip(*shown)
            trail_line.set_data(xs, ys)
            point_marker.set_data([p[0]], [p[1]])

            yaw = quat_to_yaw(q)
            hx = p[0] + ARROW_LEN * math.cos(yaw)
            hy = p[1] + ARROW_LEN * math.sin(yaw)
            heading_arrow.xy = (hx, hy)
            heading_arrow.set_position((p[0], p[1]))

            if frame % RECENTER_EVERY == 0:
                all_xs, all_ys = zip(*pts)
                cx = (max(all_xs) + min(all_xs)) / 2
                cy = (max(all_ys) + min(all_ys)) / 2
                span = max(
                    max(all_xs) - min(all_xs),
                    max(all_ys) - min(all_ys),
                    PLOT_MIN_HALF_RANGE * 2,
                ) * PLOT_PADDING / 2
                ax.set_xlim(cx - span, cx + span)
                ax.set_ylim(cy - span, cy + span)
                title.set_text(f"Live IMU tracking (top-down)  |  {len(trail)} pts")

        frame += 1
        plt.pause(0.02)  # ~50 fps cap


if __name__ == "__main__":
    main()
