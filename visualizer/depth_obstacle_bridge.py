#!/usr/bin/env python3
"""
depth_obstacle_bridge.py
Runs on the Jetson, ALONGSIDE jetson_bridge.py (that one still handles
Teensy IMU + wheel odometry -> UDP:5005; this one is a second, separate
process/port just for obstacles).

Reads the Intel RealSense D455 depth stream and forwards a compact 2D
"scan ring" of nearby obstacles to your laptop over UDP -- like a cheap
fake-lidar horizontal slice through the depth image, NOT a full 3D
point cloud. Keeps bandwidth low and the laptop-side rendering simple.

HOW IT WORKS: for a thin horizontal band of rows near the middle of
each depth frame (this dodges floor and ceiling clutter), the frame is
divided into N_BINS angular columns spanning the camera's horizontal
field of view. For each bin, the closest valid depth reading in that
band is taken as "nearest obstacle in that direction". The result is
one distance value per angular bin -- a horizontal ring scan around
the direction the camera is facing.

IMPORTANT: this does NOT use the D455's own built-in IMU. Your
Teensy + BNO055 (via jetson_bridge.py) remains the single source of
truth for orientation/position. The D455 is used here purely for
depth/obstacle sensing, kept deliberately separate to avoid two
competing orientation sources.

MOUNTING ASSUMPTIONS: this assumes the camera faces the same
"forward" direction as your robot's local +X axis.
  - If the camera is mounted at an angle (not dead-forward), set
    CAM_YAW_OFFSET_DEG.
  - If the camera isn't at the robot's origin, set CAM_FORWARD_OFFSET
    (meters ahead of the robot's odometry origin) and CAM_HEIGHT
    (meters above it -- informational only, the scan itself is
    flattened to 2D on the laptop side).

PACKET SIZE: with the defaults below (N_BINS=72), each UDP packet is
roughly 1.5-2 KB as JSON. That's within typical local Wi-Fi/Ethernet
limits but may fragment on some networks. If you see dropped/garbled
packets, lower N_BINS or SEND_HZ.

Install:  pip install pyrealsense2 numpy
Run:      python3 depth_obstacle_bridge.py
"""

import time
import json
import socket
import math

import numpy as np
import pyrealsense2 as rs

# ---- CONFIG: edit these ----
LAPTOP_IP = "10.10.1.114"      # same laptop IP as jetson_bridge.py
LAPTOP_PORT = 5006             # DIFFERENT port from the IMU bridge (5005)

DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
DEPTH_FPS = 30

N_BINS = 72                    # angular bins across the horizontal FOV
BAND_FRAC = 0.15               # vertical scan-band height, as a fraction of image height, centered on the middle row
MAX_RANGE_M = 6.0              # ignore returns beyond this (clutter/noise)
MIN_RANGE_M = 0.15             # ignore returns closer than this (D455 min range / self-occlusion)
SEND_HZ = 10.0                 # obstacle scan send rate (independent of camera fps)

CAM_YAW_OFFSET_DEG = 0.0       # rotate the scan if the camera isn't mounted dead-forward
CAM_FORWARD_OFFSET = 0.0       # meters, camera position ahead of the robot's odometry origin
CAM_HEIGHT = 0.0               # meters, camera height above the robot's odometry origin (informational)
# -----------------------------


def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, DEPTH_FPS)

    print("Starting D455 depth stream...")
    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"\npipeline.start() failed: {e}")
        print("This usually means the requested (width, height, fps) combo isn't")
        print("supported by this device/firmware. Supported depth profiles are:\n")
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            print("  (no RealSense devices detected at all -- check the USB cable/port)")
        for dev in devices:
            for sensor in dev.query_sensors():
                for sp in sensor.get_stream_profiles():
                    if sp.stream_type() == rs.stream.depth:
                        vsp = sp.as_video_stream_profile()
                        print(f"  {vsp.width()}x{vsp.height()} @ {vsp.fps()}fps  ({vsp.format()})")
        raise
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()  # meters per depth unit

    depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    intr = depth_stream.get_intrinsics()
    hfov_deg = math.degrees(2 * math.atan2(intr.width / 2.0, intr.fx))
    print(f"Depth intrinsics: {intr.width}x{intr.height}, HFOV ~= {hfov_deg:.1f} deg")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    row_lo = int(DEPTH_HEIGHT * (0.5 - BAND_FRAC / 2))
    row_hi = int(DEPTH_HEIGHT * (0.5 + BAND_FRAC / 2))
    col_edges = np.linspace(0, DEPTH_WIDTH, N_BINS + 1).astype(int)

    # Precompute the angle (deg, relative to the camera's forward axis) at
    # the center of each bin, using real intrinsics rather than a flat
    # linear split of the FOV (accounts for lens distortion near the edges).
    bin_angles = []
    for i in range(N_BINS):
        col_center = (col_edges[i] + col_edges[i + 1]) / 2.0
        ang = math.degrees(math.atan2(col_center - intr.ppx, intr.fx))
        bin_angles.append(round(ang + CAM_YAW_OFFSET_DEG, 2))

    send_period = 1.0 / SEND_HZ
    last_send = 0.0

    print(f"Forwarding obstacle scans to {LAPTOP_IP}:{LAPTOP_PORT} (Ctrl+C to stop)")
    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            now = time.monotonic()
            if now - last_send < send_period:
                continue
            last_send = now

            depth_img = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
            band = depth_img[row_lo:row_hi, :]

            dists = []
            for i in range(N_BINS):
                c0, c1 = col_edges[i], col_edges[i + 1]
                strip = band[:, c0:c1]
                valid = strip[(strip > MIN_RANGE_M) & (strip < MAX_RANGE_M)]
                if valid.size == 0:
                    dists.append(None)   # nothing in range for this bin
                else:
                    dists.append(round(float(valid.min()), 3))

            packet = {
                "t": now,
                "angles_deg": bin_angles,
                "dist_m": dists,
                "cam_forward_offset": CAM_FORWARD_OFFSET,
                "cam_height": CAM_HEIGHT,
            }
            sock.sendto(json.dumps(packet).encode("utf-8"), (LAPTOP_IP, LAPTOP_PORT))

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
