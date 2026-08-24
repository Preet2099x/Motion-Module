#!/usr/bin/env python3
"""
imu_bridge.py
Runs on the Jetson. Auto-detects the Teensy on /dev/ttyACM1 or
/dev/ttyACM2, reads lines of the form:

  IMU:gx,gy,gz,yaw,ax,ay,az

(gyro in deg/s, yaw in degrees, linear accel in m/s^2 -- see the
"Send raw IMU data" block in src/main.cpp) and forwards each sample to
your laptop over UDP as JSON, for live graphing with
imu_graph_visualizer.py.

NOTE: this reads the same Teensy serial port as jetson_bridge.py. Run
ONE of the two bridges at a time -- don't run both against the same
Teensy simultaneously, they'll fight over the port.

Install:  pip install pyserial
Run:      python3 imu_bridge.py
"""

import glob
import time
import json
import socket

import serial

# ---- CONFIG: edit these ----
CANDIDATE_PORTS = ["/dev/ttyACM1", "/dev/ttyACM2"]
SERIAL_BAUD = 115200
LAPTOP_IP = "10.10.1.106"      # laptop Wi-Fi IP (same network as Jetson)
LAPTOP_PORT = 5007             # different port from jetson_bridge (5005) / depth_obstacle_bridge (5006)
RECONNECT_DELAY = 1.0          # seconds between reconnect attempts
# -----------------------------


def find_serial_port():
    for port in CANDIDATE_PORTS:
        try:
            ser = serial.Serial(port, SERIAL_BAUD, timeout=1)
            print(f"Connected to {port}")
            return ser
        except serial.SerialException:
            continue
    return None


def connect_with_retry():
    while True:
        ser = find_serial_port()
        if ser is not None:
            return ser
        available = glob.glob("/dev/ttyACM*")
        print(f"No Teensy found on {CANDIDATE_PORTS} "
              f"(currently present: {available or 'none'}), retrying...")
        time.sleep(RECONNECT_DELAY)


def read_imu_sample(ser):
    """Read one line, return the 7 floats (gx,gy,gz,yaw,ax,ay,az) or None."""
    raw = ser.readline()
    line = raw.decode("utf-8", errors="ignore").strip()
    if not line.startswith("IMU:"):
        return None
    parts = line[len("IMU:"):].split(",")
    if len(parts) != 7:
        return None
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ser = connect_with_retry()

    n_sent = 0
    print(f"Forwarding to {LAPTOP_IP}:{LAPTOP_PORT} (Ctrl+C to stop)")

    while True:
        try:
            sample = read_imu_sample(ser)
        except (serial.SerialException, OSError):
            print("Serial connection lost, reconnecting...")
            try:
                ser.close()
            except Exception:
                pass
            ser = connect_with_retry()
            continue

        if sample is None:
            continue

        gx, gy, gz, yaw, ax, ay, az = sample

        packet = {
            "t": time.monotonic(),
            "gyro": [gx, gy, gz],   # deg/s
            "yaw": yaw,             # deg
            "accel": [ax, ay, az],  # m/s^2, gravity-compensated
        }

        sock.sendto(json.dumps(packet).encode("utf-8"), (LAPTOP_IP, LAPTOP_PORT))

        n_sent += 1
        if n_sent % 100 == 0:
            print(f"sent {n_sent} packets | gyro=({gx:.1f},{gy:.1f},{gz:.1f}) "
                  f"yaw={yaw:.1f} accel=({ax:.2f},{ay:.2f},{az:.2f})")


if __name__ == "__main__":
    main()
