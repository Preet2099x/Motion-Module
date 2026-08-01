#!/usr/bin/env python3
"""
jetson_bridge.py
Runs on the Jetson. Auto-detects the Teensy on /dev/ttyACM1 or
/dev/ttyACM2, reads lines of the form:

  SENSOR:qw,qx,qy,qz,liaX,liaY,liaZ,odomX,odomY,odomH,vLin,vAng

and forwards a clean pose packet to your laptop over UDP as JSON.

WHY THIS FIXES THE DRIFT: position now comes directly from wheel
odometry (odomX, odomY) instead of double-integrating accelerometer
data. Double-integrated IMU position error grows with time squared --
that's what caused the runaway drift before. Wheel odometry error
grows roughly linearly with distance travelled (from wheel slip / tire
diameter uncertainty), which is a fundamentally more stable source.
No calibration, leaky integration, or reset hacks needed anymore for
position.

Heading uses the IMU quaternion (qw,qx,qy,qz) rather than the
odometry heading (odomH): the BNO055 in NDOF mode fuses in the
magnetometer, so its yaw is an absolute compass-referenced heading
that doesn't drift, whereas wheel-based heading slips during turns.
odomH is still forwarded in case you want to compare/use it.

Install:  pip install pyserial
Run:      python3 jetson_bridge.py
"""

import glob
import time
import json
import socket

import serial

# ---- CONFIG: edit these ----
CANDIDATE_PORTS = ["/dev/ttyACM1", "/dev/ttyACM2"]
SERIAL_BAUD = 115200
LAPTOP_IP = "10.10.1.114"      # laptop Wi-Fi IP (same network as Jetson)
LAPTOP_PORT = 5005
RECONNECT_DELAY = 1.0          # seconds between reconnect attempts

# If your Teensy sends odomH in radians instead of degrees, set this False.
ODOM_HEADING_IS_DEGREES = True
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


def read_sensor_sample(ser):
    """Read one line, return the 12 floats or None if not a valid line."""
    raw = ser.readline()
    line = raw.decode("utf-8", errors="ignore").strip()
    if not line.startswith("SENSOR:"):
        return None
    parts = line[len("SENSOR:"):].split(",")
    if len(parts) != 12:
        return None
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


def main():
    import math

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ser = connect_with_retry()

    n_sent = 0
    print(f"Forwarding to {LAPTOP_IP}:{LAPTOP_PORT} (Ctrl+C to stop)")

    while True:
        try:
            sample = read_sensor_sample(ser)
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

        (qw, qx, qy, qz,
         lax, lay, laz,
         odom_x, odom_y, odom_h,
         v_lin, v_ang) = sample

        odom_h_rad = math.radians(odom_h) if ODOM_HEADING_IS_DEGREES else odom_h

        packet = {
            "t": time.monotonic(),
            "quat": [qw, qx, qy, qz],
            "accel": [lax, lay, laz],       # kept for reference/debugging only
            "pos": [odom_x, odom_y],        # position straight from wheel odometry
            "odom_heading": odom_h_rad,     # wheel-based heading, radians
            "v_lin": v_lin,
            "v_ang": v_ang,
        }

        sock.sendto(json.dumps(packet).encode("utf-8"), (LAPTOP_IP, LAPTOP_PORT))

        n_sent += 1
        if n_sent % 100 == 0:
            print(f"sent {n_sent} packets | pos=({odom_x:.2f}, {odom_y:.2f}) "
                  f"h={odom_h:.1f} deg | vLin={v_lin:.2f} vAng={v_ang:.2f}")


if __name__ == "__main__":
    main()
