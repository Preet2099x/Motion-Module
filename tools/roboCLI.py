#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
roboCLI - Jetson CLI for the Motion Module Teensy.

Auto-detects the Teensy on /dev/ttyACM1 or /dev/ttyACM2 at startup.
Retries until one is found, then enters the command REPL.

Usage:
    python roboCLI.py [baud]

Examples:
    python roboCLI.py            # 115200 baud (default)
    python roboCLI.py 9600       # custom baud rate

Commands:
    F3,R90,F5   => Sends  T:F3,R90,F5,S  (auto-appends S if missing)
    r           => Repeat the last track command
    home        => Reset odometry origin to current position (x=0, y=0, heading=0)
    returnhome  => Navigate autonomously back to home coordinates (0, 0)
    pos         => Print current X / Y / heading once
    debug       => Toggle live X / Y / heading stream (every 200 ms)
    1 / 15 / 0  => Sends the numeric command as-is
    m / s       => Sends the single-char Teensy command
    follow      => FOLLOW-ME: robot autonomously follows the beacon you carry
    followstop  => Stop follow-me
    followmon   => Toggle live beacon readout with motors idle (safe bring-up)
    followcfg   => Print follow-me configuration + current beacon fix
    followrev   => Invert which way "right"/"left" spins the motors
    followfb    => Invert the front/back sense (if front reads as behind)
    followturn  => TURN-ONLY: aim at the beacon and hold, never drive
    followtri   => Toggle THRESHOLD vs TRILATERATION+45deg-sector method
    followm <m> => Comparison noise margin in metres (default 0.15)
    followl <m> => Leg length: how far it drives between looks (default 1.00)
    followr <m> => Stop range in metres (default 2.00)
    followg <m> => Resume range: start following again past this (default 2.30)
    followp <m> => Hard stop when any raw anchor reads under <m> (default 0.70)
    p           => Toggle drive PAUSE / RESUME
                   First 'p'  => pauses (motors stop, task frozen)
                   Second 'p' => resumes (motors restart, task continues)
    x           => CAN lock/latch toggle (physical robot stop + track cancel / release)
    quit / exit => Close the CLI
"""

import sys
import re
import threading
import time
import subprocess
import locale
import os


try:
    import serial
except ImportError:
    print("ERROR: pyserial is not installed. Run:  pip install pyserial")
    sys.exit(1)


# ---- Configure UTF-8 for Jetson environment ----
def setup_utf8_locale():
    """Ensure UTF-8 locale for proper character encoding."""
    try:
        # Try to set UTF-8 locale
        locale.setlocale(locale.LC_ALL, "C.UTF-8")
    except locale.Error:
        try:
            locale.setlocale(locale.LC_ALL, "en_US.UTF-8")
        except locale.Error:
            # Fallback - use C locale
            locale.setlocale(locale.LC_ALL, "C")

    # Set environment variable for subprocess calls
    if "UTF" not in os.environ.get("LANG", "").upper():
        os.environ["LANG"] = "C.UTF-8"


setup_utf8_locale()


# ---- Serial reader thread ----

def reader_thread(ser: serial.Serial, stop_event: threading.Event):
    """Continuously read and print serial output from the Teensy."""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="replace").rstrip()
                if line:
                    print("  <- " + line)
            else:
                time.sleep(0.01)
        except (serial.SerialException, OSError):
            break


# ---- Helpers ----

# Matches drive-command tokens: F3, B1.5, L90, R45, S  (case insensitive)
TASK_TOKEN_RE = re.compile(r"^[FBLRSfblrs]", re.IGNORECASE)


def is_task_input(text: str) -> bool:
    """Return True if the input looks like a drive task (not a plain number)."""
    tokens = [t.strip() for t in text.split(",") if t.strip()]
    if not tokens:
        return False
    # Every token must start with a valid command letter
    return all(TASK_TOKEN_RE.match(t) for t in tokens)


def frame_task(text: str) -> str:
    """
    Convert user input like  F3,R90,F5  into  T:F3,R90,F5,S
    - Uppercases everything.
    - Prepends T: if not present.
    - Appends ,S if the last token isn't S.
    """
    text = text.strip().upper()

    # Strip T: if user accidentally typed it
    if text.startswith("T:"):
        text = text[2:]

    # Ensure it ends with S
    tokens = [t.strip() for t in text.split(",") if t.strip()]
    if tokens and tokens[-1] != "S":
        tokens.append("S")

    return "T:" + ",".join(tokens)


# ---- Main REPL ----

def find_serial_port(baud: int) -> serial.Serial:
    """Try /dev/ttyACM1 then /dev/ttyACM2. Block until one connects."""
    candidates = ["/dev/ttyACM1", "/dev/ttyACM2"]

    print("roboCLI searching for Teensy...")

    while True:
        for port in candidates:
            try:
                ser = serial.Serial(port, baud, timeout=0.1)
                print("  [OK] Connected on " + port)
                return ser
            except (serial.SerialException, OSError):
                pass

        # Neither found -- wait and retry
        print("  [FAIL] No device on " + candidates[0] + " or " + candidates[1] + " -- retrying in 2s")
        time.sleep(2)


def main():
    baud = int(sys.argv[1]) if len(sys.argv) >= 2 else 115200

    ser = find_serial_port(baud)

    print("Ready @ " + str(baud) + " baud.  Type commands or 'quit' to exit.\n")

    # Start background reader
    stop_event = threading.Event()
    reader = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    reader.start()

    robot_locked = False
    last_track: str | None = None  # last successfully sent drive task
    debug_on = False               # tracks whether live odometry stream is active
    drive_paused = False           # mirrors the Teensy's drive-pause state

    try:
        while True:
            try:
                user_input = input("robo> ").strip()
            except EOFError:
                break

            if not user_input:
                continue

            lower = user_input.lower()

            # ---- Quit ----
            if lower in ("quit", "exit"):
                break

            # ---- CAN Lock Toggle ----
            if lower == "x":
                if not robot_locked:
                    print("  [LOCK] Running CAN lock command in background...")
                    try:
                        subprocess.Popen(
                            ["sudo", "./canusb", "-d", "/dev/ttyUSB0", "-s", "500000", "-t", "-i", "300", "-j", "0100", "-n", "1"],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL
                        )
                    except Exception as e:
                        print("  Error spawning CAN lock command: " + str(e))

                    # Cancel any running autonomous track, then stop manual motors
                    ser.write(b"T:S\n")   # abort drive task queue on Teensy
                    time.sleep(0.05)
                    # Follow-me drives motion() directly and ignores manual
                    # data while running, so neither "T:S" nor "0" below can
                    # stop it -- it needs its own cancel or the latch would
                    # engage while the firmware kept commanding motion.
                    ser.write(b"followstop\n")
                    time.sleep(0.05)
                    ser.write(b"0\n")     # zero manual motor command
                    robot_locked = True
                else:
                    print("  [UNLATCH] Running CAN latch command in background...")
                    try:
                        subprocess.Popen(
                            ["sudo", "./canusb", "-d", "/dev/ttyUSB0", "-s", "500000", "-t", "-i", "300", "-j", "0000", "-n", "1"],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL
                        )
                    except Exception as e:
                        print("  Error spawning CAN latch command: " + str(e))

                    robot_locked = False
                continue

            # ---- Single-char Teensy commands (m, s) ----
            if lower in ("m", "s"):
                ser.write((lower + "\n").encode("utf-8"))
                print("  -> " + lower)
                continue

            # ---- Drive Pause / Resume toggle ----
            if lower == "p":
                ser.write(b"p\n")
                drive_paused = not drive_paused
                if drive_paused:
                    print("  [PAUSE] Drive paused -- motors stopped, task frozen.")
                    print("          Send 'p' again to resume.")
                else:
                    print("  [RESUME] Drive resumed -- task continuing.")
                continue

            # ---- Set home ----
            if lower == "home":
                if robot_locked:
                    print("  ! EMERGENCY LATCHED -- home command blocked. Unlock with x first.")
                else:
                    ser.write(b"home\n")
                    print("  -> home")
                continue

            # ---- Return to home coordinates via navigation ----
            if lower == "returnhome":
                if robot_locked:
                    print("  ! EMERGENCY LATCHED -- returnhome blocked. Unlock with x first.")
                else:
                    # Pause live debug stream so navigation messages are readable
                    if debug_on:
                        ser.write(b"d\n")
                        debug_on = False
                        print("  [debug stream paused for navigation]")
                    ser.write(b"returnhome\n")
                    print("  -> returnhome  (navigating to 0, 0)")
                continue

            # ---- One-shot position snapshot ----
            if lower == "pos":
                ser.write(b"pos\n")
                print("  -> pos")
                continue

            # ---- Toggle live odometry debug stream ----
            if lower == "debug":
                if robot_locked:
                    print("  ! EMERGENCY LATCHED -- debug blocked. Unlock with x first.")
                else:
                    ser.write(b"d\n")
                    debug_on = not debug_on
                    status = "ON" if debug_on else "OFF"
                    print("  -> d  (live odometry stream " + status + ")")
                continue

            # ---- Follow-me ----
            # followmon/followcfg/followdbg are read-only, so they stay
            # available even while latched -- being able to look at the beacon
            # fix without arming the motors is the whole point of monitor mode.
            if lower in ("followmon", "followcfg", "followdbg", "followrev", "followfb", "followtri", "followturn"):
                ser.write((lower + "\n").encode("utf-8"))
                print("  -> " + lower)
                continue

            if lower in ("follow", "followme"):
                if robot_locked:
                    print("  ! EMERGENCY LATCHED -- follow blocked. Unlock with x first.")
                else:
                    # Quieten the odometry stream first; follow-me prints its
                    # own fix line and the two interleave badly.
                    if debug_on:
                        ser.write(b"d\n")
                        debug_on = False
                        print("  [debug stream paused for follow-me]")
                    ser.write(b"follow\n")
                    # Follow_Start() calls Drive_Cancel() on the Teensy, which
                    # clears the pause latch -- mirror that here so this CLI's
                    # idea of the pause state does not go stale.
                    drive_paused = False
                    print("  -> follow  (FOLLOW-ME ARMED)")
                    print("     Robot turns to face the beacon, then drives at it.")
                    print("     Stop with 'followstop', 'p', or 'x'.")
                continue

            if lower == "followstop":
                ser.write(b"followstop\n")
                print("  -> followstop")
                continue

            # followm/followl/followr/followg/followp <m> -- tuning values
            m_follow = re.match(r"^(followm|followl|followr|followg|followp)\s+([0-9]*\.?[0-9]+)$", lower)
            if m_follow:
                ser.write((m_follow.group(1) + " " + m_follow.group(2) + "\n").encode("utf-8"))
                print("  -> " + m_follow.group(1) + " " + m_follow.group(2))
                continue

            # ---- Repeat last track ----
            if lower == "r":
                if last_track is None:
                    print("  ! No previous track to repeat")
                elif robot_locked:
                    print("  ! EMERGENCY LATCHED -- track discarded. Unlock with x first.")
                elif drive_paused:
                    print("  ! PAUSED -- track discarded. Send 'p' to resume first.")
                else:
                    ser.write((last_track + "\n").encode("utf-8"))
                    print("  -> (repeat) " + last_track)
                continue

            # ---- Task command (drive sequence) ----
            if is_task_input(user_input):
                if robot_locked:
                    print("  ! EMERGENCY LATCHED -- track discarded. Unlock with x first.")
                    continue
                if drive_paused:
                    print("  ! PAUSED -- track discarded. Send 'p' to resume first.")
                    continue
                framed = frame_task(user_input)
                ser.write((framed + "\n").encode("utf-8"))
                print("  -> " + framed)
                last_track = framed
                continue

            # ---- Numeric motion command ----
            try:
                int(user_input)
                if drive_paused:
                    print("  ! PAUSED -- motion command blocked. Send 'p' to resume first.")
                else:
                    ser.write((user_input + "\n").encode("utf-8"))
                    print("  -> " + user_input)
                continue
            except ValueError:
                pass

            # ---- Unknown ----
            print("  ? Unknown command: '" + user_input + "'")
            print("    Hint: F3,R90,F5 | r | home | returnhome | pos | debug | 1 | 0 | m | s | p | x | quit")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        stop_event.set()
        ser.close()
        print("Serial closed. Bye.")


if __name__ == "__main__":
    main()