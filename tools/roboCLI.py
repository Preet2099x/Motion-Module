#!/usr/bin/env python3
"""
roboCLI — Jetson CLI for the Motion Module Teensy.

Auto-detects the Teensy on /dev/ttyACM1 or /dev/ttyACM2 at startup.
Retries until one is found, then enters the command REPL.

Usage:
    python roboCLI.py [baud]

Examples:
    python roboCLI.py            # 115200 baud (default)
    python roboCLI.py 9600       # custom baud rate

Commands:
    F3,R90,F5   → Sends  T:F3,R90,F5,S  (auto-appends S if missing)
    1 / 15 / 0  → Sends the numeric command as-is
    m / s / p   → Sends the single-char Teensy command
    x           → CAN lock/latch toggle (physical robot stop/release)
    quit / exit → Close the CLI
"""

import sys
import re
import threading
import time
import subprocess


try:
    import serial
except ImportError:
    print("ERROR: pyserial is not installed. Run:  pip install pyserial")
    sys.exit(1)

# ─── Serial reader thread ──────────────────────────────────────


def reader_thread(ser: serial.Serial, stop_event: threading.Event):
    """Continuously read and print serial output from the Teensy."""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="replace").rstrip()
                if line:
                    print(f"  ← {line}")
            else:
                time.sleep(0.01)
        except (serial.SerialException, OSError):
            break


# ─── Helpers ────────────────────────────────────────────────────

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


# ─── Main REPL ─────────────────────────────────────────────────


def find_serial_port(baud: int) -> serial.Serial:
    """Try /dev/ttyACM1 then /dev/ttyACM2. Block until one connects."""
    candidates = ["/dev/ttyACM1", "/dev/ttyACM2"]

    print("roboCLI — searching for Teensy...")

    while True:
        for port in candidates:
            try:
                ser = serial.Serial(port, baud, timeout=0.1)
                print(f"  ✓ Connected on {port}")
                return ser
            except (serial.SerialException, OSError):
                pass

        # Neither found — wait and retry
        print(f"  ✗ No device on {candidates[0]} or {candidates[1]} — retrying in 2s")
        time.sleep(2)


def main():
    baud = int(sys.argv[1]) if len(sys.argv) >= 2 else 115200

    ser = find_serial_port(baud)

    print(f"Ready @ {baud} baud.  Type commands or 'quit' to exit.\n")

    # Start background reader
    stop_event = threading.Event()
    reader = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    reader.start()

    robot_locked = False

    try:
        while True:
            try:
                user_input = input("robo> ").strip()
            except EOFError:
                break

            if not user_input:
                continue

            lower = user_input.lower()

            # ── Quit ──
            if lower in ("quit", "exit"):
                break

            # ── CAN Lock Toggle ──
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
                        print(f"  Error spawning CAN lock command: {e}")

                    # Stop Teensy motors immediately
                    ser.write(b"0\n")
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
                        print(f"  Error spawning CAN latch command: {e}")

                    robot_locked = False
                continue

            # ── Single-char Teensy commands ──
            if lower in ("m", "s", "p"):
                ser.write((lower + "\n").encode())
                print(f"  → {lower}")
                continue

            # ── Task command (drive sequence) ──
            if is_task_input(user_input):
                framed = frame_task(user_input)
                ser.write((framed + "\n").encode())
                print(f"  → {framed}")
                continue

            # ── Numeric motion command ──
            try:
                int(user_input)
                ser.write((user_input + "\n").encode())
                print(f"  → {user_input}")
                continue
            except ValueError:
                pass

            # ── Unknown ──
            print(f"  ? Unknown command: '{user_input}'")
            print("    Hint: F3,R90,F5 | 1 | 0 | m | s | p | x | quit")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        stop_event.set()
        ser.close()
        print("Serial closed. Bye.")


if __name__ == "__main__":
    main()
