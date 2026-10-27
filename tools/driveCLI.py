#!/usr/bin/env python3
"""
driveCLI - drive the new-robot Motion Module straight from a terminal.

Type a motion command and it is re-sent every 100 ms until you stop it or type another
one (the firmware stops the robot if it hears nothing for 1 s, so it has to be streamed).

Usage:
    python driveCLI.py                 # uses /dev/ttyACM0
    python driveCLI.py /dev/ttyACM1    # or give another port (e.g. COM5 on Windows)

Drive (streams until stopped):
    fwd / back                         straight at the preset speed (code 1 / 2), or at your custom rpm
    fwd <rpm> / back <rpm>             straight at exactly <rpm> (also remembered for plain fwd / back)
    rpm <rpm>                          set the custom straight speed (changes it live if already driving)
    rpm off                            go back to the preset speed (code 1 / 2, boost applies)
    left [1-3] / right [1-3]           turn in place, 1 slow, 2 mid, 3 fast (default 2)
    <number>                           any motion code: 1, 2, 11-30, 111-130, 211-230, ...

Stop:
    x                                  HARD STOP: fires the moment you press x on an empty line (no Enter)
    <Enter> / stop / 0 / space         normal stop (gentle deceleration; the stream keeps sending 0)

One-shot (sent once, motion keeps going):
    boost                              toggle straight/arc boost (65)
    turnboost                          toggle in-place turn boost (66)

Servos / debug (sent once):
    enable  disable  reset  status  scan  id      (firmware: e d f ? c m)

    quit / exit / Ctrl+C               hard stop the robot and leave

The custom rpm is motor rpm, signed by direction, and the firmware caps it to the motor's max speed.
"""

import os
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("ERROR: pyserial is not installed. Run:  pip install pyserial")
    sys.exit(1)

DEFAULT_PORT = "/dev/ttyACM0"
SEND_PERIOD_S = 0.1  # firmware times out after 1 s, so 10 Hz is plenty
MAX_CUSTOM_RPM = 10000

TIER_BASE = {1: 0, 2: 3, 3: 6}   # tier 1/2/3 -> offset inside a 10-code range
ONE_SHOT = {
    "boost": "65", "turnboost": "66",
    "enable": "e", "disable": "d", "reset": "f", "status": "?", "scan": "c", "id": "m",
}


class Console:
    """Prompt + the line being typed, so Teensy output can be printed above it without mangling it."""

    def __init__(self):
        self.lock = threading.Lock()
        self.prompt = ""
        self.buf = ""

    def redraw(self):
        with self.lock:
            sys.stdout.write("\r\033[K" + self.prompt + self.buf)
            sys.stdout.flush()

    def message(self, text):
        with self.lock:
            sys.stdout.write("\r\033[K" + text + "\n" + self.prompt + self.buf)
            sys.stdout.flush()


class Keys:
    """Reads single keypresses (no Enter needed). Linux/Jetson via termios, Windows via msvcrt."""

    def __init__(self):
        self.fd = None
        self.saved = None

    def __enter__(self):
        if os.name != "nt":
            import termios
            import tty
            self.fd = sys.stdin.fileno()
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)  # keys arrive one by one, Ctrl+C still works
        return self

    def __exit__(self, *exc):
        if self.saved is not None:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def get(self):
        if os.name == "nt":
            import msvcrt
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):      # arrow / function key: swallow its second half
                msvcrt.getwch()
                return ""
            if ch == "\x03":
                raise KeyboardInterrupt
            return ch
        ch = os.read(self.fd, 1).decode(errors="ignore")
        if ch == "\x1b":                    # arrow key escape sequence: swallow it
            os.read(self.fd, 2)
            return ""
        return ch


def read_line(keys, console):
    """Line editor. Returns the typed line on Enter, or 'x' the instant x is pressed on an empty
    line. Returns None on Ctrl+D."""
    console.buf = ""
    console.redraw()
    while True:
        ch = keys.get()
        if not ch:
            continue
        if ch in ("x", "X") and not console.buf:
            with console.lock:
                sys.stdout.write("x  ** HARD STOP **\n")
                sys.stdout.flush()
            return "x"
        if ch in ("\r", "\n"):
            line = console.buf
            console.buf = ""
            with console.lock:
                sys.stdout.write("\n")
                sys.stdout.flush()
            return line
        if ch in ("\x7f", "\x08"):          # backspace
            console.buf = console.buf[:-1]
        elif ch == "\x04" and not console.buf:
            return None
        elif ch.isprintable():
            console.buf += ch
        console.redraw()


class Link:
    def __init__(self, ser, console):
        self.ser = ser
        self.console = console
        self.write_lock = threading.Lock()
        self.wire = None            # text being streamed (None = nothing)
        self.label = ""             # what to show in the prompt
        self.straight = 0           # +1 / -1 while driving straight with fwd / back, else 0
        self.running = True

    def send(self, text):
        with self.write_lock:
            self.ser.write((text + "\n").encode())

    def stream(self, wire, label, straight=0):
        self.wire = wire
        self.label = label
        self.straight = straight
        self.send(wire)             # don't wait for the next tick

    def sender(self):
        while self.running:
            try:
                with self.write_lock:   # read + write under the lock, so stop() can't be overtaken by a stale frame
                    if self.wire is not None:
                        self.ser.write((self.wire + "\n").encode())
            except serial.SerialException:
                self.console.message("[serial lost]")
                self.running = False
                return
            time.sleep(SEND_PERIOD_S)

    def reader(self):
        buf = b""
        while self.running:
            try:
                buf += self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException:
                self.running = False
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if text:
                    self.console.message("  < " + text)

    def stop(self, hard=False):
        """Stop and keep streaming 0 so the robot stays commanded to stop."""
        with self.write_lock:
            self.wire = None
            self.label = ""
            self.straight = 0
        try:
            if hard:
                self.send("x")      # fast deceleration; the next moving command restores the gentle one
            for _ in range(3):
                self.send("0")
                time.sleep(0.03)
        except serial.SerialException:
            return
        self.wire = "0"


def straight_wire(sign, rpm):
    """Wire text and prompt label for driving straight: custom rpm if set, else preset code 1 / 2."""
    name = "fwd" if sign > 0 else "back"
    if rpm:
        return f"r{sign * rpm},{sign * rpm}", f"{name} {rpm} rpm"
    return ("1" if sign > 0 else "2"), name


def parse_rpm(text):
    try:
        rpm = int(text)
    except ValueError:
        return None
    return rpm if 1 <= rpm <= MAX_CUSTOM_RPM else None


def parse(line):
    """Returns ('straight', sign, rpm|None) | ('setrpm', rpm|None) | ('drive', code) | ('stop',) |
    ('hardstop',) | ('once', text) | ('quit',) | ('bad', msg)."""
    words = line.strip().lstrip("> ").lower().split()   # tolerate a pasted "> " prompt
    if not words or words[0] in ("stop", "s", "0"):
        return ("stop",)
    w = words[0]
    if w == "x":
        return ("hardstop",)
    if w in ("quit", "exit", "q"):
        return ("quit",)
    if w in ONE_SHOT:
        return ("once", ONE_SHOT[w])
    if w in ("fwd", "forward", "back", "backward"):
        sign = 1 if w in ("fwd", "forward") else -1
        if len(words) == 1:
            return ("straight", sign, None)
        rpm = parse_rpm(words[1])
        if rpm is None:
            return ("bad", f"rpm must be a whole number from 1 to {MAX_CUSTOM_RPM}")
        return ("straight", sign, rpm)
    if w == "rpm":
        if len(words) < 2:
            return ("bad", "usage: rpm <number>  or  rpm off")
        if words[1] == "off":
            return ("setrpm", None)
        rpm = parse_rpm(words[1])
        if rpm is None:
            return ("bad", f"rpm must be a whole number from 1 to {MAX_CUSTOM_RPM}")
        return ("setrpm", rpm)
    if w in ("left", "right"):
        tier = 2
        if len(words) > 1:
            if words[1] not in ("1", "2", "3"):
                return ("bad", "speed must be 1, 2 or 3")
            tier = int(words[1])
        return ("drive", (21 if w == "left" else 11) + TIER_BASE[tier])
    try:
        return ("drive", int(w))
    except ValueError:
        return ("bad", f"unknown command '{w}' (see the help at the top of this file)")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    try:
        ser = serial.Serial(port, 115200, timeout=0.05)
    except serial.SerialException as e:
        print(f"Cannot open {port}: {e}")
        print("Serial ports found:")
        for p in list_ports.comports():
            print(f"  {p.device}  {p.description}")
        print("Pass the right one:  python driveCLI.py <port>")
        return 1
    print(__doc__)
    print(f"Connected on {port}\n")

    console = Console()
    link = Link(ser, console)
    custom_rpm = None               # None = preset speed (code 1 / 2)
    threading.Thread(target=link.sender, daemon=True).start()
    threading.Thread(target=link.reader, daemon=True).start()
    # Flush any half-typed line left in the Teensy's input buffer (an empty line is ignored by the
    # firmware), otherwise our first command gets glued onto the leftovers and rejected.
    ser.reset_input_buffer()
    link.send("")
    time.sleep(0.1)
    link.send("m")

    # Single-key "x" needs a real terminal; piped input falls back to plain lines.
    keys = Keys() if sys.stdin.isatty() else None
    try:
        if keys:
            keys.__enter__()
        while link.running:
            state = f"driving {link.label}" if link.label else "stopped"
            console.prompt = f"[{state}] > "
            if keys:
                line = read_line(keys, console)
                if line is None:
                    break
            else:
                try:
                    line = input(console.prompt)
                except EOFError:
                    break
            action = parse(line)
            kind = action[0]
            if kind == "quit":
                break
            elif kind == "stop":
                link.stop()
            elif kind == "hardstop":
                link.stop(hard=True)
            elif kind == "once":
                link.send(action[1])
            elif kind == "straight":
                _, sign, rpm = action
                if rpm:
                    custom_rpm = rpm
                link.stream(*straight_wire(sign, custom_rpm), straight=sign)
            elif kind == "setrpm":
                custom_rpm = action[1]
                print(f"  straight speed: {custom_rpm} rpm" if custom_rpm else "  straight speed: preset (code 1 / 2)")
                if link.straight:   # already driving straight: apply it now
                    link.stream(*straight_wire(link.straight, custom_rpm), straight=link.straight)
            elif kind == "drive":
                link.stream(str(action[1]), str(action[1]))
            else:
                print(f"  {action[1]}")
    except KeyboardInterrupt:
        pass
    finally:
        console.prompt = console.buf = ""
        print("\nStopping robot...")
        link.stop(hard=True)
        if keys:
            keys.__exit__(None, None, None)
        link.running = False
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
