#!/usr/bin/env python3
"""Bridge STM32 keypad serial commands into ArduPilot SITL via MAVLink.

    STM32 keypad --> /dev/ttyACM0 --> this bridge --> (udp) --> MAVProxy --> SITL

The STM32 sends plain-text MAVProxy-style command lines terminated with CRLF,
e.g. "mode guided\r\n", "arm throttle\r\n", "takeoff 5\r\n". This script reads
those lines from the serial port and reproduces each command as MAVLink, so the
firmware needs no changes.

Because SITL's tcp:127.0.0.1:5760 accepts only one client (MAVProxy), this
bridge connects to a MAVProxy *output* port instead of SITL directly. Start
MAVProxy so it forwards to a spare UDP port, e.g.:

    mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14551

then run this bridge (defaults match that port). MAVProxy keeps working for
manual typing; the keypad is just an extra command source.

If you are NOT running MAVProxy, you can connect straight to SITL instead:

    ./keypad_bridge.py --connect tcp:127.0.0.1:5760
"""
import argparse
import sys
import time

import serial
from pymavlink import mavutil

# Keypad key -> command string, mirroring the STM32 keypad_send_command() map.
# Documented here for reference; the bridge acts on the received *text*, not the
# key, so the STM32 remains the single source of truth for the mapping.
KEY_REFERENCE = {
    "1": "mode guided",
    "2": "arm throttle",
    "3": "takeoff 5",
    "4": "mode land",
    "5": "disarm",
    "C": "mode rtl",
}


def connect_mavlink(address, source_system=255):
    print(f"[bridge] connecting to {address} ...")
    master = mavutil.mavlink_connection(address, source_system=source_system)
    print("[bridge] waiting for heartbeat from vehicle ...")
    master.wait_heartbeat()
    print(f"[bridge] heartbeat OK  (system {master.target_system} "
          f"component {master.target_component})")
    return master


def set_mode(master, mode_name):
    mode_name = mode_name.upper()
    mapping = master.mode_mapping() or {}
    if mode_name not in mapping:
        print(f"[bridge] ERROR: unknown mode '{mode_name}'. "
              f"Known: {sorted(mapping)}")
        return
    master.set_mode(mapping[mode_name])
    print(f"[bridge] -> set mode {mode_name}")


def arm(master, do_arm):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1 if do_arm else 0, 0, 0, 0, 0, 0, 0)
    print(f"[bridge] -> {'arm' if do_arm else 'disarm'} throttle")


def takeoff(master, altitude):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, float(altitude))
    print(f"[bridge] -> takeoff {altitude} m "
          "(requires GUIDED + armed first)")


def handle_command(master, line):
    """Parse one MAVProxy-style text line and issue the MAVLink equivalent."""
    parts = line.split()
    if not parts:
        return
    cmd = parts[0].lower()

    if cmd == "mode" and len(parts) >= 2:
        set_mode(master, parts[1])
    elif cmd == "arm":
        arm(master, True)
    elif cmd == "disarm":
        arm(master, False)
    elif cmd == "takeoff":
        alt = parts[1] if len(parts) >= 2 else "5"
        takeoff(master, alt)
    else:
        print(f"[bridge] ignoring unrecognized command: {line!r}")


def run(serial_port, baud, connect):
    master = connect_mavlink(connect)

    print(f"[bridge] opening serial {serial_port} @ {baud} ...")
    ser = serial.Serial(serial_port, baud, timeout=1)
    # Drop any boot banner ("UART ready", etc.) sitting in the buffer.
    time.sleep(0.2)
    ser.reset_input_buffer()
    print("[bridge] ready. Press keypad keys to send commands. Ctrl-C to quit.\n")

    try:
        while True:
            raw = ser.readline()          # blocks up to `timeout`, split on \n
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            print(f"[serial] {line!r}")
            handle_command(master, line)
    except KeyboardInterrupt:
        print("\n[bridge] stopped.")
    finally:
        ser.close()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--serial", default="/dev/ttyACM0",
                   help="STM32 serial device (default: /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=115200,
                   help="serial baud rate (default: 115200)")
    p.add_argument("--connect", default="udpin:127.0.0.1:14551",
                   help="MAVLink endpoint. Default connects to a MAVProxy "
                        "--out=udp:127.0.0.1:14551 port. Use "
                        "tcp:127.0.0.1:5760 to talk to SITL directly.")
    args = p.parse_args()

    try:
        run(args.serial, args.baud, args.connect)
    except serial.SerialException as e:
        print(f"[bridge] serial error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
