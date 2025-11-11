#!/usr/bin/env python3
"""
ADXL355 UDP sender with simple remote control.

- Reads acceleration data from ADXL355 via SPI (using adxl355_sensor module)
- Sends data via UDP to a Linux PC (data channel)
- Listens for control UDP commands from the PC (control channel):
    - set_range:      change measurement range (±2/4/8 g)
    - set_rate:       change sampling rate (Hz)
    - set_streaming:  start/stop data streaming
    - get_status:     report current configuration

Intended use:
- This script runs continuously on the Raspberry Pi (can be run as a systemd service).
- The Linux PC acts as a receiver and controller.

Author: RoubenKe
"""

import json
import socket
import threading
import time
from typing import Tuple

from adxl355_sensor import ADXL355

# ==============================
# Configuration
# ==============================

# IP and port of the Linux PC that receives data
DATA_TARGET_IP = "192.168.9.100"   # <-- adjust to your PC's IP if needed
DATA_TARGET_PORT = 5005

# Control socket on the Raspberry Pi: listens for commands from the PC
CTRL_LISTEN_IP = "0.0.0.0"
CTRL_LISTEN_PORT = 5006

# Default sensor configuration
DEFAULT_RANGE_G = 4         # ±4 g as default
DEFAULT_SAMPLE_RATE_HZ = 200.0  # data messages per second

# ==============================
# Shared state
# ==============================

config_lock = threading.Lock()

current_range_g: int = DEFAULT_RANGE_G
current_sample_rate_hz: float = DEFAULT_SAMPLE_RATE_HZ

streaming_enabled = threading.Event()
streaming_enabled.set()  # start streaming by default

stop_event = threading.Event()


# ==============================
# Helper: send and receive JSON over UDP
# ==============================

def send_json(sock: socket.socket, addr: Tuple[str, int], obj: dict) -> None:
    """Serialize obj as JSON and send via UDP."""
    try:
        data = json.dumps(obj).encode("utf-8")
        sock.sendto(data, addr)
    except Exception as e:
        # Keep errors simple – this node has no UI
        print(f"[WARN] Failed to send JSON: {e}")


# ==============================
# Data thread: read sensor and send UDP packets
# ==============================

def data_loop(sensor: ADXL355, data_sock: socket.socket) -> None:
    """Continuously read sensor data and send via UDP."""
    global current_sample_rate_hz, current_range_g

    print("[INFO] Data thread started.")

    while not stop_event.is_set():
        # Copy config under lock
        with config_lock:
            rate_hz = float(current_sample_rate_hz)
            range_g = int(current_range_g)

        if rate_hz <= 0:
            rate_hz = 10.0  # fallback

        period = 1.0 / rate_hz

        if streaming_enabled.is_set():
            timestamp = time.time()

            try:
                x_raw, y_raw, z_raw = sensor.read_raw()
            except Exception as e:
                print(f"[ERROR] Sensor read failed: {e}")
                # avoid busy-loop on continuous errors
                time.sleep(0.1)
                continue

            msg = {
                "type": "data",
                "ts": timestamp,
                "range_g": range_g,
                "x_raw": x_raw,
                "y_raw": y_raw,
                "z_raw": z_raw,
            }

            send_json(data_sock, (DATA_TARGET_IP, DATA_TARGET_PORT), msg)

        # basic pacing – not hard real-time, but sufficient for streaming
        time.sleep(period)

    print("[INFO] Data thread stopped.")


# ==============================
# Control thread: handle commands from PC
# ==============================

def handle_command(cmd: dict, sensor: ADXL355, ctrl_sock: socket.socket, addr: Tuple[str, int]) -> None:
    """
    Handle a single command dictionary.

    Supported commands (JSON from PC):

    {
      "type": "cmd",
      "cmd": "set_range",
      "range_g": 2 | 4 | 8
    }

    {
      "type": "cmd",
      "cmd": "set_rate",
      "hz": 100.0   # sampling rate in Hz
    }

    {
      "type": "cmd",
      "cmd": "set_streaming",
      "enabled": true | false
    }

    {
      "type": "cmd",
      "cmd": "get_status"
    }
    """
    global current_range_g, current_sample_rate_hz

    if cmd.get("type") != "cmd":
        return

    command = cmd.get("cmd")
    if not command:
        return

    if command == "set_range":
        range_g = int(cmd.get("range_g", 0))
        try:
            sensor.set_range(range_g)
        except ValueError as e:
            resp = {"type": "resp", "ok": False, "error": str(e)}
            send_json(ctrl_sock, addr, resp)
            return

        with config_lock:
            current_range_g = range_g

        resp = {"type": "resp", "ok": True, "msg": f"range set to ±{range_g} g"}
        send_json(ctrl_sock, addr, resp)

    elif command == "set_rate":
        hz = float(cmd.get("hz", 0.0))
        if hz <= 0 or hz > 2000.0:
            resp = {
                "type": "resp",
                "ok": False,
                "error": "hz must be between 0 and 2000"
            }
            send_json(ctrl_sock, addr, resp)
            return

        with config_lock:
            current_sample_rate_hz = hz

        resp = {"type": "resp", "ok": True, "msg": f"sample rate set to {hz} Hz"}
        send_json(ctrl_sock, addr, resp)

    elif command == "set_streaming":
        enabled = bool(cmd.get("enabled", True))
        if enabled:
            streaming_enabled.set()
            msg = "streaming enabled"
        else:
            streaming_enabled.clear()
            msg = "streaming disabled"

        resp = {"type": "resp", "ok": True, "msg": msg}
        send_json(ctrl_sock, addr, resp)

    elif command == "get_status":
        with config_lock:
            status = {
                "type": "status",
                "range_g": current_range_g,
                "sample_rate_hz": current_sample_rate_hz,
                "streaming": streaming_enabled.is_set(),
            }
        send_json(ctrl_sock, addr, status)

    else:
        resp = {"type": "resp", "ok": False, "error": f"unknown command: {command}"}
        send_json(ctrl_sock, addr, resp)


def control_loop(sensor: ADXL355, ctrl_sock: socket.socket) -> None:
    """Listen for control UDP packets and handle commands."""
    print(f"[INFO] Control thread listening on {CTRL_LISTEN_IP}:{CTRL_LISTEN_PORT}")

    # non-blocking shutdown via timeout
    ctrl_sock.settimeout(1.0)

    while not stop_event.is_set():
        try:
            data, addr = ctrl_sock.recvfrom(2048)
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[WARN] Control socket error: {e}")
            time.sleep(0.1)
            continue

        try:
            cmd = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError:
            print("[WARN] Received non-JSON control packet.")
            continue

        handle_command(cmd, sensor, ctrl_sock, addr)

    print("[INFO] Control thread stopped.")


# ==============================
# Main entry point
# ==============================

def main():
    global current_range_g, current_sample_rate_hz

    print("[INFO] Starting ADXL355 UDP sender...")

    # Initialize sensor
    sensor = ADXL355(bus=0, device=0, max_speed_hz=1_000_000)
    sensor.set_range(DEFAULT_RANGE_G)
    with config_lock:
        current_range_g = DEFAULT_RANGE_G
        current_sample_rate_hz = DEFAULT_SAMPLE_RATE_HZ

    # Show IDs once
    devid_ad, devid_mst, partid, revid = sensor.read_ids()
    print(f"[INFO] DEVID_AD=0x{devid_ad:02X}, DEVID_MST=0x{devid_mst:02X}, PARTID=0x{partid:02X}, REVID=0x{revid:02X}")
    print(f"[INFO] Initial range: ±{current_range_g} g, sample rate: {current_sample_rate_hz} Hz")
    print(f"[INFO] Sending data to {DATA_TARGET_IP}:{DATA_TARGET_PORT}")
    print(f"[INFO] Listening for control on {CTRL_LISTEN_IP}:{CTRL_LISTEN_PORT}")

    # UDP sockets
    data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctrl_sock.bind((CTRL_LISTEN_IP, CTRL_LISTEN_PORT))

    # Threads
    data_thread = threading.Thread(target=data_loop, args=(sensor, data_sock), daemon=True)
    ctrl_thread = threading.Thread(target=control_loop, args=(sensor, ctrl_sock), daemon=True)

    data_thread.start()
    ctrl_thread.start()

    try:
        # Main thread just waits for Ctrl+C or external stop
        while not stop_event.is_set():
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt received, shutting down...")
    finally:
        stop_event.set()
        data_thread.join(timeout=2.0)
        ctrl_thread.join(timeout=2.0)
        data_sock.close()
        ctrl_sock.close()
        sensor.close()
        print("[INFO] UDP sender stopped.")


if __name__ == "__main__":
    main()
