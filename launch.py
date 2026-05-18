#!/usr/bin/env python3
"""launch rc suspension system

starts: relay, dashboard, controller
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

BASE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(BASE, "tools")
DASHBOARD = os.path.join(BASE, "dashboard")

SERIAL_PORT = "/dev/ttyUSB0"
BAUD = 115200
WS_PORT = 8765
DASH_PORT = 8050

children = []


def log(tag, msg):
    ts = time.strftime("%H:%M:%S")
    color = {"SYS": "\033[36m", "OK": "\033[32m", "ERR": "\033[31m",
             "WAIT": "\033[33m", "INFO": "\033[37m"}.get(tag, "\033[0m")
    print(f"{color}[{ts}] [{tag}] {msg}\033[0m", flush=True)


def cleanup(*_):
    log("SYS", "Shutting down all subsystems...")
    for p in reversed(children):
        try:
            p.terminate()
            p.wait(timeout=3)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
    log("SYS", "All subsystems stopped.")
    sys.exit(0)


def kill_stale():
    stale = ["telem_relay", "servo_joy", "dashboard/app.py"]
    for name in stale:
        subprocess.run(["sudo", "pkill", "-9", "-f", name],
                       capture_output=True)
    time.sleep(0.5)


def ensure_serial():
    # load drivers
    for mod in ["usbserial", "cp210x"]:
        subprocess.run(["sudo", "modprobe", mod], capture_output=True)

    log("WAIT", f"Waiting for {SERIAL_PORT}...")
    for i in range(30):
        if os.path.exists(SERIAL_PORT):
            subprocess.run(["sudo", "chmod", "666", SERIAL_PORT],
                           capture_output=True)
            log("OK", f"{SERIAL_PORT} ready")
            return True
        time.sleep(1)
        if i == 5:
            log("INFO", "hint: usbipd attach --wsl --busid <id>")

    log("ERR", f"{SERIAL_PORT} not found after 30s")
    return False


def verify_telemetry():
    import serial as ser_mod
    try:
        s = ser_mod.Serial(SERIAL_PORT, BAUD, timeout=1)
        time.sleep(1)
        s.read(s.in_waiting)  # drain
        time.sleep(0.3)
        data = s.read(s.in_waiting or 0).decode(errors="replace")
        s.close()
        if "T{" in data:
            log("OK", "ESP32 telemetry confirmed")
            return True
        else:
            log("ERR", "esp32 not streaming telemetry")
            return False
    except Exception as e:
        log("ERR", f"Serial check failed: {e}")
        return False


def start_relay():
    log("SYS", "Starting telemetry relay...")
    p = subprocess.Popen(
        ["sudo", sys.executable, os.path.join(TOOLS, "telem_relay.py"),
         "--port", SERIAL_PORT, "--ws-port", str(WS_PORT)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    children.append(p)
    time.sleep(1.5)
    if p.poll() is not None:
        out = p.stdout.read().decode(errors="replace")
        log("ERR", f"Relay crashed: {out[:200]}")
        return False
    log("OK", f"Telemetry relay on ws://0.0.0.0:{WS_PORT}")
    return True


def start_dashboard():
    log("SYS", "Starting dashboard...")
    dash_env = os.environ.copy()
    # pip paths
    user_site = subprocess.check_output(
        ["python3", "-c", "import site; print(site.getusersitepackages())"],
        text=True).strip()
    dash_env["PYTHONPATH"] = user_site + ":" + dash_env.get("PYTHONPATH", "")
    p = subprocess.Popen(
        [sys.executable, os.path.join(DASHBOARD, "app.py"),
         "--ws", f"ws://localhost:{WS_PORT}", "--port", str(DASH_PORT)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        env=dash_env,
    )
    children.append(p)
    time.sleep(2)
    if p.poll() is not None:
        out = p.stdout.read().decode(errors="replace")
        log("ERR", f"Dashboard crashed: {out[:200]}")
        return False
    log("OK", f"Dashboard at http://localhost:{DASH_PORT}")
    return True


def start_controller():
    log("SYS", "Starting Xbox controller...")
    p = subprocess.Popen(
        ["sudo", sys.executable, os.path.join(TOOLS, "servo_joy.py"),
         "--ws", f"ws://localhost:{WS_PORT}"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    children.append(p)
    time.sleep(2)
    if p.poll() is not None:
        out = p.stdout.read().decode(errors="replace")
        log("ERR", f"Controller crashed: {out[:200]}")
        return False
    log("OK", "Xbox controller active")
    return True


def health_monitor():
    reported = set()
    while True:
        for p in children:
            if p.poll() is not None and p.pid not in reported:
                reported.add(p.pid)
                out = ""
                try:
                    out = p.stdout.read(500).decode(errors="replace")
                except Exception:
                    pass
                log("ERR", f"Process {p.pid} exited with code {p.returncode}")
                if out.strip():
                    for line in out.strip().split("\n")[-3:]:
                        log("ERR", f"  {line.strip()}")
        time.sleep(5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", action="store_true",
                        help="Also start Xbox controller bridge")
    parser.add_argument("--dashboard-only", action="store_true",
                        help="Only start the dashboard (relay already running)")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print()
    log("SYS", "╔══════════════════════════════════════╗")
    log("SYS", "║   RC SUSPENSION SYSTEM LAUNCHER      ║")
    log("SYS", "╚══════════════════════════════════════╝")
    print()

    # clean slate
    kill_stale()

    if not args.dashboard_only:
        # hardware
        if not ensure_serial():
            sys.exit(1)

        if not verify_telemetry():
            sys.exit(1)

        # relay
        if not start_relay():
            cleanup()

    # dashboard
    if not start_dashboard():
        cleanup()

    # controller
    if args.controller:
        if not start_controller():
            log("ERR", "Controller failed — dashboard still running")

    print()
    log("OK", "═══ ALL SYSTEMS GO ═══")
    log("INFO", f"Dashboard:  http://localhost:{DASH_PORT}")
    log("INFO", f"Telemetry:  ws://localhost:{WS_PORT}")
    if args.controller:
        log("INFO", "Controller: Xbox 360 active")
    log("INFO", "Press Ctrl+C to stop all")
    print()

    try:
        health_monitor()
    except KeyboardInterrupt:
        cleanup()


if __name__ == "__main__":
    main()
