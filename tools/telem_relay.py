#!/usr/bin/env python3
"""serial -> ws relay

reads T{json} from esp32, broadcasts to ws clients
"""

import argparse
import asyncio
import json
import sys
import threading

import serial
import websockets

clients = set()
latest = {}

ser_port = None  # shared serial handle
ser_lock = threading.Lock()


def serial_reader(port, baud):
    global latest, ser_port
    ser_port = serial.Serial(port, baud, timeout=0.1)
    buf = ""
    while True:
        try:
            with ser_lock:
                waiting = ser_port.in_waiting
                raw = ser_port.read(waiting or 1) if waiting else ser_port.read(1)
            if not raw:
                continue
            buf += raw.decode(errors="replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line.startswith("T{"):
                    try:
                        latest = json.loads(line[1:])
                    except json.JSONDecodeError:
                        pass
        except Exception as e:
            print(f"serial error: {e}", file=sys.stderr)


def serial_write(data):
    if ser_port and ser_port.is_open:
        try:
            with ser_lock:
                ser_port.write(data)
        except Exception:
            pass


async def broadcast():
    global clients, latest
    while True:
        if clients and latest:
            msg = json.dumps(latest)
            dead = set()
            for ws in clients:
                try:
                    await ws.send(msg)
                except websockets.exceptions.ConnectionClosed:
                    dead.add(ws)
            clients -= dead
        await asyncio.sleep(0.05)


async def handler(ws):
    global clients
    clients.add(ws)
    addr = ws.remote_address
    print(f"[+] client connected: {addr}")
    try:
        async for msg in ws:
            # forward S/M/A/B to esp32
            if msg and msg[0] in "SMAB":
                serial_write(msg.encode() if isinstance(msg, str) else msg)
    finally:
        clients.discard(ws)
        print(f"[-] client disconnected: {addr}")


async def main_async(ws_port):
    async with websockets.serve(handler, "0.0.0.0", ws_port):
        print(f"WebSocket relay on ws://0.0.0.0:{ws_port}")
        await broadcast()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--ws-port", type=int, default=8765)
    args = parser.parse_args()

    # start serial reader in background thread
    t = threading.Thread(target=serial_reader, args=(args.port, args.baud), daemon=True)
    t.start()
    print(f"Serial: {args.port} @ {args.baud}")

    asyncio.run(main_async(args.ws_port))


if __name__ == "__main__":
    main()
