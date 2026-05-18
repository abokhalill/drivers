#!/usr/bin/env python3
"""xbox -> esp32 via serial or ws

sends: S<steer> M<motor> A<sus> B<mode>
"""

import argparse
import struct
import sys
import time

import usb.core
import usb.util

XBOX_VID = 0x045E
XBOX_PID = 0x028E
XBOX_EP_IN = 0x81
DEADZONE = 0.08
STICK_MAX = 32767.0
BTN_A = 0x1000
BTN_B = 0x2000


def norm(value):
    n = value / STICK_MAX
    if abs(n) < DEADZONE:
        return 0.0
    return max(-1.0, min(1.0, n))


class SerialSender:
    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0)

    def write(self, data):
        self.ser.write(data)

    def close(self):
        self.ser.write(b"S0.000\nM0.000\n")
        self.ser.close()


class WebSocketSender:
    def __init__(self, url):
        import websocket
        self.ws = websocket.WebSocket()
        self.ws.connect(url)

    def write(self, data):
        if isinstance(data, bytes):
            data = data.decode()
        self.ws.send(data)

    def close(self):
        self.ws.send("S0.000\nM0.000\n")
        self.ws.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--ws", default=None,
                        help="WebSocket relay URL (e.g. ws://localhost:8765)")
    args = parser.parse_args()

    dev = usb.core.find(idVendor=XBOX_VID, idProduct=XBOX_PID)
    if dev is None:
        print("Xbox controller not found")
        sys.exit(1)

    for cfg in dev:
        for intf in cfg:
            if dev.is_kernel_driver_active(intf.bInterfaceNumber):
                dev.detach_kernel_driver(intf.bInterfaceNumber)
    try:
        dev.set_configuration()
    except usb.core.USBError:
        pass  # already configured
    usb.util.claim_interface(dev, 0)

    # Choose transport
    if args.ws:
        sender = WebSocketSender(args.ws)
        print(f"Connected via WebSocket: {args.ws}")
    else:
        sender = SerialSender(args.port, args.baud)
        print(f"Connected via serial: {args.port}")

    steer_val = 0.0
    motor_val = 0.0
    prev_steer = None
    prev_motor = None
    prev_btns = 0
    last_send = 0.0

    print("LStickX=steer | RStickY=motor | A=sus(manual) | B=active")
    print("Ctrl+C to stop\n")

    try:
        while True:
            try:
                data = dev.read(XBOX_EP_IN, 32, timeout=4)
            except usb.core.USBTimeoutError:
                pass
            else:
                if len(data) >= 14 and data[0] == 0x00 and data[1] == 0x14:
                    btns = struct.unpack_from("<H", data, 2)[0]
                    lx = struct.unpack_from("<h", data, 6)[0]
                    ry = struct.unpack_from("<h", data, 12)[0]
                    steer_val = norm(lx)
                    motor_val = norm(ry)

                    pressed = btns & ~prev_btns
                    if pressed & BTN_A:
                        sender.write(b"A\n")
                    if pressed & BTN_B:
                        sender.write(b"B\n")
                    prev_btns = btns

            # Send at most 50Hz, and only when values change
            now = time.time()
            if now - last_send < 0.02:
                continue
            s_str = f"{steer_val:.3f}"
            m_str = f"{motor_val:.3f}"
            if s_str != prev_steer or m_str != prev_motor:
                sender.write(f"S{s_str}\n".encode())
                time.sleep(0.002)
                sender.write(f"M{m_str}\n".encode())
                prev_steer = s_str
                prev_motor = m_str
                last_send = now
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        usb.util.release_interface(dev, 0)
        sender.close()


if __name__ == "__main__":
    main()
