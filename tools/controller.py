#!/usr/bin/env python3
"""xbox 360 -> esp32 serial

protocol: C<steer> <throttle> <mode>
"""

import argparse
import struct
import sys
import threading
import time

import serial
import usb.core
import usb.util

XBOX_VID = 0x045E
XBOX_PID = 0x028E
XBOX_EP_IN = 0x81

BTN_A = 0x1000
BTN_B = 0x2000
BTN_X = 0x4000
BTN_Y = 0x8000
BTN_START = 0x0010
BTN_BACK = 0x0020

STICK_MAX = 32767.0
TRIGGER_MAX = 255.0
DEADZONE = 0.08


def normalize_stick(value):
    norm = value / STICK_MAX
    if abs(norm) < DEADZONE:
        return 0.0
    return max(-1.0, min(1.0, norm))


def normalize_trigger(value):
    return max(0.0, min(1.0, value / TRIGGER_MAX))


def serial_reader(ser):
    while True:
        try:
            line = ser.readline()
            if line:
                text = line.decode("utf-8", errors="replace").rstrip()
                if text:
                    print(f"[ESP32] {text}")
        except Exception:
            break


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=int, default=20, help="Command send rate (Hz)")
    args = parser.parse_args()

    dev = usb.core.find(idVendor=XBOX_VID, idProduct=XBOX_PID)
    if dev is None:
        print("Xbox 360 controller not found. Is it attached via usbipd?")
        sys.exit(1)

    print(f"Found: {dev.manufacturer} {dev.product} (serial: {dev.serial_number})")

    dev.set_configuration()
    if dev.is_kernel_driver_active(0):
        dev.detach_kernel_driver(0)
    usb.util.claim_interface(dev, 0)

    ser = serial.Serial(args.port, args.baud, timeout=0.1)

    # esp32 log thread
    reader = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
    reader.start()

    steer = 0.0
    throttle_fwd = 0.0
    throttle_rev = 0.0
    mode = 0
    prev_btns = 0
    last_btn_time = 0
    DEBOUNCE_S = 0.25

    print(f"\nSerial: {args.port} @ {args.baud}")
    print("\nControls:")
    print("  Left stick   : steering")
    print("  Right trigger : forward throttle")
    print("  Left trigger  : reverse throttle")
    print("  A button      : toggle ACTIVE/PASSIVE mode")
    print("  B button      : EMERGENCY STOP")
    print("  Ctrl+C        : quit\n")

    period = 1.0 / args.rate

    try:
        while True:
                    # read controller
            try:
                data = dev.read(XBOX_EP_IN, 32, timeout=int(period * 1000))
            except usb.core.USBTimeoutError:
                # no data, resend
                pass
            else:
                if len(data) >= 14 and data[0] == 0x00 and data[1] == 0x14:
                    btns = struct.unpack_from("<H", data, 2)[0]
                    lt = data[4]
                    rt = data[5]
                    lx = struct.unpack_from("<h", data, 6)[0]

                    steer = normalize_stick(lx)
                    throttle_fwd = normalize_trigger(rt)
                    throttle_rev = normalize_trigger(lt)

                    # button edges
                    pressed = btns & ~prev_btns
                    now = time.monotonic()
                    if pressed & BTN_A and (now - last_btn_time) > DEBOUNCE_S:
                        mode = 1 - mode
                        print(f">> Mode: {'ACTIVE' if mode else 'PASSIVE'}")
                        last_btn_time = now
                    if pressed & BTN_B and (now - last_btn_time) > DEBOUNCE_S:
                        steer = 0.0
                        throttle_fwd = 0.0
                        throttle_rev = 0.0
                        mode = 0
                        print(">> EMERGENCY STOP")
                        last_btn_time = now
                    prev_btns = btns

        # combine triggers
            throttle = throttle_fwd - throttle_rev

        # send cmd
            cmd = f"C {steer:.3f} {throttle:.3f} {mode}\n"
            ser.write(cmd.encode())

            time.sleep(period)

    except KeyboardInterrupt:
        ser.write(b"C 0.000 0.000 0\n")
        print("\nStopped. Sent neutral command.")
    finally:
        usb.util.release_interface(dev, 0)
        ser.close()


if __name__ == "__main__":
    main()
