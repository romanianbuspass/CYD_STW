#!/usr/bin/env python3
"""Reset the CYD via RTS (EN) and capture the boot log, surviving USB re-enumeration."""
import glob
import sys
import time

import serial

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 20


def newest_port():
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ports[-1] if ports else None


def open_port(port):
    return serial.Serial(port, 115200, timeout=0.2)


def main():
    deadline = time.time() + SECONDS
    ser = None
    # attach, then pulse RTS to reset (RTS=True -> EN low, as esptool does)
    while time.time() < deadline and ser is None:
        port = newest_port()
        if port:
            try:
                ser = open_port(port)
                ser.setRTS(True)
                time.sleep(0.15)
                ser.setRTS(False)
                print(f"[reset pulsed on {port}]", flush=True)
            except (serial.SerialException, OSError):
                ser = None
                time.sleep(0.3)
        else:
            time.sleep(0.3)

    while time.time() < deadline:
        try:
            chunk = ser.read(4096)
            if chunk:
                sys.stdout.buffer.write(chunk)
                sys.stdout.buffer.flush()
        except (serial.SerialException, OSError):
            ser = None
            while time.time() < deadline and ser is None:
                port = newest_port()
                if port:
                    try:
                        ser = open_port(port)
                    except (serial.SerialException, OSError):
                        ser = None
                        time.sleep(0.3)
                else:
                    time.sleep(0.3)


if __name__ == "__main__":
    main()
