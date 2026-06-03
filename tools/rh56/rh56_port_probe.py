#!/usr/bin/env python3
import time
import serial


def checksum(data: bytes) -> int:
    return sum(data[2:-1]) & 0xFF


def probe(port: str, hand_id: int) -> None:
    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=8,
            stopbits=1,
            parity="N",
            timeout=0.02,
        )
    except Exception as exc:
        print(port, hand_id, "OPEN_ERR", type(exc).__name__, exc, flush=True)
        return

    cmd = bytearray([0xEB, 0x90, hand_id, 0x04, 0x11, 0x0A, 0x06, 0x0C, 0x00])
    cmd[-1] = sum(cmd[2:-1]) & 0xFF

    try:
        ser.reset_input_buffer()
        ser.write(cmd)
        time.sleep(0.01)
        data = ser.read(64)
        print(port, hand_id, "LEN", len(data), "HEX", data.hex(), flush=True)
    except Exception as exc:
        print(port, hand_id, "READ_ERR", type(exc).__name__, exc, flush=True)
    finally:
        try:
            ser.close()
        except Exception:
            pass


def main() -> int:
    for port in ("/dev/ttyUSB0", "/dev/ttyUSB1"):
        for hand_id in (1, 2):
            probe(port, hand_id)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
