from __future__ import annotations

import argparse
import sys
import time

from demo_packet import (
    DEVICE_PACKET,
    USB_DEVICE_MAGIC,
    USB_HOST_MAGIC,
    build_host_packet,
    parse_device_packet,
)


def run(port: str, count: int, timeout: float, bad_checksum: bool) -> int:
    import serial

    with serial.Serial(port=port, baudrate=115200, timeout=timeout, write_timeout=timeout) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        if bad_checksum:
            request = bytearray(build_host_packet(USB_HOST_MAGIC, 0xBAD, 1.0, True, 1))
            request[-1] ^= 0x5A
            ser.write(request)
            ser.flush()
            response = ser.read(DEVICE_PACKET.size)
            if response:
                raise RuntimeError("device returned a normal response for a bad checksum packet")
            print("PASS USB bad-checksum packet was rejected")

        for seq in range(1, count + 1):
            value = 20.0 + seq * 0.25
            flag = (seq % 2) == 0
            request = build_host_packet(USB_HOST_MAGIC, seq, value, flag, seq * 5)
            ser.write(request)
            ser.flush()
            response = ser.read(DEVICE_PACKET.size)
            packet = parse_device_packet(response, USB_DEVICE_MAGIC, seq)
            if packet.status != 0 or not packet.ready:
                raise RuntimeError(f"device returned status={packet.status}, ready={packet.ready}")
            if packet.flag != flag or abs(packet.value - value) > 0.001:
                raise RuntimeError(f"payload mismatch at seq={seq}")
            print(f"PASS USB seq={seq} rx={packet.rx_count} tx={packet.tx_count} err={packet.error_count}")
            time.sleep(0.02)
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Run embedded_framework USB CDC demo test.")
    parser.add_argument("--port", required=True, help="USB CDC serial port, for example COM9 or /dev/ttyACM0")
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--bad-checksum", action="store_true", help="Send one corrupted packet before normal tests")
    args = parser.parse_args(argv)
    return run(args.port, args.count, args.timeout, args.bad_checksum)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
