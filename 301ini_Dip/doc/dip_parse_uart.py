#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DIP binary UART frame parser (protocol v2).

Wire format: sync 0x55 0xAA, packed header 22B, int16 IQ by channel bitmap, CRC16-CCITT.
Must match firmware dip_crc16_ccitt() and dip_bin_build_frame() in src/main.c.

See doc/DIP_binary_protocol.md and doc/DIP_binary_pc_parser.md.
"""

from __future__ import annotations

import argparse
import struct
import sys
from typing import Dict, List, Optional, Tuple

try:
    import serial
except ImportError:
    print("Please install pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)

SYNC1 = 0x55
SYNC2 = 0xAA
HEADER_SIZE = 22
DIP_MAX_FFT_CHANNELS = 75


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def channels_from_bitmap(bitmap: bytes) -> List[int]:
    chs: List[int] = []
    for ch in range(DIP_MAX_FFT_CHANNELS):
        if bitmap[ch // 8] & (1 << (ch % 8)):
            chs.append(ch)
    return chs


def parse_frame(frame: bytes) -> Optional[Dict]:
    if len(frame) < HEADER_SIZE + 2:
        return None

    if frame[0] != SYNC1 or frame[1] != SYNC2:
        return None

    version, ftype = frame[2], frame[3]
    payload_len, pc = struct.unpack_from("<HH", frame, 4)
    ap, iq_format, ch_count, _reserved = struct.unpack_from("<BBBB", frame, 8)
    bitmap = frame[12:22]

    iq_len = payload_len - 16
    if iq_len < 0 or iq_len % 4 != 0:
        return None

    expected = HEADER_SIZE + iq_len + 2
    if len(frame) != expected:
        return None

    body = frame[: HEADER_SIZE + iq_len]
    crc_rx = struct.unpack_from("<H", frame, HEADER_SIZE + iq_len)[0]
    if crc16_ccitt(body) != crc_rx:
        return None

    chs = channels_from_bitmap(bitmap)
    if len(chs) != ch_count or ch_count * 4 != iq_len:
        return None

    iq_bytes = frame[HEADER_SIZE : HEADER_SIZE + iq_len]
    iq_map: Dict[int, Tuple[int, int]] = {}
    off = 0
    for ch in chs:
        i, q = struct.unpack_from("<hh", iq_bytes, off)
        off += 4
        iq_map[ch] = (i, q)

    return {
        "version": version,
        "type": ftype,
        "procedure_counter": pc,
        "ap": ap,
        "iq_format": iq_format,
        "channel_count": ch_count,
        "channels": chs,
        "iq": iq_map,
        "raw": frame,
    }


class FrameReader:
    """Byte stream → complete frames (sync hunt)."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> List[bytes]:
        self._buf.extend(data)
        frames: List[bytes] = []

        while True:
            # Hunt sync
            idx = -1
            for i in range(len(self._buf) - 1):
                if self._buf[i] == SYNC1 and self._buf[i + 1] == SYNC2:
                    idx = i
                    break
            if idx < 0:
                if len(self._buf) > 1:
                    self._buf = self._buf[-1:]
                break
            if idx > 0:
                del self._buf[:idx]

            if len(self._buf) < 6:
                break

            payload_len = struct.unpack_from("<H", self._buf, 4)[0]
            iq_len = payload_len - 16
            if iq_len < 0 or iq_len > 300 or iq_len % 4 != 0:
                del self._buf[0]
                continue

            total = HEADER_SIZE + iq_len + 2
            if len(self._buf) < total:
                break

            frame = bytes(self._buf[:total])
            parsed = parse_frame(frame)
            if parsed is not None:
                frames.append(frame)
                del self._buf[:total]
            else:
                del self._buf[0]

        return frames


def main() -> int:
    parser = argparse.ArgumentParser(description="Parse DIP binary UART frames")
    parser.add_argument("port", help="Serial port, e.g. COM3 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--dump-hex", action="store_true", help="Print hex per frame")
    parser.add_argument("--csv", metavar="FILE", help="Append pc,ch,i,q rows to CSV")
    args = parser.parse_args()

    reader = FrameReader()
    csv_file = open(args.csv, "a", encoding="utf-8") if args.csv else None
    if csv_file and csv_file.tell() == 0:
        csv_file.write("procedure_counter,ch,i,q\n")

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        print(f"Listening on {args.port} @ {args.baud} … Ctrl+C to stop")
        n = 0
        try:
            while True:
                chunk = ser.read(4096)
                if not chunk:
                    continue
                for raw in reader.feed(chunk):
                    info = parse_frame(raw)
                    if info is None:
                        continue
                    n += 1
                    pc = info["procedure_counter"]
                    chs = info["channels"]
                    if chs:
                        print(
                            f"[{n}] pc={pc} ap={info['ap']} "
                            f"N={info['channel_count']} ch=[{chs[0]}..{chs[-1]}]"
                        )
                    else:
                        print(f"[{n}] pc={pc} ap={info['ap']} N=0")
                    if args.dump_hex:
                        print(raw.hex())
                    if csv_file:
                        for ch, (i, q) in info["iq"].items():
                            csv_file.write(f"{pc},{ch},{i},{q}\n")
                        csv_file.flush()
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            if csv_file:
                csv_file.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
