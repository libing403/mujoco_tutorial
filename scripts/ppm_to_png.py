#!/usr/bin/env python3
"""Convert a binary PPM image to PNG using only Python's standard library."""

import struct
import sys
import zlib
from pathlib import Path


def chunk(kind: bytes, data: bytes) -> bytes:
    body = kind + data
    return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))


source, target = map(Path, sys.argv[1:3])
with source.open("rb") as file:
    if file.readline().strip() != b"P6":
        raise SystemExit("只支持二进制 P6 PPM")
    width, height = map(int, file.readline().split())
    if file.readline().strip() != b"255":
        raise SystemExit("只支持 8-bit PPM")
    pixels = file.read()

stride = 3 * width
scanlines = b"".join(b"\0" + pixels[y:y + stride]
                     for y in range(0, len(pixels), stride))
png = b"\x89PNG\r\n\x1a\n"
png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
png += chunk(b"IDAT", zlib.compress(scanlines, 9))
png += chunk(b"IEND", b"")
target.parent.mkdir(parents=True, exist_ok=True)
target.write_bytes(png)
