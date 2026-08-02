#!/usr/bin/env python3
"""Convert the TrueColor XWD files captured by scripts/capture_viewers.sh to PNG."""

import struct
import sys
from pathlib import Path

from PIL import Image


source, target = map(Path, sys.argv[1:3])
data = source.read_bytes()
header = struct.unpack(">25I", data[:100])
(header_size, version, _, _, width, height, _, byte_order, _, _, _, bits,
 stride, _, red_mask, green_mask, blue_mask, _, _, colors, *_) = header
if version != 7 or bits != 32:
    raise SystemExit("仅支持本教材 X11 环境生成的 32-bit XWD")
offset = header_size + 12 * colors
pixels = data[offset:offset + stride * height]
order = "little" if byte_order == 0 else "big"

def shift(mask: int) -> int:
    return (mask & -mask).bit_length() - 1

shifts = [shift(red_mask), shift(green_mask), shift(blue_mask)]
rgb = bytearray(3 * width * height)
for y in range(height):
    row = pixels[y * stride:(y + 1) * stride]
    for x in range(width):
        value = int.from_bytes(row[4*x:4*x+4], order)
        for channel, (mask, bit) in enumerate(zip(
                (red_mask, green_mask, blue_mask), shifts)):
            rgb[3*(y*width+x)+channel] = (value & mask) >> bit

target.parent.mkdir(parents=True, exist_ok=True)
Image.frombytes("RGB", (width, height), bytes(rgb)).save(target, optimize=True)
