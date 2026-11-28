"""Create the shared tileable linear mask for the preview materials."""
import argparse
from pathlib import Path
import struct
import zlib

import numpy as np


SIZE = 1024
RNG = np.random.default_rng(528026)


def periodic_field(columns, rows):
    lattice = RNG.standard_normal((rows, columns)).astype(np.float32)
    x = np.arange(SIZE, dtype=np.float32) * (columns / SIZE)
    y = np.arange(SIZE, dtype=np.float32) * (rows / SIZE)
    ix, iy = x.astype(int), y.astype(int)
    u, v = x - ix, y - iy
    u, v = u * u * (3 - 2 * u), v * v * (3 - 2 * v)
    a = lattice[np.ix_(iy, ix)]
    b = lattice[np.ix_(iy, (ix + 1) % columns)]
    c = lattice[np.ix_((iy + 1) % rows, ix)]
    d = lattice[np.ix_((iy + 1) % rows, (ix + 1) % columns)]
    return ((1 - v[:, None]) * (a + (b - a) * u) + v[:, None] * (c + (d - c) * u))


def chunk(kind, payload):
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload))


def build(out):
    # Red: shallow low spots. Green: grain. Blue: uneven thin-film coverage.
    pools = (0.52 + 0.145 * periodic_field(3, 4) + 0.095 * periodic_field(5, 9)
             + 0.050 * periodic_field(16, 15) + 0.018 * periodic_field(47, 53))
    grain = (0.52 + 0.145 * periodic_field(251, 233) + 0.155 * RNG.standard_normal((SIZE, SIZE))
             + 0.072 * periodic_field(73, 99))
    film = (0.54 + 0.145 * periodic_field(7, 9) + 0.095 * periodic_field(24, 16)
            + 0.04 * periodic_field(80, 73))
    mask = np.rint(np.clip(np.stack((pools, grain, film), axis=-1), 0.05, 0.95) * 255).astype(np.uint8)
    header = struct.pack(">IIBBBBB", SIZE, SIZE, 8, 2, 0, 0, 0)
    data = zlib.compress(b"".join(b"\0" + row.tobytes() for row in mask), 6)
    out.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", data) + chunk(b"IEND", b""))
    print(out, SIZE, SIZE, out.stat().st_size)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    build(parser.parse_args().output)
