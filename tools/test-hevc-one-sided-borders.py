#!/usr/bin/env python3
"""Exercise the HEVC live border measurement on synthetic 8-bit tap surfaces."""

import ctypes
from pathlib import Path
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    driver = (root / "drivers/frame_provider/decoder/h265/vh265.c").read_text()
    start = driver.index("#define P3I_BORDER_BLACK_MAX")
    end = driver.index("/* Match Kodi's common ratios", start)
    helpers = driver[start:end]
    wrapper = """
#include <stdbool.h>
#include <stdlib.h>
typedef unsigned char u8;
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
""" + helpers + """
bool measure(const u8 *luma, int w, int h, int stride, unsigned int *border)
{
    const struct p3i_border_edge edges[4] = {
        {0, stride, 1, w, h}, {(h - 1) * stride, -stride, 1, w, h},
        {0, 1, stride, h, w}, {w - 1, -1, stride, h, w}
    };
    return p3i_measure_edges(luma, edges, border);
}
"""
    with tempfile.TemporaryDirectory() as tmp:
        lib = Path(tmp) / "edge.so"
        subprocess.run(["cc", "-std=c99", "-O2", "-shared", "-fPIC", "-x", "c", "-",
                        "-o", lib], input=wrapper, text=True, check=True)
        measure = ctypes.CDLL(str(lib)).measure
        measure.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                            ctypes.POINTER(ctypes.c_uint)]
        measure.restype = ctypes.c_bool
        run_cases(measure)


def run_cases(measure):
    width, height, stride = 160, 96, 192

    def check(label, bounds, side=0, overlay=None, expected=True):
        top, bottom, left, right = bounds
        image = bytearray([255]) * (stride * height)
        for y in range(height):
            for x in range(width):
                active = top <= y < height - bottom and left <= x < width - right
                luma = 90 if active else 16
                # Coordinates along and inward from the selected edge.
                along = x if side < 2 else y
                depth = (y, height - 1 - y, x, width - 1 - x)[side]
                if overlay:
                    luma = overlay(along, depth, luma)
                image[y * stride + x] = luma
        offsets = (ctypes.c_uint * 4)()
        valid = measure(bytes(image), width, height, stride, offsets)
        assert valid == expected, (label, valid, tuple(offsets))
        if expected:
            assert tuple(offsets) == tuple(bounds), (label, tuple(offsets), bounds)

    check("full frame", [0, 0, 0, 0])
    check("unequal paired mattes", [10, 17, 8, 13])
    check("one tap of difference", [7, 8, 0, 0])
    for side in range(4):
        span = width if side < 2 else height
        bounds = [0] * 4
        bounds[side] = 7
        check("one-sided matte", bounds, side)
        for opposite in (0, 1):
            bounds[side ^ 1] = opposite
            for far_end in (False, True):
                def jersey(along, depth, luma):
                    pos = span - 1 - along if far_end else along
                    return 44 if pos < span // 4 and depth < 7 else luma
                check("picture inside band", bounds, side, jersey, False)
        bounds[side ^ 1] = 0

        def middle_only(along, depth, luma):
            if span // 4 <= along < 3 * span // 4 or depth > 10:
                return luma
            return 16
        check("boundary visible only in center", bounds, side, middle_only, False)
        check("isolated bright speck", bounds, side,
              lambda p, d, luma: 60 if p == span // 10 and d == 1 else luma)

        # The outer probes must fall inside the perpendicular mattes.
        across = 2 if side < 2 else 0
        bounds[across] = bounds[across + 1] = span // 5
        check("genuine one-sided matte with perpendicular mattes", bounds, side)


if __name__ == "__main__":
    main()
