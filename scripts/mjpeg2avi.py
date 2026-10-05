#!/usr/bin/env python3
"""Remux a raw concatenated-JPEG stream (v4l2-ctl --stream-to) into an MJPG AVI.

No re-encode: every JPEG is copied byte-for-byte into an AVI 1.0 container,
which MATLAB's VideoReader reads on every platform. A truncated last frame is
dropped. AVI 1.0 caps the file at 1 GB.

Usage: python3 scripts/mjpeg2avi.py in.mjpeg out.avi [--fps 60] [--size 1920 1200]
"""
import argparse
import struct


def chunk(tag, data):
    return tag + struct.pack('<I', len(data)) + data + (b'\0' if len(data) % 2 else b'')


def lst(tag, data):
    return b'LIST' + struct.pack('<I', len(data) + 4) + tag + data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--fps', type=int, default=60)
    ap.add_argument('--size', nargs=2, type=int, default=[1920, 1200])
    args = ap.parse_args()
    fps, (W, H) = args.fps, args.size

    d = open(args.src, 'rb').read()
    frames, i = [], 0
    while True:
        s = d.find(b'\xff\xd8\xff', i)
        if s < 0:
            break
        e = d.find(b'\xff\xd8\xff', s + 3)
        end = len(d) if e < 0 else e
        j = d.rfind(b'\xff\xd9', s, end)
        if j > 0:
            frames.append(d[s:j + 2])
        i = end
    n, maxsz = len(frames), max(map(len, frames))

    avih = struct.pack('<14I', 1000000 // fps, maxsz * fps, 0, 0x10, n, 0, 1, maxsz,
                       W, H, 0, 0, 0, 0)
    strh = b'vids' + b'MJPG' + struct.pack('<IHHIIIIIIIIhhhh', 0, 0, 0, 0, 1, fps, 0, n,
                                           maxsz, 0xFFFFFFFF, 0, 0, 0, W, H)
    strf = struct.pack('<IiiHHIIiiII', 40, W, H, 1, 24, struct.unpack('<I', b'MJPG')[0],
                       W * H * 3, 0, 0, 0, 0)
    hdrl = lst(b'hdrl', chunk(b'avih', avih) + lst(b'strl', chunk(b'strh', strh) +
                                                     chunk(b'strf', strf)))
    movi, idx, off = bytearray(), bytearray(), 4
    for f in frames:
        c = chunk(b'00dc', f)
        idx += b'00dc' + struct.pack('<III', 0x10, off, len(f))
        movi += c
        off += len(c)
    body = b'AVI ' + hdrl + lst(b'movi', bytes(movi)) + chunk(b'idx1', bytes(idx))
    with open(args.dst, 'wb') as o:
        o.write(b'RIFF' + struct.pack('<I', len(body)) + body)
    print(n, 'frames ->', args.dst)


if __name__ == '__main__':
    main()
