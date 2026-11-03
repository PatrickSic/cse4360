#!/usr/bin/env python3
"""mkrbf.py - create a LEGO EV3 .rbf launcher that runs a shell command.

Usage:  python3 mkrbf.py "<command>" out.rbf
e.g.    python3 mkrbf.py "/media/card/myapps/proj1.elf" proj1.rbf

Byte-for-byte the same format ev3duder's `mkrbf` produces: the LEGO VM
program consists of a single opSYSTEM instruction that hands the string to
the brick's root shell.  (See ev3duder/src/mkrbf.c.)
"""
import struct
import sys


def mkrbf(cmd: str) -> bytes:
    before = (b"\x68\x00\x01\x00\x00\x00\x00\x00\x1C\x00\x00\x00"
              b"\x00\x00\x00\x00\x08\x00\x00\x00\x60\x80")
    after = b"\x44\x85\x82\xE8\x03\x40\x86\x40\x0A"
    body = before + cmd.encode() + b"\x00" + after
    total = 4 + 4 + len(body)
    return b"LEGO" + struct.pack("<I", total) + body


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    with open(sys.argv[2], "wb") as f:
        f.write(mkrbf(sys.argv[1]))
    print(f"wrote {sys.argv[2]} -> runs: {sys.argv[1]}")
