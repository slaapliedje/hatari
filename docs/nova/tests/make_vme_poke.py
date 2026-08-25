#!/usr/bin/env python3
"""Generate VMEPOKE.PRG - a minimal VME window poke test for hatari-et4000.

Hand-assembled 68000 TOS program (no toolchain needed). Dropped into an
AUTO\\ folder on a GEMDOS drive it runs in supervisor mode at boot and
performs a fixed sequence of byte/word/long reads and writes against the
VME A24 and A16 windows, then exits with Pterm0.

Against `--vme trace --trace vme` every access must appear in the trace
log and survive a read-back; against `--vme none` the machine must show
two bus-error bombs instead (the AUTO program dies, TOS carries on).

Usage: make_vme_poke.py megaste|tt <output.prg>

The A24/A16 window base differs per host machine (the VME addresses are
the same, the CPU windows are not):
  megaste : A24 window at 0x00A00000, A16 window at 0x00DF0000
  tt      : A24 window at 0xFEA00000, A16 window at 0xFEFF0000
Both reach VME A24 address 0xA00000 (the Nova/ET4000 framebuffer base
seen in docs/nova/atw800-2-megaste-probe.log) and VME A16 address 0x0000.
"""
import struct
import sys


def be16(v): return struct.pack('>H', v)
def be32(v): return struct.pack('>I', v)


def code(a24_base, a16_base):
    c = b''
    # word write / read at A24 base (framebuffer start)
    c += b'\x33\xFC' + be16(0x1234) + be32(a24_base)        # move.w #$1234,(a24).l
    c += b'\x30\x39' + be32(a24_base)                       # move.w (a24).l,d0
    # byte write / read at A24 base + 2
    c += b'\x13\xFC' + be16(0x0056) + be32(a24_base + 2)    # move.b #$56,(a24+2).l
    c += b'\x12\x39' + be32(a24_base + 2)                   # move.b (a24+2).l,d1
    # long write / read at A24 base + 4
    c += b'\x23\xFC' + be32(0xDEADBEEF) + be32(a24_base + 4)  # move.l #$deadbeef,(a24+4).l
    c += b'\x22\x39' + be32(a24_base + 4)                   # move.l (a24+4).l,d1
    # word write / read in the A16 window
    c += b'\x33\xFC' + be16(0x5AA5) + be32(a16_base)        # move.w #$5aa5,(a16).l
    c += b'\x30\x39' + be32(a16_base)                       # move.w (a16).l,d0
    # Pterm0
    c += b'\x42\x67'                                        # clr.w -(sp)
    c += b'\x4E\x41'                                        # trap #1
    return c


def prg(text):
    hdr = be16(0x601A)          # magic
    hdr += be32(len(text))      # text size
    hdr += be32(0)              # data size
    hdr += be32(0)              # bss size
    hdr += be32(0)              # symbol table size
    hdr += be32(0)              # reserved
    hdr += be32(0)              # prgflags
    hdr += be16(0)              # absflag
    return hdr + text + be32(0) # empty relocation table


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in ('megaste', 'tt'):
        sys.exit(__doc__)
    if sys.argv[1] == 'megaste':
        text = code(0x00A00000, 0x00DF0000)
    else:
        text = code(0xFEA00000, 0xFEFF0000)
    with open(sys.argv[2], 'wb') as f:
        f.write(prg(text))


if __name__ == '__main__':
    main()
