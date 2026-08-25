# Provenance of imported / derived code

Hatari is GPL-2.0-or-later; everything recorded here is licence-compatible.
Per-fork rule (CLAUDE.md): every import records source project, file,
version, what was taken and what was rewritten — in the same commit that
brings the code in.

## src/video_et4000.c

Derived from **PCem** (GPL-2.0-or-later, https://github.com/sarah-walker-pcem/pcem,
local checkout `~/dev/pcem`, checkout state of 2026-08-24):

- `src/video/vid_svga.c` (c) Sarah Walker — the SVGA register state
  machine: `svga_out`/`svga_in` port dispatch (attribute controller
  flip-flop, sequencer, CRTC protection, GDC, DAC state machine),
  `svga_write_linear`/`svga_read_linear` (the four VGA write modes, read
  modes 0/1, plane latches, chain4/odd-even addressing, the
  `svga_rotate` table), and the vertical-geometry derivation from
  `svga_recalctimings`.
- `src/video/vid_et4000.c` (c) Sarah Walker — ET4000AX specifics: the
  banking register 0x3CD, extended CRTC registers (0x33/0x35/0x36 and
  the extension bits folded into `ma_latch`/`dispend`), TS register 7
  read behaviour, "packed" chain4 addressing (`packed_chain4`), CRTC
  register masks concept.
- `src/video/vid_unk_ramdac.c` (c) Sarah Walker, Tenshi — the Sierra
  SC1502x HiColor RAMDAC command-register state machine (four reads of
  0x3C6 arm the command register; bpp decode from the command byte).
- `src/video/vid_svga_render.c` (c) Sarah Walker — the 4bpp planar and
  8bpp packed pixel-decode rules (plane interleave layout, `ma`
  stepping, `rowoffset << 3` line pitch). The renderers themselves were
  rewritten: PCem renders per scanline from a timer with change
  tracking; Hatari renders the whole frame once per VBL with
  nearest-neighbour host scaling.

Dropped from the donor: PCem's device/timer framework, I/O handler
registration, memory-mapping windows, per-scanline timing and cycle
accounting, text-mode and 2/15/16/24/32 bpp renderers, the KSC5601
Korean-font variants, and the PC BIOS ROM (Atari Nova cards do not run
PC BIOS code; the ET4000 KEY-unlock protocol is modelled instead).
All code was restructured/rewritten into Hatari style; no file was
copied verbatim.

## src/vme_nova.c (address decode)

The Nova/ET4000 VME address layout (ISA I/O ports at A24 0xDC0000 +
port, linear video memory at A24 0xC00000, MegaSTE and TT) follows
**EmuTOS** `bios/nova.c` (GPL-2.0-or-later, (c) 2018-2025 The EmuTOS
development team, author Christian Zietz) — used as documentation of the
real hardware, no code imported. A reference copy is kept at
`docs/nova/reference/emutos-nova.c`.

## docs/nova/reference/

- `emutos-nova.c`, `emutos-nova.h` — verbatim reference copies from
  EmuTOS master (fetched 2026-08-24 from
  https://raw.githubusercontent.com/emutos/emutos/master/bios/nova.c),
  GPL-2.0-or-later, original headers retained. Kept because they are the
  authoritative open documentation of Nova card init sequences and
  address maps, and because EmuTOS doubles as a copyright-free test
  driver for this fork.
