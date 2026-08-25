# Nova/ET4000 performance benchmarks (emulated MegaSTE)

2026-08-25, against the OpenUA Mega STe field session report of
2026-08-23 ("TEXT was slow in Nova mode - a full qd_present rewrote all
512,000 VME bytes per-byte on every glyph update; now row-diffed against
a shadow + the doubling loop writes aligned LONGS"). The field session
recorded mechanisms, not numbers; this run puts cycle-exact numbers on
those mechanisms. Emulation: hatari-et4000 `--vme et4000`, MegaSTE,
TOS 2.06, 4 MB, Nova NOVA-VDI 3.00 T8 stack at 640x480x256.

## 1. VME window throughput (tests/vmebench.c, synthetic)

256 KB per case against card memory at 0xC00000, timed with hz_200,
CPU mode set via 0xFF8E21. "Full frame" = one 640x400x8 present
(256,000 bytes).

| case         | 8 MHz no cache      | 16 MHz + cache      |
|--------------|---------------------|---------------------|
| byte writes  | 430 KB/s, 581 ms/f  | 609 KB/s, 410 ms/f  |
| word writes  | 867 KB/s, 288 ms/f  | 1219 KB/s, 205 ms/f |
| long writes  | 1383 KB/s, 180 ms/f | 1828 KB/s, 136 ms/f |
| long reads   | 1024 KB/s           | 1600 KB/s           |
| RAM long wr  | 1422 KB/s           | 1828 KB/s           |

- byte->long write speedup ~3.1x (the field fix's "quarter of the bus
  traffic" is the pure bus-cycle ratio; instruction stream dilutes it).
- Best case full present = 136 ms at 16 MHz+cache: a ~7 fps ceiling for
  full-screen updates - the title "wrong-palette-then-snap stretched by
  the slow VME present" window, quantified.
- **Caveat**: VME long writes == ST RAM long writes here, i.e. the
  emulated VME window currently has chip-RAM timing and models no VME
  bus arbitration/wait states. Absolute figures are therefore a lower
  bound on real-card cost. VMEBENCH.PRG runs unchanged on a real
  MegaSTE+Nova; running it there gives the calibration target for the
  bank timing (CE_MEMBANK settings in memory_map_VME).

## 2. In-game A/B: the row-diff present (FRUA, VME write counters)

Current OpenUA HEAD (401d228e) built CPU68K=68000, booted to the FRUA
menu, cursor swept to PLAY, one click to the party screen. Counters
from `info vme` (byte-lane handler ops ~= bytes). VIDEO.CFG
`novalut=off` both arms; arm B adds `novadiff=off` (verified via
"nova: row-diff present disabled" in DBG.LOG).

| VME byte-writes        | row-diff ON | row-diff OFF | factor |
|------------------------|-------------|--------------|--------|
| boot -> menu (settled) | 2,966,873   | 4,692,313    | 1.58x  |
| cursor sweep (control) | 110,320     | 110,320      | 1.00x  |
| menu -> party screen   | 125,568     | 512,128      | 4.08x  |

- The identical cursor-sweep arm is the internal control: rect presents
  are unaffected by row-diff and the procedure is repeatable to the
  byte (boot counts are bit-identical across runs of the same binary -
  cycle-exact emulation gives deterministic benchmarks).
- **512,128 bytes for the undiffed transition matches the field
  report's "512,000 VME bytes" per full update to 0.03%** (= 2 full
  640x400 frames per transition).
- Cost in time (via table 1, long-write rate at 16 MHz+cache):
  undiffed transition ~274 ms, diffed ~67 ms. On the pre-fix per-byte
  code at 8 MHz the same update was ~1.2 s - the reported text crawl.
- Cross-binary check: the pre-fix Aug-12 68000 binary (no row-diff)
  booted to menu with 4,695,997 writes - within 0.1% of the current
  build with novadiff=off (4,692,313): present volume is unchanged,
  the fix only skips redundant rows.

## Reproduction

- vmebench: `m68k-atari-mint-gcc -O2 -o ZZBENCH.PRG
  docs/nova/tests/vmebench.c`, drop into the Nova system's AUTO,
  boot with `--vme et4000`; results land in VMEBENCH.LOG.
- A/B: FRUA staging per the run-hatari-fork skill; counters via
  `hatari-debug "info vme"` (reset on machine reset).
