/*
 * et4ktest.c - synthetic Nova/ET4000 test for hatari-et4000 (--vme et4000)
 *
 * Programs the card directly - no Nova driver involved - for 640x400 at
 * 8bpp chunky ("mode 13h-like" linear chain4), loads a non-trivial DAC
 * palette and draws a gradient plus 16 palette bars. This is the phase 2
 * exit criterion from PLAN.md and the regression gate for the chunky
 * path the real Nova VDI drivers use (EmuTOS's Nova console only
 * exercises the 4bpp planar path).
 *
 * The register sequence mirrors EmuTOS bios/nova.c (init_et4000 /
 * init_nova_resolution) with the CRTC values changed for 400 lines and a
 * 640 byte pitch, and chain4 enabled instead of planar mode.
 *
 * Build:  m68k-atari-mint-gcc -O2 -o ET4KTEST.PRG et4ktest.c            (MegaSTE)
 *         m68k-atari-mint-gcc -O2 -DNOVA_TT -o ET4KTEST.PRG et4ktest.c  (TT)
 * Run:    from an AUTO folder (supervisor mode) on MegaSTE or TT with
 *         --vme et4000; the Hatari window must show the gradient/bars.
 *
 * Expected picture:
 *   top 300 lines : smooth left-to-right gradient, red rising, green
 *                   falling, blue jumping at the middle
 *   bottom 100    : 16 vertical bars stepping through palette 0,16,..240
 */

#ifdef NOVA_TT
#define NOVA_REG ((volatile unsigned char *)0xFEDC0000UL)
#define NOVA_MEM ((volatile unsigned char *)0xFEC00000UL)
#else
#define NOVA_REG ((volatile unsigned char *)0x00DC0000UL)
#define NOVA_MEM ((volatile unsigned char *)0x00C00000UL)
#endif

static volatile unsigned char *const r = NOVA_REG;

static void set_idx(unsigned short port, unsigned char reg, unsigned char val)
{
	r[port] = reg;
	r[port + 1] = val;
}

int main(void)
{
	unsigned short i, x, y;

	/* ET4000 KEY unlock */
	r[0x3BF] = 0x03;
	r[0x3D8] = 0xA0;

	r[0x3C2] = 0xE3;		/* misc output: color mode, clock */
	r[0x3C3] = 0x01;		/* video subsystem enable */

	/* Timing Sequencer reset, then mode: chain4, all planes writable */
	set_idx(0x3C4, 0x00, 0x01);
	set_idx(0x3C4, 0x00, 0x03);
	r[0x3BF] = 0x03;		/* KEY again after TS reset */
	r[0x3D8] = 0xA0;
	set_idx(0x3C4, 0x01, 0x01);
	set_idx(0x3C4, 0x02, 0x0F);
	set_idx(0x3C4, 0x03, 0x00);
	set_idx(0x3C4, 0x04, 0x0E);	/* chain 4 + extended memory */

	/* CRTC: 640 pixels, 400 lines, 640 bytes pitch, linear mode */
	set_idx(0x3D4, 0x11, 0x00);	/* unprotect regs 0-7 */
	set_idx(0x3D4, 0x01, 0x4F);	/* horizontal display end: 80 chars */
	set_idx(0x3D4, 0x07, 0x02);	/* overflow: vertical display end bit 8 */
	set_idx(0x3D4, 0x09, 0x00);	/* no scanline doubling */
	set_idx(0x3D4, 0x0C, 0x00);	/* display start 0 */
	set_idx(0x3D4, 0x0D, 0x00);
	set_idx(0x3D4, 0x12, 0x8F);	/* vertical display end: 399 */
	set_idx(0x3D4, 0x13, 0x50);	/* offset: 80 -> 640 byte pitch */
	set_idx(0x3D4, 0x33, 0x00);	/* ET4000 ext start address */
	set_idx(0x3D4, 0x35, 0x00);	/* ET4000 ext overflow */
	set_idx(0x3D4, 0x36, 0xF3);	/* linear memory, 16 bit IO */

	/* Graphics controller: 256 color shift mode, graphics map */
	set_idx(0x3CE, 0x00, 0x00);
	set_idx(0x3CE, 0x01, 0x00);
	set_idx(0x3CE, 0x03, 0x00);
	set_idx(0x3CE, 0x05, 0x40);
	set_idx(0x3CE, 0x06, 0x05);
	set_idx(0x3CE, 0x08, 0xFF);
	r[0x3CD] = 0x00;		/* segment select 0 */

	/* Attribute controller: identity palette, 8 bit color mode */
	(void)r[0x3DA];			/* reset flip-flop to index phase */
	for (i = 0; i < 16; i++)
	{
		r[0x3C0] = i;
		r[0x3C0] = i;
	}
	r[0x3C0] = 0x10;
	r[0x3C0] = 0x41;		/* graphics + 8 bit color */
	r[0x3C0] = 0x12;
	r[0x3C0] = 0x0F;		/* all planes */
	r[0x3C0] = 0x20;		/* enable screen output */

	/* DAC: full mask, distinctive 256 entry palette (6 bit per gun) */
	r[0x3C6] = 0xFF;
	r[0x3C8] = 0x00;
	for (i = 0; i < 256; i++)
	{
		r[0x3C9] = i >> 2;			/* red rises */
		r[0x3C9] = (255 - i) >> 2;		/* green falls */
		r[0x3C9] = ((i ^ 128) & 0xFF) >> 2;	/* blue flips mid-way */
	}

	/* Picture: gradient on top, palette bars below */
	for (y = 0; y < 400; y++)
	{
		volatile unsigned char *p = NOVA_MEM + 640UL * y;

		if (y < 300)
			for (x = 0; x < 640; x++)
				p[x] = (unsigned char)((x * 256UL) / 640);
		else
			for (x = 0; x < 640; x++)
				p[x] = (unsigned char)((x / 40) << 4);
	}

	return 0;
}
