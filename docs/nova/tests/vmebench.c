/*
 * vmebench.c - VME framebuffer throughput benchmark for the Nova/ET4000
 *
 * Times byte/word/long writes and long reads against the card's VME
 * memory window, plus the same long-write loop against ST RAM as the
 * bus-free baseline, at 8 MHz and at 16 MHz + cache (MegaSTE CPU control
 * register 0xFF8E21). Results in hz_200 ticks (5 ms) are printed to the
 * console and appended to VMEBENCH.LOG in the current directory, with
 * derived KB/s and the cost of one full 640x400x8 frame (256,000 bytes)
 * per access strategy.
 *
 * Context (OpenUA Mega STe field session 2026-08-23): a full qd_present
 * rewrote the card per BYTE and made text crawl; the fix batches two
 * doubled pixels into one LONG write ("a quarter of the bus traffic").
 * This benchmark puts numbers on exactly that claim - and the same PRG
 * runs unchanged on a real Mega STe with a Nova card for an
 * emulator-vs-iron comparison.
 *
 * Build:  m68k-atari-mint-gcc -O2 -o VMEBENCH.PRG vmebench.c
 * Run:    from an AUTO folder or the desktop on a MegaSTE with the card
 *         initialised (boot the Nova driver stack first, or run
 *         ET4KTEST.PRG before it); needs supervisor mode for 0xFF8E21
 *         and hz_200, so it Supexec()s the whole measurement.
 */

#include <osbind.h>
#include <string.h>

#define CARD_MEM  ((volatile unsigned char *)0x00C00000UL)
#define HZ_200    (*(volatile unsigned long *)0x4BAUL)
#define MSTE_CPU  (*(volatile unsigned char *)0xFFFF8E21UL)

#define BYTES_PER_CASE  262144UL	/* 256 KB touched per timing case */
#define FRAME_BYTES     256000UL	/* one 640x400x8 card frame */

static char logbuf[2048];
static int  loglen;

static void out(const char *s)
{
	int i;
	(void)Cconws(s);
	for (i = 0; s[i] != '\0' && loglen < (int)sizeof logbuf - 1; i++)
		logbuf[loglen++] = s[i];
}

static void out_num(const char *label, long v)
{
	char b[16];
	int  i = sizeof b;
	long n = v < 0 ? -v : v;

	out(label);
	b[--i] = '\0';
	do { b[--i] = (char)('0' + n % 10); n /= 10; } while (n && i);
	if (v < 0 && i) b[--i] = '-';
	out(&b[i]);
}

/* One timing case: f() touches BYTES_PER_CASE bytes; returns hz_200 ticks */
static long bench(void (*f)(void))
{
	long t0, t1;

	t0 = (long)HZ_200;
	f();
	t1 = (long)HZ_200;
	return t1 - t0;
}

static void case_byte_write(void)
{
	volatile unsigned char *p = CARD_MEM;
	unsigned long n = BYTES_PER_CASE / 16;
	while (n--) {
		p[0]=1; p[1]=2; p[2]=3; p[3]=4; p[4]=5; p[5]=6; p[6]=7; p[7]=8;
		p[8]=1; p[9]=2; p[10]=3; p[11]=4; p[12]=5; p[13]=6; p[14]=7; p[15]=8;
		p += 16;
	}
}

static void case_word_write(void)
{
	volatile unsigned short *p = (volatile unsigned short *)CARD_MEM;
	unsigned long n = BYTES_PER_CASE / 32;
	while (n--) {
		p[0]=1; p[1]=2; p[2]=3; p[3]=4; p[4]=5; p[5]=6; p[6]=7; p[7]=8;
		p[8]=1; p[9]=2; p[10]=3; p[11]=4; p[12]=5; p[13]=6; p[14]=7; p[15]=8;
		p += 16;
	}
}

static void case_long_write(void)
{
	volatile unsigned long *p = (volatile unsigned long *)CARD_MEM;
	unsigned long n = BYTES_PER_CASE / 64;
	while (n--) {
		p[0]=1; p[1]=2; p[2]=3; p[3]=4; p[4]=5; p[5]=6; p[6]=7; p[7]=8;
		p[8]=1; p[9]=2; p[10]=3; p[11]=4; p[12]=5; p[13]=6; p[14]=7; p[15]=8;
		p += 16;
	}
}

static void case_long_read(void)
{
	volatile unsigned long *p = (volatile unsigned long *)CARD_MEM;
	unsigned long n = BYTES_PER_CASE / 64;
	unsigned long acc = 0;
	while (n--) {
		acc += p[0]; acc += p[1]; acc += p[2]; acc += p[3];
		acc += p[4]; acc += p[5]; acc += p[6]; acc += p[7];
		acc += p[8]; acc += p[9]; acc += p[10]; acc += p[11];
		acc += p[12]; acc += p[13]; acc += p[14]; acc += p[15];
		p += 16;
	}
	*(volatile unsigned long *)CARD_MEM = acc;	/* defeat optimiser */
}

static unsigned long rambuf[16384];		/* 64 KB ST RAM baseline */

static void case_ram_long_write(void)
{
	unsigned long pass = BYTES_PER_CASE / sizeof rambuf;
	while (pass--) {
		volatile unsigned long *p = rambuf;
		unsigned long n = sizeof rambuf / 64;
		while (n--) {
			p[0]=1; p[1]=2; p[2]=3; p[3]=4; p[4]=5; p[5]=6; p[6]=7; p[7]=8;
			p[8]=1; p[9]=2; p[10]=3; p[11]=4; p[12]=5; p[13]=6; p[14]=7; p[15]=8;
			p += 16;
		}
	}
}

static void report(const char *name, long ticks)
{
	long ms   = ticks * 5;
	long kbs  = ms > 0 ? (long)(BYTES_PER_CASE / 1024UL) * 1000L / ms : -1;
	long f_ms = ms > 0 ? (long)((FRAME_BYTES * (unsigned long)ms) / BYTES_PER_CASE) : -1;

	out("  ");
	out(name);
	out_num(": ", ticks);
	out_num(" ticks = ", ms);
	out_num(" ms, ", kbs);
	out_num(" KB/s, full frame ", f_ms);
	out(" ms\r\n");
}

static void run_set(const char *title)
{
	out(title);
	report("byte writes", bench(case_byte_write));
	report("word writes", bench(case_word_write));
	report("long writes", bench(case_long_write));
	report("long reads ", bench(case_long_read));
	report("RAM long wr", bench(case_ram_long_write));
}

static long run_all(void)
{
	unsigned char cpu_save = MSTE_CPU;

	out("vmebench: 256 KB per case, card mem at 0xC00000\r\n");

	MSTE_CPU = 0x00;			/* 8 MHz, cache off */
	run_set("-- 8 MHz, cache off --\r\n");

	MSTE_CPU = 0x03;			/* 16 MHz, cache on */
	run_set("-- 16 MHz, cache on --\r\n");

	MSTE_CPU = cpu_save;
	return 0;
}

int main(void)
{
	long fh;

	Supexec(run_all);

	fh = Fcreate("VMEBENCH.LOG", 0);
	if (fh >= 0) {
		Fwrite((int)fh, (long)loglen, logbuf);
		Fclose((int)fh);
	}
	return 0;
}
