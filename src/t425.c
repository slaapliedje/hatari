/*
  Hatari - t425.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  An INMOS IMS T425 transputer, as inside the ATW800/2's FPGA (the card's
  "virtual" transputer). Written from the IMS T425 data sheet (instruction
  tables, memory map, boot, peek and poke) and the Transputer Instruction
  Set - A Compiler Writer's Guide. Julian Highfield's and Andras Pahi's t4
  emulator was read to check behaviour; no code comes from it.

  What a real FPGA card showed (sp1 tools/tram, 2026-09-25): lddevid
  answers 2, the timers stand still until sttimer, and an FPU instruction
  stops the processor. Not emulated: the links' speed (a byte moves as
  soon as the far end takes the last one), instructions interrupted part
  way (a high priority process waits for the instruction to finish), the
  external memory's wait states, the Event pin and j0 breakpoints.

  The scheduler keeps its state where the real one does: the timer queues
  in TPtrLoc0/1, an interrupted low priority process in the IntSaveLocs,
  and the process queues in the Fptr/Bptr registers, which software reads
  and writes with saveh/savel and sthf/sthb/stlf/stlb (Helios does).
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "t425.h"

#define MINT		0x80000000u
#define NOTPROCESS	MINT
#define ENABLING	(MINT + 1)
#define WAITING		(MINT + 2)
#define READY		(MINT + 3)
#define TIMESET		(MINT + 1)
#define TIMENOTSET	(MINT + 2)
#define NONESELECTED	0xFFFFFFFFu
#define MEMSTART	0x80000070u
#define LINK_OUT0	0x80000000u
#define LINK_IN0	0x80000010u
#define TPTRLOC(p)	( 0x80000024u + 4 * (p) )
#define INTSAVE_W	0x8000002Cu
#define INTSAVE_I	0x80000030u
#define INTSAVE_A	0x80000034u
#define INTSAVE_B	0x80000038u
#define INTSAVE_C	0x8000003Cu
#define INTSAVE_ST	0x80000040u
#define INTSAVE_E	0x80000044u
#define DEVICE_ID	2		/* what the FPGA's lddevid answers */

/* Workspace slots below Wptr, in bytes */
#define S_IPTR		(-4)
#define S_LINK		(-8)
#define S_STATE		(-12)		/* also Pointer.s: a message's address */
#define S_TLINK		(-16)
#define S_TIME		(-20)

#define SLICE_US	1024		/* a low priority timeslice */

#define AFTER(a, b)	( (int32_t)( (a) - (b) ) > 0 )

enum { BOOT_RUN, BOOT_WAIT, BOOT_POKE, BOOT_PEEK, BOOT_LOAD };

typedef struct
{
	uint32_t outPtr, outLen;	/* a message going out */
	uint32_t inPtr, inLen;		/* a message coming in */
	uint8_t outActive, inActive;
	int16_t outBuf;			/* sent, not yet taken by the far end; -1 */
	int16_t inBuf;			/* received, not yet taken here; -1 */
	uint8_t reply[4];		/* a peek's answer */
	uint8_t replyN, replyPos;
} link_t;

typedef struct
{
	uint32_t A, B, C, O, I, W;	/* W: Wptr, word aligned */
	uint32_t D, E, S;		/* inner registers, seen by the test
					 * instructions (Helios uses them) */
	int pri;			/* 0: high, 1: low */
	uint8_t running;		/* a current process (else idle) */
	uint8_t interrupted;		/* a low priority process is saved */
	uint8_t halted, reset, analyse;
	uint8_t error, haltOnError, j0break;
	uint8_t timersGo, timerOn[2];
	uint8_t boot, bootLink, bootN;
	uint8_t bootBuf[8];
	uint32_t bootAddr, bootLeft;
	uint32_t haltI, haltW;		/* where it stopped, for Analyse */
	uint32_t Fptr[2], Bptr[2];
	uint32_t Clock[2];
	uint32_t lowTick;		/* microseconds into the low clock's tick */
	uint64_t cyc;
	uint64_t tickDue;		/* the cycle of the next microsecond */
	uint64_t sliceStart;
	uint32_t m2dRows, m2dDstStride, m2dSrcStride;
	link_t link[T425_LINKS];
} t425_t;

static t425_t T;
static uint8_t *Ram;
static uint32_t RamSize;
static uint8_t *Shm;
static uint32_t ShmBase, ShmSize;
static t425_bus_t Bus;
static int CycPerUs = 25;
static bool Trace;
#ifdef T425_HISTORY
static void history_dump(void);
#endif


static void tlog(const char *fmt, ...)
{
	char buf[256];
	va_list ap;

	if (!Bus.log)
		return;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	Bus.log(buf);
}


/* ---- memory (little-endian) ---- */

static uint32_t rd32_slow(uint32_t a)
{
	uint32_t o = a - ShmBase;

	if (Shm && o < ShmSize)
		return Shm[o] | Shm[o+1] << 8 | Shm[o+2] << 16 | (uint32_t)Shm[o+3] << 24;
	return Bus.read ? Bus.read(a, 4) : 0;
}

static inline uint32_t rd32(uint32_t a)
{
	uint32_t o;

	a &= ~3u;
	o = a - MINT;
	if (o < RamSize)
		return Ram[o] | Ram[o+1] << 8 | Ram[o+2] << 16 | (uint32_t)Ram[o+3] << 24;
	return rd32_slow(a);
}

static void wr32_slow(uint32_t a, uint32_t v)
{
	uint32_t o = a - ShmBase;

	if (Shm && o < ShmSize)
	{
		Shm[o] = v; Shm[o+1] = v >> 8; Shm[o+2] = v >> 16; Shm[o+3] = v >> 24;
	}
	else
	{
#ifdef T425_HISTORY
		static bool dumped;

		if (( a >> 28 ) != 4 && !dumped)
		{
			dumped = true;
			tlog("t425: write to %08x, nothing there\n", a);
			history_dump();
		}
#endif
		if (Bus.write)
			Bus.write(a, v, 4);
	}
}

#ifdef T425_HISTORY
static uint32_t Watch;
#define WATCH(a, v)	do { if (Watch && ( (a) & ~3u ) == Watch) tlog("t425: watch %08x <- %08x at %08x\n", (a), (v), T.I); } while (0)
#else
#define WATCH(a, v)
#endif

static inline void wr32(uint32_t a, uint32_t v)
{
	uint32_t o;

	a &= ~3u;
	WATCH(a, v);
	o = a - MINT;
	if (o < RamSize)
	{
		Ram[o] = v; Ram[o+1] = v >> 8; Ram[o+2] = v >> 16; Ram[o+3] = v >> 24;
	}
	else
		wr32_slow(a, v);
}

static inline uint8_t rd8(uint32_t a)
{
	uint32_t o = a - MINT;

	if (o < RamSize)
		return Ram[o];
	o = a - ShmBase;
	if (Shm && o < ShmSize)
		return Shm[o];
	return Bus.read ? Bus.read(a, 1) : 0;
}

static inline void wr8(uint32_t a, uint8_t v)
{
	uint32_t o = a - MINT;

	WATCH(a, v);
	if (o < RamSize)
		Ram[o] = v;
	else if (Shm && ( o = a - ShmBase ) < ShmSize)
		Shm[o] = v;
	else
	{
#ifdef T425_HISTORY
		static bool dumped;

		if (( a >> 28 ) != 4 && !dumped)
		{
			dumped = true;
			tlog("t425: byte write to %08x, nothing there\n", a);
			history_dump();
		}
#endif
		if (Bus.write)
			Bus.write(a, v, 1);
	}
}

/* A block move (move, messages), from the start: a move to just above
 * its source repeats the first bytes, which programs use to fill memory */
static void copy(uint32_t dst, uint32_t src, uint32_t n)
{
	uint32_t d = dst - MINT, s = src - MINT;

	if (d < RamSize && s < RamSize && n <= RamSize - d && n <= RamSize - s)
	{
		if (d <= s || d - s >= n)
			memmove(Ram + d, Ram + s, n);
		else
		{
			uint8_t *p = Ram + d, *q = Ram + s;

			while (n--)
				*p++ = *q++;
		}
		return;
	}
	while (n--)
		wr8(dst++, rd8(src++));
}


/* ---- the scheduler ---- */

static void schedule(uint32_t wdesc)
{
	uint32_t w = wdesc & ~3u;
	int p = wdesc & 1;

	if (T.Fptr[p] == NOTPROCESS)
		T.Fptr[p] = w;
	else
		wr32(T.Bptr[p] + S_LINK, w);
	T.Bptr[p] = w;
}

/* The current process stops; the main loop picks the next */
static void deschedule(void)
{
	wr32(T.W + S_IPTR, T.I);
	T.running = 0;
}

static uint32_t status(void)
{
	return ( T.error ? 1 : 0 ) | ( T.haltOnError ? 2 : 0 );
}

static void set_error(void)
{
	T.error = 1;
	if (T.haltOnError)
	{
		T.halted = 1;
		T.haltI = T.I;
		T.haltW = T.W | T.pri;
		tlog("t425: halted on error at %08x\n", T.I);
#ifdef T425_HISTORY
		history_dump();
#endif
	}
}

/* A high priority process is ready: the low priority one stops where it is */
static void interrupt_low(void)
{
	wr32(INTSAVE_W, T.W | 1);
	wr32(INTSAVE_I, T.I);
	wr32(INTSAVE_A, T.A);
	wr32(INTSAVE_B, T.B);
	wr32(INTSAVE_C, T.C);
	wr32(INTSAVE_ST, status());
	wr32(INTSAVE_E, T.E);
	T.interrupted = 1;
	T.running = 0;
}

static bool pick(void)
{
	int p;

	if (T.Fptr[0] != NOTPROCESS)
		p = 0;
	else if (T.interrupted)
	{
		uint32_t st = rd32(INTSAVE_ST);

		T.W = rd32(INTSAVE_W) & ~3u;
		T.I = rd32(INTSAVE_I);
		T.A = rd32(INTSAVE_A);
		T.B = rd32(INTSAVE_B);
		T.C = rd32(INTSAVE_C);
		T.E = rd32(INTSAVE_E);
		T.error = st & 1;
		T.haltOnError = ( st >> 1 ) & 1;
		T.interrupted = 0;
		T.pri = 1;
		T.running = 1;
		return true;
	}
	else if (T.Fptr[1] != NOTPROCESS)
	{
		p = 1;
		T.sliceStart = T.cyc;
	}
	else
	{
		T.pri = 1;
		return false;
	}
	T.W = T.Fptr[p];
	if (T.W == T.Bptr[p])
		T.Fptr[p] = NOTPROCESS;
	else
		T.Fptr[p] = rd32(T.W + S_LINK);
	T.I = rd32(T.W + S_IPTR);
	T.pri = p;
	T.running = 1;
#ifdef T425_HISTORY
	if (T.W - MINT >= RamSize || T.I - MINT >= RamSize)
	{
		tlog("t425: picked W=%08x I=%08x from the %s queue (next %08x)\n",
		     T.W, T.I, p ? "low" : "high", T.Fptr[p]);
		history_dump();
		T.halted = 1;
	}
#endif
	return true;
}

/* At j and lend: a low priority process that has had its timeslice goes
 * to the back of the queue */
static void timeslice(void)
{
	if (T.pri == 1 && T.Fptr[1] != NOTPROCESS
	    && T.cyc - T.sliceStart >= (uint64_t)SLICE_US * CycPerUs)
	{
		wr32(T.W + S_IPTR, T.I);
		schedule(T.W | 1);
		T.running = 0;
	}
}


/* ---- timers ---- */

/* Wait in the timer queue of priority p until the clock is AFTER time
 * (Time.s holds the first clock value that is) */
static void timer_insert(uint32_t w, int p, uint32_t time)
{
	uint32_t prev = 0, q = rd32(TPTRLOC(p));

	time++;
	wr32(w + S_TIME, time);
	while (q != NOTPROCESS && (int32_t)( time - rd32(q + S_TIME) ) >= 0)
	{
		prev = q;
		q = rd32(q + S_TLINK);
	}
	wr32(w + S_TLINK, q);
	if (prev)
		wr32(prev + S_TLINK, w);
	else
		wr32(TPTRLOC(p), w);
}

static void timer_remove(uint32_t w, int p)
{
	uint32_t prev = 0, q = rd32(TPTRLOC(p));

	while (q != NOTPROCESS)
	{
		uint32_t next = rd32(q + S_TLINK);

		if (q == w)
		{
			if (prev)
				wr32(prev + S_TLINK, next);
			else
				wr32(TPTRLOC(p), next);
			return;
		}
		prev = q;
		q = next;
	}
}

static void timers_check(void)
{
	int p;

	for (p = 0; p < 2; p++)
	{
		uint32_t q;

		if (!T.timerOn[p])
			continue;
		while (( q = rd32(TPTRLOC(p)) ) != NOTPROCESS
		       && (int32_t)( T.Clock[p] - rd32(q + S_TIME) ) >= 0)
		{
			wr32(TPTRLOC(p), rd32(q + S_TLINK));
			/* an ALT waiting on a time: its time has come */
			if (rd32(q + S_STATE) == WAITING)
				wr32(q + S_STATE, READY);
			schedule(q | p);
		}
	}
}

static void clocks_advance(uint32_t us)
{
	if (!T.timersGo)
		return;
	T.Clock[0] += us;
	T.lowTick += us;
	T.Clock[1] += T.lowTick / 64;
	T.lowTick %= 64;
}

/* The microseconds until a timer queue wakes something (UINT32_MAX: none) */
static uint32_t timers_next(void)
{
	uint32_t best = UINT32_MAX;
	int p;

	if (!T.timersGo)
		return best;
	for (p = 0; p < 2; p++)
	{
		uint32_t q = rd32(TPTRLOC(p));
		int32_t d;
		uint32_t us;

		if (!T.timerOn[p] || q == NOTPROCESS)
			continue;
		d = (int32_t)( rd32(q + S_TIME) - T.Clock[p] );
		if (d <= 0)
			return 0;
		us = p == 0 ? (uint32_t)d : (uint32_t)d * 64 - T.lowTick;
		if (us < best)
			best = us;
	}
	return best;
}

static inline void tick(void)
{
	uint64_t us = ( T.cyc - T.tickDue ) / CycPerUs + 1;

	clocks_advance((uint32_t)us);
	T.tickDue += us * CycPerUs;
	timers_check();
}


/* ---- channels and ALTs ---- */

static inline int link_out_of(uint32_t chan)
{
	return chan - LINK_OUT0 < 16 && !( chan & 3 ) ? (int)( chan - LINK_OUT0 ) / 4 : -1;
}

static inline int link_in_of(uint32_t chan)
{
	return chan - LINK_IN0 < 16 && !( chan & 3 ) ? (int)( chan - LINK_IN0 ) / 4 : -1;
}

/* A guard of the ALTing process wdesc became ready */
static void alt_ready(uint32_t wdesc)
{
	uint32_t w = wdesc & ~3u;
	uint32_t st = rd32(w + S_STATE);

	if (st == ENABLING)
		wr32(w + S_STATE, READY);
	else if (st == WAITING)
	{
		wr32(w + S_STATE, READY);
		timer_remove(w, wdesc & 1);
		schedule(wdesc);
	}
}

static void link_pump_out(int l)
{
	link_t *k = &T.link[l];

	if (!k->outActive || k->outBuf >= 0)
		return;
	if (k->outLen)
	{
		k->outBuf = rd8(k->outPtr++);
		k->outLen--;
	}
	else
	{
		uint32_t wd = rd32(LINK_OUT0 + 4 * l);

		k->outActive = 0;
		wr32(LINK_OUT0 + 4 * l, NOTPROCESS);
		schedule(wd);
	}
}

static void link_pump_in(int l)
{
	link_t *k = &T.link[l];
	uint32_t wd;

	if (k->inActive)
	{
		while (k->inLen && k->inBuf >= 0)
		{
			wr8(k->inPtr++, k->inBuf);
			k->inLen--;
			k->inBuf = -1;
		}
		if (!k->inLen)
		{
			wd = rd32(LINK_IN0 + 4 * l);
			k->inActive = 0;
			wr32(LINK_IN0 + 4 * l, NOTPROCESS);
			schedule(wd);
		}
	}
	else if (k->inBuf >= 0 && ( wd = rd32(LINK_IN0 + 4 * l) ) != NOTPROCESS)
		alt_ready(wd);
}

/* out/in: A = length, B = channel, C = the message */
static void chan_out(uint32_t len, uint32_t chan, uint32_t ptr)
{
	uint32_t wd, w, st;
	int l = link_out_of(chan);

	T.cyc += 19 + ( len + 3 ) / 2;
	if (l >= 0)
	{
		link_t *k = &T.link[l];

		wr32(chan, T.W | T.pri);
		k->outPtr = ptr;
		k->outLen = len;
		k->outActive = 1;
		deschedule();
		link_pump_out(l);
		return;
	}
	wd = rd32(chan);
	if (wd == NOTPROCESS)
	{
		wr32(chan, T.W | T.pri);
		wr32(T.W + S_STATE, ptr);
		deschedule();
		return;
	}
	w = wd & ~3u;
	st = rd32(w + S_STATE);
	if (st == ENABLING || st == WAITING || st == READY)
	{
		/* the other end is ALTing: wait in the channel for it */
		wr32(chan, T.W | T.pri);
		wr32(T.W + S_STATE, ptr);
		alt_ready(wd);
		deschedule();
		return;
	}
	copy(st, ptr, len);
	wr32(chan, NOTPROCESS);
	schedule(wd);
}

static void chan_in(uint32_t len, uint32_t chan, uint32_t ptr)
{
	uint32_t wd;
	int l = link_in_of(chan);

	T.cyc += 19 + ( len + 3 ) / 2;
	if (l >= 0)
	{
		link_t *k = &T.link[l];

		wr32(chan, T.W | T.pri);
		k->inPtr = ptr;
		k->inLen = len;
		k->inActive = 1;
		deschedule();
		link_pump_in(l);
		return;
	}
	wd = rd32(chan);
	if (wd == NOTPROCESS)
	{
		wr32(chan, T.W | T.pri);
		wr32(T.W + S_STATE, ptr);
		deschedule();
		return;
	}
	copy(ptr, rd32(( wd & ~3u ) + S_STATE), len);
	wr32(chan, NOTPROCESS);
	schedule(wd);
}


/* ---- arithmetic helpers ---- */

static inline void push(uint32_t v)
{
	T.C = T.B;
	T.B = T.A;
	T.A = v;
}

static inline void pop(void)
{
	T.A = T.B;
	T.B = T.C;
}

static uint32_t bitcount(uint32_t x)
{
	uint32_t n = 0;

	while (x)
	{
		x &= x - 1;
		n++;
	}
	return n;
}

static uint32_t bitrev(uint32_t x)
{
	uint32_t r = 0;
	int i;

	for (i = 0; i < 32; i++, x >>= 1)
		r = ( r << 1 ) | ( x & 1 );
	return r;
}

static int highbit(uint32_t x)
{
	int n = -1;

	while (x)
	{
		x >>= 1;
		n++;
	}
	return n;
}

/* the CRC step of crcword/crcbyte: A = data (from its top), B = CRC,
 * C = generator */
static void crc(int bits)
{
	uint32_t data = T.A, c = T.B;

	while (bits--)
	{
		uint32_t carry = c >> 31;

		c = ( c << 1 ) | ( data >> 31 );
		data <<= 1;
		if (carry)
			c ^= T.C;
	}
	T.A = c;
	T.B = T.C;
}


/* ---- instructions ---- */

static void bad(uint32_t op)
{
	tlog("t425: no instruction opr #%x at %08x: halted\n", op, T.I);
#ifdef T425_HISTORY
	history_dump();
#endif
	T.halted = 1;
	T.haltI = T.I;
	T.haltW = T.W | T.pri;
}

static void operate(uint32_t op)
{
	uint32_t t, u;
	uint64_t v;
	int64_t s;

	switch (op)
	{
	case 0x00:	/* rev */
		t = T.A; T.A = T.B; T.B = t;
		break;
	case 0x01:	/* lb */
		T.A = rd8(T.A);
		T.cyc += 4;
		break;
	case 0x02:	/* bsub */
		T.A += T.B; T.B = T.C;
		break;
	case 0x03:	/* endp */
		t = rd32(T.A + 4);
		wr32(T.A + 4, t - 1);
		T.cyc += 12;
		if (t == 1)
		{
			T.W = T.A & ~3u;
			T.I = rd32(T.A);
		}
		else
			T.running = 0;
		break;
	case 0x04:	/* diff */
		T.A = T.B - T.A; T.B = T.C;
		break;
	case 0x05:	/* add */
		t = T.B + T.A;
		if (( ~( T.B ^ T.A ) & ( T.B ^ t ) ) >> 31)
			set_error();
		T.A = t; T.B = T.C;
		break;
	case 0x06:	/* gcall */
		t = T.I; T.I = T.A; T.A = t;
		T.cyc += 3;
		break;
	case 0x07:	/* in */
		chan_in(T.A, T.B, T.C);
		break;
	case 0x08:	/* prod */
		T.cyc += 4 + highbit(T.A);
		T.A *= T.B; T.B = T.C;
		break;
	case 0x09:	/* gt */
		T.A = (int32_t)T.B > (int32_t)T.A; T.B = T.C;
		T.cyc += 1;
		break;
	case 0x0A:	/* wsub */
		T.A += T.B * 4; T.B = T.C;
		T.cyc += 1;
		break;
	case 0x0B:	/* out */
		chan_out(T.A, T.B, T.C);
		break;
	case 0x0C:	/* sub */
		t = T.B - T.A;
		if (( ( T.B ^ T.A ) & ( T.B ^ t ) ) >> 31)
			set_error();
		T.A = t; T.B = T.C;
		break;
	case 0x0D:	/* startp */
		t = T.A & ~3u;
		wr32(t + S_IPTR, T.I + T.B);
		schedule(t | T.pri);
		T.cyc += 11;
		break;
	case 0x0E:	/* outbyte */
	case 0x0F:	/* outword */
		wr32(T.W, T.A);
		chan_out(op == 0x0E ? 1 : 4, T.B, T.W);
		break;
	case 0x10:	/* seterr */
		set_error();
		break;
	case 0x12:	/* resetch */
		t = rd32(T.A);
		wr32(T.A, NOTPROCESS);
		if (link_out_of(T.A) >= 0)
			T.link[link_out_of(T.A)].outActive = 0;
		if (link_in_of(T.A) >= 0)
		{
			T.link[link_in_of(T.A)].inActive = 0;
			T.link[link_in_of(T.A)].inBuf = -1;
		}
		T.A = t;
		T.cyc += 2;
		break;
	case 0x13:	/* csub0 */
		if (T.B >= T.A)
			set_error();
		T.A = T.B; T.B = T.C;
		T.cyc += 1;
		break;
	case 0x15:	/* stopp */
		deschedule();
		T.cyc += 10;
		break;
	case 0x16:	/* ladd */
		s = (int64_t)(int32_t)T.B + (int32_t)T.A + ( T.C & 1 );
		if (s != (int32_t)s)
			set_error();
		T.A = (uint32_t)s; T.B = T.C;
		T.cyc += 1;
		break;
	case 0x17:	/* stlb */
		T.Bptr[1] = T.A; pop();
		break;
	case 0x18:	/* sthf */
		T.Fptr[0] = T.A; pop();
		break;
	case 0x19:	/* norm */
		v = (uint64_t)T.B << 32 | T.A;
		if (!v)
			t = 64;
		else
			for (t = 0; !( v >> 63 ); t++)
				v <<= 1;
		T.A = (uint32_t)v; T.B = (uint32_t)( v >> 32 ); T.C = t;
		T.cyc += t < 32 ? t + 4 : t < 64 ? t - 27 : 2;
		break;
	case 0x1A:	/* ldiv */
		if (T.C >= T.A)
		{
			set_error();
			T.A = T.B; T.B = T.C;
		}
		else
		{
			v = (uint64_t)T.C << 32 | T.B;
			t = (uint32_t)( v / T.A );
			T.B = (uint32_t)( v % T.A );
			T.A = t;
			T.C = T.B;
		}
		T.cyc += 34;
		break;
	case 0x1B:	/* ldpi */
		T.A += T.I;
		T.cyc += 1;
		break;
	case 0x1C:	/* stlf */
		T.Fptr[1] = T.A; pop();
		break;
	case 0x1D:	/* xdble */
		T.C = T.B; T.B = (int32_t)T.A < 0 ? 0xFFFFFFFFu : 0;
		T.cyc += 1;
		break;
	case 0x1E:	/* ldpri */
		push(T.pri);
		break;
	case 0x1F:	/* rem */
		if (T.A == 0)
		{
			set_error();
			T.A = T.B;
		}
		else if (T.A == 0xFFFFFFFFu)
			T.A = 0;
		else
			T.A = (uint32_t)( (int32_t)T.B % (int32_t)T.A );
		T.B = T.C;
		T.cyc += 36;
		break;
	case 0x20:	/* ret */
		T.I = rd32(T.W);
		T.W += 16;
		T.cyc += 4;
		break;
	case 0x21:	/* lend: B = the loop's control block, A = how far back */
		t = rd32(T.B + 4);
		wr32(T.B + 4, t - 1);
		if ((int32_t)t > 1)
		{
			wr32(T.B, rd32(T.B) + 1);
			T.I -= T.A;
			T.cyc += 9;
			timeslice();
		}
		else
			T.cyc += 4;
		break;
	case 0x22:	/* ldtimer */
		push(T.Clock[T.pri]);
		T.cyc += 1;
		break;
	case 0x23:	/* testlds */
		push(T.S);
		break;
	case 0x24:	/* testlde */
		push(T.E);
		break;
	case 0x25:	/* testldd */
		push(T.D);
		break;
	case 0x26:	/* teststs */
		T.S = T.A; pop();
		break;
	case 0x27:	/* testste */
		T.E = T.A; pop();
		break;
	case 0x28:	/* teststd */
		T.D = T.A; pop();
		break;
	case 0x29:	/* testerr */
		push(T.error ? 0 : 1);
		T.error = 0;
		T.cyc += 1;
		break;
	case 0x2A:	/* testpranal */
		push(T.analyse);
		T.cyc += 1;
		break;
	case 0x2B:	/* tin */
		if (AFTER(T.Clock[T.pri], T.A))
			T.cyc += 3;
		else
		{
			timer_insert(T.W, T.pri, T.A);
			deschedule();
			T.cyc += 29;
		}
		break;
	case 0x2C:	/* div */
		if (T.A == 0 || ( T.A == 0xFFFFFFFFu && T.B == MINT ))
			set_error();
		else
			T.A = (uint32_t)( (int32_t)T.B / (int32_t)T.A );
		T.B = T.C;
		T.cyc += 38;
		break;
	case 0x2E:	/* dist: A = offset, B = guard, C = time */
		if (T.B && AFTER(T.Clock[T.pri], T.C) && rd32(T.W) == NONESELECTED)
		{
			wr32(T.W, T.A);
			T.A = 1;
		}
		else
			T.A = 0;
		T.cyc += 22;
		break;
	case 0x2F:	/* disc: A = offset, B = guard, C = channel */
	{
		int l = link_in_of(T.C);
		bool ready;

		t = rd32(T.C);
		if (l >= 0)
			ready = !T.link[l].inActive && T.link[l].inBuf >= 0;
		else
			ready = t != NOTPROCESS && t != ( T.W | T.pri );
		if (t == ( T.W | T.pri ))
			wr32(T.C, NOTPROCESS);
		if (T.B && ready && rd32(T.W) == NONESELECTED)
		{
			wr32(T.W, T.A);
			T.A = 1;
		}
		else
			T.A = 0;
		T.cyc += 7;
		break;
	}
	case 0x30:	/* diss: A = offset, B = guard */
		if (T.B && rd32(T.W) == NONESELECTED)
		{
			wr32(T.W, T.A);
			T.A = 1;
		}
		else
			T.A = 0;
		T.B = T.C;
		T.cyc += 3;
		break;
	case 0x31:	/* lmul */
		v = (uint64_t)T.B * T.A + T.C;
		T.A = (uint32_t)v; T.B = (uint32_t)( v >> 32 ); T.C = T.B;
		T.cyc += 32;
		break;
	case 0x32:	/* not */
		T.A = ~T.A;
		break;
	case 0x33:	/* xor */
		T.A ^= T.B; T.B = T.C;
		break;
	case 0x34:	/* bcnt */
		T.A *= 4;
		T.cyc += 1;
		break;
	case 0x35:	/* lshr */
	case 0x36:	/* lshl */
		v = (uint64_t)T.C << 32 | T.B;
		if (T.A >= 64)
			v = 0;
		else if (op == 0x35)
			v >>= T.A;
		else
			v <<= T.A;
		/* the shifter takes a cycle a place, however far */
		T.cyc += T.A < 32 ? (uint64_t)T.A + 2 : (uint64_t)T.A - 29;
		T.A = (uint32_t)v; T.B = (uint32_t)( v >> 32 );
		break;
	case 0x37:	/* lsum */
		v = (uint64_t)T.B + T.A + ( T.C & 1 );
		T.A = (uint32_t)v; T.B = (uint32_t)( v >> 32 );
		T.cyc += 2;
		break;
	case 0x38:	/* lsub */
		s = (int64_t)(int32_t)T.B - (int32_t)T.A - ( T.C & 1 );
		if (s != (int32_t)s)
			set_error();
		T.A = (uint32_t)s; T.B = T.C;
		T.cyc += 1;
		break;
	case 0x39:	/* runp */
		schedule(T.A);
		T.cyc += 9;
		break;
	case 0x3A:	/* xword: B as a part word whose sign bit is A */
		if (bitcount(T.A) == 1)
			T.A = ( T.B & T.A ) ? ( T.B | ~( T.A - 1 ) ) : T.B;
		T.B = T.C;
		T.cyc += 3;
		break;
	case 0x3B:	/* sb */
		wr8(T.A, T.B);
		T.A = T.C;
		T.cyc += 3;
		break;
	case 0x3C:	/* gajw */
		t = T.W; T.W = T.A & ~3u; T.A = t;
		T.cyc += 1;
		break;
	case 0x3D:	/* savel */
	case 0x3E:	/* saveh */
		t = op == 0x3E ? 0 : 1;
		wr32(T.A, T.Fptr[t]);
		wr32(T.A + 4, T.Bptr[t]);
		pop();
		T.cyc += 3;
		break;
	case 0x3F:	/* wcnt */
		T.C = T.B; T.B = T.A & 3; T.A = (uint32_t)( (int32_t)T.A >> 2 );
		T.cyc += 4;
		break;
	case 0x40:	/* shr */
	case 0x41:	/* shl */
		T.cyc += (uint64_t)T.A + 1;	/* rspy times shl to find the speed */
		if (T.A >= 32)
			T.A = 0;
		else
			T.A = op == 0x40 ? T.B >> T.A : T.B << T.A;
		T.B = T.C;
		break;
	case 0x42:	/* mint */
		push(MINT);
		break;
	case 0x43:	/* alt */
		wr32(T.W + S_STATE, ENABLING);
		T.cyc += 1;
		break;
	case 0x44:	/* altwt */
		wr32(T.W, NONESELECTED);
		if (rd32(T.W + S_STATE) != READY)
		{
			wr32(T.W + S_STATE, WAITING);
			deschedule();
			T.cyc += 16;
		}
		else
			T.cyc += 4;
		break;
	case 0x45:	/* altend */
		T.I += rd32(T.W);
		T.cyc += 3;
		break;
	case 0x46:	/* and */
		T.A &= T.B; T.B = T.C;
		break;
	case 0x47:	/* enbt: A = guard, B = time */
		if (T.A)
		{
			t = rd32(T.W + S_TLINK);
			if (t == TIMENOTSET)
			{
				wr32(T.W + S_TLINK, TIMESET);
				wr32(T.W + S_TIME, T.B);
			}
			else if (t == TIMESET && AFTER(rd32(T.W + S_TIME), T.B))
				wr32(T.W + S_TIME, T.B);
		}
		T.B = T.C;
		T.cyc += 7;
		break;
	case 0x48:	/* enbc: A = guard, B = channel */
		if (T.A)
		{
			int l = link_in_of(T.B);

			t = rd32(T.B);
			if (l >= 0 && !T.link[l].inActive && T.link[l].inBuf >= 0)
				wr32(T.W + S_STATE, READY);
			else if (t == NOTPROCESS)
				wr32(T.B, T.W | T.pri);
			else if (t != ( T.W | T.pri ) && l < 0)
				wr32(T.W + S_STATE, READY);
		}
		T.B = T.C;
		T.cyc += 6;
		break;
	case 0x49:	/* enbs: A = guard */
		if (T.A)
			wr32(T.W + S_STATE, READY);
		T.cyc += 2;
		break;
	case 0x4A:	/* move: A = length, B = to, C = from (no length: nothing) */
		if ((int32_t)T.A > 0)
		{
			copy(T.B, T.C, T.A);
			T.cyc += 7 + ( T.A + 3 ) / 2;
		}
		break;
	case 0x4B:	/* or */
		T.A |= T.B; T.B = T.C;
		break;
	case 0x4C:	/* csngl */
		if ((int32_t)T.A < 0 ? T.B != 0xFFFFFFFFu : T.B != 0)
			set_error();
		T.B = T.C;
		T.cyc += 2;
		break;
	case 0x4D:	/* ccnt1 */
		if (T.B == 0 || T.B > T.A)
			set_error();
		T.A = T.B; T.B = T.C;
		T.cyc += 2;
		break;
	case 0x4E:	/* talt */
		wr32(T.W + S_STATE, ENABLING);
		wr32(T.W + S_TLINK, TIMENOTSET);
		T.cyc += 3;
		break;
	case 0x4F:	/* ldiff */
		v = (uint64_t)T.B - T.A - ( T.C & 1 );
		T.A = (uint32_t)v; T.B = (uint32_t)( v >> 32 ) & 1;
		T.cyc += 2;
		break;
	case 0x50:	/* sthb */
		T.Bptr[0] = T.A; pop();
		break;
	case 0x51:	/* taltwt */
		wr32(T.W, NONESELECTED);
		T.cyc += 14;
		if (rd32(T.W + S_STATE) == READY)
			break;
		if (rd32(T.W + S_TLINK) == TIMESET)
		{
			t = rd32(T.W + S_TIME);
			if (AFTER(T.Clock[T.pri], t))
			{
				wr32(T.W + S_STATE, READY);
				break;
			}
			timer_insert(T.W, T.pri, t);
		}
		wr32(T.W + S_STATE, WAITING);
		deschedule();
		T.cyc += 33;
		break;
	case 0x52:	/* sum */
		T.A += T.B; T.B = T.C;
		break;
	case 0x53:	/* mul */
		s = (int64_t)(int32_t)T.B * (int32_t)T.A;
		if (s != (int32_t)s)
			set_error();
		T.A = (uint32_t)s; T.B = T.C;
		T.cyc += 37;
		break;
	case 0x54:	/* sttimer */
		T.Clock[0] = T.Clock[1] = T.A;
		T.timersGo = 1;
		pop();
		break;
	case 0x55:	/* stoperr */
		if (T.error)
			deschedule();
		T.cyc += 1;
		break;
	case 0x56:	/* cword: A = the sign bit of a part word, B = value */
		if (bitcount(T.A) == 1 && T.A != MINT
		    && ( (int32_t)T.B >= (int32_t)T.A || (int32_t)T.B < -(int32_t)T.A ))
			set_error();
		T.A = T.B; T.B = T.C;
		T.cyc += 4;
		break;
	case 0x57:	/* clrhalterr */
		T.haltOnError = 0;
		break;
	case 0x58:	/* sethalterr */
		T.haltOnError = 1;
		break;
	case 0x59:	/* testhalterr */
		push(T.haltOnError);
		T.cyc += 1;
		break;
	case 0x5A:	/* dup */
		push(T.A);
		break;
	case 0x5B:	/* move2dinit: A = rows, B = to's stride, C = from's */
		T.m2dRows = T.A;
		T.m2dDstStride = T.B;
		T.m2dSrcStride = T.C;
		T.cyc += 7;
		break;
	case 0x5C:	/* move2dall: A = width, B = to, C = from */
	case 0x5D:	/* move2dnonzero */
	case 0x5E:	/* move2dzero */
		if ((int32_t)T.A >= 0 && (int32_t)T.m2dRows >= 0)
		{
			uint32_t r, x, dst = T.B, src = T.C;

			for (r = 0; r < T.m2dRows; r++)
			{
				if (op == 0x5C)
					copy(dst, src, T.A);
				else
					for (x = 0; x < T.A; x++)
					{
						uint8_t b = rd8(src + x);

						if (( b != 0 ) == ( op == 0x5D ))
							wr8(dst + x, b);
					}
				src += T.m2dSrcStride;
				dst += T.m2dDstStride;
			}
			T.cyc += (uint64_t)( 23 + T.A / 2 ) * T.m2dRows;
		}
		break;
	case 0x63:	/* unpacksn: A = a single, as B = exponent, A = fraction,
			 * C = old B << 2 | 0 zero, 1 number, 2 infinity, 3 NaN */
		t = T.A;
		T.C = T.B << 2;
		T.A = ( t & 0x007FFFFF ) << 8;
		T.B = ( t >> 23 ) & 0xFF;
		if (!( t & 0x7FFFFFFF ))
			u = 0;
		else if (T.B == 0xFF)
			u = T.A ? 3 : 2;
		else
		{
			u = 1;
			if (T.B == 0)
				T.B = 1;	/* denormalised */
			else
				T.A |= MINT;
		}
		T.C |= u;
		T.cyc += 14;
		break;
	case 0x6C:	/* postnormsn */
		s = (int32_t)rd32(T.W) - (int32_t)T.C;
		if (s <= -32)
			T.A = T.B = T.C = 0;
		else if (s > 0xFF)
			T.C = 0xFF;
		else if (s <= 0)
		{
			int n = 1 - (int)s;

			v = (uint64_t)T.B << 32 | T.A;
			t = T.A;
			v >>= n;
			T.A = (uint32_t)v | t;
			T.B = (uint32_t)( v >> 32 );
			T.C = 0;
		}
		else
			T.C = (uint32_t)s;
		T.cyc += 5;
		break;
	case 0x6D:	/* roundsn: C = exponent, B = fraction, A = guard bits */
		if ((int32_t)T.C >= 0xFF)
		{
			T.A = 0x7F800000;
			T.C = T.B << 1;
		}
		else
		{
			t = ( ( T.C & 0x1FF ) << 23 ) | ( ( T.B & 0x7FFFFF00 ) >> 8 );
			if (!( T.B & 0x80 ))
				T.A = t;
			else if (T.A | ( T.B & 0x7F ))
				T.A = t + 1;
			else
				T.A = t + ( t & 1 );
			T.B = T.A;
			T.C >>= 9;
		}
		T.cyc += 12;
		break;
	case 0x71:	/* ldinf */
		push(0x7F800000);
		break;
	case 0x72:	/* fmul: fractions, rounded to nearest even */
		if (T.A == MINT && T.B == MINT)
		{
			set_error();
			T.A = MINT;
		}
		else
		{
			s = (int64_t)(int32_t)T.A * (int32_t)T.B;
			t = (uint32_t)( s >> 31 );
			u = (uint32_t)s & 0x7FFFFFFF;
			if (u > 0x40000000 || ( u == 0x40000000 && ( t & 1 ) ))
				t++;
			T.A = t;
		}
		T.B = T.C;
		T.cyc += 34;
		break;
	case 0x73:	/* cflerr */
		if (( T.A & 0x7F800000 ) == 0x7F800000)
			set_error();
		T.cyc += 2;
		break;
	case 0x74:	/* crcword */
		crc(32);
		T.cyc += 34;
		break;
	case 0x75:	/* crcbyte */
		crc(8);
		T.cyc += 10;
		break;
	case 0x76:	/* bitcnt */
		T.cyc += 2 + highbit(T.A);
		T.A = bitcount(T.A) + T.B; T.B = T.C;
		break;
	case 0x77:	/* bitrevword */
		T.A = bitrev(T.A);
		T.cyc += 35;
		break;
	case 0x78:	/* bitrevnbits: the bottom A bits of B, reversed */
		if (T.A == 0 || T.A >= 64)
			t = 0;
		else if (T.A <= 32)
			t = bitrev(T.B) >> ( 32 - T.A );
		else
			t = bitrev(T.B) << ( T.A - 32 );
		T.cyc += 3 + ( T.A < 64 ? T.A : 64 );
		T.A = t; T.B = T.C;
		break;
	case 0x79:	/* pop */
		pop();
		break;
	case 0x7A:	/* timerdisableh */
	case 0x7B:	/* timerdisablel */
		T.timerOn[op - 0x7A] = 0;
		break;
	case 0x7C:	/* timerenableh */
	case 0x7D:	/* timerenablel */
		T.timerOn[op - 0x7C] = 1;
		T.cyc += 5;
		break;
	case 0x7E:	/* ldmemstartval */
		push(MEMSTART);
		break;
	case 0x81:	/* wsubdb */
		T.A += T.B * 8; T.B = T.C;
		T.cyc += 2;
		break;
	case 0x9C:	/* fptesterr: no FPU, so no FPU error */
		push(1);
		break;
	case 0xB1:	/* break */
		tlog("t425: break at %08x (breakpoints are not emulated)\n", T.I);
		T.cyc += 8;
		break;
	case 0xB2:	/* clrj0break */
		T.j0break = 0;
		break;
	case 0xB3:	/* setj0break */
		T.j0break = 1;
		break;
	case 0xB4:	/* testj0break */
		push(T.j0break);
		T.cyc += 1;
		break;
	case 0x17C:	/* lddevid */
		push(DEVICE_ID);
		break;
	default:
		bad(op);
		break;
	}
}

#ifdef T425_HISTORY
/* the last instructions, dumped when the processor halts */
static struct { uint32_t I, A, B, C, W; uint8_t ins; } Hist[4096];
static unsigned HistN;

static void history_dump(void)
{
	unsigned i;

	for (i = HistN - 4096; i != HistN; i++)
	{
		unsigned k = i & 4095;

		tlog("t425: %08x %02x  A=%08x B=%08x C=%08x W=%08x\n", Hist[k].I,
		     Hist[k].ins, Hist[k].A, Hist[k].B, Hist[k].C, Hist[k].W);
	}
}
#endif

static void trace(uint8_t ins)
{
	tlog("t425: %08x %02x  A=%08x B=%08x C=%08x O=%08x W=%08x%s\n",
	     T.I, ins, T.A, T.B, T.C, T.O, T.W | T.pri, T.error ? " E" : "");
}

static void step(void)
{
	uint8_t ins = rd8(T.I);
	uint32_t op = T.O | ( ins & 15 );

	if (Trace)
		trace(ins);
#ifdef T425_HISTORY
	if (T.I - MINT >= RamSize)
	{
		tlog("t425: I=%08x is outside the RAM\n", T.I);
		history_dump();
		T.halted = 1;
		return;
	}
	{
		unsigned k = HistN++ & 4095;

		Hist[k].I = T.I; Hist[k].ins = ins; Hist[k].A = T.A;
		Hist[k].B = T.B; Hist[k].C = T.C; Hist[k].W = T.W | T.pri;
	}
#endif
	T.I++;
	T.cyc++;
	switch (ins >> 4)
	{
	case 0x0:	/* j */
		T.O = 0;
		T.I += op;
		T.cyc += 2;
		timeslice();
		return;
	case 0x1:	/* ldlp */
		push(T.W + op * 4);
		break;
	case 0x2:	/* pfix */
		T.O = op << 4;
		return;
	case 0x3:	/* ldnl */
		T.A = rd32(T.A + op * 4);
		T.cyc++;
		break;
	case 0x4:	/* ldc */
		push(op);
		break;
	case 0x5:	/* ldnlp */
		T.A += op * 4;
		break;
	case 0x6:	/* nfix */
		T.O = ~op << 4;
		return;
	case 0x7:	/* ldl */
		push(rd32(T.W + op * 4));
		T.cyc++;
		break;
	case 0x8:	/* adc */
	{
		uint32_t t = T.A + op;

		if (( ~( T.A ^ op ) & ( T.A ^ t ) ) >> 31)
			set_error();
		T.A = t;
		break;
	}
	case 0x9:	/* call */
		T.W -= 16;
		wr32(T.W, T.I);
		wr32(T.W + 4, T.A);
		wr32(T.W + 8, T.B);
		wr32(T.W + 12, T.C);
		T.A = T.I;
		T.I += op;
		T.cyc += 6;
		break;
	case 0xA:	/* cj */
		if (T.A == 0)
		{
			T.I += op;
			T.cyc += 3;
		}
		else
		{
			pop();
			T.cyc++;
		}
		break;
	case 0xB:	/* ajw */
		T.W += op * 4;
		break;
	case 0xC:	/* eqc */
		T.A = T.A == op;
		T.cyc++;
		break;
	case 0xD:	/* stl */
		wr32(T.W + op * 4, T.A);
		pop();
		break;
	case 0xE:	/* stnl */
		wr32(T.A + op * 4, T.B);
		T.A = T.C;
		T.cyc++;
		break;
	case 0xF:	/* opr */
		T.O = 0;
		operate(op);
		return;
	}
	T.O = 0;
}


/* ---- reset and boot ---- */

static void reset_state(void)
{
	int l, p;

	T.running = 0;
	T.interrupted = 0;
	T.halted = 0;
	T.error = 0;
	T.haltOnError = 0;
	T.j0break = 0;
	T.timersGo = 0;
	T.timerOn[0] = T.timerOn[1] = 1;
	T.O = 0;
	T.pri = 1;
	for (p = 0; p < 2; p++)
	{
		T.Fptr[p] = T.Bptr[p] = NOTPROCESS;
		wr32(TPTRLOC(p), NOTPROCESS);
	}
	for (l = 0; l < T425_LINKS; l++)
	{
		memset(&T.link[l], 0, sizeof(T.link[l]));
		T.link[l].inBuf = T.link[l].outBuf = -1;
		wr32(LINK_OUT0 + 4 * l, NOTPROCESS);
		wr32(LINK_IN0 + 4 * l, NOTPROCESS);
	}
	wr32(LINK_IN0 + 16, NOTPROCESS);	/* the Event channel */
	T.boot = BOOT_WAIT;
	T.bootN = 0;
}

void T425_SetReset(bool high)
{
	if (high && !T.reset)
	{
		if (!T.halted)
		{
			T.haltI = T.I;
			T.haltW = T.W | T.pri;
		}
		T.reset = 1;
	}
	else if (!high && T.reset)
	{
		T.reset = 0;
		reset_state();
	}
}

void T425_SetAnalyse(bool high)
{
	if (high && !T.analyse && T.boot == BOOT_RUN && !T.halted)
	{
		/* (the real processor stops at its next descheduling point) */
		T.halted = 1;
		T.haltI = T.I;
		T.haltW = T.W | T.pri;
	}
	T.analyse = high;
}

bool T425_Error(void)
{
	return T.error;
}

static void boot_start(void)
{
	T.W = ( T.bootAddr + 3 ) & ~3u;
	T.I = MEMSTART;
	T.pri = 1;
	T.A = T.haltI;
	T.B = T.haltW;
	T.C = LINK_IN0 + 4 * T.bootLink;
	T.running = 1;
	T.boot = BOOT_RUN;
	T.sliceStart = T.cyc;
	tlog("t425: booted %u bytes from link %d\n", T.bootAddr - MEMSTART, T.bootLink);
}

static void boot_byte(int l, uint8_t b)
{
	switch (T.boot)
	{
	case BOOT_WAIT:
		T.bootLink = l;
		T.bootN = 0;
		if (b == 0)
			T.boot = BOOT_POKE;
		else if (b == 1)
			T.boot = BOOT_PEEK;
		else
		{
			T.boot = BOOT_LOAD;
			T.bootAddr = MEMSTART;
			T.bootLeft = b;
		}
		return;
	case BOOT_POKE:
	case BOOT_PEEK:
		if (l != T.bootLink)
			return;
		T.bootBuf[T.bootN++] = b;
		if (T.boot == BOOT_POKE && T.bootN == 8)
		{
			wr32(T.bootBuf[0] | T.bootBuf[1] << 8 | T.bootBuf[2] << 16 | (uint32_t)T.bootBuf[3] << 24,
			     T.bootBuf[4] | T.bootBuf[5] << 8 | T.bootBuf[6] << 16 | (uint32_t)T.bootBuf[7] << 24);
			T.boot = BOOT_WAIT;
		}
		else if (T.boot == BOOT_PEEK && T.bootN == 4)
		{
			link_t *k = &T.link[l];
			uint32_t v = rd32(T.bootBuf[0] | T.bootBuf[1] << 8 | T.bootBuf[2] << 16 | (uint32_t)T.bootBuf[3] << 24);

			k->reply[0] = v; k->reply[1] = v >> 8;
			k->reply[2] = v >> 16; k->reply[3] = v >> 24;
			k->replyN = 4;
			k->replyPos = 0;
			T.boot = BOOT_WAIT;
		}
		return;
	case BOOT_LOAD:
		if (l != T.bootLink)
			return;
		wr8(T.bootAddr++, b);
		if (!--T.bootLeft)
			boot_start();
		return;
	}
}


/* ---- the far ends of the links ---- */

bool T425_LinkCanWrite(int l)
{
	if (T.reset)
		return false;
	if (T.boot != BOOT_RUN)
		return T.link[l].replyN == 0;
	return T.link[l].inBuf < 0;
}

void T425_LinkWrite(int l, uint8_t b)
{
	if (!T425_LinkCanWrite(l))
		return;
	if (T.boot != BOOT_RUN)
	{
		boot_byte(l, b);
		return;
	}
	T.link[l].inBuf = b;
	link_pump_in(l);
}

bool T425_LinkCanRead(int l)
{
	link_t *k = &T.link[l];

	return k->replyN > 0 || k->outBuf >= 0;
}

uint8_t T425_LinkRead(int l)
{
	link_t *k = &T.link[l];
	uint8_t b;

	if (k->replyN)
	{
		b = k->reply[k->replyPos++];
		if (k->replyPos == k->replyN)
			k->replyN = 0;
		return b;
	}
	if (k->outBuf < 0)
		return 0;
	b = k->outBuf;
	k->outBuf = -1;
	link_pump_out(l);
	return b;
}


/* ---- running ---- */

uint64_t T425_Run(uint64_t cycles)
{
	uint64_t start = T.cyc, end = T.cyc + cycles;

	while (T.cyc < end)
	{
		if (T.cyc >= T.tickDue)
			tick();
		if (T.reset || T.halted || T.boot != BOOT_RUN
		    || ( !T.running && !pick() ))
		{
			/* nothing to run: on to the next timer, or the end */
			uint32_t us = T.boot == BOOT_RUN && !T.reset && !T.halted
			              ? timers_next() : UINT32_MAX;
			uint64_t to = us == UINT32_MAX ? end
			              : T.tickDue + (uint64_t)( us ? us - 1 : 0 ) * CycPerUs;

			if (to > end)
				to = end;
			if (to <= T.cyc)
				to = T.cyc + 1;
			T.cyc = to;
			continue;
		}
		/* (never between a prefix and its instruction) */
		if (T.pri == 1 && T.Fptr[0] != NOTPROCESS && !T.O)
		{
			interrupt_low();
			continue;
		}
		step();
	}
	return T.cyc - start;
}

uint64_t T425_Cycles(void)
{
	return T.cyc;
}

void T425_SetTrace(bool on)
{
	Trace = on;
}


/* ---- set up ---- */

bool T425_Init(uint32_t ram_size, int cycles_per_us, const t425_bus_t *bus)
{
	free(Ram);
	Ram = calloc(1, ram_size);
	if (!Ram)
		return false;
	RamSize = ram_size;
	CycPerUs = cycles_per_us > 0 ? cycles_per_us : 25;
	if (bus)
		Bus = *bus;
	else
		memset(&Bus, 0, sizeof(Bus));
	memset(&T, 0, sizeof(T));
#ifdef T425_HISTORY
	if (getenv("T425_WATCH"))
		Watch = strtoul(getenv("T425_WATCH"), NULL, 16);
#endif
	T.tickDue = CycPerUs;
	T.reset = 0;
	reset_state();
	return true;
}

void T425_UnInit(void)
{
	free(Ram);
	Ram = NULL;
	RamSize = 0;
	Shm = NULL;
}

void T425_SetShared(uint8_t *mem, uint32_t base, uint32_t size)
{
	Shm = mem;
	ShmBase = base;
	ShmSize = mem ? size : 0;
}

uint32_t T425_StateSize(void)
{
	return sizeof(T);
}

void *T425_State(void)
{
	return &T;
}

uint8_t *T425_Ram(uint32_t *size)
{
	*size = RamSize;
	return Ram;
}

void T425_Info(void (*print)(const char *line))
{
	static const char *const boot[] = { "running", "waiting for a boot",
		"poke", "peek", "loading" };
	char buf[160];
	int l, p;

	snprintf(buf, sizeof(buf), "T425: %s%s%s%s, %llu cycles, clocks %08x/%08x%s\n",
	         T.reset ? "in reset" : boot[T.boot], T.halted ? ", halted" : "",
	         T.analyse ? ", analyse" : "", T.error ? ", error" : "",
	         (unsigned long long)T.cyc, T.Clock[0], T.Clock[1],
	         T.timersGo ? "" : " (stopped)");
	print(buf);
	snprintf(buf, sizeof(buf), "  %s: I=%08x W=%08x A=%08x B=%08x C=%08x%s\n",
	         T.running ? ( T.pri ? "low" : "high" ) : "idle",
	         T.I, T.W, T.A, T.B, T.C, T.interrupted ? " (low interrupted)" : "");
	print(buf);
	for (p = 0; p < 2; p++)
	{
		snprintf(buf, sizeof(buf), "  %s: queue %08x-%08x timers %08x\n",
		         p ? "low" : "high", T.Fptr[p], T.Bptr[p], rd32(TPTRLOC(p)));
		print(buf);
	}
	for (l = 0; l < T425_LINKS; l++)
	{
		link_t *k = &T.link[l];

		snprintf(buf, sizeof(buf), "  link %d: out %s %u left%s, in %s %u left%s\n", l,
		         k->outActive ? "busy" : "idle", k->outLen,
		         k->outBuf >= 0 ? " (a byte waits)" : "",
		         k->inActive ? "busy" : "idle", k->inLen,
		         k->inBuf >= 0 ? " (a byte waits)" : "");
		print(buf);
	}
}
