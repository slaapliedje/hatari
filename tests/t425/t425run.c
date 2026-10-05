/*
 * t425run - boot a transputer program (a .btl, as made by the INMOS
 * toolsets' icollect) on the T425 core alone and be its host: the iserver
 * protocol's common requests on link 0 (files, the console, the time,
 * the environment, exit). For testing src/t425.c against other
 * transputers and emulators.
 *
 * usage: t425run [-t] [-m MHZ] [-r RAMKB] [-c MAXSECONDS] FILE.btl [ARGS]
 *
 * This file is distributed under the GNU General Public License, version 2
 * or at your option any later version. Read the file gpl.txt for details.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "t425.h"

#define SP_OK		0
#define SP_ERROR	129

static int mhz = 50;
static double maxSec = 60;
static char cmdline[1024];
static FILE *streams[64];
static uint64_t budget;

static void logline(const char *s)
{
	fputs(s, stderr);
}

static void die(const char *why)
{
	uint32_t size;
	uint8_t *ram = T425_Ram(&size);
	FILE *f = getenv("T425RUN_DUMP") ? fopen(getenv("T425RUN_DUMP"), "wb") : NULL;

	fprintf(stderr, "t425run: %s\n", why);
	T425_Info(logline);
	if (f)
	{
		fwrite(ram, 1, size, f);
		fclose(f);
	}
	exit(2);
}

/* run the transputer until cond() or the time is up */
static void run_until(bool (*cond)(void))
{
	while (!cond())
	{
		if (T425_Cycles() > budget)
			die("time is up");
		T425_Run(200);
	}
}

static bool can_write(void) { return T425_LinkCanWrite(0); }
static bool can_read(void) { return T425_LinkCanRead(0); }

static void put(uint8_t b)
{
	run_until(can_write);
	T425_LinkWrite(0, b);
}

static uint8_t get(void)
{
	run_until(can_read);
	return T425_LinkRead(0);
}

/* a reply: rp[0..n), sent with its length, padded to an even length of
 * at least 6 */
static uint8_t rp[1100];
static int rn;

static void r8(int v) { rp[rn++] = v; }
static void r16(int v) { r8(v & 255); r8(v >> 8 & 255); }
static void r32(uint32_t v) { r16(v & 0xFFFF); r16(v >> 16); }
static void rstr(const char *s, int n)
{
	r16(n);
	memcpy(rp + rn, s, n);
	rn += n;
}

static void reply(void)
{
	int i;

	if (rn < 6)
		rn = 6;
	if (rn & 1)
		rn++;
	put(rn & 255);
	put(rn >> 8);
	for (i = 0; i < rn; i++)
		put(rp[i]);
}

static uint8_t *q;		/* the request, after its tag */

static int q16(void) { int v = q[0] | q[1] << 8; q += 2; return v; }
static uint32_t q32(void) { uint32_t v = q[0] | q[1] << 8 | q[2] << 16 | (uint32_t)q[3] << 24; q += 4; return v; }

static FILE *stream(uint32_t id)
{
	return id < 64 ? streams[id] : NULL;
}

static int serve(void)
{
	static uint8_t rq[1100];
	int len, tag, i;
	char name[512];

	len = get();
	len |= get() << 8;
	if (len < 1 || len > (int)sizeof(rq))
		die("bad request length");
	for (i = 0; i < len; i++)
		rq[i] = get();
	tag = rq[0];
	q = rq + 1;
	rn = 0;
	switch (tag)
	{
	case 10:	/* open: name, type, mode */
	{
		int n = q16(), mode;
		FILE *f;

		memcpy(name, q, n < 511 ? n : 511);
		name[n < 511 ? n : 511] = 0;
		q += n;
		q++;
		mode = *q++;
		f = fopen(name, mode == 1 ? "rb" : mode == 2 ? "wb" : mode == 3 ? "ab" :
		          mode == 4 ? "r+b" : mode == 5 ? "w+b" : "a+b");
		for (i = 3; i < 64 && streams[i]; i++)
			;
		if (!f || i == 64)
		{
			r8(SP_ERROR);
			if (f)
				fclose(f);
		}
		else
		{
			streams[i] = f;
			r8(SP_OK);
			r32(i);
		}
		break;
	}
	case 11:	/* close */
	{
		uint32_t id = q32();
		FILE *f = stream(id);

		if (f && id > 2)
		{
			fclose(f);
			streams[id] = NULL;
		}
		r8(f ? SP_OK : SP_ERROR);
		break;
	}
	case 12:	/* read: id, count */
	{
		FILE *f = stream(q32());
		int n = q16(), got;
		char buf[1024];

		if (n > 1024)
			n = 1024;
		got = f ? (int)fread(buf, 1, n, f) : 0;
		r8(f ? SP_OK : SP_ERROR);
		rstr(buf, got);
		break;
	}
	case 13:	/* write: id, data */
	case 15:	/* puts: id, data */
	{
		FILE *f = stream(q32());
		int n = q16();

		if (f)
		{
			fwrite(q, 1, n, f);
			if (tag == 15)
				fputc('\n', f);
			fflush(f);
		}
		r8(f ? SP_OK : SP_ERROR);
		if (tag == 13)
			r16(f ? n : 0);
		break;
	}
	case 14:	/* gets: id, limit */
	{
		FILE *f = stream(q32());
		int n = q16();
		char buf[1024];

		if (n > 1023)
			n = 1023;
		if (f && fgets(buf, n + 1, f))
		{
			int l = strlen(buf);

			if (l && buf[l - 1] == '\n')
				l--;
			r8(SP_OK);
			rstr(buf, l);
		}
		else
			r8(SP_ERROR);
		break;
	}
	case 16:	/* flush */
	{
		FILE *f = stream(q32());

		if (f)
			fflush(f);
		r8(f ? SP_OK : SP_ERROR);
		break;
	}
	case 19:	/* eof */
	{
		FILE *f = stream(q32());

		r8(f && feof(f) ? SP_OK : SP_ERROR);
		break;
	}
	case 30:	/* getkey */
	case 31:	/* pollkey */
	{
		int ch = getchar();

		if (ch == EOF)
			r8(SP_ERROR);
		else
		{
			r8(SP_OK);
			r8(ch);
		}
		break;
	}
	case 32:	/* getenv: name */
	{
		int n = q16();
		const char *v;

		memcpy(name, q, n < 511 ? n : 511);
		name[n < 511 ? n : 511] = 0;
		v = getenv(name);
		if (!v && !strcmp(name, "IBOARDSIZE"))
			v = "#100000";
		if (v)
		{
			r8(SP_OK);
			rstr(v, strlen(v));
		}
		else
			r8(SP_ERROR);
		break;
	}
	case 33:	/* time: local, UTC */
	{
		time_t t = time(NULL);

		r8(SP_OK);
		r32((uint32_t)t);
		r32((uint32_t)t);
		break;
	}
	case 35:	/* exit: status */
	{
		int32_t st = (int32_t)q32();

		r8(SP_OK);
		reply();
		return st == 999999999 ? 0 : st == -999999999 ? 1 : (int)st;
	}
	case 40:	/* command line: all, or only the program's arguments */
	{
		const char *c = cmdline;

		if (!*q)
			c = strchr(cmdline, ' ') ? strchr(cmdline, ' ') + 1 : "";
		r8(SP_OK);
		rstr(c, strlen(c));
		break;
	}
	case 42:	/* version: version, host (PC), OS, board */
		r8(SP_OK);
		r8(20); r8(1); r8(1); r8(0);
		break;
	default:
		fprintf(stderr, "t425run: request %d not served\n", tag);
		r8(1);		/* SP.NOTIMPLEMENTED */
		break;
	}
	reply();
	return -1;
}

int main(int argc, char **argv)
{
	FILE *f;
	int c, st;
	long ramkb = 6 * 1024;
	bool trace = false;
	t425_bus_t bus = { NULL, NULL, logline };

	while ((c = getopt(argc, argv, "tm:r:c:")) != -1)
		switch (c)
		{
		case 't': trace = true; break;
		case 'm': mhz = atoi(optarg); break;
		case 'r': ramkb = atol(optarg); break;
		case 'c': maxSec = atof(optarg); break;
		default:
			fprintf(stderr, "usage: t425run [-t] [-m MHZ] [-r RAMKB] [-c MAXSECONDS] FILE.btl [ARGS]\n");
			return 2;
		}
	if (optind >= argc)
		return 2;
	for (c = optind; c < argc; c++)
	{
		if (c > optind)
			strcat(cmdline, " ");
		strncat(cmdline, argv[c], sizeof(cmdline) - strlen(cmdline) - 2);
	}
	f = fopen(argv[optind], "rb");
	if (!f)
	{
		perror(argv[optind]);
		return 2;
	}
	streams[0] = stdin;
	streams[1] = stdout;
	streams[2] = stderr;
	if (!T425_Init(ramkb * 1024, mhz, &bus))
		return 2;
	T425_SetTrace(trace);
	budget = (uint64_t)( maxSec * 1e6 * mhz );
	T425_SetReset(true);
	T425_SetReset(false);
	while ((c = getc(f)) != EOF)
		put(c);
	fclose(f);
	while ((st = serve()) < 0)
		;
	fprintf(stderr, "t425run: exit %d after %.3f s of transputer time\n",
	        st, T425_Cycles() / ( mhz * 1e6 ));
	return st;
}
