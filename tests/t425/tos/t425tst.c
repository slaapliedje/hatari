/*
 * T425TST.PRG - run the ATW800/2's transputer tools with their output in a
 * file, for testing the emulated T425 from an AUTO folder: each line of
 * C:\T425TST.INF is "PROGRAM ARGUMENTS"; what they print goes to
 * C:\T425.LOG, each run headed by its command and followed by its exit
 * code. The environment is the ATW800/2 profile's (TRANSPUTER etc.).
 *
 * Build: m68k-atari-mint-gcc -O2 -o T425TST.PRG t425tst.c
 *
 * This file is distributed under the GNU General Public License, version 2
 * or at your option any later version. Read the file gpl.txt for details.
 */
#include <osbind.h>
#include <string.h>

static char env[] =
	"TRANSPUTER=C:\\LINKALIA.TOS@4711\0"
	"IBOARDSIZE=#200000\0"
	"IDEBUGSIZE=#200000\0"
	"PATH=C:\\\0"
	"HOME=C:\\\0";

static long logfh;

static void say(const char *s)
{
	Fwrite(logfh, strlen(s), s);
}

static void num(long v)
{
	char b[16];
	int i = 15;

	b[i] = 0;
	if (v < 0)
	{
		say("-");
		v = -v;
	}
	do
		b[--i] = '0' + v % 10;
	while (v /= 10);
	say(b + i);
}

int main(void)
{
	static char inf[2048];
	long fh, n, save;
	char *line, *next;

	logfh = Fcreate("C:\\T425.LOG", 0);
	fh = Fopen("C:\\T425TST.INF", 0);
	if (logfh < 0 || fh < 0)
		return 1;
	n = Fread(fh, sizeof(inf) - 1, inf);
	Fclose(fh);
	inf[n > 0 ? n : 0] = 0;
	for (line = inf; *line; line = next)
	{
		char cmd[128], *sp;
		long r;

		next = line + strcspn(line, "\r\n");
		if (*next)
			*next++ = 0;
		while (*next == '\r' || *next == '\n')
			next++;
		if (!*line || *line == '#')
			continue;
		sp = strchr(line, ' ');
		if (sp)
			*sp++ = 0;
		strncpy(cmd + 1, sp ? sp : "", 125);
		cmd[126] = 0;
		cmd[0] = strlen(cmd + 1);
		say("== ");
		say(line);
		say(" ");
		say(cmd + 1);
		say("\r\n");
		save = Fdup(1);
		Fforce(1, logfh);
		r = Pexec(0, line, cmd, env);
		Fforce(1, save);
		Fclose(save);
		say("== exit ");
		num(r);
		say("\r\n");
	}
	say("== done\r\n");
	Fclose(logfh);
	return 0;
}
