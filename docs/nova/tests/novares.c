/*
 * novares.c - switch a running NOVA-VDI system to a given resolution
 *
 * Uses the documented NOVA cookie interface (nova_xcb_t / p_chres, same
 * contract as SDL's src/video/xbios/SDL_xbios_nova.c): loads the wanted
 * nova_resolution_t entry from \AUTO\STA_VDI.BIB and calls the driver's
 * own change-resolution routine with a0 = &entry, d0 = 0.
 *
 * Build:  m68k-atari-mint-gcc -O2 -DRES_INDEX=1 -o ZNOVARES.PRG novares.c
 * Run:    from the AUTO folder AFTER STA_VDI.PRG (the Z prefix keeps it
 *         last in Hatari's alphasorted GEMDOS AUTO order), or from the
 *         desktop. RES_INDEX is the 0-based entry in STA_VDI.BIB.
 */

#include <osbind.h>

#ifndef RES_INDEX
#define RES_INDEX 1
#endif

typedef struct {
	unsigned char	name[33];
	unsigned char	dummy1;
	unsigned short	mode;
	unsigned short	pitch;
	unsigned short	planes;
	unsigned short	colors;
	unsigned short	hc_mode;
	unsigned short	max_x, max_y;
	unsigned short	real_x, real_y;
	unsigned short	freq;
	unsigned char	freq2;
	unsigned char	low_res;
	unsigned char	r_3c2;
	unsigned char	r_3d4[25];
	unsigned char	extended[3];
	unsigned char	dummy2;
} nova_resolution_t;

typedef struct {
	unsigned char	version[4];
	unsigned char	resolution;
	unsigned char	blnk_time;
	unsigned char	ms_speed;
	unsigned char	old_res;
	void		(*p_chres)(void);
	/* more fields follow, not needed here */
} nova_xcb_t;

static nova_resolution_t res;
static nova_xcb_t *xcb;

static long find_nova(void)
{
	long *jar = *(long **)0x5a0;

	xcb = 0;
	if (!jar)
		return 0;
	for (; jar[0]; jar += 2)
	{
		if (jar[0] == 0x4E4F5641L)	/* 'NOVA' */
		{
			xcb = (nova_xcb_t *)jar[1];
			return 0;
		}
	}
	return 0;
}

static long do_chres(void)
{
	register nova_resolution_t *a0 __asm__("a0") = &res;
	register void (*a1)(void) __asm__("a1") = xcb->p_chres;

	__asm__ volatile (
		"moveq	#0,%%d0\n\t"
		"jsr	(%1)"
		: /* no output */
		: "a"(a0), "a"(a1)
		: "d0", "d1", "d2", "cc", "memory");
	return 0;
}

int main(void)
{
	long fd;

	Supexec(find_nova);
	if (!xcb)
	{
		(void)Cconws("novares: no NOVA cookie\r\n");
		return 1;
	}

	fd = Fopen("\\AUTO\\STA_VDI.BIB", 0);
	if (fd < 0)
	{
		(void)Cconws("novares: no \\AUTO\\STA_VDI.BIB\r\n");
		return 1;
	}
	Fseek((long)RES_INDEX * sizeof(nova_resolution_t), (int)fd, 0);
	if (Fread((int)fd, sizeof(res), &res) != sizeof(res))
	{
		Fclose((int)fd);
		(void)Cconws("novares: BIB read failed\r\n");
		return 1;
	}
	Fclose((int)fd);

	(void)Cconws("novares: switching to ");
	(void)Cconws((char *)res.name);
	(void)Cconws("\r\n");

	Supexec(do_chres);
	return 0;
}
