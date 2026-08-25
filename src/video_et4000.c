/*
  Hatari - video_et4000.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  Tseng Labs ET4000AX emulation, for the Nova VME graphics card of the
  MegaSTE and TT (see vme_nova.c for the bus side).

  The SVGA register state machine (sequencer / CRTC / graphics controller /
  attribute controller / RAMDAC), the VGA plane write/read logic and the
  Tseng specifics (banking register 0x3CD, extended CRTC registers, Sierra
  HiColor RAMDAC command state) are derived from PCem (GPL-2.0-or-later):
    pcem/src/video/vid_svga.c, vid_svga_render.c  (c) Sarah Walker
    pcem/src/video/vid_et4000.c                   (c) Sarah Walker
    pcem/src/video/vid_unk_ramdac.c               (c) Sarah Walker, Tenshi
  PCem's device/timer framework and per-scanline renderer are not used :
  the screen is rendered whole once per Hatari VBL, and there is no PC BIOS
  ROM (Atari Nova drivers program the chip directly).
  See docs/nova/PROVENANCE.md for details.

  What is intentionally NOT emulated (yet):
  - text modes and the 9-dot character path (no Atari driver uses them)
  - 2bpp CGA-compatible modes
  - 15/16/24bpp HiColor modes (the Sierra RAMDAC command register is
    modelled so drivers can probe it, but rendering logs one warning)
  - accurate dot-clock derived timing: the vertical retrace status bit
    follows Hatari's own VBL instead
*/
const char VideoEt4000_fileid[] = "Hatari video_et4000.c";

#include "main.h"
#include "configuration.h"
#include "conv_gen.h"
#include "log.h"
#include "m68000.h"
#include "memorySnapShot.h"
#include "screen.h"
#include "video.h"
#include "video_et4000.h"

#define ET4000_VRAM_SIZE	0x100000		/* 1 MB, ET4000AX class */
#define ET4000_VRAM_MASK	(ET4000_VRAM_SIZE-1)

typedef struct {
	/* Miscellaneous output register (write 0x3C2, read 0x3CC) */
	uint8_t		miscout;
	/* Video subsystem enable (0x3C3) */
	uint8_t		vidsub;
	/* Feature control (write 0x3DA, read 0x3CA) */
	uint8_t		featcon;

	/* Sequencer ("Timing Sequencer") 0x3C4/0x3C5 */
	uint8_t		seqaddr;
	uint8_t		seqregs[0x10];

	/* CRT Controller 0x3D4/0x3D5 (0x3B4/0x3B5 in mono mapping) */
	uint8_t		crtcaddr;
	uint8_t		crtc[0x40];

	/* Graphics controller 0x3CE/0x3CF */
	uint8_t		gdcaddr;
	uint8_t		gdcreg[0x10];

	/* Attribute controller 0x3C0/0x3C1 */
	uint8_t		attraddr;
	uint8_t		attrff;			/* index/data flip-flop */
	uint8_t		attr_palette_enable;	/* bit 5 of the index write */
	uint8_t		attrregs[0x20];

	/* RAMDAC 0x3C6-0x3C9 (VGA part) */
	uint8_t		dac_mask;
	uint8_t		dac_status;
	uint8_t		dac_read, dac_write, dac_pos;
	uint8_t		dac_r, dac_g;
	uint8_t		vgapal[256][3];

	/* Sierra SC1502x HiColor RAMDAC command state (4 reads of 0x3C6 arm it) */
	uint8_t		ramdac_state;
	uint8_t		ramdac_ctrl;
	uint8_t		bpp;			/* pixel depth the RAMDAC outputs */

	/* Tseng banking (0x3CD) */
	uint8_t		banking;

	/* ET4000 KEY protection (0x3BF then mode control) */
	uint8_t		key_state;
	bool		key_unlocked;

	/* Hercules/CGA compatibility mode control latches (0x3B8/0x3D8) */
	uint8_t		mode_3x8;

	/* VGA data path state derived from the registers above */
	uint8_t		writemask;		/* seq 2 */
	uint8_t		readplane;		/* gdc 4 */
	uint8_t		writemode, readmode;	/* gdc 5 */
	uint8_t		chain2_write, chain2_read;
	uint8_t		chain4;			/* seq 4 bit 3 */
	uint8_t		colourcompare, colournocare;
	uint8_t		la, lb, lc, ld;		/* the 4 VGA latches */

	uint8_t		plane_mask;		/* attr 0x12 */
} ET4000_REGS;

static ET4000_REGS	et4000;
static uint8_t		*pEt4000Vram;

/* GDC data rotate lookup, from PCem (svga_rotate) */
static uint8_t		Et4000Rotate[8][256];

/* Host colors for the 256 palette entries; rebuilt when the DAC changes */
static uint32_t		Et4000HostPal[256];
static bool		bEt4000PalDirty;

/* Set when a mode is reached that the renderer cannot show, to log only once */
static uint8_t		Et4000UnsupportedLogged;

static void		ET4000_RecalcHostPalette ( void );


/**
 * Allocate VRAM. Called from the VME device when the card is mapped.
 */
void	ET4000_Init ( void )
{
	int c, d, e;

	if ( !pEt4000Vram )
	{
		pEt4000Vram = malloc ( ET4000_VRAM_SIZE );
		if ( !pEt4000Vram )
			Main_ErrorExit ( "Out of memory (ET4000 VRAM)", NULL, 1 );
		memset ( pEt4000Vram, 0, ET4000_VRAM_SIZE );
	}

	for ( c = 0 ; c < 256 ; c++ )
		for ( d = 0 ; d < 8 ; d++ )
		{
			e = c;
			if ( d )
				e = ( ( c >> d ) | ( c << ( 8 - d ) ) ) & 0xff;
			Et4000Rotate[d][c] = e;
		}
}


void	ET4000_UnInit ( void )
{
	free ( pEt4000Vram );
	pEt4000Vram = NULL;
}


/**
 * Reset the chip. VRAM content is preserved (as on real hardware).
 */
void	ET4000_Reset ( bool bCold )
{
	memset ( &et4000, 0, sizeof(et4000) );
	et4000.bpp = 8;
	et4000.dac_mask = 0xff;
	bEt4000PalDirty = true;
	Et4000UnsupportedLogged = 0;
}


/*-----------------------------------------------------------------------*/
/* Register file access                                                  */
/*-----------------------------------------------------------------------*/

/**
 * The ET4000 KEY: writing 0x03 to port 0x3BF followed by 0xA0 to the mode
 * control register (0x3D8/0x3B8) unlocks the extended registers. Tracked
 * for tracing/debugging; the extended registers are not actually gated
 * (like in PCem - every Atari driver unlocks first anyway).
 */
static void ET4000_Key_Write ( uint16_t port, uint8_t val )
{
	if ( port == 0x3BF )
	{
		et4000.key_state = ( val == 0x03 ) ? 1 : 0;
	}
	else						/* mode control 0x3x8 */
	{
		et4000.mode_3x8 = val;
		if ( et4000.key_state == 1 && val == 0xA0 && !et4000.key_unlocked )
		{
			et4000.key_unlocked = true;
			LOG_TRACE(TRACE_VME, "et4000 KEY unlocked pc=%x\n", M68000_GetPC());
		}
	}
}


/**
 * Update the derived "fast path" state after sequencer/GDC writes
 */
static void ET4000_UpdateDataPath ( void )
{
	et4000.chain4 = et4000.seqregs[4] & 8;
	et4000.chain2_write = !( et4000.seqregs[4] & 4 );
	et4000.writemask = et4000.seqregs[2] & 0xf;
	et4000.writemode = et4000.gdcreg[5] & 3;
	et4000.readmode = et4000.gdcreg[5] & 8;
	et4000.chain2_read = et4000.gdcreg[5] & 0x10;
	et4000.readplane = et4000.gdcreg[4] & 3;
	et4000.colourcompare = et4000.gdcreg[2] & 0xf;
	et4000.colournocare = et4000.gdcreg[7] & 0xf;
}


/**
 * Vertical/horizontal retrace approximation for Input Status Register 1
 * (0x3DA): the card's sync is not derived from a dot clock but follows
 * the position of Hatari's own video beam, which is what the polling
 * loops in the drivers (vsync waits, EmuTOS count_vbls) need.
 */
static uint8_t ET4000_Status1_Read ( void )
{
	int FrameCycles, HblCounterVideo, LineCycles;
	uint8_t status = 0;

	Video_GetPosition ( &FrameCycles, &HblCounterVideo, &LineCycles );

	if ( HblCounterVideo < nStartHBL || HblCounterVideo >= nEndHBL )
		status |= 0x09;				/* vertical retrace + display disabled */
	else if ( LineCycles > 320*2 )			/* rough horizontal blank portion */
		status |= 0x01;

	return status;
}


void	ET4000_IO_WriteByte ( uint16_t port, uint8_t val )
{
	/* Mono register mapping: with miscout bit 0 clear, 0x3Dx moves to 0x3Bx */
	if ( ( ( port & 0xFFF0 ) == 0x3D0 || ( port & 0xFFF0 ) == 0x3B0 ) && !( et4000.miscout & 1 ) )
		port ^= 0x60;

	LOG_TRACE(TRACE_VME, "et4000 out port=%03x val=0x%02x pc=%x\n", port, val, M68000_GetPC());

	switch ( port )
	{
	 case 0x3B8:					/* KEY / Hercules mode control */
	 case 0x3D8:
	 case 0x3BF:
		ET4000_Key_Write ( port, val );
		break;

	 case 0x3C0:					/* attribute controller: index/data alternate */
		if ( !et4000.attrff )
		{
			et4000.attraddr = val & 31;
			et4000.attr_palette_enable = val & 0x20;
		}
		else
		{
			et4000.attrregs[et4000.attraddr & 31] = val;
			if ( et4000.attraddr == 0x12 )
				et4000.plane_mask = val & 0xf;
			bEt4000PalDirty = true;
		}
		et4000.attrff ^= 1;
		break;

	 case 0x3C2:					/* misc output */
		et4000.miscout = val;
		break;

	 case 0x3C3:					/* video subsystem enable */
		et4000.vidsub = val;
		break;

	 case 0x3C4:					/* sequencer index */
		et4000.seqaddr = val;
		break;
	 case 0x3C5:
		if ( ( et4000.seqaddr & 0xf ) > 0xf )
			break;
		et4000.seqregs[et4000.seqaddr & 0xf] = val;
		ET4000_UpdateDataPath ();
		break;

	 case 0x3C6:					/* DAC mask / Sierra HiColor command */
		if ( et4000.ramdac_state == 4 )
		{
			et4000.ramdac_state = 0;
			if ( val != 0xFF )
			{
				et4000.ramdac_ctrl = val;
				switch ( ( val & 1 ) | ( ( val & 0xC0 ) >> 5 ) )
				{
				 case 0:	et4000.bpp = 8; break;
				 case 2: case 3: et4000.bpp = ( val & 0x20 ) ? 24 : 32; break;
				 case 4: case 5: et4000.bpp = 15; break;
				 case 6:	et4000.bpp = 16; break;
				 case 7:	et4000.bpp = ( val & 4 ) ? ( ( val & 0x20 ) ? 24 : 32 ) : 16; break;
				 default:	break;
				}
			}
			break;
		}
		et4000.ramdac_state = 0;
		et4000.dac_mask = val;
		bEt4000PalDirty = true;
		break;
	 case 0x3C7:					/* DAC read index */
		et4000.ramdac_state = 0;
		et4000.dac_read = val;
		et4000.dac_pos = 0;
		break;
	 case 0x3C8:					/* DAC write index */
		et4000.ramdac_state = 0;
		et4000.dac_write = val;
		et4000.dac_read = val - 1;
		et4000.dac_pos = 0;
		break;
	 case 0x3C9:					/* DAC data */
		et4000.ramdac_state = 0;
		et4000.dac_status = 0;
		switch ( et4000.dac_pos )
		{
		 case 0:
			et4000.dac_r = val;
			et4000.dac_pos++;
			break;
		 case 1:
			et4000.dac_g = val;
			et4000.dac_pos++;
			break;
		 case 2:
			et4000.vgapal[et4000.dac_write][0] = et4000.dac_r;
			et4000.vgapal[et4000.dac_write][1] = et4000.dac_g;
			et4000.vgapal[et4000.dac_write][2] = val;
			bEt4000PalDirty = true;
			et4000.dac_pos = 0;
			et4000.dac_write = ( et4000.dac_write + 1 ) & 255;
			break;
		}
		break;

	 case 0x3CD:					/* Tseng banking */
		et4000.banking = val;
		break;

	 case 0x3CE:					/* graphics controller index */
		et4000.gdcaddr = val;
		break;
	 case 0x3CF:
		et4000.gdcreg[et4000.gdcaddr & 15] = val;
		ET4000_UpdateDataPath ();
		break;

	 case 0x3B4:					/* CRTC index */
	 case 0x3D4:
		et4000.crtcaddr = val & 0x3f;
		break;
	 case 0x3B5:					/* CRTC data */
	 case 0x3D5:
		if ( ( et4000.crtcaddr < 7 ) && ( et4000.crtc[0x11] & 0x80 ) )
			break;				/* CRTC write protection */
		if ( ( et4000.crtcaddr == 7 ) && ( et4000.crtc[0x11] & 0x80 ) )
			val = ( et4000.crtc[7] & ~0x10 ) | ( val & 0x10 );
		et4000.crtc[et4000.crtcaddr] = val;
		break;

	 case 0x3BA:					/* feature control */
	 case 0x3DA:
		et4000.featcon = val;
		break;

	 case 0x3C1:					/* attribute data read port: writes ignored */
	 default:
		break;
	}
}


uint8_t	ET4000_IO_ReadByte ( uint16_t port )
{
	uint8_t val = 0xff;

	if ( ( ( port & 0xFFF0 ) == 0x3D0 || ( port & 0xFFF0 ) == 0x3B0 ) && !( et4000.miscout & 1 ) )
		port ^= 0x60;

	switch ( port )
	{
	 case 0x3C0:
		val = et4000.attraddr | et4000.attr_palette_enable;
		break;
	 case 0x3C1:
		val = et4000.attrregs[et4000.attraddr & 31];
		break;
	 case 0x3C2:					/* input status 0 */
		val = 0x10;				/* monitor sense: color display attached */
		break;
	 case 0x3C3:
		val = et4000.vidsub;
		break;
	 case 0x3C4:
		val = et4000.seqaddr;
		break;
	 case 0x3C5:
		val = et4000.seqregs[et4000.seqaddr & 0xf];
		if ( ( et4000.seqaddr & 0xf ) == 7 )
			val |= 4;			/* ET4000: TS 7 reads back with bit 2 set */
		break;
	 case 0x3C6:
		if ( et4000.ramdac_state == 4 )
			val = et4000.ramdac_ctrl;	/* Sierra HiColor command register */
		else
		{
			et4000.ramdac_state++;
			val = et4000.dac_mask;
		}
		break;
	 case 0x3C7:
		et4000.ramdac_state = 0;
		val = et4000.dac_status;
		break;
	 case 0x3C8:
		et4000.ramdac_state = 0;
		val = et4000.dac_write;
		break;
	 case 0x3C9:
		et4000.ramdac_state = 0;
		et4000.dac_status = 3;
		switch ( et4000.dac_pos )
		{
		 case 0:
			et4000.dac_pos++;
			val = et4000.vgapal[et4000.dac_read & 0xff][0];
			break;
		 case 1:
			et4000.dac_pos++;
			val = et4000.vgapal[et4000.dac_read & 0xff][1];
			break;
		 default:
			et4000.dac_pos = 0;
			et4000.dac_read = ( et4000.dac_read + 1 ) & 255;
			val = et4000.vgapal[( et4000.dac_read - 1 ) & 255][2];
			break;
		}
		break;
	 case 0x3CA:
		val = et4000.featcon;
		break;
	 case 0x3CC:
		val = et4000.miscout;
		break;
	 case 0x3CD:
		val = et4000.banking;
		break;
	 case 0x3CE:
		val = et4000.gdcaddr;
		break;
	 case 0x3CF:
		val = et4000.gdcreg[et4000.gdcaddr & 15];
		break;
	 case 0x3B4:
	 case 0x3D4:
		val = et4000.crtcaddr;
		break;
	 case 0x3B5:
	 case 0x3D5:
		val = et4000.crtc[et4000.crtcaddr];
		break;
	 case 0x3BA:
	 case 0x3DA:
		et4000.attrff = 0;			/* reading 3DA resets the ATC flip-flop */
		val = ET4000_Status1_Read ();
		break;
	 default:
		break;
	}

	LOG_TRACE(TRACE_VME, "et4000 in  port=%03x val=0x%02x pc=%x\n", port, val, M68000_GetPC());
	return val;
}


/*-----------------------------------------------------------------------*/
/* Video memory access (VGA plane logic, from PCem svga_write_linear /   */
/* svga_read_linear with packed chain4 as on the real ET4000)            */
/*-----------------------------------------------------------------------*/

/**
 * Translate a byte offset in the card's memory window to the VRAM address
 * used by the plane logic. In linear mode (CRTC 0x36 bit 4, the way all
 * Atari drivers run the card) the window maps VRAM from 0; otherwise the
 * Tseng segment select (0x3CD) provides the 64k banking.
 */
static uint32_t ET4000_MapAddr ( uint32_t offset, bool bWrite )
{
	if ( et4000.crtc[0x36] & 0x10 )			/* linear mode */
		return offset;
	if ( bWrite )
		return ( offset & 0xffff ) + ( ( et4000.banking & 0xf ) << 16 );
	return ( offset & 0xffff ) + ( ( ( et4000.banking >> 4 ) & 0xf ) << 16 );
}


void	ET4000_Mem_WriteByte ( uint32_t offset, uint8_t val )
{
	uint32_t addr = ET4000_MapAddr ( offset, true );
	int writemask2 = et4000.writemask;
	uint8_t vala, valb, valc, vald, wm;

	if ( et4000.chain4 )
	{
		/* ET4000: chain4 is fully packed/linear */
		writemask2 = 1 << ( addr & 3 );
		addr &= ~3;
	}
	else if ( et4000.chain2_write )
	{
		writemask2 &= ~0xa;
		if ( addr & 1 )
			writemask2 <<= 1;
		addr &= ~1;
		addr <<= 2;
	}
	else
	{
		addr <<= 2;
	}

	if ( addr >= ET4000_VRAM_SIZE )
		return;

	switch ( et4000.writemode )
	{
	 case 1:
		if ( writemask2 & 1 ) pEt4000Vram[addr]     = et4000.la;
		if ( writemask2 & 2 ) pEt4000Vram[addr | 1] = et4000.lb;
		if ( writemask2 & 4 ) pEt4000Vram[addr | 2] = et4000.lc;
		if ( writemask2 & 8 ) pEt4000Vram[addr | 3] = et4000.ld;
		break;

	 case 0:
		if ( et4000.gdcreg[3] & 7 )
			val = Et4000Rotate[et4000.gdcreg[3] & 7][val];
		if ( et4000.gdcreg[8] == 0xff && !( et4000.gdcreg[3] & 0x18 ) && !et4000.gdcreg[1] )
		{
			if ( writemask2 & 1 ) pEt4000Vram[addr]     = val;
			if ( writemask2 & 2 ) pEt4000Vram[addr | 1] = val;
			if ( writemask2 & 4 ) pEt4000Vram[addr | 2] = val;
			if ( writemask2 & 8 ) pEt4000Vram[addr | 3] = val;
		}
		else
		{
			vala = ( et4000.gdcreg[1] & 1 ) ? ( ( et4000.gdcreg[0] & 1 ) ? 0xff : 0 ) : val;
			valb = ( et4000.gdcreg[1] & 2 ) ? ( ( et4000.gdcreg[0] & 2 ) ? 0xff : 0 ) : val;
			valc = ( et4000.gdcreg[1] & 4 ) ? ( ( et4000.gdcreg[0] & 4 ) ? 0xff : 0 ) : val;
			vald = ( et4000.gdcreg[1] & 8 ) ? ( ( et4000.gdcreg[0] & 8 ) ? 0xff : 0 ) : val;
			goto do_logic;
		}
		break;

	 case 2:
		vala = ( val & 1 ) ? 0xff : 0;
		valb = ( val & 2 ) ? 0xff : 0;
		valc = ( val & 4 ) ? 0xff : 0;
		vald = ( val & 8 ) ? 0xff : 0;
		goto do_logic;

	 case 3:
		if ( et4000.gdcreg[3] & 7 )
			val = Et4000Rotate[et4000.gdcreg[3] & 7][val];
		wm = et4000.gdcreg[8];
		et4000.gdcreg[8] &= val;
		vala = ( et4000.gdcreg[0] & 1 ) ? 0xff : 0;
		valb = ( et4000.gdcreg[0] & 2 ) ? 0xff : 0;
		valc = ( et4000.gdcreg[0] & 4 ) ? 0xff : 0;
		vald = ( et4000.gdcreg[0] & 8 ) ? 0xff : 0;
		switch ( et4000.gdcreg[3] & 0x18 )
		{
		 case 0x00:				/* set */
			if ( writemask2 & 1 ) pEt4000Vram[addr]     = ( vala & et4000.gdcreg[8] ) | ( et4000.la & ~et4000.gdcreg[8] );
			if ( writemask2 & 2 ) pEt4000Vram[addr | 1] = ( valb & et4000.gdcreg[8] ) | ( et4000.lb & ~et4000.gdcreg[8] );
			if ( writemask2 & 4 ) pEt4000Vram[addr | 2] = ( valc & et4000.gdcreg[8] ) | ( et4000.lc & ~et4000.gdcreg[8] );
			if ( writemask2 & 8 ) pEt4000Vram[addr | 3] = ( vald & et4000.gdcreg[8] ) | ( et4000.ld & ~et4000.gdcreg[8] );
			break;
		 case 0x08:				/* and */
			if ( writemask2 & 1 ) pEt4000Vram[addr]     = ( vala | ~et4000.gdcreg[8] ) & et4000.la;
			if ( writemask2 & 2 ) pEt4000Vram[addr | 1] = ( valb | ~et4000.gdcreg[8] ) & et4000.lb;
			if ( writemask2 & 4 ) pEt4000Vram[addr | 2] = ( valc | ~et4000.gdcreg[8] ) & et4000.lc;
			if ( writemask2 & 8 ) pEt4000Vram[addr | 3] = ( vald | ~et4000.gdcreg[8] ) & et4000.ld;
			break;
		 case 0x10:				/* or */
			if ( writemask2 & 1 ) pEt4000Vram[addr]     = ( vala & et4000.gdcreg[8] ) | et4000.la;
			if ( writemask2 & 2 ) pEt4000Vram[addr | 1] = ( valb & et4000.gdcreg[8] ) | et4000.lb;
			if ( writemask2 & 4 ) pEt4000Vram[addr | 2] = ( valc & et4000.gdcreg[8] ) | et4000.lc;
			if ( writemask2 & 8 ) pEt4000Vram[addr | 3] = ( vald & et4000.gdcreg[8] ) | et4000.ld;
			break;
		 case 0x18:				/* xor */
			if ( writemask2 & 1 ) pEt4000Vram[addr]     = ( vala & et4000.gdcreg[8] ) ^ et4000.la;
			if ( writemask2 & 2 ) pEt4000Vram[addr | 1] = ( valb & et4000.gdcreg[8] ) ^ et4000.lb;
			if ( writemask2 & 4 ) pEt4000Vram[addr | 2] = ( valc & et4000.gdcreg[8] ) ^ et4000.lc;
			if ( writemask2 & 8 ) pEt4000Vram[addr | 3] = ( vald & et4000.gdcreg[8] ) ^ et4000.ld;
			break;
		}
		et4000.gdcreg[8] = wm;
		break;
	}
	return;

do_logic:
	switch ( et4000.gdcreg[3] & 0x18 )
	{
	 case 0x00:					/* set */
		if ( writemask2 & 1 ) pEt4000Vram[addr]     = ( vala & et4000.gdcreg[8] ) | ( et4000.la & ~et4000.gdcreg[8] );
		if ( writemask2 & 2 ) pEt4000Vram[addr | 1] = ( valb & et4000.gdcreg[8] ) | ( et4000.lb & ~et4000.gdcreg[8] );
		if ( writemask2 & 4 ) pEt4000Vram[addr | 2] = ( valc & et4000.gdcreg[8] ) | ( et4000.lc & ~et4000.gdcreg[8] );
		if ( writemask2 & 8 ) pEt4000Vram[addr | 3] = ( vald & et4000.gdcreg[8] ) | ( et4000.ld & ~et4000.gdcreg[8] );
		break;
	 case 0x08:					/* and */
		if ( writemask2 & 1 ) pEt4000Vram[addr]     = ( vala | ~et4000.gdcreg[8] ) & et4000.la;
		if ( writemask2 & 2 ) pEt4000Vram[addr | 1] = ( valb | ~et4000.gdcreg[8] ) & et4000.lb;
		if ( writemask2 & 4 ) pEt4000Vram[addr | 2] = ( valc | ~et4000.gdcreg[8] ) & et4000.lc;
		if ( writemask2 & 8 ) pEt4000Vram[addr | 3] = ( vald | ~et4000.gdcreg[8] ) & et4000.ld;
		break;
	 case 0x10:					/* or */
		if ( writemask2 & 1 ) pEt4000Vram[addr]     = ( vala & et4000.gdcreg[8] ) | et4000.la;
		if ( writemask2 & 2 ) pEt4000Vram[addr | 1] = ( valb & et4000.gdcreg[8] ) | et4000.lb;
		if ( writemask2 & 4 ) pEt4000Vram[addr | 2] = ( valc & et4000.gdcreg[8] ) | et4000.lc;
		if ( writemask2 & 8 ) pEt4000Vram[addr | 3] = ( vald & et4000.gdcreg[8] ) | et4000.ld;
		break;
	 case 0x18:					/* xor */
		if ( writemask2 & 1 ) pEt4000Vram[addr]     = ( vala & et4000.gdcreg[8] ) ^ et4000.la;
		if ( writemask2 & 2 ) pEt4000Vram[addr | 1] = ( valb & et4000.gdcreg[8] ) ^ et4000.lb;
		if ( writemask2 & 4 ) pEt4000Vram[addr | 2] = ( valc & et4000.gdcreg[8] ) ^ et4000.lc;
		if ( writemask2 & 8 ) pEt4000Vram[addr | 3] = ( vald & et4000.gdcreg[8] ) ^ et4000.ld;
		break;
	}
}


uint8_t	ET4000_Mem_ReadByte ( uint32_t offset )
{
	uint32_t addr = ET4000_MapAddr ( offset, false );
	int readplane = et4000.readplane;
	uint32_t latch_addr;
	uint8_t temp, temp2, temp3, temp4;

	if ( et4000.chain4 )
	{
		if ( addr >= ET4000_VRAM_SIZE )
			return 0xff;
		return pEt4000Vram[addr];
	}
	else if ( et4000.chain2_read )
	{
		readplane = ( readplane & 2 ) | ( addr & 1 );
		addr &= ~1;
		addr <<= 2;
	}
	else
	{
		addr <<= 2;
	}

	latch_addr = addr & ~3;
	if ( latch_addr >= ET4000_VRAM_SIZE )
	{
		et4000.la = et4000.lb = et4000.lc = et4000.ld = 0xff;
		return 0xff;
	}
	et4000.la = pEt4000Vram[latch_addr];
	et4000.lb = pEt4000Vram[latch_addr | 1];
	et4000.lc = pEt4000Vram[latch_addr | 2];
	et4000.ld = pEt4000Vram[latch_addr | 3];

	if ( addr >= ET4000_VRAM_SIZE )
		return 0xff;

	if ( et4000.readmode )
	{
		temp  = et4000.la ^ ( ( et4000.colourcompare & 1 ) ? 0xff : 0 );
		temp  &= ( et4000.colournocare & 1 ) ? 0xff : 0;
		temp2 = et4000.lb ^ ( ( et4000.colourcompare & 2 ) ? 0xff : 0 );
		temp2 &= ( et4000.colournocare & 2 ) ? 0xff : 0;
		temp3 = et4000.lc ^ ( ( et4000.colourcompare & 4 ) ? 0xff : 0 );
		temp3 &= ( et4000.colournocare & 4 ) ? 0xff : 0;
		temp4 = et4000.ld ^ ( ( et4000.colourcompare & 8 ) ? 0xff : 0 );
		temp4 &= ( et4000.colournocare & 8 ) ? 0xff : 0;
		return ~( temp | temp2 | temp3 | temp4 );
	}

	return pEt4000Vram[( addr & ~3 ) | readplane];
}


/*-----------------------------------------------------------------------*/
/* Rendering                                                             */
/*-----------------------------------------------------------------------*/

/**
 * Rebuild the 256 host colors from the DAC state (6 bit per gun as on the
 * ET4000AX standard VGA DAC)
 */
static void ET4000_RecalcHostPalette ( void )
{
	int i;

	for ( i = 0 ; i < 256 ; i++ )
	{
		int j = i & et4000.dac_mask;
		Et4000HostPal[i] = Screen_MapRGB ( ( et4000.vgapal[j][0] & 0x3f ) << 2,
		                                   ( et4000.vgapal[j][1] & 0x3f ) << 2,
		                                   ( et4000.vgapal[j][2] & 0x3f ) << 2 );
	}
	bEt4000PalDirty = false;
}


/**
 * Screen geometry derived from the CRTC (including the ET4000 extended
 * overflow bits in CRTC 0x33/0x35), evaluated per frame
 */
typedef struct {
	int width;			/* pixels */
	int height;			/* scanlines shown */
	int rowbytes;			/* VRAM bytes to the next character row */
	uint32_t ma_latch;		/* display start address (VRAM bytes) */
	int rowcount;			/* extra repeats of each character row */
	bool linedbl;
	int mode;			/* 0 = blank, 4 = 4bpp planar, 8 = 8bpp packed, -1 = unsupported */
} ET4000_GEO;

static void ET4000_GetGeometry ( ET4000_GEO *geo )
{
	int dispend = et4000.crtc[0x12];
	int hdisp = et4000.crtc[1] + 1;
	int rowoffset = et4000.crtc[0x13];

	if ( et4000.crtc[7] & 0x02 )	dispend |= 0x100;
	if ( et4000.crtc[7] & 0x40 )	dispend |= 0x200;
	if ( et4000.crtc[0x35] & 0x04 )	dispend += 0x400;	/* ET4000 extension */
	dispend++;

	if ( !rowoffset )
		rowoffset = 0x100;				/* ET4000: offset 0 means 256 */

	geo->ma_latch = ( ( et4000.crtc[0xc] << 8 ) | et4000.crtc[0xd] )
	                + ( ( et4000.crtc[8] & 0x60 ) >> 5 );
	geo->ma_latch |= ( et4000.crtc[0x33] & 3 ) << 16;	/* ET4000 extension */
	geo->ma_latch <<= 2;

	geo->rowbytes = rowoffset << 3;
	geo->rowcount = et4000.crtc[9] & 31;
	geo->linedbl = ( et4000.crtc[9] & 0x80 ) != 0;
	geo->height = dispend;

	/* Select the pixel mode the same way the VGA does */
	if ( ( et4000.seqregs[1] & 0x20 ) || !et4000.attr_palette_enable || !( et4000.vidsub & 1 ) )
	{
		geo->mode = 0;					/* screen blanked / card disabled */
		geo->width = hdisp << 3;
	}
	else if ( !( et4000.gdcreg[6] & 1 ) && !( et4000.attrregs[0x10] & 1 ) )
	{
		geo->mode = -1;					/* text mode: not emulated */
		geo->width = hdisp << 3;
	}
	else
	{
		switch ( et4000.gdcreg[5] & 0x60 )
		{
		 case 0x00:
			geo->mode = 4;				/* 16 color planar */
			geo->width = hdisp << 3;
			break;
		 case 0x40:
		 case 0x60:
			if ( et4000.bpp == 8 )
			{
				geo->mode = 8;			/* 256 color packed */
				geo->width = hdisp << 3;
				if ( et4000.attrregs[0x16] & 0x20 )	/* ET4000 hres doubling */
					geo->width <<= 1;
			}
			else
				geo->mode = -1;			/* HiColor: not emulated yet */
			break;
		 default:
			geo->mode = -1;				/* 2bpp CGA modes: not emulated */
			geo->width = hdisp << 3;
			break;
		}
	}

	if ( geo->width < 64 )		geo->width = 64;
	if ( geo->width > 2048 )	geo->width = 2048;
	if ( geo->height < 64 )		geo->height = 64;
	if ( geo->height > 1024 )	geo->height = 1024;
}


/**
 * Return true when the Hatari window should show the card instead of the
 * internal video: the card's video subsystem is enabled and a graphics
 * mode is programmed and unblanked (which only ever happens once a Nova
 * driver or EmuTOS has initialised the card).
 */
bool	ET4000_UseCardScreen ( void )
{
	if ( !pEt4000Vram )
		return false;
	if ( !( et4000.vidsub & 1 ) )			/* video subsystem off */
		return false;
	if ( et4000.seqregs[1] & 0x20 )			/* screen blanked */
		return false;
	if ( !et4000.attr_palette_enable )		/* ATC not enabled yet */
		return false;
	if ( !( et4000.gdcreg[6] & 1 ) )		/* still in text mode = not driven */
		return false;
	return true;
}


/**
 * Render the current frame, whole, into the host SDL frame buffer.
 * Called once per VBL from ConvST_Refresh when the card screen is active.
 * Nearest-neighbour scaling copes with any host surface size.
 */
void	ET4000_Render ( void )
{
	static int prev_w, prev_h;
	ET4000_GEO geo;
	uint32_t *hvram;
	int scrwidth, scrheight, pitch;
	int x, y;

	ET4000_GetGeometry ( &geo );

	if ( geo.mode == -1 && !( Et4000UnsupportedLogged & 1 ) )
	{
		Log_Printf(LOG_WARN, "ET4000: unsupported screen mode (gdc5=%02x bpp=%d seq1=%02x), showing blank\n",
		           et4000.gdcreg[5], et4000.bpp, et4000.seqregs[1]);
		Et4000UnsupportedLogged |= 1;
	}

	if ( geo.width != prev_w || geo.height != prev_h )
	{
		ConvGen_SetSize ( geo.width, geo.height, false );
		prev_w = geo.width;
		prev_h = geo.height;
	}

	if ( bEt4000PalDirty )
		ET4000_RecalcHostPalette ();

	if ( ConfigureParams.Screen.DisableVideo || !Screen_Lock() )
		return;

	Screen_GetDimension ( &hvram, NULL, NULL, &pitch );
	scrwidth = Screen_GetGenConvWidth ();
	scrheight = Screen_GetGenConvHeight ();
	pitch /= sizeof(uint32_t);

	for ( y = 0 ; y < scrheight ; y++ )
	{
		uint32_t *dst = hvram + y * pitch;
		int srcy = y * geo.height / scrheight;
		uint32_t base;

		/* character row repetition and line doubling shrink the effective row */
		if ( geo.linedbl )
			srcy >>= 1;
		if ( geo.rowcount )
			srcy /= ( geo.rowcount + 1 );

		base = ( geo.ma_latch + (uint32_t)srcy * geo.rowbytes ) & ET4000_VRAM_MASK;

		switch ( geo.mode )
		{
		 case 8:					/* 256 colors, packed */
			for ( x = 0 ; x < scrwidth ; x++ )
			{
				int srcx = x * geo.width / scrwidth;
				dst[x] = Et4000HostPal[ pEt4000Vram[( base + srcx ) & ET4000_VRAM_MASK] ];
			}
			break;

		 case 4:					/* 16 colors, 4 planes */
			for ( x = 0 ; x < scrwidth ; x++ )
			{
				int srcx = x * geo.width / scrwidth;
				uint32_t a = ( base + ( ( srcx >> 3 ) << 2 ) ) & ET4000_VRAM_MASK;
				int bit = 7 - ( srcx & 7 );
				int idx = ( ( pEt4000Vram[a]     >> bit ) & 1 )
				        | ( ( ( pEt4000Vram[a | 1] >> bit ) & 1 ) << 1 )
				        | ( ( ( pEt4000Vram[a | 2] >> bit ) & 1 ) << 2 )
				        | ( ( ( pEt4000Vram[a | 3] >> bit ) & 1 ) << 3 );
				idx &= et4000.plane_mask;
				/* through the attribute controller palette */
				idx = et4000.attrregs[idx & 0xf] & 0x3f;
				dst[x] = Et4000HostPal[idx];
			}
			break;

		 default:					/* blank / unsupported */
			for ( x = 0 ; x < scrwidth ; x++ )
				dst[x] = Et4000HostPal[0];
			break;
		}
	}

	Screen_UnLock ();
	Screen_GenConvUpdate ( true );
}


/*-----------------------------------------------------------------------*/
/**
 * Save/Restore snapshot of the ET4000 state
 */
void	ET4000_MemorySnapShot_Capture ( bool bSave )
{
	bool bAllocated = ( pEt4000Vram != NULL );

	MemorySnapShot_Store(&et4000, sizeof(et4000));
	MemorySnapShot_Store(&bAllocated, sizeof(bAllocated));

	if ( bAllocated )
	{
		if ( !bSave && !pEt4000Vram )
			ET4000_Init ();
		MemorySnapShot_Store(pEt4000Vram, ET4000_VRAM_SIZE);
	}

	if ( !bSave )
		bEt4000PalDirty = true;
}


/**
 * Show ET4000 state (debugger 'info vme')
 */
void	ET4000_Info ( FILE *fp, uint32_t arg )
{
	ET4000_GEO geo;

	ET4000_GetGeometry ( &geo );

	fprintf(fp, "ET4000 vidsub=%d key=%s miscout=0x%02x banking=0x%02x bpp=%d\n",
	        et4000.vidsub & 1, et4000.key_unlocked ? "unlocked" : "locked",
	        et4000.miscout, et4000.banking, et4000.bpp);
	fprintf(fp, "  mode=%d %dx%d rowbytes=%d ma=0x%x chain4=%d writemode=%d planemask=0x%x\n",
	        geo.mode, geo.width, geo.height, geo.rowbytes, geo.ma_latch,
	        et4000.chain4 ? 1 : 0, et4000.writemode, et4000.writemask);
	fprintf(fp, "  card screen: %s\n", ET4000_UseCardScreen() ? "active" : "inactive");
}
