/*
  Hatari - vme_atw800.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  ATW800/2 "Seurat" graphics emulation (VME / Mega bus card).

  The ATW800/2 (geekdot.com, "The Transputer Gentlemen") is an FPGA
  graphics + Transputer card for the Mega ST bus and the MegaSTE/TT VME
  bus. Only the graphics half ("Seurat") is emulated here; the Transputer
  link interfaces respond as "nothing connected". The register model
  follows the ATW800/2 Programmer's Manual V1.0a (memory map p.5, VTG and
  LUT pp.6-7 and 12-14, including its set_adr() / set_fpga() example
  sources) plus a register probe of a real 2MB card.

  Model: a 2 MB card with the ADDR jumper at 0x(FE)A00000 — the variant
  measured on real hardware. All FPGA structures live at the TOP of the
  video memory:

    +0x1FF000  LUT       256 x 16bit RGB565 (big-endian words), R/W
    +0x1FF200  FPGA info 32-byte version string, " cpm" at offset 24
                         (the card's ID, 0x2063706D)
    +0x1FF800  VTG       video timing generator register file (word regs)
    +0x1FF900  blitter   2D copy engine, verified on a real V0205 card
                         (VRAM read back from Atari System V, 2026-09-24):
                         +0x00 src addr (long), +0x04 dst addr (long),
                         +0x08 src stride (word, signed), +0x0A dst
                         stride (word, signed), +0x0C width in bytes,
                         +0x0E row count, +0x10 command: bit 0 = GO,
                         bit 1 = backwards, bit 2 = fill; 1 = copy
                         ascending, 3 = copy DESCENDING (src/dst = the
                         last byte, negative strides), 5 = fill from one
                         source row (sstride 0). Write-only: every offset
                         reads back the status word, bit 0 = busy (the
                         real engine is asynchronous; ~38 MB/s copy).
                         Emulated blits complete at once, status 0.
    +0x1FFAC0  FPGA link virtual Transputer C011 interface: status reads
                         return 0 = no data / not ready

  A 2 MB card decodes ONLY its 2 MB window: the 4 MB-offset probes of
  set_adr() (e.g. 0xDFF218 on a MegaSTE) fall outside and must BUS ERROR
  — that is how a driver sizes the memory (the manual's set_adr()
  probes with the bus-error vector guarded).

  The card's bus CPLD ("Absinth") also decodes a small register block
  OUTSIDE the video window: the CPLD version register (low 3 bits) at
  0x00DFFA98 (MegaSTE) / 0xFEFFFAD8 (TT) / 0x000FFAD8 (Mega ST) and the
  physical C011 Transputer link of the "+T" model nearby. The CPLD
  version must answer: a driver may read it unguarded, and a bus error
  there costs it two bombs. The C011
  registers answer 0 = no Transputer.

  VTG control register (manual p.12-13, "-gfe dcba"):
    bit0 a = VTG enable          bit3 d = 0->1 transfers PLL registers
    bit1 b = positive Hsync      bits5:4 fe = depth: 00=1bpp 01=8bpp+LUT
    bit2 c = positive Vsync                  10=16bpp 11=15bpp (V1.0a;
                                             32bpp on V0205+, see Render)
    bit6 g = LUT select (0 = Atari, 1 = Transputer)
  8bpp with LUT enabled = ctrl 0x19.
*/
const char VmeAtw800_fileid[] = "Hatari vme_atw800.c";

#include "main.h"
#include "configuration.h"
#include "conv_gen.h"
#include "log.h"
#include "m68000.h"
#include "memorySnapShot.h"
#include "screen.h"
#include "statusbar.h"
#include "vme_atw800.h"

/*
 * Video memory size. The A0/A1 jumpers (CPLD firmware 2+) select a 2 MB
 * window, or - both closed, TT only - a 4 MB window at 0x(FE)A00000
 * (--vme-vram 4 with --vme-base 0xA00000). With the 4 MB window, the
 * card still starts in a 2 MB layout: the upper 2 MB MIRROR the lower
 * (measured on a real V0205 card with register 15 = 1: identical data at
 * +2 MB and the id block at both 0xFEBFF200 and 0xFEDFF200). Register
 * 15 = 3 (bit 1: "A21" by analogy with nano_vtg.h's "A20GATE 0 = 1 MB,
 * 1 = 2 MB") switches to the full 4 MB, with the FPGA structures at the
 * top of the 4 MB (measured on a real card: write 1 via both aliases,
 * write 3 via the lower, look for " cpm" at 0xFEDFF218).
 * The FPGA structures always sit at the top of the CURRENT size.
 */
#define ATW_VRAM_ALLOC	0x400000		/* the 4 MB card's worth */
#define ATW_WINDOW	( ConfigureParams.System.nVMEVram == 4 ? 0x400000u : 0x200000u )
#define ATW_VRAM_SIZE	AtwSize			/* current layout: 2 or 4 MB */
#define ATW_VRAM_MASK	(ATW_VRAM_SIZE-1)
#define ATW_BASE_A24	((uint32_t)ConfigureParams.System.nVMEBase)	/* ADDR jumper: --vme-base */

/* FPGA structure offsets inside the video memory (top of the current size) */
#define ATW_LUT_OFF	(ATW_VRAM_SIZE - 0x1000)	/* 0x1FF000 / 0x3FF000 */
#define ATW_LUT_SIZE	0x200
#define ATW_INFO_OFF	(ATW_VRAM_SIZE - 0xE00)		/* 0x1FF200 */
#define ATW_INFO_SIZE	0x20
#define ATW_VTG_OFF	(ATW_VRAM_SIZE - 0x800)		/* 0x1FF800 */
#define ATW_VTG_SIZE	0x20
#define ATW_BLIT_OFF	(ATW_VRAM_SIZE - 0x700)		/* 0x1FF900 */
#define ATW_BLIT_SIZE	0x40
#define ATW_LINK_OFF	(ATW_VRAM_SIZE - 0x540)		/* 0x1FFAC0 */
#define ATW_LINK_SIZE	0x20
#define ATW_AUX_OFF	0x400000		/* pseudo-offset: CPLD/C011 block */
#define ATW_CPLD_VERSION	4		/* firmware version, low 3 bits */

/* VTG word register indices (offset/2) */
#define VTG_CTRL	0x00
#define VTG_HFP		0x01
#define VTG_HSY		0x02
#define VTG_HBP		0x03
#define VTG_HDI		0x04
#define VTG_VFP		0x05
#define VTG_VSY		0x06
#define VTG_VBP		0x07
#define VTG_VDI		0x08
#define VTG_PLLFB	0x09
#define VTG_PLLID	0x0A
#define VTG_PLLOD	0x0B
#define VTG_VMEM_LO	0x0C
#define VTG_VMEM_HI	0x0D
#define VTG_MEM_REG	0x0F

#define VTG_CTRL_ENABLE	0x01
#define VTG_CTRL_DEPTH(c)	( ( (c) >> 4 ) & 3 )

/* 32-byte FPGA info block. The long at byte 24 must read " cpm"
 * (0x2063706D), as on a real card (the manual's "_cpm" comment is
 * wrong, its own question mark was earned).
 * The rest imitates the real block's "build date, version and
 * copyright" content. */
static const char AtwInfoBlock[ATW_INFO_SIZE] =
	"Seurat v0106 Hatari.\0\0\0\0 cpm\0\0\0";

static uint8_t	*pAtwVram;
static uint16_t	AtwVtg[ATW_VTG_SIZE/2];
static uint16_t	AtwLut[256];
static uint8_t	AtwBlitRegs[ATW_BLIT_SIZE];
static uint16_t	AtwMemReg;
static uint32_t	AtwSize = 0x200000;		/* current layout, from AtwMemReg */

static void AtwUpdateSize ( void )
{
	AtwSize = ( ConfigureParams.System.nVMEVram == 4 && ( AtwMemReg & 2 ) )
	          ? 0x400000 : 0x200000;
}

static uint32_t	AtwHostPal[256];		/* LUT -> host pixels */

/* Big-endian register fetch out of the blit register file */
static uint32_t AtwBlitLong ( int off )
{
	return ( (uint32_t)AtwBlitRegs[off] << 24 ) | ( AtwBlitRegs[off+1] << 16 )
	     | ( AtwBlitRegs[off+2] << 8 ) | AtwBlitRegs[off+3];
}
static int16_t AtwBlitWord ( int off )
{
	return (int16_t)( ( AtwBlitRegs[off] << 8 ) | AtwBlitRegs[off+1] );
}

/*
 * Execute a blit, instantly. The register model and the commands were
 * verified on a real V0205 card (2026-09-24, VRAM read back through
 * /dev/mem from Atari System V):
 *   bit 0 = go, bit 1 = backwards, bit 2 = fill
 *   1 = copy, addresses ascending from src/dst
 *   3 = copy DESCENDING: src/dst name the LAST byte of the first row
 *       walked, each row is copied downwards, strides are negative
 *   5 = fill (src stride 0): each dst row gets the first 64 src bytes,
 *       then the src's second 32-byte block repeated - a 64-byte source
 *       of one colour fills any width (measured: a 1024-byte row from a
 *       64-byte pattern came out bytes 0-63, then 32-63 over and over)
 * The real engine runs asynchronously (the CPU is not held off; drivers
 * wait on the busy bit); here it completes at once.
 */
static void ATW800_DoBlit ( void )
{
	uint32_t src = AtwBlitLong ( 0x00 ) & ATW_VRAM_MASK;
	uint32_t dst = AtwBlitLong ( 0x04 ) & ATW_VRAM_MASK;
	int32_t sstride = AtwBlitWord ( 0x08 );
	int32_t dstride = AtwBlitWord ( 0x0A );
	uint16_t width  = (uint16_t)AtwBlitWord ( 0x0C );
	uint16_t rows   = (uint16_t)AtwBlitWord ( 0x0E );
	uint16_t cmd    = (uint16_t)AtwBlitWord ( 0x10 );
	uint16_t y, x;

	LOG_TRACE(TRACE_VME, "vme atw blit run cmd=0x%04x src=$%06x dst=$%06x sstr=%d dstr=%d w=%u h=%u\n",
	          cmd, src, dst, sstride, dstride, width, rows);

	if ( width != 0 )
		for ( y = 0 ; y < rows ; y++ )
		{
			if ( cmd & 4 )		/* fill: 64 bytes, then 32-byte tiles */
				for ( x = 0 ; x < width ; x++ )
					pAtwVram[( dst + x ) & ATW_VRAM_MASK] =
						pAtwVram[( src + ( x < 64 ? x : 32 + ( x & 31 ) ) ) & ATW_VRAM_MASK];
			else if ( cmd & 2 )	/* backwards: from the last byte down */
				for ( x = 0 ; x < width ; x++ )
					pAtwVram[( dst - x ) & ATW_VRAM_MASK] =
						pAtwVram[( src - x ) & ATW_VRAM_MASK];
			else if ( src + width <= ATW_VRAM_SIZE && dst + width <= ATW_VRAM_SIZE )
				memmove ( pAtwVram + dst, pAtwVram + src, width );
			src = ( src + sstride ) & ATW_VRAM_MASK;
			dst = ( dst + dstride ) & ATW_VRAM_MASK;
		}
}

static bool	bAtwPalDirty;


void	ATW800_Init ( void )
{
	if ( !pAtwVram )
	{
		pAtwVram = malloc ( ATW_VRAM_ALLOC );
		if ( !pAtwVram )
			Main_ErrorExit ( "Out of memory (ATW800/2 video memory)", NULL, 1 );
		memset ( pAtwVram, 0, ATW_VRAM_ALLOC );
	}
	AtwUpdateSize ();
	bAtwPalDirty = true;
}


void	ATW800_UnInit ( void )
{
	free ( pAtwVram );
	pAtwVram = NULL;
}


void	ATW800_Reset ( bool bCold )
{
	/* The FPGA keeps running across an Atari warm reset (a frozen
	 * picture survives it); only a cold
	 * boot clears the mode. */
	if ( bCold )
	{
		memset ( AtwVtg, 0, sizeof(AtwVtg) );
		memset ( AtwLut, 0, sizeof(AtwLut) );
		AtwMemReg = 0;
		AtwUpdateSize ();
		if ( pAtwVram )
			memset ( pAtwVram, 0, ATW_VRAM_ALLOC );
		bAtwPalDirty = true;
	}
}


/**
 * The card responds only inside its VidMem window (2 or 4 MB, per the
 * jumpers) at the ADDR jumper base. Everything else on the bus stays
 * bus-error — including, on a 2 MB window, the 4 MB-offset register
 * addresses, which is how set_adr() discovers a 2 MB card. Inside the
 * window, a 2 MB layout mirrors (see ATW_VRAM_ALLOC).
 */
bool	ATW800_Decode ( uint32_t addr24, uint32_t *pOffset )
{
	if ( addr24 >= ATW_BASE_A24 && addr24 < ATW_BASE_A24 + ATW_WINDOW )
	{
		*pOffset = ( addr24 - ATW_BASE_A24 ) & ATW_VRAM_MASK;
		return true;
	}
	/* CPLD / C011 register block: 0xDFFA80-0xDFFAFF on the MegaSTE,
	 * 0x(FE)FFFA80-0x(FE)FFFAFF on the TT. Mapped to pseudo-offsets
	 * ATW_AUX_OFF+reg so the byte handlers can tell them apart. */
	if ( ( addr24 >= 0x00DFFA80 && addr24 < 0x00DFFB00 )
	  || ( addr24 >= 0x00FFFA80 && addr24 < 0x00FFFB00 ) )
	{
		*pOffset = ATW_AUX_OFF | ( addr24 & 0xFF );
		return true;
	}
	return false;
}


uint8_t	ATW800_ReadByte ( uint32_t offset )
{
	if ( offset & ATW_AUX_OFF )
	{
		int reg = offset & 0xFF;
		LOG_TRACE(TRACE_VME, "vme atw aux rd $%02x pc=%x\n", reg, M68000_GetPC());
		/* CPLD version register (word): MegaSTE 0x98, TT / Mega ST 0xD8 */
		if ( reg == 0x99 || reg == 0xD9 )
			return ATW_CPLD_VERSION;
		return 0;				/* C011: no data, not ready */
	}
	if ( offset >= ATW_LUT_OFF && offset < ATW_LUT_OFF + ATW_LUT_SIZE )
	{
		uint16_t w = AtwLut[ ( offset - ATW_LUT_OFF ) >> 1 ];
		return ( offset & 1 ) ? ( w & 0xff ) : ( w >> 8 );
	}
	if ( offset >= ATW_INFO_OFF && offset < ATW_INFO_OFF + ATW_INFO_SIZE )
	{
		uint8_t v = AtwInfoBlock[ offset - ATW_INFO_OFF ];
		LOG_TRACE(TRACE_VME, "vme atw info rd $%06x val=0x%02x pc=%x\n", offset, v, M68000_GetPC());
		return v;
	}
	/* The VTG registers are write-only: a read returns the video memory
	 * underneath (measured on a V0205 card: 0xFEDFF800.. read back the
	 * screen's own pattern), so a driver cannot read a mode back. */
	if ( offset >= ATW_BLIT_OFF && offset < ATW_BLIT_OFF + ATW_BLIT_SIZE )
	{
		/* The registers are write-only: every offset of the window
		 * reads back the engine's status word (measured on V0205), bit
		 * 0 = busy. The emulated engine is always done. */
		LOG_TRACE(TRACE_VME, "vme atw blit rd $%06x pc=%x\n", offset, M68000_GetPC());
		return 0;
	}
	if ( offset >= ATW_LINK_OFF && offset < ATW_LINK_OFF + ATW_LINK_SIZE )
	{
		LOG_TRACE(TRACE_VME, "vme atw link rd $%06x pc=%x\n", offset, M68000_GetPC());
		return 0;				/* no Transputer: no data, not ready */
	}
	return pAtwVram[ offset & ATW_VRAM_MASK ];
}


void	ATW800_WriteByte ( uint32_t offset, uint8_t val )
{
	if ( offset & ATW_AUX_OFF )
	{
		LOG_TRACE(TRACE_VME, "vme atw aux wr $%02x val=0x%02x pc=%x\n",
		          (int)( offset & 0xFF ), val, M68000_GetPC());
		return;					/* CPLD/C011 writes: accepted, ignored */
	}
	if ( offset >= ATW_LUT_OFF && offset < ATW_LUT_OFF + ATW_LUT_SIZE )
	{
		int idx = ( offset - ATW_LUT_OFF ) >> 1;
		if ( offset & 1 )
			AtwLut[idx] = ( AtwLut[idx] & 0xff00 ) | val;
		else
			AtwLut[idx] = ( AtwLut[idx] & 0x00ff ) | ( val << 8 );
		bAtwPalDirty = true;
		LOG_TRACE(TRACE_VME, "vme atw lut wr $%06x val=0x%02x pc=%x\n", offset, val, M68000_GetPC());
		return;
	}
	if ( offset >= ATW_INFO_OFF && offset < ATW_INFO_OFF + ATW_INFO_SIZE )
		return;					/* version block is read-only */
	if ( offset >= ATW_VTG_OFF && offset < ATW_VTG_OFF + ATW_VTG_SIZE )
	{
		int reg = ( offset - ATW_VTG_OFF ) >> 1;
		uint16_t *pw = ( reg == VTG_MEM_REG ) ? &AtwMemReg : &AtwVtg[reg];
		if ( offset & 1 )
			*pw = ( *pw & 0xff00 ) | val;
		else
			*pw = ( *pw & 0x00ff ) | ( val << 8 );
		if ( reg == VTG_MEM_REG )
			AtwUpdateSize ();
		LOG_TRACE(TRACE_VME, "vme atw vtg wr $%06x val=0x%02x pc=%x\n", offset, val, M68000_GetPC());
		if ( reg == VTG_CTRL && ( offset & 1 ) )
			LOG_TRACE(TRACE_VME, "vme atw vtg ctrl=0x%04x %dx%d depth=%d pc=%x\n",
			          AtwVtg[VTG_CTRL], AtwVtg[VTG_HDI], AtwVtg[VTG_VDI],
			          VTG_CTRL_DEPTH(AtwVtg[VTG_CTRL]), M68000_GetPC());
		return;
	}
	if ( offset >= ATW_BLIT_OFF && offset < ATW_BLIT_OFF + ATW_BLIT_SIZE )
	{
		LOG_TRACE(TRACE_VME, "vme atw blit wr $%06x val=0x%02x pc=%x\n", offset, val, M68000_GetPC());
		AtwBlitRegs[ offset - ATW_BLIT_OFF ] = val;
		/* GO bit in the command word low byte: run the blit now */
		if ( offset - ATW_BLIT_OFF == 0x11 && ( val & 1 ) )
			ATW800_DoBlit ();
		return;
	}
	if ( offset >= ATW_LINK_OFF && offset < ATW_LINK_OFF + ATW_LINK_SIZE )
	{
		LOG_TRACE(TRACE_VME, "vme atw link wr $%06x val=0x%02x pc=%x\n", offset, val, M68000_GetPC());
		return;
	}
	pAtwVram[ offset & ATW_VRAM_MASK ] = val;
}


/**
 * LUT entry format measured on the real card: RGB565 in a big-endian
 * word (measured).
 */
static void ATW800_RecalcHostPalette ( void )
{
	int i;

	for ( i = 0 ; i < 256 ; i++ )
	{
		uint16_t w = AtwLut[i];
		uint8_t r = ( ( w >> 11 ) & 0x1f ) << 3;
		uint8_t g = ( ( w >> 5 ) & 0x3f ) << 2;
		uint8_t b = ( w & 0x1f ) << 3;
		AtwHostPal[i] = Screen_MapRGB ( r | ( r >> 5 ), g | ( g >> 6 ), b | ( b >> 5 ) );
	}
	bAtwPalDirty = false;
}


bool	ATW800_UseCardScreen ( void )
{
	if ( !pAtwVram )
		return false;
	if ( !( AtwVtg[VTG_CTRL] & VTG_CTRL_ENABLE ) )
		return false;
	if ( AtwVtg[VTG_HDI] < 64 || AtwVtg[VTG_VDI] < 64 )
		return false;
	return true;
}


/**
 * Render the card frame whole, once per VBL (same scheme as
 * ET4000_Render). Depths: 1bpp and 8bpp through the LUT, and the two
 * direct-colour codes:
 *   2 = 16bpp. The manual's word is G2G1G0B4B3B2B1B0 R4R3R2R1R0G5G4G3,
 *       i.e. a LITTLE-endian RGB565 word (the even byte holds the low
 *       half) - unlike the LUT, which is a big-endian RGB565 word.
 *       Confirmed on a real card.
 *   3 = 32bpp on firmware V0205+ (ctrl 0x39, bypl = 4 x width; the
 *       V1.0a manual still calls code 3 15bpp). Bytes R, G, B, x in memory order - undocumented, and
 *       measured on a real V0205 card (atari-sysv-sp1 tools/atw/atwtest
 *       on a monitor, 2026-09-24; the 16bpp order was confirmed the same
 *       way). A guess of B, G, R, x by analogy with 16bpp was WRONG.
 */
static uint32_t AtwDirR[256], AtwDirG[256], AtwDirB[256];	/* host pixel parts */
static bool	bAtwDirReady;

static void ATW800_InitDirect ( void )
{
	int i;

	/* host formats are packed channels, so a pixel is the OR of its
	 * three single-channel mappings */
	for ( i = 0 ; i < 256 ; i++ )
	{
		AtwDirR[i] = Screen_MapRGB ( i, 0, 0 );
		AtwDirG[i] = Screen_MapRGB ( 0, i, 0 );
		AtwDirB[i] = Screen_MapRGB ( 0, 0, i );
	}
	bAtwDirReady = true;
}

void	ATW800_Render ( void )
{
	static int prev_w, prev_h;
	uint32_t *hvram;
	int scrwidth, scrheight, pitch;
	int width, height, depth, rowbytes;
	uint32_t vbase;
	int x, y;

	width  = AtwVtg[VTG_HDI];
	height = AtwVtg[VTG_VDI];
	depth  = VTG_CTRL_DEPTH ( AtwVtg[VTG_CTRL] );
	vbase  = ( (uint32_t)( AtwVtg[VTG_VMEM_HI] & 0x7f ) << 16 ) | AtwVtg[VTG_VMEM_LO];
	/* A start of 1 does NOT move the picture on a real V0205 card (32 bpp
	 * test pattern, 2026-09-24): the low bits are ignored, or the unit
	 * is a 32-bit word. Which one is not measured yet; ignoring the low
	 * two bits is right for either as long as the start is below 4. */
	vbase &= ~3u;

	if ( width > 2048 )	width = 2048;
	if ( height > 1200 )	height = 1200;
	rowbytes = ( depth == 0 ) ? width / 8 : width << ( depth - 1 );	/* 1, 2, 4 bytes */

	if ( width != prev_w || height != prev_h )
	{
		ConvGen_SetSize ( width, height, false );
		prev_w = width;
		prev_h = height;
	}

	if ( bAtwPalDirty )
	{
		ATW800_RecalcHostPalette ();
		bAtwDirReady = false;		/* host format may have changed too */
	}
	if ( depth >= 2 && !bAtwDirReady )
		ATW800_InitDirect ();

	if ( ConfigureParams.Screen.DisableVideo || !Screen_Lock() )
		return;

	Screen_GetDimension ( &hvram, NULL, NULL, &pitch );
	scrwidth = Screen_GetGenConvWidth ();
	scrheight = Screen_GetGenConvHeight ();
	pitch /= sizeof(uint32_t);

	for ( y = 0 ; y < scrheight ; y++ )
	{
		uint32_t *dst = hvram + y * pitch;
		int srcy = y * height / scrheight;
		uint32_t base = ( vbase + (uint32_t)srcy * rowbytes ) & ATW_VRAM_MASK;

		switch ( depth )
		{
		 case 1:				/* 8bpp through the LUT */
			for ( x = 0 ; x < scrwidth ; x++ )
			{
				int srcx = x * width / scrwidth;
				dst[x] = AtwHostPal[ pAtwVram[( base + srcx ) & ATW_VRAM_MASK] ];
			}
			break;

		 case 0:				/* 1bpp: LUT entries 0/1 */
			for ( x = 0 ; x < scrwidth ; x++ )
			{
				int srcx = x * width / scrwidth;
				uint8_t byte = pAtwVram[( base + ( srcx >> 3 ) ) & ATW_VRAM_MASK];
				dst[x] = AtwHostPal[ ( byte >> ( 7 - ( srcx & 7 ) ) ) & 1 ];
			}
			break;

		 case 2:				/* 16bpp: little-endian RGB565 */
			for ( x = 0 ; x < scrwidth ; x++ )
			{
				uint32_t a = ( base + ( ( x * width / scrwidth ) << 1 ) ) & ATW_VRAM_MASK;
				uint16_t w = pAtwVram[a] | ( pAtwVram[( a + 1 ) & ATW_VRAM_MASK] << 8 );
				uint8_t r = ( w >> 11 ) << 3, g = ( ( w >> 5 ) & 0x3f ) << 2, b = ( w & 0x1f ) << 3;
				dst[x] = AtwDirR[r | ( r >> 5 )] | AtwDirG[g | ( g >> 6 )] | AtwDirB[b | ( b >> 5 )];
			}
			break;

		 default:				/* 32bpp: bytes R, G, B, x (measured) */
			for ( x = 0 ; x < scrwidth ; x++ )
			{
				uint32_t a = ( base + ( ( x * width / scrwidth ) << 2 ) ) & ATW_VRAM_MASK;
				dst[x] = AtwDirR[pAtwVram[a]]
				       | AtwDirG[pAtwVram[( a + 1 ) & ATW_VRAM_MASK]]
				       | AtwDirB[pAtwVram[( a + 2 ) & ATW_VRAM_MASK]];
			}
			break;
		}
	}

	Screen_UnLock ();
	Screen_GenConvUpdate ( true );
}


void	ATW800_MemorySnapShot_Capture ( bool bSave )
{
	bool bAllocated = ( pAtwVram != NULL );

	MemorySnapShot_Store(&AtwVtg, sizeof(AtwVtg));
	MemorySnapShot_Store(&AtwLut, sizeof(AtwLut));
	MemorySnapShot_Store(&AtwBlitRegs, sizeof(AtwBlitRegs));
	MemorySnapShot_Store(&AtwMemReg, sizeof(AtwMemReg));
	MemorySnapShot_Store(&bAllocated, sizeof(bAllocated));

	if ( bAllocated )
	{
		if ( !bSave && !pAtwVram )
			ATW800_Init ();
		MemorySnapShot_Store(pAtwVram, ATW_VRAM_ALLOC);
	}

	if ( !bSave )
	{
		AtwUpdateSize ();
		bAtwPalDirty = true;
	}
}


void	ATW800_Info ( FILE *fp, uint32_t arg )
{
	fprintf(fp, "ATW800/2 Seurat: %uMB window at A24 0x%06x, %uMB layout, VTG ctrl=0x%04x\n",
	        ATW_WINDOW >> 20, ATW_BASE_A24, AtwSize >> 20, AtwVtg[VTG_CTRL]);
	fprintf(fp, "  mode: %dx%d depth-code=%d (0=1bpp 1=8bpp 2=16bpp 3=32bpp) vmem=0x%x memreg=0x%x\n",
	        AtwVtg[VTG_HDI], AtwVtg[VTG_VDI], VTG_CTRL_DEPTH(AtwVtg[VTG_CTRL]),
	        ( (uint32_t)( AtwVtg[VTG_VMEM_HI] & 0x7f ) << 16 ) | AtwVtg[VTG_VMEM_LO],
	        AtwMemReg);
	fprintf(fp, "  card screen: %s\n", ATW800_UseCardScreen() ? "active" : "inactive");
}
