/*
  Hatari - xbus.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  Expansion bus windows beyond the TT's own VME decode, for bus adapter
  hardware that decodes the top address byte (addr[31:24]) into distinct
  buses:

      0xFE  VME16 : A24:D16 - the TT's standard VME window (see vme_nova.c)
      0xFD  VME32 : A32:D32
      0xFC  ISA

  Only reachable with 32 bit addressing (TT, Falcon, or a MegaSTE set up
  with --addr24 off, e.g. for a 68030 accelerator): a 68000's 24 address
  lines never see these ranges.

  With --xbus trace, the 0xFC and 0xFD windows are backed by 16 MB of plain
  RAM each and every access is logged with the 'vme' trace flag (window,
  24 bit offset, size, R/W, value, PC) - the same recon approach as
  --vme trace: a driver probing for its card finds memory instead of a bus
  error, and the log shows the register map a card model must satisfy.
  The ISA memory / I-O split, byte lane order and bus widths inside these
  windows are not modelled yet: the trace is how to find them.
  Without it (--xbus none, the default) the windows stay bus errors.
*/
const char XBus_fileid[] = "Hatari xbus.c";

#include "main.h"
#include "configuration.h"
#include "log.h"
#include "m68000.h"
#include "xbus.h"

#define XBUS_MASK	(XBUS_WINDOW_SIZE - 1)

static uint8_t	*pIsaRam;			/* RAM image of the 0xFC window */
static uint8_t	*pVme32Ram;			/* RAM image of the 0xFD window */
static uint64_t	XBusReads[2], XBusWrites[2];	/* [0] ISA, [1] VME32 */


/**
 * Return true if the trace windows are enabled and the CPU can reach them
 */
bool	XBus_IsAvailable ( void )
{
	return ConfigureParams.System.nXBusType == XBUS_TYPE_TRACE
	       && !ConfigureParams.System.bAddressSpace24;
}


void	XBus_Init ( void )
{
	if ( !XBus_IsAvailable() )
		return;
	if ( !pIsaRam )
		pIsaRam = calloc ( 1, XBUS_WINDOW_SIZE );
	if ( !pVme32Ram )
		pVme32Ram = calloc ( 1, XBUS_WINDOW_SIZE );
	if ( !pIsaRam || !pVme32Ram )
		Main_ErrorExit ( "Out of memory (ISA / VME32 trace windows)", NULL, 1 );
}


void	XBus_UnInit ( void )
{
	free ( pIsaRam );
	pIsaRam = NULL;
	free ( pVme32Ram );
	pVme32Ram = NULL;
}


/**
 * The RAM images keep their content across a reset, only the statistics
 * restart.
 */
void	XBus_Reset ( bool bCold )
{
	XBusReads[0] = XBusReads[1] = 0;
	XBusWrites[0] = XBusWrites[1] = 0;
}


/**
 * Map a CPU address to its window: RAM pointer, window index and name
 */
static uint8_t *XBus_Decode ( uaecptr addr, int *pWin, const char **pName )
{
	uint32_t off = addr & XBUS_MASK;

	if ( ( addr & 0xff000000 ) == XBUS_ISA_START )
	{
		*pWin = 0;
		*pName = "isa";
		return pIsaRam + off;
	}
	*pWin = 1;
	*pName = "vme32";
	return pVme32Ram + off;
}

/* A word or long that runs past the end of a window wraps inside it */
#define XB(p, i, base)	( ( base )[ ( ( p ) - ( base ) + ( i ) ) & XBUS_MASK ] )

uae_u32 REGPARAM3 XBus_Mem_bget ( uaecptr addr )
{
	const char *name;
	int win;
	uint8_t *p = XBus_Decode ( addr, &win, &name );
	uint8_t val = p[0];

	XBusReads[win]++;
	LOG_TRACE(TRACE_VME, "%s rd.b $%06x val=0x%02x pc=%x\n", name, addr & XBUS_MASK, val, M68000_GetPC());
	return val;
}

uae_u32 REGPARAM3 XBus_Mem_wget ( uaecptr addr )
{
	const char *name;
	int win;
	uint8_t *p = XBus_Decode ( addr, &win, &name );
	uint8_t *base = win ? pVme32Ram : pIsaRam;
	uint16_t val = ( XB(p, 0, base) << 8 ) | XB(p, 1, base);

	XBusReads[win]++;
	LOG_TRACE(TRACE_VME, "%s rd.w $%06x val=0x%04x pc=%x\n", name, addr & XBUS_MASK, val, M68000_GetPC());
	return val;
}

uae_u32 REGPARAM3 XBus_Mem_lget ( uaecptr addr )
{
	const char *name;
	int win;
	uint8_t *p = XBus_Decode ( addr, &win, &name );
	uint8_t *base = win ? pVme32Ram : pIsaRam;
	uint32_t val = ( (uint32_t)XB(p, 0, base) << 24 ) | ( XB(p, 1, base) << 16 )
	             | ( XB(p, 2, base) << 8 ) | XB(p, 3, base);

	XBusReads[win]++;
	LOG_TRACE(TRACE_VME, "%s rd.l $%06x val=0x%08x pc=%x\n", name, addr & XBUS_MASK, val, M68000_GetPC());
	return val;
}

void REGPARAM3 XBus_Mem_bput ( uaecptr addr, uae_u32 val )
{
	const char *name;
	int win;
	uint8_t *p = XBus_Decode ( addr, &win, &name );

	p[0] = val;
	XBusWrites[win]++;
	LOG_TRACE(TRACE_VME, "%s wr.b $%06x val=0x%02x pc=%x\n", name, addr & XBUS_MASK, val & 0xff, M68000_GetPC());
}

void REGPARAM3 XBus_Mem_wput ( uaecptr addr, uae_u32 val )
{
	const char *name;
	int win;
	uint8_t *p = XBus_Decode ( addr, &win, &name );
	uint8_t *base = win ? pVme32Ram : pIsaRam;

	XB(p, 0, base) = val >> 8;
	XB(p, 1, base) = val;
	XBusWrites[win]++;
	LOG_TRACE(TRACE_VME, "%s wr.w $%06x val=0x%04x pc=%x\n", name, addr & XBUS_MASK, val & 0xffff, M68000_GetPC());
}

void REGPARAM3 XBus_Mem_lput ( uaecptr addr, uae_u32 val )
{
	const char *name;
	int win;
	uint8_t *p = XBus_Decode ( addr, &win, &name );
	uint8_t *base = win ? pVme32Ram : pIsaRam;

	XB(p, 0, base) = val >> 24;
	XB(p, 1, base) = val >> 16;
	XB(p, 2, base) = val >> 8;
	XB(p, 3, base) = val;
	XBusWrites[win]++;
	LOG_TRACE(TRACE_VME, "%s wr.l $%06x val=0x%08x pc=%x\n", name, addr & XBUS_MASK, val, M68000_GetPC());
}

int REGPARAM3 XBus_Mem_check ( uaecptr addr, uae_u32 size )
{
	return 0;					/* no direct access */
}

uae_u8 * REGPARAM3 XBus_Mem_xlate ( uaecptr addr )
{
	const char *name;
	int win;

	return XBus_Decode ( addr, &win, &name );
}


void	XBus_Info ( FILE *fp, uint32_t arg )
{
	if ( ConfigureParams.System.nXBusType != XBUS_TYPE_TRACE )
	{
		fprintf(fp, "ISA (0xFC) / VME32 (0xFD) windows: none (bus errors; --xbus trace to enable)\n");
		return;
	}
	if ( ConfigureParams.System.bAddressSpace24 )
	{
		fprintf(fp, "ISA (0xFC) / VME32 (0xFD) windows: enabled, but the CPU has 24 bit addressing (--addr24 off)\n");
		return;
	}
	fprintf(fp, "ISA   window 0x%08x-0x%08x: trace, %llu reads, %llu writes since reset\n",
	        XBUS_ISA_START, XBUS_ISA_START + XBUS_WINDOW_SIZE - 1,
	        (unsigned long long)XBusReads[0], (unsigned long long)XBusWrites[0]);
	fprintf(fp, "VME32 window 0x%08x-0x%08x: trace, %llu reads, %llu writes since reset\n",
	        XBUS_VME32_START, XBUS_VME32_START + XBUS_WINDOW_SIZE - 1,
	        (unsigned long long)XBusReads[1], (unsigned long long)XBusWrites[1]);
}
