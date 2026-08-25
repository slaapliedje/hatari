/*
  Hatari - vme_nova.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  VME bus expansion card emulation for MegaSTE and TT.

  The MegaSTE and TT decode two windows onto the VME bus (see also the
  notes in scu_vme.c and cpu/memory.c):
  - TT :      0xFE000000-0xFEFEFFFF A24:D16 and 0xFEFF0000-0xFEFFFFFF A16:D16
  - MegaSTE : 0x00A00000-0x00DEFFFF A24:D16 and 0x00DF0000-0x00DFFFFF A16:D16
  On both machines the low 24 bits of the CPU address are the VME A24
  address, so a card responds at the same VME address whichever host it
  is plugged into (e.g. the Nova/ET4000 framebuffer at A24 0xA00000 is
  seen at 0x00A00000 on a MegaSTE and at 0xFEA00000 on a TT).

  When no VME card emulation is enabled these windows return bus errors
  (mapped to the bus error bank in cpu/memory.c).

  With --vme trace, the windows are backed by plain RAM images of the
  A24 and A16 address spaces and every access is logged with the
  'vme' trace flag (addr, size, R/W, value, PC).  This is a recon tool :
  booting a Nova / NVDI ET4000 driver against it produces the
  register map a real card model must satisfy, without the driver
  crashing on a bus error when it probes for its hardware.

  References :
    - Atari TT030 Hardware Reference Manual - June 1990
    - The Atari Compendium, memory map pages 737 / 754 / 761 / 766
    - a register probe of a real ATW800/2 card in a MegaSTE
*/
const char VmeNova_fileid[] = "Hatari vme_nova.c";

#include "main.h"
#include "configuration.h"
#include "log.h"
#include "m68000.h"
#include "memorySnapShot.h"
#include "video_et4000.h"
#include "vme_nova.h"


#define VME_A24_SIZE	0x1000000		/* full 16 MB VME A24 address space */
#define VME_A24_MASK	(VME_A24_SIZE-1)
#define VME_A16_SIZE	0x10000			/* 64 kB VME A16 address space */
#define VME_A16_MASK	(VME_A16_SIZE-1)

/* A16:D16 window position inside the host's address space */
#define VME_A16_START_TT	0xFEFF0000
#define VME_A16_START_MEGASTE	0x00DF0000
#define VME_A16_END_MEGASTE	0x00E00000

/* Nova/ET4000 VME adapter address decode (MegaSTE and TT layout, the one
   EmuTOS calls "Nova/ET4000 in Atari MegaSTe or TT"): the ET4000's ISA I/O
   ports appear at A24 0xDC0000 + port, the video memory as a linear 1 MB
   window at A24 0xC00000. Everything else on the bus stays bus-error, which
   the drivers' probe loops rely on. */
#define VME_NOVA_REG_BASE	0x00DC0000
#define VME_NOVA_REG_SIZE	0x00010000
#define VME_NOVA_MEM_BASE	0x00C00000
#define VME_NOVA_MEM_SIZE	0x00100000

static uint8_t	*pVmeA24Ram;			/* RAM image of the A24 space (trace mode) */
static uint8_t	*pVmeA16Ram;			/* RAM image of the A16 space (trace mode) */

static uint64_t	VmeReadCount;			/* access statistics for the 'info vme' command */
static uint64_t	VmeWriteCount;


/**
 * Return true if a VME card emulation is enabled on a machine with a VME bus
 */
bool	VME_IsAvailable ( void )
{
	return ( Config_IsMachineTT() || Config_IsMachineMegaSTE() )
	       && ( ConfigureParams.System.nVMEType != VME_TYPE_NONE );
}


/**
 * Allocate the VME address space images. Called when cpu/memory.c maps
 * the VME windows to the VME bank; can be called several times.
 */
void	VME_Init ( void )
{
	if ( !VME_IsAvailable() )
		return;

	if ( ConfigureParams.System.nVMEType == VME_TYPE_ET4000 )
	{
		ET4000_Init ();
		return;
	}

	if ( !pVmeA24Ram )
	{
		pVmeA24Ram = malloc ( VME_A24_SIZE );
		if ( pVmeA24Ram )
			memset ( pVmeA24Ram, 0, VME_A24_SIZE );
	}
	if ( !pVmeA16Ram )
	{
		pVmeA16Ram = malloc ( VME_A16_SIZE );
		if ( pVmeA16Ram )
			memset ( pVmeA16Ram, 0, VME_A16_SIZE );
	}
	if ( !pVmeA24Ram || !pVmeA16Ram )
	{
		Main_ErrorExit ( "Out of memory (VME address space)", NULL, 1 );
	}
}


void	VME_UnInit ( void )
{
	free ( pVmeA24Ram );
	pVmeA24Ram = NULL;
	free ( pVmeA16Ram );
	pVmeA16Ram = NULL;
	ET4000_UnInit ();
}


/**
 * Reset the VME card. The RAM images keep their content (as real VRAM /
 * card registers would across a warm reset), only the statistics restart.
 */
void	VME_Reset ( bool bCold )
{
	if ( !VME_IsAvailable() )
		return;

	VmeReadCount = 0;
	VmeWriteCount = 0;

	if ( ConfigureParams.System.nVMEType == VME_TYPE_ET4000 )
		ET4000_Reset ( bCold );
}


/**
 * Translate a host CPU address inside a VME window into a pointer to the
 * backing byte and its VME address. Returns the space name for tracing.
 */
static uint8_t	*VME_DecodeAddr ( uaecptr addr, uint32_t *pVmeAddr, const char **ppSpace )
{
	uaecptr addr24 = addr & 0x00ffffff;

	if ( ( Config_IsMachineMegaSTE() && addr24 >= VME_A16_START_MEGASTE && addr24 < VME_A16_END_MEGASTE )
	  || ( Config_IsMachineTT() && addr >= VME_A16_START_TT ) )
	{
		*pVmeAddr = addr & VME_A16_MASK;
		*ppSpace = "a16";
		return pVmeA16Ram + *pVmeAddr;
	}

	*pVmeAddr = addr24;
	*ppSpace = "a24";
	return pVmeA24Ram + *pVmeAddr;
}


/**
 * Nova/ET4000 card decode: map a CPU address inside the VME windows onto
 * the card's register file or memory window. Returns false when no card
 * hardware responds there (-> bus error, like on the real bus).
 */
static bool VME_ET4000_Decode ( uaecptr addr, uint32_t *pOffset, bool *pIsMem )
{
	uint32_t addr24 = addr & 0x00ffffff;

	if ( addr24 >= VME_NOVA_MEM_BASE && addr24 < VME_NOVA_MEM_BASE + VME_NOVA_MEM_SIZE )
	{
		*pIsMem = true;
		*pOffset = addr24 - VME_NOVA_MEM_BASE;
		return true;
	}
	if ( addr24 >= VME_NOVA_REG_BASE && addr24 < VME_NOVA_REG_BASE + VME_NOVA_REG_SIZE )
	{
		*pIsMem = false;
		*pOffset = addr24 & 0xffff;
		return true;
	}
	return false;
}

/* Byte-wide card access helpers. Word accesses are split high byte first :
 * whether the real Nova adapter swaps the byte lanes on 16 bit transfers
 * (EmuTOS notes it does for register accesses) is to be settled with a
 * real driver trace in phase 3 ; EmuTOS itself only does byte accesses on
 * the ET4000 path and the framebuffer of the ATW800/2 is not swapped. */
static uint8_t VME_ET4000_ReadByte ( uint32_t offset, bool is_mem )
{
	VmeReadCount++;
	if ( is_mem )
		return ET4000_Mem_ReadByte ( offset );
	return ET4000_IO_ReadByte ( offset );
}

static void VME_ET4000_WriteByte ( uint32_t offset, bool is_mem, uint8_t val )
{
	VmeWriteCount++;
	if ( is_mem )
		ET4000_Mem_WriteByte ( offset, val );
	else
		ET4000_IO_WriteByte ( offset, val );
}


/**
 * Read / write handlers for the VME windows, hooked into the memory
 * banks in cpu/memory.c. The bus is D16 : the CPU splits long accesses
 * into 2 word cycles itself, so no extra handling is needed here.
 */
uae_u32 REGPARAM3 VME_Mem_bget ( uaecptr addr )
{
	uint32_t VmeAddr;
	const char *space;
	uint8_t *p;
	uint8_t val;

	if ( ConfigureParams.System.nVMEType == VME_TYPE_ET4000 )
	{
		uint32_t offset;
		bool is_mem;

		if ( !VME_ET4000_Decode ( addr, &offset, &is_mem ) )
		{
			M68000_BusError ( addr, BUS_ERROR_READ, BUS_ERROR_SIZE_BYTE, BUS_ERROR_ACCESS_DATA, 0 );
			return -1;
		}
		return VME_ET4000_ReadByte ( offset, is_mem );
	}

	p = VME_DecodeAddr ( addr, &VmeAddr, &space );
	val = p[0];

	VmeReadCount++;
	LOG_TRACE(TRACE_VME, "vme %s rd.b $%06x val=0x%02x pc=%x\n", space, VmeAddr, val, M68000_GetPC());
	return val;
}

uae_u32 REGPARAM3 VME_Mem_wget ( uaecptr addr )
{
	uint32_t VmeAddr;
	const char *space;
	uint8_t *p;
	uint16_t val;

	if ( ConfigureParams.System.nVMEType == VME_TYPE_ET4000 )
	{
		uint32_t offset;
		bool is_mem;

		if ( !VME_ET4000_Decode ( addr, &offset, &is_mem ) )
		{
			M68000_BusError ( addr, BUS_ERROR_READ, BUS_ERROR_SIZE_WORD, BUS_ERROR_ACCESS_DATA, 0 );
			return -1;
		}
		return ( VME_ET4000_ReadByte ( offset, is_mem ) << 8 )
		       | VME_ET4000_ReadByte ( offset + 1, is_mem );
	}

	p = VME_DecodeAddr ( addr, &VmeAddr, &space );
	val = ( p[0] << 8 ) | p[1];

	VmeReadCount++;
	LOG_TRACE(TRACE_VME, "vme %s rd.w $%06x val=0x%04x pc=%x\n", space, VmeAddr, val, M68000_GetPC());
	return val;
}

uae_u32 REGPARAM3 VME_Mem_lget ( uaecptr addr )
{
	uint32_t VmeAddr;
	const char *space;
	uint8_t *p;
	uint32_t val;

	if ( ConfigureParams.System.nVMEType == VME_TYPE_ET4000 )
	{
		uint32_t offset;
		bool is_mem;

		if ( !VME_ET4000_Decode ( addr, &offset, &is_mem ) )
		{
			M68000_BusError ( addr, BUS_ERROR_READ, BUS_ERROR_SIZE_LONG, BUS_ERROR_ACCESS_DATA, 0 );
			return -1;
		}
		return ( (uint32_t)VME_ET4000_ReadByte ( offset, is_mem ) << 24 )
		       | ( VME_ET4000_ReadByte ( offset + 1, is_mem ) << 16 )
		       | ( VME_ET4000_ReadByte ( offset + 2, is_mem ) << 8 )
		       | VME_ET4000_ReadByte ( offset + 3, is_mem );
	}

	p = VME_DecodeAddr ( addr, &VmeAddr, &space );
	val = ( (uint32_t)p[0] << 24 ) | ( p[1] << 16 ) | ( p[2] << 8 ) | p[3];

	VmeReadCount++;
	LOG_TRACE(TRACE_VME, "vme %s rd.l $%06x val=0x%08x pc=%x\n", space, VmeAddr, val, M68000_GetPC());
	return val;
}

void REGPARAM3 VME_Mem_bput ( uaecptr addr, uae_u32 val )
{
	uint32_t VmeAddr;
	const char *space;
	uint8_t *p;

	if ( ConfigureParams.System.nVMEType == VME_TYPE_ET4000 )
	{
		uint32_t offset;
		bool is_mem;

		if ( !VME_ET4000_Decode ( addr, &offset, &is_mem ) )
		{
			M68000_BusError ( addr, BUS_ERROR_WRITE, BUS_ERROR_SIZE_BYTE, BUS_ERROR_ACCESS_DATA, val );
			return;
		}
		VME_ET4000_WriteByte ( offset, is_mem, val );
		return;
	}

	p = VME_DecodeAddr ( addr, &VmeAddr, &space );
	p[0] = val;
	VmeWriteCount++;
	LOG_TRACE(TRACE_VME, "vme %s wr.b $%06x val=0x%02x pc=%x\n", space, VmeAddr, (uint8_t)val, M68000_GetPC());
}

void REGPARAM3 VME_Mem_wput ( uaecptr addr, uae_u32 val )
{
	uint32_t VmeAddr;
	const char *space;
	uint8_t *p;

	if ( ConfigureParams.System.nVMEType == VME_TYPE_ET4000 )
	{
		uint32_t offset;
		bool is_mem;

		if ( !VME_ET4000_Decode ( addr, &offset, &is_mem ) )
		{
			M68000_BusError ( addr, BUS_ERROR_WRITE, BUS_ERROR_SIZE_WORD, BUS_ERROR_ACCESS_DATA, val );
			return;
		}
		VME_ET4000_WriteByte ( offset, is_mem, val >> 8 );
		VME_ET4000_WriteByte ( offset + 1, is_mem, val );
		return;
	}

	p = VME_DecodeAddr ( addr, &VmeAddr, &space );
	p[0] = val >> 8;
	p[1] = val;
	VmeWriteCount++;
	LOG_TRACE(TRACE_VME, "vme %s wr.w $%06x val=0x%04x pc=%x\n", space, VmeAddr, (uint16_t)val, M68000_GetPC());
}

void REGPARAM3 VME_Mem_lput ( uaecptr addr, uae_u32 val )
{
	uint32_t VmeAddr;
	const char *space;
	uint8_t *p;

	if ( ConfigureParams.System.nVMEType == VME_TYPE_ET4000 )
	{
		uint32_t offset;
		bool is_mem;

		if ( !VME_ET4000_Decode ( addr, &offset, &is_mem ) )
		{
			M68000_BusError ( addr, BUS_ERROR_WRITE, BUS_ERROR_SIZE_LONG, BUS_ERROR_ACCESS_DATA, val );
			return;
		}
		VME_ET4000_WriteByte ( offset, is_mem, val >> 24 );
		VME_ET4000_WriteByte ( offset + 1, is_mem, val >> 16 );
		VME_ET4000_WriteByte ( offset + 2, is_mem, val >> 8 );
		VME_ET4000_WriteByte ( offset + 3, is_mem, val );
		return;
	}

	p = VME_DecodeAddr ( addr, &VmeAddr, &space );
	p[0] = val >> 24;
	p[1] = val >> 16;
	p[2] = val >> 8;
	p[3] = val;
	VmeWriteCount++;
	LOG_TRACE(TRACE_VME, "vme %s wr.l $%06x val=0x%08x pc=%x\n", space, VmeAddr, val, M68000_GetPC());
}

int REGPARAM3 VME_Mem_check ( uaecptr addr, uae_u32 size )
{
	return 0;
}

uae_u8 * REGPARAM3 VME_Mem_xlate ( uaecptr addr )
{
	static uint8_t dummy[4];
	uint32_t VmeAddr;
	const char *space;

	if ( ConfigureParams.System.nVMEType == VME_TYPE_ET4000 || !pVmeA24Ram )
		return dummy;				/* no direct/executable access to the card */

	return VME_DecodeAddr ( addr, &VmeAddr, &space );
}


/*-----------------------------------------------------------------------*/
/**
 * Save/Restore snapshot of VME variables
 */
void	VME_MemorySnapShot_Capture ( bool bSave )
{
	bool bAllocated = ( pVmeA24Ram != NULL );

	MemorySnapShot_Store(&VmeReadCount, sizeof(VmeReadCount));
	MemorySnapShot_Store(&VmeWriteCount, sizeof(VmeWriteCount));
	MemorySnapShot_Store(&bAllocated, sizeof(bAllocated));

	ET4000_MemorySnapShot_Capture ( bSave );

	if ( !bAllocated )
		return;

	if ( !bSave && !pVmeA24Ram )
	{
		pVmeA24Ram = malloc ( VME_A24_SIZE );
		pVmeA16Ram = malloc ( VME_A16_SIZE );
		if ( !pVmeA24Ram || !pVmeA16Ram )
			Main_ErrorExit ( "Out of memory (VME address space)", NULL, 1 );
	}
	MemorySnapShot_Store(pVmeA24Ram, VME_A24_SIZE);
	MemorySnapShot_Store(pVmeA16Ram, VME_A16_SIZE);
}


/**
 * Show VME card emulation state
 */
void VME_Info ( FILE *fp, uint32_t arg )
{
	if ( !( Config_IsMachineTT() || Config_IsMachineMegaSTE() ) )
	{
		fprintf(fp, "No MegaSTE/TT -> no VME bus\n\n");
		return;
	}

	switch ( ConfigureParams.System.nVMEType )
	{
	 case VME_TYPE_NONE:
		fprintf(fp, "VME card emulation: none (VME windows return bus errors)\n");
		break;
	 case VME_TYPE_TRACE:
		fprintf(fp, "VME card emulation: trace (RAM-backed windows, log with --trace vme)\n");
		fprintf(fp, "VME accesses since reset: %llu reads, %llu writes\n",
		        (unsigned long long)VmeReadCount, (unsigned long long)VmeWriteCount);
		break;
	 case VME_TYPE_ET4000:
		fprintf(fp, "VME card emulation: Nova/ET4000 (regs at A24 0x%06x, mem at A24 0x%06x)\n",
		        VME_NOVA_REG_BASE, VME_NOVA_MEM_BASE);
		fprintf(fp, "VME accesses since reset: %llu reads, %llu writes\n",
		        (unsigned long long)VmeReadCount, (unsigned long long)VmeWriteCount);
		ET4000_Info ( fp, arg );
		break;
	 default:
		fprintf(fp, "VME card emulation: unknown type %d\n", ConfigureParams.System.nVMEType);
		break;
	}
}
