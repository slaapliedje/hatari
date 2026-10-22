/*
  Hatari - vme_lightning.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  The Lightning VME USB card for the MegaSTE and TT: a Philips ISP1160
  host controller (usb_isp1160.c) in the VME A16 space, with a CPLD that
  passes the chip's interrupt on when asked.

    0x8000  word  the chip's data port
    0x8004  word  the chip's address port (bit 7 = write)
    0x8009  byte  the CPLD control register (bit 2: interrupt enable)

  The card crosses the data lanes: a word on the bus reaches the chip
  with its bytes swapped, both ways (the drivers write the register
  index into the high byte, and the FIFO data as the bytes sit in
  memory). An empty slot bus-errors, which is how the drivers probe.

  The devices on the two ports: a USB stick on an image file
  (--usb-disk) on port 1, the keyboard and mouse (--usb-input) on port
  2. The drivers for Atari System V and FreeMiNT poll the chip; its
  interrupt line is not wired to the SCU here (yet).
*/
const char VmeLightning_fileid[] = "Hatari vme_lightning.c";

#include "main.h"
#include "configuration.h"
#include "m68000.h"
#include "log.h"
#include "memorySnapShot.h"
#include "usb_isp1160.h"
#include "usb_dev.h"
#include "vme_lightning.h"

#define LIGHTNING_BASE	0x8000		/* inside the A16 space */
#define LIGHTNING_SIZE	0x10
#define REG_DATA	0x0
#define REG_ADDR	0x4
#define REG_CPLD	0x9
#define CPLD_INTMASK	(1 << 2)

/* a function, not a macro: its argument is a bus access with a side effect */
static inline uint16_t SWAP16(uint16_t w)
{
	return (uint16_t)((w << 8) | (w >> 8));
}

static uint8_t CpldCr;
static usb_device_t *pStick;
static usb_device_t *pHid;
static uint64_t Accesses;


bool Lightning_IsEnabled(void)
{
	return (Config_IsMachineTT() || Config_IsMachineMegaSTE())
	       && ConfigureParams.Usb.nUsbType == USB_TYPE_LIGHTNING;
}


/* the chip and what is plugged into it; called when the memory map is
 * (re)built, so it may run several times */
void Lightning_Init(void)
{
	if (!Lightning_IsEnabled())
		return;
	ISP1160_Init();
	if (!pStick && ConfigureParams.Usb.szDiskImage[0])
	{
		pStick = usb_storage_create(ConfigureParams.Usb.szDiskImage);
		ISP1160_Attach(0, pStick);
	}
	if (!pHid && ConfigureParams.Usb.bHostInput)
	{
		pHid = usb_hid_create();
		ISP1160_Attach(1, pHid);
	}
}


void Lightning_UnInit(void)
{
	ISP1160_Detach(0);
	ISP1160_Detach(1);
	usb_storage_destroy(pStick);
	pStick = NULL;
	usb_hid_destroy(pHid);
	pHid = NULL;
	ISP1160_UnInit();
}


void Lightning_Reset(bool bCold)
{
	CpldCr = 0;
	Accesses = 0;
	ISP1160_Reset(bCold);
}


/* the card answers at its 16 bytes of the A16 space */
bool Lightning_Decode(uaecptr addr, uint32_t *pOffset)
{
	uint32_t off = addr & 0xffff;

	if (!Lightning_IsEnabled())
		return false;
	if (Config_IsMachineTT())
	{
		if (addr < 0xFEFF0000)
			return false;
	}
	else
	{
		if ((addr & 0x00ff0000) != 0x00ff0000)
			return false;
	}
	if (off < LIGHTNING_BASE || off >= LIGHTNING_BASE + LIGHTNING_SIZE)
		return false;
	*pOffset = off - LIGHTNING_BASE;
	return true;
}


uint16_t Lightning_ReadWord(uint32_t offset)
{
	uint16_t v;

	Accesses++;
	switch (offset)
	{
	 case REG_DATA:
		v = SWAP16(ISP1160_ReadData());
		LOG_TRACE(TRACE_USB, "lightning: data rd 0x%04x (chip 0x%04x) pc=%x\n", v, SWAP16(v), M68000_GetPC());
		break;
	 case REG_ADDR:
		v = 0;
		break;
	 case REG_CPLD - 1:
		v = CpldCr;
		break;
	 default:
		v = 0;
		break;
	}
	return v;
}


void Lightning_WriteWord(uint32_t offset, uint16_t val)
{
	Accesses++;
	switch (offset)
	{
	 case REG_DATA:
		LOG_TRACE(TRACE_USB, "lightning: data wr 0x%04x (chip 0x%04x) pc=%x\n", val, SWAP16(val), M68000_GetPC());
		ISP1160_WriteData(SWAP16(val));
		break;
	 case REG_ADDR:
		LOG_TRACE(TRACE_USB, "lightning: addr wr 0x%04x (reg 0x%02x) pc=%x\n", val, SWAP16(val) & 0xff, M68000_GetPC());
		ISP1160_WriteAddr(SWAP16(val));
		break;
	 case REG_CPLD - 1:
		CpldCr = val & 0xff;
		LOG_TRACE(TRACE_USB, "lightning: cpld cr = 0x%02x\n", CpldCr);
		break;
	 default:
		break;
	}
}


/* byte accesses: the halves of the words, and the CPLD register */
uint8_t Lightning_ReadByte(uint32_t offset)
{
	uint16_t w;

	if (offset == REG_CPLD)
	{
		Accesses++;
		return CpldCr;
	}
	w = Lightning_ReadWord(offset & ~1);
	return (offset & 1) ? (w & 0xff) : (w >> 8);
}


void Lightning_WriteByte(uint32_t offset, uint8_t val)
{
	if (offset == REG_CPLD)
	{
		Accesses++;
		CpldCr = val;
		LOG_TRACE(TRACE_USB, "lightning: cpld cr = 0x%02x\n", CpldCr);
		return;
	}
	/* a byte into a word port: the other lane reads as zero */
	Lightning_WriteWord(offset & ~1, (offset & 1) ? val : (val << 8));
}


void Lightning_MemorySnapShot_Capture(bool bSave)
{
	MemorySnapShot_Store(&CpldCr, sizeof(CpldCr));
	ISP1160_MemorySnapShot_Capture(bSave);
}


void Lightning_Info(FILE *fp, uint32_t arg)
{
	fprintf(fp, "Lightning VME: ISP1160 at A16 0x%04x, CPLD cr 0x%02x, %llu accesses\n",
	        LIGHTNING_BASE, CpldCr, (unsigned long long)Accesses);
	ISP1160_Info(fp);
}
