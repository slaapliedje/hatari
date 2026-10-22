/*
  Hatari - usb_isp1160.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  The Philips ISP1160 USB host controller, as the Lightning VME and the
  NetUSBee carry it: an OHCI-like register set reached through an address
  port and a 16-bit data port, a two-port root hub, and the ATL buffer
  the driver fills with transfer descriptors (PTDs) and data through the
  HcATLBufferPort, counted by HcTransferCounter.

  The bus glue (vme_lightning.c) hands over the 16-bit words as the chip
  sees them. 32-bit registers come as two words, low half first. A PTD
  is four little-endian 16-bit fields and its data, padded to 4 bytes.

  The ATL is run as soon as the driver has written it whole: every PTD
  marked active is carried out against the device at its address, the
  result (count, condition code, toggle, active) written back, and the
  ATL done flag and interrupt set; the read-back clears them. Frame
  timing is taken from the CPU clock for HcFmNumber. Isochronous
  transfers (the ITL) are not supported; no driver here uses them.
*/
const char UsbIsp1160_fileid[] = "Hatari usb_isp1160.c";

#include <string.h>
#include "main.h"
#include "configuration.h"
#include "log.h"
#include "cycles.h"
#include "clocks_timings.h"
#include "memorySnapShot.h"
#include "usb_isp1160.h"

/* registers (the index written to the address port; bit 7 = write) */
#define HCREVISION	0x00
#define HCCONTROL	0x01
#define HCCMDSTAT	0x02
#define HCINTSTAT	0x03
#define HCINTENB	0x04
#define HCINTDIS	0x05
#define HCFMINTVL	0x0d
#define HCFMREM		0x0e
#define HCFMNUM		0x0f
#define HCLSTHRESH	0x11
#define HCRHDESCA	0x12
#define HCRHDESCB	0x13
#define HCRHSTATUS	0x14
#define HCRHPORT1	0x15
#define HCRHPORT2	0x16
#define HCHWCFG		0x20
#define HCDMACFG	0x21
#define HCXFERCTR	0x22
#define HCuPINT		0x24
#define HCuPINTENB	0x25
#define HCCHIPID	0x27
#define HCSCRATCH	0x28
#define HCSWRES		0x29
#define HCITLBUFLEN	0x2a
#define HCATLBUFLEN	0x2b
#define HCBUFSTAT	0x2c
#define HCRDITL0LEN	0x2d
#define HCRDITL1LEN	0x2e
#define HCITLPORT	0x40
#define HCATLPORT	0x41
#define NREGS		0x30

#define HCCMDSTAT_HCR	(1 << 0)
#define HCINT_RHSC	(1 << 6)
#define HCINT_SF	(1 << 2)

#define RH_A_NDP	ISP1160_PORTS
#define RH_HS_LPS	(1UL << 0)
#define RH_HS_LPSC	(1UL << 16)
#define RH_HS_OCIC	(1UL << 17)

#define RH_PS_CCS	(1UL << 0)
#define RH_PS_PES	(1UL << 1)
#define RH_PS_PSS	(1UL << 2)
#define RH_PS_POCI	(1UL << 3)
#define RH_PS_PRS	(1UL << 4)
#define RH_PS_PPS	(1UL << 8)
#define RH_PS_LSDA	(1UL << 9)
#define RH_PS_CSC	(1UL << 16)
#define RH_PS_PESC	(1UL << 17)
#define RH_PS_PSSC	(1UL << 18)
#define RH_PS_OCIC	(1UL << 19)
#define RH_PS_PRSC	(1UL << 20)
#define RH_PS_CHANGE	(0x1fUL << 16)

#define HCuPINT_SOF	(1 << 0)
#define HCuPINT_ATL	(1 << 1)
#define HCuPINT_AIIEOT	(1 << 2)
#define HCuPINT_OPR	(1 << 4)

#define HCBUFSTAT_ATL_FULL	(1 << 2)
#define HCBUFSTAT_ATL_DONE	(1 << 5)

#define HCSWRES_MAGIC	0x00f6
#define CHIP_ID		0x6120

/* PTD fields */
#define PTD_COUNT_MSK	0x3ff
#define PTD_TOGGLE	(1 << 10)
#define PTD_ACTIVE	(1 << 11)
#define PTD_CC_SHIFT	12
#define PTD_MPS_MSK	0x3ff
#define PTD_LAST	(1 << 11)
#define PTD_EP_SHIFT	12
#define PTD_LEN_MSK	0x3ff
#define PTD_DIR_SHIFT	10
#define PTD_DIR_SETUP	0
#define PTD_DIR_OUT	1
#define PTD_DIR_IN	2
#define PTD_FA_MSK	0x7f

#define CC_NOERROR	0x0
#define CC_STALL	0x4
#define CC_DEVNOTRESP	0x5
#define CC_DATAUNDERRUN	0x9
#define CC_NOTACCESSED	0xf

#define ATL_SIZE	4096

static uint32_t Regs[NREGS];
static int AddrReg;			/* the register selected (index, bit 7 = write) */
static bool AddrWrite;
static int WordPhase;			/* 0 = low half, 1 = high half of a 32-bit register */
static uint8_t Atl[ATL_SIZE];
static int XferPos;			/* bytes moved through the port so far */
static int XferLen;			/* HcTransferCounter at the start of the transfer */
static bool XferActive;
static uint32_t WordLow;		/* the low half of a 32-bit register access */
static usb_device_t *Port[ISP1160_PORTS];
static uint64_t Transfers, Nak, Errors;

/* 32-bit registers: the OHCI set */
static bool is32(int reg)
{
	return reg <= HCRHPORT2;
}


/* a connect or change on a port: the root hub status change interrupt */
static void rhsc(void)
{
	Regs[HCINTSTAT] |= HCINT_RHSC;
	Regs[HCuPINT] |= HCuPINT_OPR;
}


/* the chip after a hardware or software reset */
static void chip_reset(void)
{
	int i;

	memset(Regs, 0, sizeof(Regs));
	Regs[HCREVISION] = 0x10;
	Regs[HCCONTROL] = 0;
	Regs[HCFMINTVL] = 0x2edf;
	Regs[HCLSTHRESH] = 0x0628;
	Regs[HCRHDESCA] = RH_A_NDP | (1 << 9) | (1 << 12);	/* NPS, NOCP */
	Regs[HCRHDESCB] = 0;
	Regs[HCRHSTATUS] = 0;
	for (i = 0; i < ISP1160_PORTS; i++)
		Regs[HCRHPORT1 + i] = 0;
	Regs[HCHWCFG] = 0;
	Regs[HCCHIPID] = CHIP_ID;
	Regs[HCITLBUFLEN] = 0;
	Regs[HCATLBUFLEN] = 0;
	Regs[HCBUFSTAT] = 0;
	WordPhase = 0;
	XferActive = false;
	XferPos = XferLen = 0;
	for (i = 0; i < ISP1160_PORTS; i++)
		if (Port[i])
			usb_dev_reset(Port[i]);
}


void ISP1160_Init(void)
{
	chip_reset();
}


void ISP1160_UnInit(void)
{
	int i;

	for (i = 0; i < ISP1160_PORTS; i++)
		Port[i] = NULL;
}


void ISP1160_Reset(bool bCold)
{
	chip_reset();
	Transfers = Nak = Errors = 0;
}


void ISP1160_Attach(int port, usb_device_t *dev)
{
	if (port < 0 || port >= ISP1160_PORTS)
		return;
	Port[port] = dev;
	if (dev && (Regs[HCRHPORT1 + port] & RH_PS_PPS))
	{
		Regs[HCRHPORT1 + port] |= RH_PS_CCS | RH_PS_CSC;
		if (dev->low_speed)
			Regs[HCRHPORT1 + port] |= RH_PS_LSDA;
		rhsc();
	}
}


void ISP1160_Detach(int port)
{
	if (port < 0 || port >= ISP1160_PORTS)
		return;
	Port[port] = NULL;
	if (Regs[HCRHPORT1 + port] & RH_PS_CCS)
	{
		Regs[HCRHPORT1 + port] &= ~(RH_PS_CCS | RH_PS_PES | RH_PS_LSDA);
		Regs[HCRHPORT1 + port] |= RH_PS_CSC | RH_PS_PESC;
		rhsc();
	}
}


usb_device_t *ISP1160_Device(int port)
{
	return (port >= 0 && port < ISP1160_PORTS) ? Port[port] : NULL;
}


/* port power on: a device there shows up as connected */
static void port_power(int port, bool on)
{
	uint32_t *ps = &Regs[HCRHPORT1 + port];

	if (on)
	{
		if (*ps & RH_PS_PPS)
			return;
		*ps |= RH_PS_PPS;
		if (Port[port])
		{
			*ps |= RH_PS_CCS | RH_PS_CSC;
			if (Port[port]->low_speed)
				*ps |= RH_PS_LSDA;
			rhsc();
		}
	}
	else
	{
		if (!(*ps & RH_PS_PPS))
			return;
		if (*ps & RH_PS_CCS)
			*ps |= RH_PS_CSC;
		*ps &= ~(RH_PS_PPS | RH_PS_CCS | RH_PS_PES | RH_PS_LSDA | RH_PS_PRS | RH_PS_PSS);
		rhsc();
	}
}


/* a write to HcRhPortStatus: OHCI's set/clear semantics */
static void port_write(int port, uint32_t val)
{
	uint32_t *ps = &Regs[HCRHPORT1 + port];

	if (val & RH_PS_CCS)			/* ClearPortEnable */
		*ps &= ~RH_PS_PES;
	if (val & RH_PS_PES)			/* SetPortEnable */
	{
		if (*ps & RH_PS_CCS)
			*ps |= RH_PS_PES;
		else
			*ps |= RH_PS_CSC;
	}
	if (val & RH_PS_PSS)			/* SetPortSuspend */
	{
		if (*ps & RH_PS_CCS)
			*ps |= RH_PS_PSS;
		else
			*ps |= RH_PS_CSC;
	}
	if (val & RH_PS_POCI)			/* ClearSuspendStatus */
	{
		if (*ps & RH_PS_PSS)
		{
			*ps &= ~RH_PS_PSS;
			*ps |= RH_PS_PSSC;
		}
	}
	if (val & RH_PS_PRS)			/* SetPortReset: done at once */
	{
		if (*ps & RH_PS_CCS)
		{
			*ps &= ~RH_PS_PRS;
			*ps |= RH_PS_PRSC | RH_PS_PES;
			if (Port[port])
				usb_dev_reset(Port[port]);
			LOG_TRACE(TRACE_USB, "usb hc: port %d reset\n", port + 1);
		}
		else
			*ps |= RH_PS_CSC;
	}
	if (val & RH_PS_PPS)			/* SetPortPower */
		port_power(port, true);
	if (val & RH_PS_LSDA)			/* ClearPortPower */
		port_power(port, false);
	*ps &= ~(val & RH_PS_CHANGE);		/* the change bits clear on a 1 */
}


/* the frame counter follows the CPU clock: a frame a millisecond */
static uint32_t frame_number(void)
{
	uint64_t ms = Cycles_GetClockCounterImmediate() / (MachineClocks.CPU_Freq_Emul / 1000);
	return (uint32_t)(ms & 0xffff);
}


/* ---- the ATL ---------------------------------------------------------- */

/* the device at a USB address, on an enabled port */
static usb_device_t *find_device(int fa)
{
	int i;

	for (i = 0; i < ISP1160_PORTS; i++)
		if (Port[i] && (Regs[HCRHPORT1 + i] & RH_PS_PES) && Port[i]->address == fa)
			return Port[i];
	return NULL;
}


/* one PTD, in place in the ATL */
static void run_ptd(int off)
{
	uint8_t *p = Atl + off;
	uint16_t count = usb_le16(p), mps = usb_le16(p + 2), len = usb_le16(p + 4), faddr = usb_le16(p + 6);
	int dir = (len >> PTD_DIR_SHIFT) & 3, ep = (mps >> PTD_EP_SHIFT) & 0xf, fa = faddr & PTD_FA_MSK;
	int bytes = len & PTD_LEN_MSK, maxp = mps & PTD_MPS_MSK, toggle = (count & PTD_TOGGLE) != 0;
	int cc = CC_NOERROR, done = 0, npackets, r;
	bool active = false;
	usb_device_t *dev;
	uint8_t *data = p + 8;

	Transfers++;
	dev = find_device(fa);
	if (!dev)
	{
		LOG_TRACE(TRACE_USB, "usb hc: no device at address %d\n", fa);
		cc = CC_DEVNOTRESP;
		Errors++;
		goto out;
	}
	if (maxp == 0)
		maxp = 8;
	switch (dir)
	{
	 case PTD_DIR_SETUP:
		r = usb_dev_setup(dev, data);
		if (r == USB_DEV_STALL)
			cc = CC_STALL;
		else
			done = 8;
		npackets = 1;
		break;
	 case PTD_DIR_OUT:
		r = usb_dev_out(dev, ep, data, bytes);
		if (r == USB_DEV_STALL)
			cc = CC_STALL;
		else if (r == USB_DEV_NAK)
		{
			cc = CC_NOTACCESSED;
			active = true;
			Nak++;
		}
		else
			done = bytes;
		npackets = bytes ? (bytes + maxp - 1) / maxp : 1;
		break;
	 case PTD_DIR_IN:
		r = usb_dev_in(dev, ep, data, bytes);
		if (r == USB_DEV_STALL)
		{
			cc = CC_STALL;
			npackets = 1;
		}
		else if (r == USB_DEV_NAK)
		{
			cc = CC_NOTACCESSED;
			active = true;
			npackets = 0;
			Nak++;
		}
		else
		{
			done = r;
			if (done < bytes)
				cc = CC_DATAUNDERRUN;
			npackets = done ? (done + maxp - 1) / maxp : 1;
			/* a short last packet, or none, closes the transfer:
			 * a full-length one is followed by nothing more */
		}
		break;
	 default:
		cc = CC_STALL;
		npackets = 1;
		break;
	}
	if (cc != CC_NOERROR && cc != CC_DATAUNDERRUN && cc != CC_NOTACCESSED)
		Errors++;
	if (!active)
		toggle ^= npackets & 1;
	LOG_TRACE(TRACE_USB, "usb hc: ptd fa=%d ep=%d %s len=%d -> count=%d cc=%d%s\n", fa, ep,
	          dir == PTD_DIR_SETUP ? "setup" : dir == PTD_DIR_OUT ? "out" : "in", bytes, done, cc,
	          active ? " (nak)" : "");
out:
	count = (done & PTD_COUNT_MSK) | (toggle ? PTD_TOGGLE : 0) | (active ? PTD_ACTIVE : 0) | (cc << PTD_CC_SHIFT);
	p[0] = count;
	p[1] = count >> 8;
}


/* the ATL as the driver wrote it: PTD after PTD until the one marked last */
static void run_atl(void)
{
	int off = 0;

	while (off + 8 <= XferLen)
	{
		uint16_t count = usb_le16(Atl + off), mps = usb_le16(Atl + off + 2), len = usb_le16(Atl + off + 4);
		int bytes = len & PTD_LEN_MSK;

		if (count & PTD_ACTIVE)
			run_ptd(off);
		if (mps & PTD_LAST)
			break;
		off += 8 + ((bytes + 3) & ~3);
	}
	Regs[HCBUFSTAT] |= HCBUFSTAT_ATL_DONE;
	Regs[HCuPINT] |= HCuPINT_ATL;
}


/* ---- the bus ---------------------------------------------------------- */

void ISP1160_WriteAddr(uint16_t val)
{
	AddrReg = val & 0x7f;
	AddrWrite = (val & 0x80) != 0;
	WordPhase = 0;
	if (AddrReg == HCATLPORT || AddrReg == HCITLPORT)
	{
		XferPos = 0;
		XferLen = Regs[HCXFERCTR] & 0xffff;
		XferActive = true;
	}
	else
		XferActive = false;
}


static uint32_t reg_read(int reg)
{
	switch (reg)
	{
	 case HCFMNUM:
		return frame_number();
	 case HCFMREM:
	 {
		uint64_t per_ms = MachineClocks.CPU_Freq_Emul / 1000;
		uint32_t fi = Regs[HCFMINTVL] & 0x3fff;
		return fi - (uint32_t)((Cycles_GetClockCounterImmediate() % per_ms) * fi / per_ms);
	 }
	 case HCRHSTATUS:
		return Regs[HCRHSTATUS] & ~RH_HS_LPS;
	 case HCRDITL0LEN:
	 case HCRDITL1LEN:
		return 0;
	 case HCSWRES:
		return 0;
	 default:
		if (reg < NREGS)
			return Regs[reg];
	}
	return 0;
}


static void reg_write(int reg, uint32_t val)
{
	switch (reg)
	{
	 case HCREVISION:
	 case HCCHIPID:
	 case HCFMNUM:
	 case HCFMREM:
	 case HCBUFSTAT:
	 case HCRDITL0LEN:
	 case HCRDITL1LEN:
		return;				/* read only */
	 case HCCMDSTAT:
		if (val & HCCMDSTAT_HCR)
		{
			LOG_TRACE(TRACE_USB, "usb hc: host controller reset\n");
			chip_reset();		/* done at once: HCR reads back 0 */
		}
		return;
	 case HCINTSTAT:
		Regs[HCINTSTAT] &= ~val;
		if (!(Regs[HCINTSTAT] & 0x7f))
			Regs[HCuPINT] &= ~HCuPINT_OPR;
		return;
	 case HCINTENB:
		Regs[HCINTENB] |= val;
		return;
	 case HCINTDIS:
		Regs[HCINTENB] &= ~val;
		return;
	 case HCRHDESCA:
		Regs[HCRHDESCA] = (val & ~0xffUL) | RH_A_NDP;
		return;
	 case HCRHSTATUS:
		if (val & RH_HS_LPS)		/* ClearGlobalPower */
		{
			port_power(0, false);
			port_power(1, false);
		}
		if (val & RH_HS_LPSC)		/* SetGlobalPower */
		{
			port_power(0, true);
			port_power(1, true);
		}
		if (val & RH_HS_OCIC)
			Regs[HCRHSTATUS] &= ~RH_HS_OCIC;
		Regs[HCRHSTATUS] = (Regs[HCRHSTATUS] & ~(1UL << 15)) | (val & (1UL << 15));
		return;
	 case HCRHPORT1:
	 case HCRHPORT2:
		port_write(reg - HCRHPORT1, val);
		return;
	 case HCuPINT:
		Regs[HCuPINT] &= ~val;
		return;
	 case HCSWRES:
		if ((val & 0xffff) == HCSWRES_MAGIC)
		{
			LOG_TRACE(TRACE_USB, "usb hc: software reset\n");
			chip_reset();
		}
		return;
	 default:
		if (reg < NREGS)
			Regs[reg] = val;
		return;
	}
}


void ISP1160_WriteData(uint16_t val)
{
	if (XferActive)
	{
		if (AddrReg == HCATLPORT && AddrWrite && XferPos + 1 < ATL_SIZE)
		{
			Atl[XferPos] = val & 0xff;
			Atl[XferPos + 1] = val >> 8;
		}
		XferPos += 2;
		if (XferPos >= XferLen)
		{
			XferActive = false;
			Regs[HCuPINT] |= HCuPINT_AIIEOT;
			if (AddrReg == HCATLPORT && AddrWrite)
			{
				Regs[HCBUFSTAT] |= HCBUFSTAT_ATL_FULL;
				LOG_TRACE(TRACE_USB, "usb hc: atl written, %d bytes\n", XferLen);
				run_atl();
			}
		}
		return;
	}
	if (!AddrWrite)
		return;
	if (is32(AddrReg))
	{
		if (WordPhase == 0)
		{
			WordLow = val;		/* the low half waits for the high one */
			WordPhase = 1;
			return;
		}
		WordPhase = 0;
		LOG_TRACE(TRACE_USB, "usb hc: wr reg 0x%02x = 0x%08x\n", AddrReg, ((uint32_t)val << 16) | WordLow);
		reg_write(AddrReg, ((uint32_t)val << 16) | WordLow);
		return;
	}
	LOG_TRACE(TRACE_USB, "usb hc: wr reg 0x%02x = 0x%04x\n", AddrReg, val);
	reg_write(AddrReg, val);
}


uint16_t ISP1160_ReadData(void)
{
	uint16_t val;

	if (XferActive)
	{
		val = 0;
		if (AddrReg == HCATLPORT && !AddrWrite && XferPos + 1 < ATL_SIZE)
			val = Atl[XferPos] | (Atl[XferPos + 1] << 8);
		XferPos += 2;
		if (XferPos >= XferLen)
		{
			XferActive = false;
			Regs[HCuPINT] |= HCuPINT_AIIEOT;
			if (AddrReg == HCATLPORT && !AddrWrite)
			{
				Regs[HCBUFSTAT] &= ~(HCBUFSTAT_ATL_FULL | HCBUFSTAT_ATL_DONE);
				LOG_TRACE(TRACE_USB, "usb hc: atl read back, %d bytes\n", XferLen);
			}
		}
		return val;
	}
	if (AddrWrite)
		return 0;
	if (is32(AddrReg))
	{
		uint32_t v = reg_read(AddrReg);
		if (WordPhase == 0)
		{
			WordLow = v;
			WordPhase = 1;
			return v & 0xffff;
		}
		WordPhase = 0;
		LOG_TRACE(TRACE_USB, "usb hc: rd reg 0x%02x = 0x%08x\n", AddrReg, WordLow);
		return WordLow >> 16;
	}
	val = reg_read(AddrReg) & 0xffff;
	LOG_TRACE(TRACE_USB, "usb hc: rd reg 0x%02x = 0x%04x\n", AddrReg, val);
	return val;
}


/* the INT pin: a uP interrupt that is enabled, with the interrupt output enabled */
bool ISP1160_IntPending(void)
{
	return (Regs[HCHWCFG] & 1) && (Regs[HCuPINT] & Regs[HCuPINTENB] & 0xff);
}


void ISP1160_MemorySnapShot_Capture(bool bSave)
{
	MemorySnapShot_Store(Regs, sizeof(Regs));
	MemorySnapShot_Store(&AddrReg, sizeof(AddrReg));
	MemorySnapShot_Store(&AddrWrite, sizeof(AddrWrite));
	MemorySnapShot_Store(&WordPhase, sizeof(WordPhase));
	MemorySnapShot_Store(&WordLow, sizeof(WordLow));
	MemorySnapShot_Store(Atl, sizeof(Atl));
	MemorySnapShot_Store(&XferPos, sizeof(XferPos));
	MemorySnapShot_Store(&XferLen, sizeof(XferLen));
	MemorySnapShot_Store(&XferActive, sizeof(XferActive));
}


void ISP1160_Info(FILE *fp)
{
	int i;

	fprintf(fp, "ISP1160: control 0x%08x, int status 0x%08x, uP int 0x%04x/enable 0x%04x, bufstat 0x%04x, frame %u\n",
	        Regs[HCCONTROL], Regs[HCINTSTAT], Regs[HCuPINT], Regs[HCuPINTENB], Regs[HCBUFSTAT], frame_number());
	fprintf(fp, "  %llu transfers, %llu naks, %llu errors\n",
	        (unsigned long long)Transfers, (unsigned long long)Nak, (unsigned long long)Errors);
	for (i = 0; i < ISP1160_PORTS; i++)
	{
		fprintf(fp, "  port %d: status 0x%08x", i + 1, Regs[HCRHPORT1 + i]);
		if (Port[i])
		{
			fprintf(fp, ", %s at address %d, configuration %d\n", Port[i]->name, Port[i]->address, Port[i]->config);
			if (Port[i]->info)
				Port[i]->info(Port[i], fp);
		}
		else
			fprintf(fp, ", nothing attached\n");
	}
}
