/*
  Hatari - usb_isp1160.h

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.
*/
#ifndef HATARI_USB_ISP1160_H
#define HATARI_USB_ISP1160_H

#include "sysdeps.h"
#include "usb_dev.h"

#define ISP1160_PORTS	2

extern void ISP1160_Init(void);
extern void ISP1160_UnInit(void);
extern void ISP1160_Reset(bool bCold);
extern void ISP1160_Attach(int port, usb_device_t *dev);
extern void ISP1160_Detach(int port);
extern usb_device_t *ISP1160_Device(int port);

/* the 16-bit bus: what the chip's pins see */
extern void ISP1160_WriteAddr(uint16_t val);
extern void ISP1160_WriteData(uint16_t val);
extern uint16_t ISP1160_ReadData(void);
extern bool ISP1160_IntPending(void);

extern void ISP1160_MemorySnapShot_Capture(bool bSave);
extern void ISP1160_Info(FILE *fp);

#endif /* HATARI_USB_ISP1160_H */
