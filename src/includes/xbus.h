/*
  Hatari - xbus.h

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.
*/

#ifndef HATARI_XBUS_H
#define HATARI_XBUS_H

#include "sysdeps.h"

#define XBUS_ISA_START		0xFC000000	/* addr[31:24] == 0xFC : ISA */
#define XBUS_VME32_START	0xFD000000	/* addr[31:24] == 0xFD : VME A32:D32 */
#define XBUS_WINDOW_SIZE	0x01000000

extern bool XBus_IsAvailable(void);
extern void XBus_Init(void);
extern void XBus_UnInit(void);
extern void XBus_Reset(bool bCold);
extern void XBus_Info(FILE *fp, uint32_t dummy);

extern uae_u32 REGPARAM3 XBus_Mem_bget(uaecptr addr);
extern uae_u32 REGPARAM3 XBus_Mem_wget(uaecptr addr);
extern uae_u32 REGPARAM3 XBus_Mem_lget(uaecptr addr);
extern void REGPARAM3 XBus_Mem_bput(uaecptr addr, uae_u32 val);
extern void REGPARAM3 XBus_Mem_wput(uaecptr addr, uae_u32 val);
extern void REGPARAM3 XBus_Mem_lput(uaecptr addr, uae_u32 val);
extern int REGPARAM3 XBus_Mem_check(uaecptr addr, uae_u32 size);
extern uae_u8 * REGPARAM3 XBus_Mem_xlate(uaecptr addr);

#endif /* HATARI_XBUS_H */
