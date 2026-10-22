/*
  Hatari - vme_lightning.h

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.
*/
#ifndef HATARI_VME_LIGHTNING_H
#define HATARI_VME_LIGHTNING_H

#include "sysdeps.h"

extern bool Lightning_IsEnabled(void);
extern void Lightning_Init(void);
extern void Lightning_UnInit(void);
extern void Lightning_Reset(bool bCold);
extern bool Lightning_Decode(uaecptr addr, uint32_t *pOffset);
extern uint8_t Lightning_ReadByte(uint32_t offset);
extern uint16_t Lightning_ReadWord(uint32_t offset);
extern void Lightning_WriteByte(uint32_t offset, uint8_t val);
extern void Lightning_WriteWord(uint32_t offset, uint16_t val);
extern void Lightning_MemorySnapShot_Capture(bool bSave);
extern void Lightning_Info(FILE *fp, uint32_t arg);

#endif /* HATARI_VME_LIGHTNING_H */
