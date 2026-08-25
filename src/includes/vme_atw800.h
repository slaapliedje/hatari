/*
  Hatari - vme_atw800.h

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.
*/

#ifndef HATARI_VME_ATW800_H
#define HATARI_VME_ATW800_H

#include "sysdeps.h"

extern void ATW800_Init(void);
extern void ATW800_UnInit(void);
extern void ATW800_Reset(bool bCold);

/* Card decode: true when the card responds at this VME address */
extern bool ATW800_Decode(uint32_t addr24, uint32_t *pOffset);

extern uint8_t ATW800_ReadByte(uint32_t offset);
extern void ATW800_WriteByte(uint32_t offset, uint8_t val);

extern bool ATW800_UseCardScreen(void);
extern void ATW800_Render(void);

extern void ATW800_MemorySnapShot_Capture(bool bSave);
extern void ATW800_Info(FILE *fp, uint32_t arg);

#endif /* HATARI_VME_ATW800_H */
