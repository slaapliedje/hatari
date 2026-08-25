/*
  Hatari - video_et4000.h

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.
*/

#ifndef HATARI_VIDEO_ET4000_H
#define HATARI_VIDEO_ET4000_H

#include "sysdeps.h"

extern void ET4000_Init(void);
extern void ET4000_UnInit(void);
extern void ET4000_Reset(bool bCold);

extern uint8_t ET4000_IO_ReadByte(uint16_t port);
extern void ET4000_IO_WriteByte(uint16_t port, uint8_t val);
extern uint8_t ET4000_Mem_ReadByte(uint32_t offset);
extern void ET4000_Mem_WriteByte(uint32_t offset, uint8_t val);

extern bool ET4000_UseCardScreen(void);
extern void ET4000_Render(void);

extern void ET4000_MemorySnapShot_Capture(bool bSave);
extern void ET4000_Info(FILE *fp, uint32_t arg);

#endif /* HATARI_VIDEO_ET4000_H */
