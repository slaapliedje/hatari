/*
  Hatari - vme_nova.h

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.
*/

#ifndef HATARI_VME_NOVA_H
#define HATARI_VME_NOVA_H

#include "sysdeps.h"

extern void VME_Init(void);
extern void VME_UnInit(void);
extern bool VME_IsAvailable(void);
extern void VME_Reset(bool bCold);
extern void VME_MemorySnapShot_Capture(bool bSave);
extern void VME_Info(FILE *fp, uint32_t dummy);

extern uae_u32 REGPARAM3 VME_Mem_bget(uaecptr addr);
extern uae_u32 REGPARAM3 VME_Mem_wget(uaecptr addr);
extern uae_u32 REGPARAM3 VME_Mem_lget(uaecptr addr);
extern void REGPARAM3 VME_Mem_bput(uaecptr addr, uae_u32 val);
extern void REGPARAM3 VME_Mem_wput(uaecptr addr, uae_u32 val);
extern void REGPARAM3 VME_Mem_lput(uaecptr addr, uae_u32 val);
extern int REGPARAM3 VME_Mem_check(uaecptr addr, uae_u32 size);
extern uae_u8 * REGPARAM3 VME_Mem_xlate(uaecptr addr);

#endif /* HATARI_VME_NOVA_H */
