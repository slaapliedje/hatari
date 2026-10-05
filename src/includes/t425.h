/*
  Hatari - t425.h

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  An INMOS IMS T425 transputer (the ATW800/2's FPGA transputer). The core
  depends only on the C library, so a test harness can run it alone.
*/

#ifndef HATARI_T425_H
#define HATARI_T425_H

#include <stdint.h>
#include <stdbool.h>

#define T425_LINKS	4

/* Addresses outside the RAM go to these (an address: the byte's; size: 1
 * or 4, a word's address is aligned). NULL: reads 0, writes ignored. */
typedef struct
{
	uint32_t (*read)(uint32_t addr, int size);
	void (*write)(uint32_t addr, uint32_t val, int size);
	void (*log)(const char *msg);		/* NULL: quiet */
} t425_bus_t;

/* ram_size: the memory from 0x80000000 (the 4 KB on chip included).
 * cycles_per_us: the processor cycles in a microsecond of its timers.
 * false when out of memory. */
extern bool T425_Init(uint32_t ram_size, int cycles_per_us, const t425_bus_t *bus);
extern void T425_UnInit(void);

/* A window of memory shared with something else (the card's video RAM),
 * at base, size bytes; NULL removes it. */
extern void T425_SetShared(uint8_t *mem, uint32_t base, uint32_t size);

/* The Reset and Analyse pins. Reset going low starts a wait for a boot
 * on any link; with Analyse high, the halted state is kept for a look. */
extern void T425_SetReset(bool high);
extern void T425_SetAnalyse(bool high);
extern bool T425_Error(void);		/* the Error pin */

/* Run up to cycles processor cycles; returns those used (idle time
 * counts: an idle transputer's clock still moves). */
extern uint64_t T425_Run(uint64_t cycles);
extern uint64_t T425_Cycles(void);	/* since T425_Init */

/* The far end of a link (a C011's view): a byte can go in when the last
 * one was taken, and one waits to come out. */
extern bool T425_LinkCanWrite(int link);
extern void T425_LinkWrite(int link, uint8_t byte);
extern bool T425_LinkCanRead(int link);
extern uint8_t T425_LinkRead(int link);

/* What it is doing, for the debugger and logs. */
extern void T425_Info(void (*print)(const char *line));

/* Trace each instruction (to the log). */
extern void T425_SetTrace(bool on);

/* The state, for memory snapshots: the blob holds everything but the RAM. */
extern uint32_t T425_StateSize(void);
extern void *T425_State(void);
extern uint8_t *T425_Ram(uint32_t *size);

#endif /* HATARI_T425_H */
