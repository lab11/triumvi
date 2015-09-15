#ifndef __TRIUMVIFRAMRTC_H__
#define __TRIUMVIFRAMRTC_H__

#define FM25V02_MAX_ADDR 32760
#define FM25V02_MIN_ADDR 16			// reserve first 16 bytes 
#define FM25V02_WRITE_LOC_ADDR 12	// Addr 12~13
#define FM25V02_READ_LOC_ADDR 14	// Addr 14~15
#define READ_PTR_TYPE 0x0
#define WRITE_PTR_TYPE 0x1
#define TRIUMVI_RECORD_SIZE 8		// size of each record, 6 bytes time, 2 bytes power

#include <stdint.h>
#include "rv3049.h"

typedef struct {
	uint16_t year;
	uint8_t month;
	uint8_t days;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint16_t powerReading;
} triumviData_t;

int triumviFramWrite(uint16_t powerReading, rv3049_time_t* rtctime);
int triumviFramRead(triumviData_t* record);
void triumviFramClear();

inline void triumviLEDinit();
inline void triumviLEDON();
inline void triumviLEDOFF();
inline void triumviLEDToggle();

#endif
