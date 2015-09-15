
#include <stdint.h>
#include <stdio.h>

#include "contiki.h"
#include "fm25v02.h"
#include "rv3049.h"
#include "triumviFramRTC.h"

uint16_t getReadWritePtr(uint8_t ptrType){
	uint8_t readBuf[2];
	if (ptrType==READ_PTR_TYPE)
		fm25v02_read(FM25V02_READ_LOC_ADDR, 2, readBuf);
	else
		fm25v02_read(FM25V02_WRITE_LOC_ADDR, 2, readBuf);
	uint16_t myPtr = (readBuf[0]<<8 | readBuf[1]);
	return myPtr;
}

void triumviFramClear(){
	// using Big Endianness
	uint8_t writeBuf[2];
	writeBuf[0] = (FM25V02_MIN_ADDR&0xff00)>>8;
	writeBuf[1] = FM25V02_MIN_ADDR&0xff;
	// Dummy read, wake up FRAM
	fm25v02_dummyWakeup();
	fm25v02_write(FM25V02_WRITE_LOC_ADDR, 2, writeBuf);
	fm25v02_write(FM25V02_READ_LOC_ADDR, 2, writeBuf);
	fm25v02_sleep();
}

void updatePtr(uint8_t ptrType, uint16_t readWritePtr){
	if (readWritePtr < FM25V02_MAX_ADDR)
		readWritePtr += TRIUMVI_RECORD_SIZE;
	else
		readWritePtr = FM25V02_MIN_ADDR;

	uint8_t writeBuf[2];
	writeBuf[0] = (readWritePtr & 0xff00)>>8;
	writeBuf[1] = readWritePtr & 0xff;

	if (ptrType==READ_PTR_TYPE){
		fm25v02_write(FM25V02_READ_LOC_ADDR, 2, writeBuf);
	}
	else{
		fm25v02_write(FM25V02_WRITE_LOC_ADDR, 2, writeBuf);
	}
}

// Write record into FRAM, return -1 if FRAM is full
// Otherwise, return 0 (success)
int triumviFramWrite(uint16_t powerReading, rv3049_time_t* rtctime){
	// Dummy read, wake up FRAM
	fm25v02_dummyWakeup();
	uint16_t readPtr = getReadWritePtr(READ_PTR_TYPE);
	uint16_t writePtr = getReadWritePtr(WRITE_PTR_TYPE);
	static uint8_t writeBuf[TRIUMVI_RECORD_SIZE] = {0xff};
	if (writePtr < readPtr){
		// FRAM is full
		if (readPtr == (writePtr + TRIUMVI_RECORD_SIZE))
			return -1;
	}
	else{
		// FRAM is full
		if ((readPtr==FM25V02_MIN_ADDR) && (writePtr==FM25V02_MAX_ADDR)) 
			return -1;
	}
	writeBuf[0] = (rtctime->year & 0xff0)>>4;
	writeBuf[1] = ((rtctime->year & 0xf)<<4) | ((uint8_t)rtctime->month & 0xf);
	writeBuf[2] = rtctime->days;
	writeBuf[3] = rtctime->hours;
	writeBuf[4] = rtctime->minutes;
	writeBuf[5] = rtctime->seconds;
	writeBuf[6] = (powerReading&0xff00)>>8;
	writeBuf[7] = powerReading&0xff;
	fm25v02_write(writePtr, TRIUMVI_RECORD_SIZE, writeBuf);
	updatePtr(WRITE_PTR_TYPE, writePtr);
	fm25v02_sleep();
	return 0;
}

int triumviFramRead(triumviData_t* record){
	// Dummy read, wake up FRAM
	fm25v02_dummyWakeup();
	uint16_t readPtr = getReadWritePtr(READ_PTR_TYPE);
	uint16_t writePtr = getReadWritePtr(WRITE_PTR_TYPE);
	static uint8_t readBuf[TRIUMVI_RECORD_SIZE];
	// FRAM is empty
	if (readPtr==writePtr){
		return -1;
	}
	fm25v02_read(readPtr, TRIUMVI_RECORD_SIZE, readBuf);
	record->year = readBuf[0]<<4 | ((readBuf[1]&0xf0)>>4);
	record->month = (readBuf[1] & 0xf);
	record->days = readBuf[2];
	record->hours = readBuf[3];
	record->minutes = readBuf[4];
	record->seconds = readBuf[5];
	record->powerReading = ((readBuf[6]<<8) | readBuf[7]);
	updatePtr(READ_PTR_TYPE, readPtr);
	fm25v02_sleep();
	return 0;
}

inline void triumviLEDinit(){
	GPIO_SET_OUTPUT(LED_RED_BASE, LED_RED_MASK);
	GPIO_SET_PIN(LED_RED_BASE, LED_RED_MASK);
}

inline void triumviLEDON(){
	GPIO_CLR_PIN(LED_RED_BASE, LED_RED_MASK);
}

inline void triumviLEDOFF(){
	GPIO_SET_PIN(LED_RED_BASE, LED_RED_MASK);
}

inline void triumviLEDToggle(){
	uint8_t portState = GPIO_READ_PIN(LED_RED_BASE, LED_RED_MASK);
	if (portState > 0)
		GPIO_CLR_PIN(LED_RED_BASE, LED_RED_MASK);
	else
		GPIO_SET_PIN(LED_RED_BASE, LED_RED_MASK);
}

