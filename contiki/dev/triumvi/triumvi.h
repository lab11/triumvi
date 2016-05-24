#ifndef __TRIUMVIFRAMRTC_H__
#define __TRIUMVIFRAMRTC_H__

#include <stdint.h>
#include "rv3049.h"
#include "ioc.h"

#define FM25V02_MAX_ADDR 32760
#define FM25V02_MIN_ADDR 16			// reserve first 16 bytes
#define FM25V02_WRITE_LOC_ADDR 12	// Addr 12~13
#define FM25V02_READ_LOC_ADDR 14	// Addr 14~15
#define READ_PTR_TYPE 0x0
#define WRITE_PTR_TYPE 0x1
#define TRIUMVI_RECORD_SIZE 8		// size of each record, 6 bytes time, 2 bytes power

#define VOLTAGE 0x0
#define CURRENT 0x1
#define SENSE_ENABLE 0x1
#define SENSE_DISABLE 0x0

#define BATTERY_PACK_LED_RED 0x08
#define BATTERY_PACK_LED_GREEN 0x04
#define BATTERY_PACK_LED_BLUE 0x02


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
void triumviFramPtrClear();

void triumviLEDinit();
void triumviLEDON();
void triumviLEDOFF();
void triumviLEDToggle();

#ifdef VERSION9
// Enable/Disable LDO on Sensing board
void meterSenseVREn(uint8_t en);

// Dessert READYn signal
void unitReady();

// Return 1 if all units in the chain are ready, 0 otherwise
uint8_t allUnitsReady();

// Return 1 if vcap is looping back, 0 otherwise
uint8_t vcapLoopBack();
#endif

// Enable/Disable Voltage/Current Sensing
void meterSenseConfig(uint8_t type, uint8_t en);

// Enable comparater interrupt
void meterVoltageComparator(uint8_t en);

#ifndef VERSION9
#define MAX_INA_GAIN_IDX 3
#define MAX_INA_GAIN 10
#define MIN_INA_GAIN 1
void meterMUXConfig(uint8_t en);
static const uint8_t inaGainArr[4] = {1, 2, 5, 10};
uint8_t getINAGain();
void increaseINAGain();
void decreaseINAGain();
#endif
void setINAGain(uint8_t gain);


void disableSPI();
void reenableSPI();

// unregister i2c gpio
void disableI2C();


uint8_t externalVoltSel();
uint8_t isButtonPressed();

// Battery Pack functions
#ifdef VERSION9
void batteryPackVoltageEn(uint8_t en);
#endif
uint8_t batteryPackIsAttached();
uint8_t batteryPackIsUSBAttached();
void batteryPackInit();
void batteryPackLEDDriverDisable();
uint8_t batteryPackReadPanelID();
uint8_t batteryPackReadCircuitID();

void batteryPackLEDOn(uint8_t leds);
void batteryPackLEDOff(uint8_t leds);
void batteryPackLEDToggle(uint8_t leds);

void batteryPackLEDDriverInit();
uint8_t batteryPackLEDIntensityDecrease(uint8_t leds);
uint8_t batteryPackLEDIntensityIncrease(uint8_t leds);
void batteryPackLEDIntensitySet(uint8_t leds, uint8_t iOnVal);

#endif
