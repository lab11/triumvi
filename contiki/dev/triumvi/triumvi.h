#ifndef __TRIUMVIFRAMRTC_H__
#define __TRIUMVIFRAMRTC_H__

#include <stdint.h>
#include "rv3049.h"
#include "ioc.h"

#if defined(FM25V02)
#define FRAM_DATA_MAX_LOC_ADDR 32732
int (*fram_write)(uint16_t, uint16_t, uint8_t*);
int (*fram_read)(uint16_t, uint16_t, uint8_t*);
#elif defined(FM25CL64B)
#define FRAM_DATA_MAX_LOC_ADDR 8152
int (*fram_write)(uint16_t, uint16_t, uint8_t*);
int (*fram_read)(uint16_t, uint16_t, uint8_t*);
#endif
void (*fram_erase_all)();
#define TRIUMVI_RECORD_SIZE 20  // size of each record, 6 bytes time, 13 bytes power, 1 byte reserved
#define FRAM_WRITE_PTR_LOC_ADDR 12	// Addr 12~13
#define FRAM_READ_PTR_LOC_ADDR 14	// Addr 14~15
#define FRAM_CALIBRATION_DATA_VALID_LOC_ADDR 16         // 1 byte, 7 bits idx, 1 bits valid
#define FRAM_CALIBRATION_DATA_PHASE_OFFSET_LOC_ADDR 18  // 4 bytes
#define FRAM_CALIBRATION_DATA_I_FIT_LOC_ADDR 22         // 4 bytes, 22 + 24*idx
#define FRAM_CALIBRATION_DATA_P_FIT_LOC_ADDR 34         // 4 bytes, 34 + 24*idx

#define CURRENT_FIT_TYPE 0x0
#define POWER_FIT_TYPE 0x1

#define FRAM_DATA_MIN_LOC_ADDR 192
#define READ_PTR_TYPE 0x0
#define WRITE_PTR_TYPE 0x1

#define VOLTAGE 0x0
#define CURRENT 0x1
#define SENSE_ENABLE 0x1
#define SENSE_DISABLE 0x0

#define BATTERY_PACK_LED_RED 0x08
#define BATTERY_PACK_LED_GREEN 0x04
#define BATTERY_PACK_LED_BLUE 0x02

#if defined(FM25V02) || defined(FM25CL64B)
#define FRAM_ENABLE
#endif

typedef struct{
    int avgPower;
    uint8_t triumviStatusReg;
    uint8_t panelID;
    uint8_t circuitID;
    uint16_t pf;
    uint16_t VRMS;
    uint16_t IRMS;
} triumvi_record_t;


typedef struct {
    triumvi_record_t sample;
	uint16_t year;
	uint8_t month;
	uint8_t days;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
} triumviData_t;

typedef struct {
    uint8_t type;
    uint32_t numerator;
    uint32_t denumerator;
    int offset;
    uint8_t gain_idx;
} linearFitCalData_t;

typedef struct {
    uint16_t dc_Offset;
    uint16_t phase_Offset;
} phaseOffsetCalData_t;

#ifdef FRAM_ENABLE
int triumviFramWrite(triumvi_record_t* thisSample, rv3049_time_t* rtctime);
int triumviFramRead(triumviData_t* record);
void triumviFramPtrClear();
// clear data valid flag
void triumviFramCalibrateDataValidClear();
uint8_t triumviFramCalibrateDataValidRead();
void triumviFramCalibrateDataPhaseWrite(phaseOffsetCalData_t* calData);
void triumviFramCalibrateDataPhaseRead(phaseOffsetCalData_t* calData);
void triumviFramCalibrateDataFitWrite(linearFitCalData_t* calData);
void triumviFramCalibrateDataFitRead(linearFitCalData_t* calData);
#endif

void triumviLEDinit();
void triumviLEDON();
void triumviLEDOFF();
void triumviLEDToggle();

#ifndef VERSION8 
// Enable/Disable LDO on Sensing board
void meterSenseVREn(uint8_t en);

// signals unit is ready
void unitReady();

// clear the ready signal
void unitClrReady();

// Return 1 if all units in the chain are ready, 0 otherwise
uint8_t allUnitsReady();

// Return 1 if vcap is looping back, 0 otherwise
uint8_t vcapLoopBack();
#endif

// Enable/Disable Voltage/Current Sensing
void meterSenseConfig(uint8_t type, uint8_t en);

// Enable comparater interrupt
void meterVoltageComparator(uint8_t en);

#ifdef VERSION8
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

uint8_t externalVoltSel();
uint8_t isButtonPressed();

// Battery Pack functions
#ifndef VERSION8
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

uint16_t mysqrt(uint32_t n);

// return variance of phase offsets 
uint16_t getVariance(uint16_t* data, uint16_t length);

// return mean(data)
uint16_t getAverage(uint16_t* data, uint16_t length);
// return mean(data)
uint16_t getAverage32(int* data, uint16_t length);

// split a uint32_t into 4 uint8_t
void packData(uint8_t* dest, int src, uint8_t len);

#endif
