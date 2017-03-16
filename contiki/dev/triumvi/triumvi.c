
#include <stdint.h>
#include <stdio.h>

#include "contiki.h"
#include "clock.c"
#include "fm25v02.h"
#include "fm25cl64b.h"
#include "rv3049.h"
#include "sx1509b.h"
#include "triumvi.h"
#include "ad5274.h"
#include "ioc.h"

// FRAM hold pin shares the same pin as analog switch in version 10
// This imposes a problem that FRAM can only be accessed when the sensing
// frontend is enabled and gain is not set to 1.

#if defined(FM25V02)
int (*fram_write)(uint16_t, uint16_t, uint8_t*) = &fm25v02_write;
int (*fram_read)(uint16_t, uint16_t, uint8_t*) = &fm25v02_read;
void (*fram_erase_all)() = &fm25v02_eraseAll;
#elif defined(FM25CL64B)
int (*fram_write)(uint16_t, uint16_t, uint8_t*) = &fm25cl64b_write;
int (*fram_read)(uint16_t, uint16_t, uint8_t*) = &fm25cl64b_read;
void (*fram_erase_all)() = &fm25cl64b_eraseAll;
#endif

#ifdef FRAM_ENABLE
void triumviFramCalibrateDataValidClear(){
    uint8_t writeBuf = 0x00;
    (*fram_write)(FRAM_CALIBRATION_DATA_VALID_LOC_ADDR, 1, &writeBuf);
}

uint8_t triumviFramCalibrateDataValidRead(){
    uint8_t readBuf;
    (*fram_read)(FRAM_CALIBRATION_DATA_VALID_LOC_ADDR, 1, &readBuf);
    return readBuf;
}

void triumviFramCalibrateDataPhaseWrite(phaseOffsetCalData_t* calData){
    uint8_t writeBuf[4];
    uint8_t valid = 0x01;
    packData(&writeBuf[0], calData->dc_Offset, 2);
    packData(&writeBuf[2], calData->phase_Offset, 2);
    (*fram_write)(FRAM_CALIBRATION_DATA_PHASE_OFFSET_LOC_ADDR, 4, writeBuf);
    (*fram_write)(FRAM_CALIBRATION_DATA_VALID_LOC_ADDR, 1, &valid);
}

void triumviFramCalibrateDataPhaseRead(phaseOffsetCalData_t* calData){
    uint8_t readBuf[4];
    (*fram_read)(FRAM_CALIBRATION_DATA_PHASE_OFFSET_LOC_ADDR, 4, readBuf);
    calData->dc_Offset = (readBuf[1]<<8 | readBuf[0]);
    calData->phase_Offset = (readBuf[3]<<8 | readBuf[2]);
}

void triumviFramCalibrateDataFitWrite(linearFitCalData_t* calData){
    uint8_t writeBuf[12];
    uint16_t addr = (calData->type==CURRENT_FIT_TYPE)? FRAM_CALIBRATION_DATA_I_FIT_LOC_ADDR : FRAM_CALIBRATION_DATA_P_FIT_LOC_ADDR; 
    uint8_t valid;
    valid = triumviFramCalibrateDataValidRead();
    addr += calData->gain_idx*24;
    packData(&writeBuf[0], calData->numerator, 4);
    packData(&writeBuf[4], calData->denumerator, 4);
    packData(&writeBuf[8], calData->offset, 4);
    (*fram_write)(addr, 12, writeBuf);
    valid |= 1<<((calData->gain_idx)+1);
    (*fram_write)(FRAM_CALIBRATION_DATA_VALID_LOC_ADDR, 1, &valid);
}

void triumviFramCalibrateDataFitRead(linearFitCalData_t* calData){
    uint8_t readBuf[12];
    uint16_t addr = (calData->type==CURRENT_FIT_TYPE)? FRAM_CALIBRATION_DATA_I_FIT_LOC_ADDR : FRAM_CALIBRATION_DATA_P_FIT_LOC_ADDR; 
    addr += calData->gain_idx*24;
    (*fram_read)(addr, 12, readBuf);
    calData->numerator = (readBuf[3]<<24 | readBuf[2]<<16 | readBuf[1]<<8 | readBuf[0]);
    calData->denumerator = (readBuf[7]<<24 | readBuf[6]<<16 | readBuf[5]<<8 | readBuf[4]);
    calData->offset = (readBuf[11]<<24 | readBuf[10]<<16 | readBuf[9]<<8 | readBuf[8]);
}

void triumviFramCounterWrite(uint32_t counterVal){
    uint8_t writeBuf[4];
    packData(writeBuf, counterVal, 4);
    (*fram_write)(COUNTER_LOC_ADDR, 4, writeBuf);
}

uint32_t triumviFramCounterRead(){
    uint8_t readBuf[4];
    (*fram_read)(COUNTER_LOC_ADDR, 4, readBuf);
    return (readBuf[3]<<24 | readBuf[2]<<16 | readBuf[1]<<8 | readBuf[0]);
}

void triumviFramDCOffsetWrite(uint16_t dc_offset, uint8_t inaGainIdx){
    uint8_t writeBuf[2];
    packData(writeBuf, dc_offset, 2);
    (*fram_write)(DC_OFFSET_LOC_ADDR+inaGainIdx*2, 2, writeBuf);
}

uint16_t triumviFramDCOffsetRead(uint8_t inaGainIdx){
    uint8_t readBuf[2];
    (*fram_read)(DC_OFFSET_LOC_ADDR+inaGainIdx*2, 2, readBuf);
    return (readBuf[1]<<8 | readBuf[0]);
}

uint16_t getReadWritePtr(uint8_t ptrType){
    uint8_t readBuf[2];
    uint16_t destAddr = (ptrType==READ_PTR_TYPE)? FRAM_READ_PTR_LOC_ADDR : FRAM_WRITE_PTR_LOC_ADDR; 
    (*fram_read)(destAddr, 2, readBuf);
    uint16_t myPtr = (readBuf[0]<<8 | readBuf[1]);
    return myPtr;
}

void triumviFramPtrClear(){
    // using Big Endianness
    uint8_t writeBuf[2];
    writeBuf[0] = (FRAM_DATA_MIN_LOC_ADDR&0xff00)>>8;
    writeBuf[1] = FRAM_DATA_MIN_LOC_ADDR&0xff;
    #ifdef FM25V02
    fm25v02_dummyWakeup();
    #endif
    (*fram_write)(FRAM_WRITE_PTR_LOC_ADDR, 2, writeBuf);
    (*fram_write)(FRAM_READ_PTR_LOC_ADDR, 2, writeBuf);
    #ifdef FM25V02
    fm25v02_sleep();
    #endif
}

void updatePtr(uint8_t ptrType, uint16_t readWritePtr){
    if (readWritePtr < FRAM_DATA_MAX_LOC_ADDR){
        readWritePtr += TRIUMVI_RECORD_SIZE;
    } else{
        readWritePtr = FRAM_DATA_MIN_LOC_ADDR;
    }

    uint8_t writeBuf[2];
    writeBuf[0] = (readWritePtr & 0xff00)>>8;
    writeBuf[1] = readWritePtr & 0xff;
    uint16_t destAddr = (ptrType==READ_PTR_TYPE)? FRAM_READ_PTR_LOC_ADDR : FRAM_WRITE_PTR_LOC_ADDR; 
    (*fram_write)(destAddr, 2, writeBuf);
}

// Write record into FRAM, return -1 if FRAM is full
// Otherwise, return 0 (success)
int triumviFramWrite(triumvi_record_t* thisSample, rv3049_time_t* rtctime){
    // Dummy read, wake up FRAM
    #ifdef FM25V02
    fm25v02_dummyWakeup();
    #endif
    uint16_t readPtr = getReadWritePtr(READ_PTR_TYPE);
    uint16_t writePtr = getReadWritePtr(WRITE_PTR_TYPE);
    static uint8_t writeBuf[TRIUMVI_RECORD_SIZE] = {0xff};
    if (writePtr < readPtr){
        // FRAM is full
        if (readPtr == (writePtr + TRIUMVI_RECORD_SIZE)){
            return -1;
        }
    } else {
        // FRAM is full
        if ((readPtr==FRAM_DATA_MIN_LOC_ADDR ) && (writePtr==FRAM_DATA_MAX_LOC_ADDR )){
            return -1;
        }
    }
    // triumvi_record_t 
    packData(&writeBuf[0], thisSample->avgPower, 4); // 0~3
    writeBuf[4] = thisSample->triumviStatusReg;
    writeBuf[5] = thisSample->panelID;
    writeBuf[6] = thisSample->circuitID;
    packData(&writeBuf[7], thisSample->pf, 2); 
    packData(&writeBuf[9], thisSample->VRMS, 2); 
    packData(&writeBuf[11], thisSample->IRMS, 2); 
    writeBuf[13] = 0x00; // reserved byte, 0 for now
    // rv3049_time_t
    writeBuf[14] = (rtctime->year - 2000);
    writeBuf[15] = rtctime->month;
    writeBuf[16] = rtctime->days;
    writeBuf[17] = rtctime->hours;
    writeBuf[18] = rtctime->minutes;
    writeBuf[19] = rtctime->seconds;

    (*fram_write)(writePtr, TRIUMVI_RECORD_SIZE, writeBuf);
    updatePtr(WRITE_PTR_TYPE, writePtr);

    #ifndef FM25V02
    fm25v02_sleep();
    #endif
    return 0;
}

int triumviFramRead(triumviData_t* record){
    // Dummy read, wake up FRAM
    #ifdef FM25V02
    fm25v02_dummyWakeup();
    #endif
    uint16_t readPtr = getReadWritePtr(READ_PTR_TYPE);
    uint16_t writePtr = getReadWritePtr(WRITE_PTR_TYPE);
    static uint8_t readBuf[TRIUMVI_RECORD_SIZE];
    // FRAM is empty
    if (readPtr==writePtr){
        return -1;
    }
    (*fram_read)(readPtr, TRIUMVI_RECORD_SIZE, readBuf);

    // triumvi_record_t
    record->sample.avgPower = (readBuf[3]<<24 | readBuf[2]<<16 | readBuf[1]<<8 | readBuf[0]);
    record->sample.triumviStatusReg = readBuf[4];
    record->sample.panelID = readBuf[5];
    record->sample.circuitID = readBuf[6];
    record->sample.pf = (readBuf[8]<<8 | readBuf[7]);
    record->sample.VRMS = (readBuf[10]<<8 | readBuf[9]);
    record->sample.IRMS = (readBuf[12]<<8 | readBuf[11]);

    // time stamp
    record->year = readBuf[14] + 2000;
    record->month = readBuf[15];
    record->days = readBuf[16];
    record->hours = readBuf[17];
    record->minutes = readBuf[18];
    record->seconds = readBuf[19];

    updatePtr(READ_PTR_TYPE, readPtr);
    #ifndef FM25CL64B
    fm25v02_sleep();
    #endif
    return 0;
}
#endif

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

#ifndef VERSION8
void meterSenseVREn(uint8_t en){
    // enable voltage regulator
    if (en==SENSE_ENABLE)
        GPIO_SET_PIN(SENSE_VR_EN_GPIO_BASE, 0x1<<SENSE_VR_EN_GPIO_PIN );
    else
        GPIO_CLR_PIN(SENSE_VR_EN_GPIO_BASE, 0x1<<SENSE_VR_EN_GPIO_PIN );
}

void unitReady(){
    #ifdef VERSION9
    GPIO_CLR_PIN(TRIUMVI_READYn_OUT_GPIO_BASE, 0x1<<TRIUMVI_READYn_OUT_GPIO_PIN);
    #else
    GPIO_SET_PIN(TRIUMVI_READYn_OUT_GPIO_BASE, 0x1<<TRIUMVI_READYn_OUT_GPIO_PIN);
    #endif
}

void unitClrReady(){
    #ifdef VERSION9
    GPIO_SET_PIN(TRIUMVI_READYn_OUT_GPIO_BASE, 0x1<<TRIUMVI_READYn_OUT_GPIO_PIN);
    #else
    GPIO_CLR_PIN(TRIUMVI_READYn_OUT_GPIO_BASE, 0x1<<TRIUMVI_READYn_OUT_GPIO_PIN);
    #endif
}

uint8_t allUnitsReady(){
    uint8_t tmp = GPIO_READ_PIN(TRIUMVI_RDYn_IN_GPIO_BASE, (0x1<<TRIUMVI_RDYn_IN_GPIO_PIN));
    #ifdef VERSION9
    if (tmp>0) return 0; else return 1;
    #else
    if (tmp>0) return 1; else return 0;
    #endif
}

uint8_t vcapLoopBack(){
    uint8_t tmp = GPIO_READ_PIN(CONFIG_VCAP_LOOPBACK_GPIO_BASE, (0x1<<CONFIG_VCAP_LOOPBACK_GPIO_PIN));
    #ifdef VERSION9
    if (tmp>0) return 1; else return 0;
    #else
    if (tmp>0) return 0; else return 1;
    #endif

}
#endif

void meterSenseConfig(uint8_t type, uint8_t en){
	if (type==VOLTAGE){
		if (en==SENSE_ENABLE)
			GPIO_SET_PIN(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
		else
			GPIO_CLR_PIN(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
	}
	else{
		if (en==SENSE_ENABLE){
			GPIO_SET_PIN(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);
            #if defined(VERSION10) || defined(VERSION11)
            GPIO_SET_PIN(GPIO_PORT_TO_BASE(FM25V02_HOLD_N_PORT_NUM), 
                        0x1<<FM25V02_HOLD_N_PIN);
            #elif defined(VERSION12)
            GPIO_SET_PIN(ADG604_EN_GPIO_BASE, 0x1<<ADG604_EN_PIN);
            #endif
        }
		else{
            GPIO_CLR_PIN(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);
            #if defined(VERSION10) || defined(VERSION11) || defined(VERSION12)
            setINAGain(1);
            #endif
        }
	}
}

inline void meterVoltageComparator(uint8_t en){
	GPIO_CLEAR_INTERRUPT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	if (en==SENSE_ENABLE){
		GPIO_ENABLE_INTERRUPT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
		nvic_interrupt_enable(V_REF_CROSS_INT_NVIC_PORT);
	}
	else{
		GPIO_DISABLE_INTERRUPT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
		nvic_interrupt_disable(V_REF_CROSS_INT_NVIC_PORT);
	}
}

#ifdef VERSION8
inline void meterMUXConfig(uint8_t en){
	if (en==SENSE_ENABLE)
		GPIO_SET_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_EN_GPIO_PIN);
	else
		GPIO_CLR_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_EN_GPIO_PIN);
}

inline uint8_t getINAIDX(){
	uint8_t mux_a0_sel = GPIO_READ_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN)>>MUX_A0_GPIO_PIN;
	uint8_t mux_a1_sel = GPIO_READ_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN)>>MUX_A1_GPIO_PIN;
	return (mux_a1_sel<<1 | mux_a0_sel);
}

void increaseINAGain(){
	uint8_t inaIDX = getINAIDX();
	if (inaIDX<MAX_INA_GAIN_IDX){
		setINAGain(inaGainArr[(inaIDX+1)]);
	}
}

void decreaseINAGain(){
	uint8_t inaIDX = getINAIDX();
	if (inaIDX>0){
		setINAGain(inaGainArr[(inaIDX-1)]);
	}
}

inline uint8_t getINAGain(){
	return inaGainArr[getINAIDX()];
}
#endif

void setINAGain(uint8_t gain){
    #ifdef VERSION8
	switch (gain){
		case 1: // Select S1
			GPIO_CLR_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN);
			GPIO_CLR_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN);
		break;
		case 2: // Select S2
			GPIO_SET_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN);
			GPIO_CLR_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN);
		break;
		case 5: // Select S3
			GPIO_CLR_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN);
			GPIO_SET_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN);
		break;
		case 10: // Select S4
			GPIO_SET_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN);
			GPIO_SET_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN);
		break;
		default:
		break;
	}
    #elif defined(VERSION9)
    switch (gain){
        case 1:
            // shutdown --> Rg of INA 333 open
            ad5274_shutdown(0x1);
        break;

        case 2:
            ad5274_rdac_write(1023);
        break;

        case 3:
            ad5274_rdac_write(512);
        break;
        
        case 5:
            ad5274_rdac_write(256);
        break;

        case 9:
            ad5274_rdac_write(128);
        break;

        case 17:
            ad5274_rdac_write(64);
        break;

        default:
        break;
    }
    #elif defined(VERSION10)
    switch (gain){
        case 1:
            // This pin is shared with mux enable 
            GPIO_CLR_PIN(GPIO_PORT_TO_BASE(FM25V02_HOLD_N_PORT_NUM), 
                        0x1<<FM25V02_HOLD_N_PIN);
        break;

        case 2:
            GPIO_SET_PIN(GPIO_PORT_TO_BASE(FM25V02_HOLD_N_PORT_NUM), 
                        0x1<<FM25V02_HOLD_N_PIN);
            ad5274_rdac_write(1023);
        break;

        case 3:
            GPIO_SET_PIN(GPIO_PORT_TO_BASE(FM25V02_HOLD_N_PORT_NUM), 
                        0x1<<FM25V02_HOLD_N_PIN);
            ad5274_rdac_write(512);
        break;
        
        case 5:
            GPIO_SET_PIN(GPIO_PORT_TO_BASE(FM25V02_HOLD_N_PORT_NUM), 
                        0x1<<FM25V02_HOLD_N_PIN);
            ad5274_rdac_write(256);
        break;

        case 9:
            GPIO_SET_PIN(GPIO_PORT_TO_BASE(FM25V02_HOLD_N_PORT_NUM), 
                        0x1<<FM25V02_HOLD_N_PIN);
            ad5274_rdac_write(128);
        break;

        case 17:
            GPIO_SET_PIN(GPIO_PORT_TO_BASE(FM25V02_HOLD_N_PORT_NUM), 
                        0x1<<FM25V02_HOLD_N_PIN);
            ad5274_rdac_write(64);
        break;

        default:
        break;
    }
    #elif defined(VERSION11)
    switch (gain){
        case 1: // disable switch, and set to s1
            // This pin is shared with mux enable 
            GPIO_CLR_PIN(GPIO_PORT_TO_BASE(FM25V02_HOLD_N_PORT_NUM), 
                        0x1<<FM25V02_HOLD_N_PIN);
            GPIO_CLR_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A0_PIN);
            GPIO_CLR_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A1_PIN);
        break;
        case 3: // Select S1
            GPIO_SET_PIN(GPIO_PORT_TO_BASE(FM25V02_HOLD_N_PORT_NUM), 
                        0x1<<FM25V02_HOLD_N_PIN);
            GPIO_CLR_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A0_PIN);
            GPIO_CLR_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A1_PIN);
        break;
        case 5: // Select S2
            GPIO_SET_PIN(GPIO_PORT_TO_BASE(FM25V02_HOLD_N_PORT_NUM), 
                        0x1<<FM25V02_HOLD_N_PIN);
            GPIO_SET_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A0_PIN);
            GPIO_CLR_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A1_PIN);
        break;
        case 9: // Select S3
            GPIO_SET_PIN(GPIO_PORT_TO_BASE(FM25V02_HOLD_N_PORT_NUM), 
                        0x1<<FM25V02_HOLD_N_PIN);
            GPIO_CLR_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A0_PIN);
            GPIO_SET_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A1_PIN);
        break;
        case 17: // Select S4
            GPIO_SET_PIN(GPIO_PORT_TO_BASE(FM25V02_HOLD_N_PORT_NUM), 
                        0x1<<FM25V02_HOLD_N_PIN);
            GPIO_SET_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A0_PIN);
            GPIO_SET_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A1_PIN);
        break;
        default:
        break;
    }
    #elif defined(VERSION12)
    switch (gain){
        case 1: // disable switch, and set to s1
            GPIO_CLR_PIN(ADG604_EN_GPIO_BASE, 0x1<<ADG604_EN_PIN);
            GPIO_CLR_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A0_PIN);
            GPIO_CLR_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A1_PIN);
        break;
        case 3: // Select S1
            GPIO_SET_PIN(ADG604_EN_GPIO_BASE, 0x1<<ADG604_EN_PIN);
            GPIO_CLR_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A0_PIN);
            GPIO_CLR_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A1_PIN);
        break;
        case 5: // Select S2
            GPIO_SET_PIN(ADG604_EN_GPIO_BASE, 0x1<<ADG604_EN_PIN);
            GPIO_SET_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A0_PIN);
            GPIO_CLR_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A1_PIN);
        break;
        case 9: // Select S3
            GPIO_SET_PIN(ADG604_EN_GPIO_BASE, 0x1<<ADG604_EN_PIN);
            GPIO_CLR_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A0_PIN);
            GPIO_SET_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A1_PIN);
        break;
        case 17: // Select S4
            GPIO_SET_PIN(ADG604_EN_GPIO_BASE, 0x1<<ADG604_EN_PIN);
            GPIO_SET_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A0_PIN);
            GPIO_SET_PIN(ADG604_GPIO_BASE, 0x1<<ADG604_A1_PIN);
        break;
        default:
        break;
    }
    #endif
}


void disableSPI(){
	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SPI_CLK_PORT), 0x1<<SPI_CLK_PIN);
	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SPI_MOSI_PORT), 0x1<<SPI_MOSI_PIN);
	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SPI_MISO_PORT), 0x1<<SPI_MISO_PIN);
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(SPI_CLK_PORT), 0x1<<SPI_CLK_PIN);
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(SPI_MOSI_PORT), 0x1<<SPI_MOSI_PIN);
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(SPI_MISO_PORT), 0x1<<SPI_MISO_PIN);
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(SPI_CLK_PORT), 0x1<<SPI_CLK_PIN);
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(SPI_MOSI_PORT), 0x1<<SPI_MOSI_PIN);
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(SPI_MISO_PORT), 0x1<<SPI_MISO_PIN);
}

void reenableSPI(){
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(SPI_CLK_PORT), 0x1<<SPI_CLK_PIN);
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(SPI_MOSI_PORT), 0x1<<SPI_MOSI_PIN);
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(SPI_MISO_PORT), 0x1<<SPI_MISO_PIN);
}


inline uint8_t externalVoltSel(){
    ioc_set_over(EXT_VOLT_IN_SEL_GPIO_NUM, EXT_VOLT_IN_SEL_GPIO_PIN, IOC_OVERRIDE_PDE);
    if (GPIO_READ_PIN(EXT_VOLT_IN_SEL_GPIO_BASE, 0x1<<EXT_VOLT_IN_SEL_GPIO_PIN))
        return 1;
    else
        return 0;
}

inline uint8_t isButtonPressed(){
	if (GPIO_READ_PIN(MEM_RST_GPIO_BASE, 0x1<<MEM_RST_GPIO_PIN)>>MEM_RST_GPIO_PIN)
		return 0;
	else
		return 1;
}

#ifndef VERSION8
void batteryPackVoltageEn(uint8_t en){
    GPIO_SET_OUTPUT(CONFIG_PWR_SW_GPIO_BASE , 0x1<<CONFIG_PWR_SW_GPIO_PIN);
    if (en==SENSE_ENABLE)
        GPIO_SET_PIN(CONFIG_PWR_SW_GPIO_BASE , 0x1<<CONFIG_PWR_SW_GPIO_PIN);
    else
        GPIO_CLR_PIN(CONFIG_PWR_SW_GPIO_BASE , 0x1<<CONFIG_PWR_SW_GPIO_PIN);
}
#endif

uint8_t batteryPackIsAttached(){
    #ifdef VERSION8
	// set input, disable internal pull-up
	// If external battery pack is attached, I2C should be hold high
	GPIO_SET_INPUT(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
	// enable pull down resistor for 10 us
	ioc_set_over(I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN, IOC_OVERRIDE_PDE);
	clock_delay_usec(10);
	// disable pull down resistor
	ioc_set_over(I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN, IOC_OVERRIDE_DIS);
	uint8_t result = GPIO_READ_PIN(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN)>>I2C_SCL_GPIO_PIN;
	// If pin is high, set both pins input and returns. 
	if (result>0){
		GPIO_SET_INPUT(I2C_SDA_GPIO_BASE, 0x1<<I2C_SDA_GPIO_PIN);
		ioc_set_over(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, IOC_OVERRIDE_DIS);
		return 1;
	}
	// Otherwise, set it to output and drive it low
	GPIO_SET_OUTPUT(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
	GPIO_CLR_PIN(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
	return 0;
    #else
    ioc_set_over(CONFIG_PWR_LOOPBAK_GPIO_NUM, CONFIG_PWR_LOOPBAK_GPIO_PIN, IOC_OVERRIDE_PDE);
    clock_delay_usec(5);
    ioc_set_over(CONFIG_PWR_LOOPBAK_GPIO_NUM, CONFIG_PWR_LOOPBAK_GPIO_PIN, IOC_OVERRIDE_DIS);
    batteryPackVoltageEn(SENSE_ENABLE);
    clock_delay_usec(5);
    uint8_t tmp = GPIO_READ_PIN(CONFIG_PWR_LOOPBAK_GPIO_BASE, 0x1<<CONFIG_PWR_LOOPBAK_GPIO_PIN);
    batteryPackVoltageEn(SENSE_DISABLE);
    if (tmp > 0)
        return 1;
    else
        return 0;
    #endif
}

void batteryPackInit(){
	// I2C init
	sx1509b_init();
    #ifdef VERSION8
	sx1509b_software_reset();
    #endif
	// Port A pin 0 is connected to VUSB directly
	sx1509b_high_voltage_input_enable(SX1509B_PORTA, 0x1, SX1509B_HIGH_INPUT_ENABLE);
	// Setup RGB led IOs
	sx1509b_gpio_pullup_cfg(SX1509B_PORTA, 0x0e, SX1509B_OUTPUT_RESISTOR_DISABLE);
	sx1509b_gpio_set_output(SX1509B_PORTA, 0x0e);
	sx1509b_gpio_output_type(SX1509B_PORTA, 0x0e, SX1509B_OUTPUT_TYPE_OPENDRAIN);
	sx1509b_gpio_set_pin(SX1509B_PORTA, 0x0e);
	// Disable all pull-up and pull-down resistors
	sx1509b_gpio_pullup_cfg(SX1509B_PORTA, 0xff, SX1509B_OUTPUT_RESISTOR_DISABLE);
	sx1509b_gpio_pullup_cfg(SX1509B_PORTB, 0xff, SX1509B_OUTPUT_RESISTOR_DISABLE);
	sx1509b_gpio_pulldown_cfg(SX1509B_PORTA, 0xff, SX1509B_OUTPUT_RESISTOR_DISABLE);
	sx1509b_gpio_pulldown_cfg(SX1509B_PORTB, 0xff, SX1509B_OUTPUT_RESISTOR_DISABLE);
}

uint8_t batteryPackReadPanelID(){
	// enable pull-up resistor before read IO state
	sx1509b_gpio_pullup_cfg(SX1509B_PORTA, 0xf0, SX1509B_OUTPUT_RESISTOR_ENABLE);
	uint8_t panelID = 15 - (sx1509b_gpio_read_port(SX1509B_PORTA)>>4);
	sx1509b_gpio_pullup_cfg(SX1509B_PORTA, 0xf0, SX1509B_OUTPUT_RESISTOR_DISABLE);
	return panelID;
}

uint8_t batteryPackReadCircuitID(){
	sx1509b_gpio_pullup_cfg(SX1509B_PORTB, 0xff, SX1509B_OUTPUT_RESISTOR_ENABLE);
	uint8_t portBReg = sx1509b_gpio_read_port(SX1509B_PORTB);
    #ifdef VERSION8
	uint8_t circuitID = (15 - (portBReg&0x0f)) + (15 - (portBReg>>4))*10;
    #else
	uint8_t circuitID = (15 - (portBReg>>4)) + (15 - (portBReg&0x0f))*10;
    #endif
	sx1509b_gpio_pullup_cfg(SX1509B_PORTB, 0xff, SX1509B_OUTPUT_RESISTOR_DISABLE);
	return circuitID;
}

void batteryPackLEDOn(uint8_t leds){
	uint8_t myLED = (leds&0x0e);
	sx1509b_gpio_clr_pin(SX1509B_PORTA, myLED);
}

void batteryPackLEDOff(uint8_t leds){
	uint8_t myLED = (leds&0x0e);
	sx1509b_gpio_set_pin(SX1509B_PORTA, myLED);
}

void batteryPackLEDToggle(uint8_t leds){
	uint8_t myLED = (leds&0x0e);
	uint8_t ledState = (sx1509b_gpio_read_port(SX1509B_PORTA) & 0x0e);
	sx1509b_gpio_write_port(SX1509B_PORTA, myLED, ~ledState);
}

uint8_t batteryPackIsUSBAttached(){
	uint8_t portAReg = sx1509b_gpio_read_port(SX1509B_PORTA);
	if (portAReg & 0x1)
		return 1;
	else
		return 0;
}

void batteryPackLEDDriverInit(){
	// select internal 2 MHz oscillator
	sx1509b_oscillator_source_select(SX1509B_OSCI_SOURCE_INT);
	// set fOSCOUT = 2 MHz / 2 ^ (9-1) = 7812.5 Hz
	sx1509b_oscillator_freq_divider(9);
	// set CLKx to 977 / 2 ^ (1-1) = 7812.5 Hz
	sx1509b_led_driver_freq_divider(1);
	// Enable LED driver for RGB LEDs
	sx1509b_led_driver_enable(SX1509B_PORTA, 0x0e, SX1509B_LED_DRIVER_ENABLE);
	// Set IOn, this register won't get reset by Reset pin nor software reset
	batteryPackLEDIntensitySet(BATTERY_PACK_LED_RED, 255);
	batteryPackLEDIntensitySet(BATTERY_PACK_LED_GREEN, 255);
	batteryPackLEDIntensitySet(BATTERY_PACK_LED_BLUE, 255);
}

void batteryPackLEDDriverDisable(){
	sx1509b_oscillator_source_select(SX1509B_OSCI_SOURCE_OFF);
	sx1509b_led_driver_enable(SX1509B_PORTA, 0x0e, SX1509B_LED_DRIVER_DISABLE);
}

uint8_t batteryPackLEDGetPin(uint8_t leds){
	uint8_t pin = 0;
	switch (leds){
		case BATTERY_PACK_LED_RED:
			pin = 3;
		break;
		case BATTERY_PACK_LED_GREEN:
			pin = 2;
		break;
		case BATTERY_PACK_LED_BLUE:
			pin = 1;
		break;
		default:
		break;
	}
	return pin;
}

// return 1 if success, 0 otherwise
uint8_t batteryPackLEDIntensityDecrease(uint8_t leds){
	uint8_t pin = batteryPackLEDGetPin(leds);
	uint8_t iOnVal = sx1509b_led_driver_get_ION(pin);
	if (iOnVal > 0){
		if (iOnVal >= 5)
			sx1509b_led_driver_set_ION(pin, iOnVal-5);
		else
			sx1509b_led_driver_set_ION(pin, 0);
		return 1;
	}
	return 0;
}

// return 1 if success, 0 otherwise
uint8_t batteryPackLEDIntensityIncrease(uint8_t leds){
	uint8_t pin = batteryPackLEDGetPin(leds);
	uint8_t iOnVal = sx1509b_led_driver_get_ION(pin);
	if (iOnVal < 255){
		if (iOnVal<250)
			sx1509b_led_driver_set_ION(pin, iOnVal+5);
		else
			sx1509b_led_driver_set_ION(pin, 255);
		return 1;
	}
	return 0;
}

void batteryPackLEDIntensitySet(uint8_t leds, uint8_t iOnVal){
	uint8_t pin = batteryPackLEDGetPin(leds);
	sx1509b_led_driver_set_ION(pin, iOnVal);
}

#ifdef VERSION12
#ifdef CHARGING_ENABLE
void batteryChargingEnable(uint8_t en){
    GPIO_SET_OUTPUT(LTC3200_GPIO_BASE, 0x1<<LTC3200_EN_PIN);
    if (en>0){
        GPIO_SET_PIN(LTC3200_GPIO_BASE, 0x1<<LTC3200_EN_PIN);
    } else {
        GPIO_CLR_PIN(LTC3200_GPIO_BASE, 0x1<<LTC3200_EN_PIN);
    }
}
#endif
#endif

uint16_t mysqrt(uint32_t n){
    uint32_t xn = n;
    uint8_t i;
    for (i=0; i<20; i++){
        xn = (xn + n/xn)/2;
    }
    return (uint16_t)xn;
}

// return average value
uint16_t getAverage(uint16_t* data, uint16_t length){
    uint16_t i;
    uint32_t sum = 0;
    for (i=0; i<length; i++)
        sum += data[i];
    return (uint16_t)(sum/length);
}

// return average value
uint16_t getAverage32(int* data, uint16_t length){
    uint16_t i;
    uint32_t sum = 0;
    for (i=0; i<length; i++)
        sum += data[i];
    return (uint16_t)(sum/length);
}

// calculate variance of phase offset array 
uint16_t getVariance(uint16_t* data, uint16_t length){
    uint16_t i;
    uint16_t avg = getAverage(data, length);
    int sqr;
    uint16_t variance = 0;
    for (i=0; i<length; i++){
        sqr = data[i] - avg;
        variance += (sqr*sqr);
    }
    variance /= length;
    return variance;
}

// dest[0] = src&0xff
// dest[1] = (src&0xff00)>>8
// dest[2] = (src&0xff0000)>>16
// dest[3] = (src&0xff000000)>>24
void packData(uint8_t* dest, int src, uint8_t len){
    uint8_t i;
    for (i=0; i<len; i++){
        dest[i] = (src&(0xff<<(i<<3)))>>(i<<3);
    }
}


