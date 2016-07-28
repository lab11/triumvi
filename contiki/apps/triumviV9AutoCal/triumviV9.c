
#include "contiki.h"
#include "cpu.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "dev/watchdog.h"
#include "dev/sys-ctrl.h"
#include "dev/gpio.h"
#include "dev/gptimer.h"
#include "dev/crypto.h"
#include "dev/ccm.h"
#include "dev/random.c"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include "adc.h"
#include "soc-adc.h"
#include "ioc.h"
#include "nvic.h"
#include "systick.h"
#include "i2c.h"
#include "cc2538-rf.h"
#include "fm25v02.h"
#include "triumvi.h"
#include "sx1509b.h"
#include "ad5274.h"
#include "dev/rom-util.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define I_TRANSFORM 48.34468

#define FIRSTSAMPLE_STATUSREG  0x0100
#define EXTERNALVOLT_STATUSREG 0x0080
#define BATTERYPACK_STATUSREG  0x0040
#define FRAMWRITE_STATUSREG    0x0008
#define POWERFACTOR_STATUSREG  0x0004

#define FLASH_BASE 0x200000
#define FLASH_SIZE 0x80000 //rom_util_get_flash_size()
#define FLASH_ERASE_SIZE 0x800

// Adjusted (DC removal) ADC sample thresholds
#define UPPERTHRESHOLD  430 // any value above this, gain is too large
#define LOWERTHRESHOLD  185 // max value below this, gain is too small

// number of samples per cycle
#define BUF_SIZE 120        // sample current only, 11-bit resolution, 1 cycles
#define BUF_SIZE2 225       // sample both current and voltage, 10-bit resolution, 2 cycles
#define MAX_BUF_SIZE 225    // max(BUF_SIZE, BUF_SIZE2)

// number of calibration cycles
#define CALIBRATION_CYCLES 512

// phase lock threshold
#define PHASE_VARIANCE_THRESHOLD 15

// INA gain indices
#define MAX_INA_GAIN_IDX 5
#define MIN_INA_GAIN_IDX 0

// voltage isolation filter offset
#define VOLTAGE_SAMPLE_OFFSET 19
// voltage scaling constant
#define VOLTAGE_SCALING 1092


// calibration coefficient
#define GAIN2_D0  35.2934
#define GAIN2_D1   0.9909

#define GAIN3_D0  46.6898
#define GAIN3_D1   0.9887

#define GAIN5_D0  49.4408
#define GAIN5_D1   0.9861

#define GAIN9_D0  44.9526
#define GAIN9_D1   0.9861

#define GAIN17_D0 61.7154
#define GAIN17_D1  0.9654

const uint8_t inaGainArr[6] = {1, 2, 3, 5, 9, 17};

typedef enum {
    MODE_CALIBRATION,
    MODE_NORMAL
} triumvi_mode_t;

typedef enum {
    GAIN_TOO_LOW,
    GAIN_OK,
    GAIN_TOO_HIGH
} gainSetting_t;

typedef enum {
    STATE_INIT,
    STATE_WAITING_VOLTAGE_STABLE,
    STATE_WAITING_COMPARATOR_STABLE,
    STATE_BATTERYPACK_LEDBLINK,
    STATE_TRIUMVI_LEDBLINK
} triumvi_state_t;

typedef enum {
    STATE_CALIBRATION_IN_PROGRESS,
    STATE_CALIBRATION_COMPLETED
} triumvi_state_calibration_t;

volatile static triumvi_state_t myState;

/* global variables */

const uint32_t flash_addr = FLASH_BASE - 2*FLASH_ERASE_SIZE + FLASH_SIZE;
volatile uint32_t flash_data;

uint32_t timerVal[2];
uint16_t currentADCVal[MAX_BUF_SIZE];
int adjustedCurrSamples[MAX_BUF_SIZE];
uint16_t voltADCVal[BUF_SIZE2]; 
int adjustedVoltSamples[BUF_SIZE2];

volatile uint16_t phaseOffset;
volatile uint16_t dcOffset;


volatile static triumvi_mode_t operation_mode;

static struct etimer calibration_timer;
static struct rtimer myRTimer;
volatile uint8_t rTimerExpired;
volatile uint8_t referenceInt;
volatile uint8_t backOffTime;
volatile uint8_t backOffHistory;
volatile uint8_t inaGainIdx;
volatile uint8_t allInitsAreReadyInt;

/* End of global variables */

#include "sineTable.h"

/* function prototypes */

/* functions for calibration process only */
// return product of voltage and current for a cycle
int cycleProduct(uint16_t* adcSamples, uint16_t offset, uint16_t currentRef);
// return the phase has maximum correlation
uint16_t phaseMatchFilter(uint16_t* adcSamples, uint16_t* currentAVG);
// return variance of phase offsets 
uint16_t getVariance(uint16_t* data, uint16_t length);

// return mean(data)
uint16_t getAverage(uint16_t* data, uint16_t length);
// gain control, adjust gain if necessary
gainSetting_t gainCtrl(uint16_t* adcSamples, uint8_t externalVolt);
// initialize GPIO, peripherals
void meterInit();
// samples ADC, and computes power
int sampleAndCalculate(uint16_t triumviStatusReg);
// samples current ADC, 11-bit
void sampleCurrentWaveform();
// samples current, voltage ADCs, 10-bit each
void sampleCurrentVoltageWaveform();
// power gate to current sensing, disable POT
void disablePOT();
// encrypt data using AES, and wirelessly transmit packet
void encryptAndTransmit(uint16_t triumviStatusReg, int avgPower, float pf, 
        uint16_t VRMS, uint16_t IRMS, uint8_t* myNonce, uint32_t nonceCounter);
// split a uint32_t into 4 uint8_t
void packData(uint8_t* dest, int reading);
// split a uint16_t into 2 uint8_t
void packDataHalf(uint8_t* dest, uint16_t reading);

int currentDataTransform(int currentReading, uint8_t externalVolt);
int voltDataTransform(uint16_t voltReading, uint16_t voltReference);
// functions do not use in data dump mode
static void disable_all_ioc_override();


uint16_t currentRMS(uint16_t triumviStatusReg);
uint16_t mysqrt(uint32_t n);
uint16_t voltageRMS(uint16_t triumviStatusReg);

// ISRs
static void rtimerEvent(struct rtimer *t, void *ptr);
static void referenceIntCallBack(uint8_t port, uint8_t pin);
static void unitReadyCallBack(uint8_t port, uint8_t pin);

/* End of function prototypes */

/*---------------------------------------------------------------------------*/
PROCESS(startupProcess, "Startup");
PROCESS(calibrationProcess, "Calibration");
PROCESS(triumviProcess, "Triumvi");
AUTOSTART_PROCESSES(&startupProcess);
/*---------------------------------------------------------------------------*/

// Startup process, check flash
PROCESS_THREAD(startupProcess, ev, data) {
    PROCESS_BEGIN();

    random_init(0);
    CC2538_RF_CSP_ISRFOFF();
    fm25v02_sleep();
    disableSPI();

    #ifdef ERASE_FRAM
    fm25v02_eraseAll();
    #endif

    // Initialize peripherals
    meterInit();
    
    // Erase FRAM
    if (isButtonPressed()){
        triumviFramPtrClear();
        triumviLEDON();
    }
    
    // Disable sensing frontend
    disablePOT();
    meterSenseConfig(VOLTAGE, SENSE_DISABLE);
    meterVoltageComparator(SENSE_DISABLE);

    // read internal flash
    flash_data = REG(flash_addr);

    // non-calibrated device
    if (flash_data==0xffffffff){
        process_start(&calibrationProcess, NULL);
        operation_mode = MODE_CALIBRATION;
    }
    // calibrated device, load phase offset and dc offset
    else{
        phaseOffset = (flash_data & 0xffff);
        dcOffset = ((flash_data & 0xffff0000)>>16);
        process_start(&triumviProcess, NULL);
        operation_mode = MODE_NORMAL;
    }
    PROCESS_END();
}

PROCESS_THREAD(calibrationProcess, ev, data) {
    PROCESS_BEGIN();

    static uint16_t cycleCnt = 0;
    uint16_t i;
    gainSetting_t gainSetting;
    static triumvi_state_calibration_t calibration_state = STATE_CALIBRATION_IN_PROGRESS;
    #ifdef DATADUMP
    uint8_t inaGain;
    #else
    uint16_t currentRef;
    uint16_t calculatedPhase;
    static uint16_t phaseOffset_array[CALIBRATION_CYCLES];
    static uint32_t dcOffset_accum = 0;
    static uint32_t variance = 0;
    uint32_t tmp = 0;
    #endif
    static uint32_t timerExp, currentTime;

    rom_util_page_erase(flash_addr, FLASH_ERASE_SIZE);
    triumviLEDON();
    etimer_set(&calibration_timer, CLOCK_SECOND*10);

    // enable LDO, release power gating
    meterSenseVREn(SENSE_ENABLE);
    meterSenseConfig(VOLTAGE, SENSE_ENABLE);
    meterSenseConfig(CURRENT, SENSE_ENABLE);

    while (1){
        PROCESS_YIELD();
        switch (calibration_state){
            case STATE_CALIBRATION_IN_PROGRESS:
                if (etimer_expired(&calibration_timer)){
                    triumviLEDON();

                    // Enable digital pot
                    ad5274_init();
                    ad5274_ctrl_reg_write(AD5274_REG_RDAC_RP);
                    setINAGain(inaGainArr[inaGainIdx]);
                    i2c_disable(AD527X_SDA_GPIO_NUM, AD527X_SDA_GPIO_PIN, AD527X_SCL_GPIO_NUM, AD527X_SCL_GPIO_PIN); 

                    // Enable comparator interrupt
                    ungate_gpt(GPTIMER_1);
                    REG(SYSTICK_STCTRL) &= (~SYSTICK_STCTRL_INTEN);
                    timerExp = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A) - 320000;
                    GPIO_DETECT_RISING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
                    meterVoltageComparator(SENSE_ENABLE);

                    // waiting for comparator interrupt
                    do {
                        currentTime = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
                    }
                    while ((currentTime > timerExp) && (referenceInt==0));

                    // time out, retry
                    if (currentTime <= timerExp){
                        gainSetting = GAIN_TOO_HIGH;
                    }
                    // captured interrupt
                    else{
                        sampleCurrentWaveform();
                        referenceInt = 0;

                        // resume systick
                        REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
                        gate_gpt(GPTIMER_1);

                        // check INA gain setting
                        gainSetting = gainCtrl(currentADCVal, 0x0);
                    }

                    if (gainSetting == GAIN_OK){
                        #ifdef DATADUMP
                        // print data through UART
                        inaGain = inaGainArr[inaGainIdx];
                        printf("ADC reference: %u\r\n", getAverage(currentADCVal, BUF_SIZE));
                        printf("Time difference: %lu\r\n", (timerVal[0]-timerVal[1]));
                        printf("INA Gain: %u\r\n", inaGain);
                        for (i=0; i<BUF_SIZE; i+=1){
                            printf("Current reading: %d\r\n", currentADCVal[i]);
                        }
                        #else
                        // perform phase, dc offset calculation
                        calculatedPhase = phaseMatchFilter(currentADCVal, &currentRef);
                        phaseOffset_array[cycleCnt] = calculatedPhase;
                        dcOffset_accum += currentRef;
                        #endif
                        cycleCnt += 1;
                        
                        if (cycleCnt < CALIBRATION_CYCLES){
                            triumviLEDOFF();
                            etimer_set(&calibration_timer, CLOCK_SECOND*0.1);
                        }
                        else{
                            #ifdef DATADUMP
                            etimer_set(&calibration_timer, CLOCK_SECOND*1);
                            #else
                            meterSenseVREn(SENSE_DISABLE);
                            meterSenseConfig(VOLTAGE, SENSE_DISABLE);
                            meterSenseConfig(CURRENT, SENSE_DISABLE);

                            variance = getVariance(phaseOffset_array, CALIBRATION_CYCLES);
                            // cannot lock phase, check if the phase across 360
                            if (variance > PHASE_VARIANCE_THRESHOLD){
                                #ifdef DEBUG_ON
                                for (i=0; i<CALIBRATION_CYCLES; i++){
                                    printf("phase samples: %u\r\n", phaseOffset_array[i]);
                                }
                                printf("variance: %u\r\n", variance);
                                #endif
                                for (i=0; i<CALIBRATION_CYCLES; i++){
                                    if (phaseOffset_array[i] < 180)
                                        phaseOffset_array[i] += 360;
                                }
                                variance = getVariance(phaseOffset_array, CALIBRATION_CYCLES);
                                #ifdef DEBUG_ON
                                printf("new variance: %u\r\n", variance);
                                #endif
                                if (variance > PHASE_VARIANCE_THRESHOLD){
                                    if (batteryPackIsAttached()){
                                        batteryPackVoltageEn(SENSE_ENABLE);
                                        batteryPackInit();
                                        batteryPackLEDDriverInit();
                                        batteryPackLEDOn(BATTERY_PACK_LED_RED);
                                        i2c_disable(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN); 
                                    }
                                    while(1){}
                                }
                            }
                            phaseOffset = getAverage(phaseOffset_array, CALIBRATION_CYCLES);
                            if (phaseOffset >= 360)
                                phaseOffset -= 360;
                            dcOffset = (dcOffset_accum/CALIBRATION_CYCLES);
                            
                            #ifdef DEBUG_ON
                            printf("phase offset: %u\r\n", phaseOffset);
                            printf("dc offset: %u\r\n", dcOffset);
                            printf("variance: %u\r\n", variance);
                            //for (i=0; i<CALIBRATION_CYCLES; i++){
                            //    printf("phase Offset %u: %u\r\n", i, phaseOffset_array[i]);
                            //}
                            #endif

                            // write to flash
                            tmp = (dcOffset<<16) | phaseOffset;
                            rom_util_program_flash(&tmp, flash_addr, 4);

                            if (batteryPackIsAttached()){
                                batteryPackVoltageEn(SENSE_ENABLE);
                                batteryPackInit();
                                batteryPackLEDDriverInit();
                                batteryPackLEDOn(BATTERY_PACK_LED_GREEN);
                                i2c_disable(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN); 
                            }

                            // advances state
                            calibration_state = STATE_CALIBRATION_COMPLETED;
                            etimer_set(&calibration_timer, CLOCK_SECOND*3);
                            #endif // end of DATADUMP
                        }
                    }
                    else{
                        etimer_set(&calibration_timer, CLOCK_SECOND*1);
                    }
                }
            break;

            case STATE_CALIBRATION_COMPLETED:
                if (etimer_expired(&calibration_timer)){
                    operation_mode = MODE_NORMAL;
                    process_start(&triumviProcess, NULL);
                    triumviLEDOFF();
                    if (batteryPackIsAttached()){
                        batteryPackLEDOff(BATTERY_PACK_LED_GREEN);
                        batteryPackVoltageEn(SENSE_DISABLE);
                    }
                    break;
                }
            break;

            default:
            break;
        }
    }

    PROCESS_END();
}

PROCESS_THREAD(triumviProcess, ev, data) {
    PROCESS_BEGIN();
    // packet preprocessing
    static uint8_t extAddr[8];
    NETSTACK_RADIO.get_object(RADIO_PARAM_64BIT_ADDR, extAddr, 8);

    // AES Preprocessing
    static uint8_t aesKey[] = AES_KEY;
    static uint8_t myNonce[13] = {0};
    memcpy(myNonce, extAddr, 8);
    static uint32_t nonceCounter = 0;
    aes_load_keys(aesKey, AES_KEY_STORE_SIZE_KEY_SIZE_128, 1, 0);

	static int avgPower;
    uint8_t rdy;

    uint16_t triumviStatusReg;

    float pf;
    static uint16_t VRMS, IRMS;
    uint16_t inaGain;

	// Keep a counter of the number of samples we have taken
	// since we have been on. This will obviously get reset if we lose power.
	// We would have to store this counter in FRAM for it to be persistent.
	static uint32_t sampleCount = 0;
    
    static uint32_t timerExp, currentTime;

    unitReady();
    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND, 1, &rtimerEvent, NULL);
    myState = STATE_INIT;

    while(1){
        PROCESS_YIELD();
        switch (myState){
            // initialization state, check READYn is low before moving forward
            case STATE_INIT:
                rdy = 0;
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                    if (allUnitsReady())
                        rdy = 1;
                    // not all units are ready, go back to sleep
                    else{
                        GPIO_DETECT_FALLING(TRIUMVI_RDYn_IN_GPIO_BASE, 0x1<<TRIUMVI_RDYn_IN_GPIO_PIN);
                        GPIO_ENABLE_INTERRUPT(TRIUMVI_RDYn_IN_GPIO_BASE, 0x1<<TRIUMVI_RDYn_IN_GPIO_PIN);
                        nvic_interrupt_enable(TRIUMVI_RDYn_IN_INT_NVIC_PORT);
                    }
                }
                else if (allInitsAreReadyInt==1){
                    allInitsAreReadyInt = 0;
                    rdy = 1;
                }
                if (rdy==1){
                    meterSenseVREn(SENSE_ENABLE);
                    meterSenseConfig(VOLTAGE, SENSE_ENABLE);
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.4, 1, &rtimerEvent, NULL);
                    myState = STATE_WAITING_VOLTAGE_STABLE;
                    // consecutive 4 samples, decreases sampling interval
                    if (((backOffHistory&0x0f)==0x0f)&&(backOffTime>2))
                        backOffTime = (backOffTime>>1);
                    backOffHistory = (backOffHistory<<1) | 0x01;
                }
            break;

            // voltage is sattled, enable current measurement
            case STATE_WAITING_VOLTAGE_STABLE:
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                    meterSenseConfig(CURRENT, SENSE_ENABLE);
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.3, 1, &rtimerEvent, NULL);
                    myState = STATE_WAITING_COMPARATOR_STABLE;
                }
            break;

            case STATE_WAITING_COMPARATOR_STABLE:
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                    // Layout of Status Reg:
                    // Bit 9: First sample
                    // Bit 8: External Volt Selected
                    // Bit 7: Battery Pack attached
                    // Bit 6:5: Three Phase Slave Selected
                    // 00 --> non, 01 --> Master, 10 --> slave 1, 11 --> slave2
                    // Bit 4: FRAM WRITE Enabled
                    // Note: bit 6 & 7 cannot be set simultaneously
                    triumviStatusReg = 0x0000;
                    if (externalVoltSel())
                        triumviStatusReg |= EXTERNALVOLT_STATUSREG;
                    if (sampleCount == 0)
                        triumviStatusReg |= FIRSTSAMPLE_STATUSREG;
                    if (vcapLoopBack())
                        triumviStatusReg |= ((THREEPHASE_ID & 0x3)<<4);
                    #ifdef FRAM_WRITE
                    triumviStatusReg |= FRAMWRITE_STATUSREG;
                    #endif

                    #ifdef POWERFACTOR_EN
                    triumviStatusReg |= POWERFACTOR_STATUSREG;
                    #endif

                    // Check if configuration board is attached
                    if (batteryPackIsAttached())
                        triumviStatusReg |= BATTERYPACK_STATUSREG;

                    // Enable digital pot
                    ad5274_init();
                    ad5274_ctrl_reg_write(AD5274_REG_RDAC_RP);
                    setINAGain(inaGainArr[inaGainIdx]);

                    // Enable comparator interrupt
                    ungate_gpt(GPTIMER_1);
                    REG(SYSTICK_STCTRL) &= (~SYSTICK_STCTRL_INTEN);
                    timerExp = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A) - 320000;
                    GPIO_DETECT_RISING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
                    meterVoltageComparator(SENSE_ENABLE);

                    // waiting for comparator interrupt
                    do {
                        currentTime = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
                    }
                    while ((currentTime > timerExp) && (referenceInt==0));

                    // time out, retry
                    if (currentTime <= timerExp){
                        meterSenseConfig(VOLTAGE, SENSE_DISABLE);
                        disablePOT();
                        REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
                        gate_gpt(GPTIMER_1);
                        avgPower = -1;
                    }
                    // captured interrupt
                    else{
                        // voltage sensing and comparator interrupt can be disabled first (disableALL())
                        avgPower = sampleAndCalculate(triumviStatusReg);
                        referenceInt = 0;
                    }


                    if (avgPower>=0){
                        sampleCount++;
                        inaGain = inaGainArr[inaGainIdx];
                        if (triumviStatusReg & POWERFACTOR_STATUSREG){
                            IRMS = currentRMS(triumviStatusReg);
                            VRMS = voltageRMS(triumviStatusReg);
                            if ((IRMS==0) || (avgPower==0))
                                pf = 0;
                            else
                                pf = (float)avgPower/(VRMS*IRMS);
                            if (pf > 1)
                                pf = 1;
                            switch (inaGain){
                                case 2:
                                    IRMS = (int)((float)IRMS*GAIN2_D1 + GAIN2_D0);
                                break;
                                case 3:
                                    IRMS = (int)((float)IRMS*GAIN3_D1 + GAIN3_D0);
                                break;
                                case 5:
                                    IRMS = (int)((float)IRMS*GAIN5_D1 + GAIN5_D0);
                                break;
                                case 9:
                                    IRMS = (int)((float)IRMS*GAIN9_D1 + GAIN9_D0);
                                break;
                                case 17:
                                    IRMS = (int)((float)IRMS*GAIN17_D1 + GAIN17_D0);
                                break;
                            }
                        }
                        #ifdef DATADUMP2
                        uint8_t i;
                        printf("ADC reference: %u\r\n", getAverage(currentADCVal, BUF_SIZE));
                        printf("Time difference: %lu\r\n", (timerVal[0]-timerVal[1]));
                        printf("INA Gain: %u\r\n", inaGain);
                        for (i=0; i<BUF_SIZE; i+=1)
                            printf("Current reading: %d\r\n", currentADCVal[i]);
                        //printf("INA Gain: %u\r\n", inaGain);
                        //printf("IRMS: %u\r\n", IRMS);
                        //printf("Average Power: %u\r\n", avgPower);
                        // Write data into FRAM
                        #elif defined(FRAM_WRITE)
                        writeFRAM((uint16_t)(avgPower/1000), &rtctime);
                        #else
                        
                        uint16_t rand0, rand1;
                        rand0 = random_rand();
                        rand1 = random_rand();
                        nonceCounter = (rand0<<16) | rand1;
                        
                        // Battery pack attached, turn it on ans samples switches
                        if (triumviStatusReg & BATTERYPACK_STATUSREG){
                            batteryPackVoltageEn(SENSE_ENABLE);
                            sx1509b_init();
                            sx1509b_high_voltage_input_enable(SX1509B_PORTA, 0x1, SX1509B_HIGH_INPUT_ENABLE);
                        }
                        encryptAndTransmit(triumviStatusReg, avgPower, pf, (inaGain<<8)|VRMS, IRMS, myNonce, nonceCounter);
                        #endif
                        // First sample, blinks battery pack blue LED
                        if (batteryPackIsUSBAttached() &&
                            (triumviStatusReg & FIRSTSAMPLE_STATUSREG)){
                            batteryPackVoltageEn(SENSE_ENABLE);
                            batteryPackInit();
                            batteryPackLEDDriverInit();
                            batteryPackLEDOn(BATTERY_PACK_LED_BLUE);
                            myState = STATE_BATTERYPACK_LEDBLINK;
                        }
                        else{
                            triumviLEDON();
                            myState = STATE_TRIUMVI_LEDBLINK;
                        }
                        rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.1, 1, &rtimerEvent, NULL);
                    }
                    else{
                        if (triumviStatusReg & BATTERYPACK_STATUSREG){
                            batteryPackVoltageEn(SENSE_DISABLE);
                            i2c_disable(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, 
                                        I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN); 
                        }
                        rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.1, 1, &rtimerEvent, NULL);
                        myState = STATE_TRIUMVI_LEDBLINK;
                    }
                }
            break;

            case STATE_BATTERYPACK_LEDBLINK:
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                    batteryPackVoltageEn(SENSE_DISABLE);
                    i2c_disable(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, 
                                I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN); 
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
                    myState = STATE_INIT;
                }
            break;

            case STATE_TRIUMVI_LEDBLINK:
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                    triumviLEDOFF();
                    #ifdef DATADUMP2
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.1, 1, &rtimerEvent, NULL);
                    #else
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
                    #endif
                    myState = STATE_INIT;
                }
            break;

            default:
            break;
        }
    }
    PROCESS_END();
}


static void rtimerEvent(struct rtimer *t, void *ptr){
    rTimerExpired = 1;
    process_poll(&triumviProcess);
}

// return phase offset has max product, muct be in calibration mode
uint16_t phaseMatchFilter(uint16_t* adcSamples, uint16_t* currentAVG){
    uint16_t i;
    int prod[360];
    int maxVal = 0;
    uint16_t maxVal_phaseOffset = 0;
    uint16_t currentRef;
    currentRef = getAverage(adcSamples, BUF_SIZE);
    for (i=0; i<360; i++){
        prod[i] = cycleProduct(adcSamples, i, currentRef);
        if (prod[i] > maxVal){
            maxVal = prod[i];
            maxVal_phaseOffset = i;
        }
    }
    *currentAVG = currentRef;
    return maxVal_phaseOffset;
}

int cycleProduct(uint16_t* adcSamples, uint16_t offset, uint16_t currentRef){
    uint16_t i;
    uint16_t tmp;
    uint16_t length = BUF_SIZE;
    int product = 0;
    for (i=0; i<length; i++){
        tmp = i*3 + offset;
        if (tmp>=360)
            tmp -= 360;
        product += (adcSamples[i] - currentRef)*stdSineTable[tmp];
    }
    return product;
}

// return average value
uint16_t getAverage(uint16_t* data, uint16_t length){
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

// Fine tune to 2.99 degree / sample
void sampleCurrentWaveform(){
	uint16_t sampleCnt = 0;
	uint16_t temp;
    timerVal[0] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
	while (sampleCnt < BUF_SIZE){
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
		temp = adc_get(I_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_512);
		currentADCVal[sampleCnt] = ((temp>>4)>2047)? 0 : (temp>>4);
		sampleCnt++;
	}
    timerVal[1] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
}

void meterInit(){

    // GPIO default Input
    // Set all un-used pins to output and clear output
    #if defined(DATADUMP) || defined(DATADUMP2) || defined(DEBUG_ON)
    GPIO_SET_OUTPUT(GPIO_A_BASE, 0x40);
    GPIO_CLR_PIN(GPIO_A_BASE, 0x00);
    GPIO_SET_PIN(TRIUMVI_READYn_OUT_GPIO_BASE, 0x1<<TRIUMVI_READYn_OUT_GPIO_PIN); // not ready yet
    #else
    // Port A
    GPIO_SET_OUTPUT(GPIO_A_BASE, 0x47);
    GPIO_CLR_PIN(GPIO_A_BASE, 0x07);
    GPIO_SET_PIN(TRIUMVI_READYn_OUT_GPIO_BASE, 0x1<<TRIUMVI_READYn_OUT_GPIO_PIN); // not ready yet
    #endif

    // Port B
    GPIO_SET_OUTPUT(GPIO_B_BASE, 0x06);
    GPIO_CLR_PIN(GPIO_B_BASE, 0x06);
    GPIO_SET_INPUT(GPIO_B_BASE, 
        (0x1<<TRIUMVI_RDYn_IN_GPIO_PIN) | 
        (0x1<<CONFIG_PWR_LOOPBAK_GPIO_PIN) |
        (0x1<<EXT_VOLT_IN_SEL_GPIO_PIN)); 

    // Port C
    GPIO_SET_OUTPUT(GPIO_C_BASE, 0xeb);
    GPIO_CLR_PIN(GPIO_C_BASE, 0xeb);

    // Port D
    GPIO_SET_OUTPUT(GPIO_D_BASE, 0x1f);
    GPIO_SET_PIN(GPIO_D_BASE, 0x0f);
    GPIO_CLR_PIN(GPIO_D_BASE, 0x10);
    GPIO_SET_INPUT(CONFIG_VCAP_LOOPBACK_GPIO_BASE, (0x1<<CONFIG_VCAP_LOOPBACK_GPIO_PIN));
    #if !defined(DATADUMP) && !defined(DATADUMP2) && !defined(DEBUG_ON)
    // disable all pull-up resistors
    disable_all_ioc_override();
    #endif


	// ADC for current sense
	adc_init();
	ioc_set_over(I_ADC_GPIO_NUM, I_ADC_GPIO_PIN, IOC_OVERRIDE_ANA);
	ioc_set_over(V_REF_ADC_GPIO_NUM, V_REF_ADC_GPIO_PIN, IOC_OVERRIDE_ANA);

	// External voltage waveform inputs
	ioc_set_over(EXT_VOLT_IN_GPIO_NUM, EXT_VOLT_IN_GPIO_PIN, IOC_OVERRIDE_ANA);
	ioc_set_over(EXT_VOLT_IN_SEL_GPIO_NUM, EXT_VOLT_IN_SEL_GPIO_PIN, IOC_OVERRIDE_PDE);

	// Enable interrupt on rising edge
	ioc_set_over(V_REF_CROSS_INT_GPIO_NUM, V_REF_CROSS_INT_GPIO_PIN, IOC_OVERRIDE_DIS);
	GPIO_DETECT_EDGE(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	GPIO_TRIGGER_SINGLE_EDGE(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	// interrupt
	gpio_register_callback(referenceIntCallBack, V_REF_CROSS_INT_GPIO_NUM, V_REF_CROSS_INT_GPIO_PIN);
	GPIO_DISABLE_INTERRUPT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	nvic_interrupt_disable(V_REF_CROSS_INT_NVIC_PORT);

    // READYn_IN, enable pull down resistor
	ioc_set_over(TRIUMVI_RDYn_IN_GPIO_NUM, TRIUMVI_RDYn_IN_GPIO_PIN, IOC_OVERRIDE_PDE);
	GPIO_DETECT_EDGE(TRIUMVI_RDYn_IN_GPIO_BASE, 0x1<<TRIUMVI_RDYn_IN_GPIO_PIN);
	GPIO_TRIGGER_SINGLE_EDGE(TRIUMVI_RDYn_IN_GPIO_BASE, 0x1<<TRIUMVI_RDYn_IN_GPIO_PIN);
	// interrupt
	gpio_register_callback(unitReadyCallBack, TRIUMVI_RDYn_IN_GPIO_NUM, TRIUMVI_RDYn_IN_GPIO_PIN);
	GPIO_DISABLE_INTERRUPT(TRIUMVI_RDYn_IN_GPIO_BASE, 0x1<<TRIUMVI_RDYn_IN_GPIO_PIN);
	nvic_interrupt_disable(TRIUMVI_RDYn_IN_INT_NVIC_PORT);


	// timer1, used for counting samples
	ungate_gpt(GPTIMER_1);
	gpt_set_mode(GPTIMER_1, GPTIMER_SUBTIMER_A, GPTIMER_TAMR_TAMR_PERIODIC);
	gpt_set_count_dir(GPTIMER_1, GPTIMER_SUBTIMER_A, GPTIMER_TnMR_TnCDIR_COUNT_DOWN);
	gpt_set_interval_value(GPTIMER_1, GPTIMER_SUBTIMER_A, 0xffffffff);
	gpt_enable_event(GPTIMER_1, GPTIMER_SUBTIMER_A);
	gate_gpt(GPTIMER_1);

	rTimerExpired = 0;
	referenceInt = 0;
    allInitsAreReadyInt = 0;
    //inaGainIdx = MAX_INA_GAIN_IDX;
    inaGainIdx = 3; // use larger gain setting

	backOffTime = 4;
	backOffHistory = 0;

}

// this function check if the ADC samples are within a proper range
gainSetting_t gainCtrl(uint16_t* adcSamples, uint8_t externalVolt){
    uint16_t i;
    uint16_t upperThreshold = UPPERTHRESHOLD;
    uint16_t lowerThreshold = LOWERTHRESHOLD;
    uint16_t length = BUF_SIZE;
    uint16_t currentRef;
    int currentCal;
    int maxVal = 0;
    gainSetting_t res = GAIN_OK;
    
    // in calibration mode, use ADC sample as reference
    if (operation_mode==MODE_CALIBRATION){
        currentRef = adc_get(V_REF_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_512);
        currentRef = ((currentRef>>4)>2047)? 0 : (currentRef>>4);
    }
    // using hardcoded value as reference
    else{
        if (externalVolt){
            #ifdef AVG_VREF
            currentRef = getAverage(adcSamples, BUF_SIZE2);
            #else
            currentRef = (dcOffset>>1);
            #endif
            upperThreshold = (UPPERTHRESHOLD>>1);
            lowerThreshold= (LOWERTHRESHOLD>>1);
            length = BUF_SIZE2;
        }
        else{
            #ifdef AVG_VREF
            currentRef = getAverage(adcSamples, BUF_SIZE);
            #else
            currentRef = dcOffset;
            #endif
        }
    }

    // loop the entire samples, substract offset and update max ADC value
    for (i=0; i<length; i++){
        currentCal = adcSamples[i] - currentRef;
        if ((currentCal>upperThreshold)&&(inaGainIdx>MIN_INA_GAIN_IDX)){
            res = GAIN_TOO_HIGH;
            inaGainIdx -= 1;
            setINAGain(inaGainArr[inaGainIdx]);
            return res;
        }
        if (currentCal > maxVal){
            maxVal = currentCal;
        }
        adjustedCurrSamples[i] = currentCal;
    }
    
    if ((maxVal < lowerThreshold) && (inaGainIdx<MAX_INA_GAIN_IDX)){
        res = GAIN_TOO_LOW;
        // AD5272 is in shutdown mode, turn it on
        if (inaGainIdx == 0)
            ad5274_shutdown(0x0);
        inaGainIdx += 1;
        setINAGain(inaGainArr[inaGainIdx]);
    }

    return res;
}

void encryptAndTransmit(uint16_t triumviStatusReg, int avgPower, float pf, 
        uint16_t VRMS, uint16_t IRMS, uint8_t* myNonce, uint32_t nonceCounter){
	// 1 byte Identifier, 4 bytes nonce, 5~13 bytes payload, 4 byte MIC
	static uint8_t packetData[22];
	// 4 bytes reading, 1 byte status reg, 
    // (1 bytes panel ID, 1 bytes circuit ID, 2 bytes PF, 2 bytes VRMS, 2 bytes IRMS optional)
	static uint8_t readingBuf[13];
	uint8_t* aData = myNonce;
	uint8_t* pData = readingBuf;
	uint8_t myMic[8] = {0x0};
	uint8_t myPDATA_LEN = PDATA_LEN;
	uint8_t packetLen = 14; // minimum length, 1 byte ID, 4 bytes nonce, 5 bytes payload, 4 byte MIC
	uint16_t randBackOff;

	packetData[0] = TRIUMVI_PKT_IDENTIFIER;
	if (triumviStatusReg & BATTERYPACK_STATUSREG){
		readingBuf[myPDATA_LEN] = batteryPackReadPanelID();
		readingBuf[myPDATA_LEN+1] = batteryPackReadCircuitID();
		myPDATA_LEN += 2;
		packetLen += 2;
        batteryPackVoltageEn(SENSE_DISABLE);
        i2c_disable(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, 
                    I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN); 
	}
	readingBuf[4] = triumviStatusReg;
	packData(&packetData[1], nonceCounter);
	packData(&myNonce[9], nonceCounter);
	packData(readingBuf, avgPower);
    if (triumviStatusReg & POWERFACTOR_STATUSREG){
        packDataHalf(&readingBuf[myPDATA_LEN], (uint16_t)(pf*1000));
        packDataHalf(&readingBuf[myPDATA_LEN+2], VRMS);
        packDataHalf(&readingBuf[myPDATA_LEN+4], IRMS);
        myPDATA_LEN += 6;
        packetLen += 6;
    }

	ccm_auth_encrypt_start(LEN_LEN, 0, myNonce, aData, ADATA_LEN,
		pData, myPDATA_LEN, MIC_LEN, NULL);
	while(ccm_auth_encrypt_check_status()!=AES_CTRL_INT_STAT_RESULT_AV){}
	ccm_auth_encrypt_get_result(myMic, MIC_LEN);
	memcpy(&packetData[5], readingBuf, myPDATA_LEN);
	memcpy(&packetData[5+myPDATA_LEN], myMic, MIC_LEN);
	packetbuf_copyfrom(packetData, packetLen);

    // Random delay before transmits a packet
	randBackOff = random_rand();
	clock_delay_usec(randBackOff);
	clock_delay_usec(randBackOff);

    //REG(RFCORE_XREG_TXPOWER) = 0xb6;    // set TX power to 0 dBm
	cc2538_on_and_transmit();
	CC2538_RF_CSP_ISRFOFF();
}

int sampleAndCalculate(uint16_t triumviStatusReg){
    int tempPower;
    int tempPower2 = 0;
    uint16_t i, j, k;
    uint8_t numOfCycles = 8;
    uint8_t numOfBitShift = 3; // log2(numOfCycles)
    uint16_t voltRef;
    int energyCal = 0;
    gainSetting_t gainSetting;

    if (triumviStatusReg & EXTERNALVOLT_STATUSREG){
        for (i=0; i<numOfCycles; i++){
            sampleCurrentVoltageWaveform();
            gainSetting = gainCtrl(currentADCVal, 0x1);
            if (gainSetting == GAIN_OK){
                voltRef = getAverage(voltADCVal, BUF_SIZE2);
                energyCal = 0;
                for (j=0; j<BUF_SIZE2; j++){
                    k = ((j + VOLTAGE_SAMPLE_OFFSET)>=BUF_SIZE2)? 
                        (j+VOLTAGE_SAMPLE_OFFSET-BUF_SIZE2) : 
                        j+VOLTAGE_SAMPLE_OFFSET;
                    adjustedCurrSamples[j] = currentDataTransform(adjustedCurrSamples[j], 0x1);
                    adjustedVoltSamples[k] = voltDataTransform(voltADCVal[k], voltRef);
                    energyCal += (adjustedCurrSamples[j]*adjustedVoltSamples[k])/1000;
                }
                tempPower = (energyCal/BUF_SIZE2); // unit is mW
                // Fix phase oppsite down
                if (tempPower < 0)
                    tempPower = -1*tempPower;
                tempPower2 += tempPower;
            }
            else{
                tempPower2 = -1;
                break;
            }
        }
        REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
        gate_gpt(GPTIMER_1);
        meterSenseConfig(VOLTAGE, SENSE_DISABLE);
        disablePOT();
        if (tempPower2 < 0)
            return -1;
        else
            return (tempPower2>>numOfBitShift);
    }
    else{
        sampleCurrentWaveform();
        meterSenseConfig(VOLTAGE, SENSE_DISABLE);
        gainSetting = gainCtrl(currentADCVal, 0x0);
        disablePOT();
        REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
        gate_gpt(GPTIMER_1);
        if (gainSetting==GAIN_OK){
            for (i=0; i<BUF_SIZE; i++){
                j = ((i*3+phaseOffset) >= 360)? i*3+phaseOffset-360 : i*3+phaseOffset;
                adjustedCurrSamples[i] = currentDataTransform(adjustedCurrSamples[i], 0x0);
                energyCal += (adjustedCurrSamples[i]*stdSineTable[j]);
            }
            tempPower = (energyCal/BUF_SIZE); // unit is mW
            // Fix phase oppsite down
            if (tempPower < 0)
                tempPower = -1*tempPower;
            return tempPower;
        }
    }
    return -1;
}


void disablePOT(){
    meterSenseConfig(CURRENT, SENSE_DISABLE);
    meterSenseVREn(SENSE_DISABLE);
    i2c_disable(AD527X_SDA_GPIO_NUM, AD527X_SDA_GPIO_PIN, 
                AD527X_SCL_GPIO_NUM, AD527X_SCL_GPIO_PIN); 
}

int currentDataTransform(int currentReading, uint8_t externalVolt){
    uint8_t inaGain = inaGainArr[inaGainIdx];
    float tmp = (externalVolt)? currentReading*I_TRANSFORM*2/inaGain : currentReading*I_TRANSFORM/inaGain;
    return (int)tmp;
}

int voltDataTransform(uint16_t voltReading, uint16_t voltReference){
    return (voltReading - voltReference)*VOLTAGE_SCALING;
}

void packData(uint8_t* dest, int reading){
	uint8_t i;
	for (i=0; i<4; i++){
		dest[i] = (reading&(0xff<<(i<<3)))>>(i<<3);
	}
}

void packDataHalf(uint8_t* dest, uint16_t data){
	uint8_t i;
	for (i=0; i<2; i++){
		dest[i] = (data&(0xff<<(i<<3)))>>(i<<3);
	}
}

void sampleCurrentVoltageWaveform(){
	uint16_t sampleCnt = 0;
	uint16_t temp;
	timerVal[0] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
	while (sampleCnt < BUF_SIZE2){
		//timerVal[(sampleCnt%2)] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
		temp = adc_get(I_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_256);
		currentADCVal[sampleCnt] = ((temp>>5)>1023)? 0 : (temp>>5);
		temp = adc_get(EXT_VOLT_IN_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_256);
		voltADCVal[sampleCnt] = ((temp>>5)>1023)? 0 : (temp>>5);
		sampleCnt++;
	}
	timerVal[1] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
}


static void disable_all_ioc_override() {
	uint8_t portnum = 0;
	uint8_t pinnum = 0;
	for(portnum = 0; portnum < 4; portnum++) {
		for(pinnum = 0; pinnum < 8; pinnum++) {
			ioc_set_over(portnum, pinnum, IOC_OVERRIDE_DIS);
		}
	}
}

// interrupt when all units are ready
static void unitReadyCallBack(uint8_t port, uint8_t pin){
	GPIO_DISABLE_INTERRUPT(TRIUMVI_RDYn_IN_GPIO_BASE, 0x1<<TRIUMVI_RDYn_IN_GPIO_PIN);
	nvic_interrupt_disable(TRIUMVI_RDYn_IN_INT_NVIC_PORT);
	process_poll(&triumviProcess);
    allInitsAreReadyInt = 1;
}

static void referenceIntCallBack(uint8_t port, uint8_t pin){
    meterVoltageComparator(SENSE_DISABLE);
    referenceInt = 1;
    // not sure if this is necessary
    if (operation_mode==MODE_CALIBRATION)
        process_poll(&calibrationProcess);
    else
        process_poll(&triumviProcess);
}

// unit is mA
// FIX ME, OVERFLOW!!
uint16_t currentRMS(uint16_t triumviStatusReg){
    uint16_t i;
    uint32_t result = 0;
    uint16_t length = (triumviStatusReg & EXTERNALVOLT_STATUSREG)? BUF_SIZE2 : BUF_SIZE;

    for (i=0; i<length; i++){
        result += ((adjustedCurrSamples[i]*adjustedCurrSamples[i])>>2);
    }
    result /= length;
    return (mysqrt(result)<<1);
}

// unit is V
uint16_t voltageRMS(uint16_t triumviStatusReg){
    uint16_t i;
    float tmp;
    float result = 0;
    if (triumviStatusReg & EXTERNALVOLT_STATUSREG){
        for (i=0; i<BUF_SIZE2; i++){
            tmp = adjustedVoltSamples[i]/1000;
            result += tmp*tmp;
        }
        result /= BUF_SIZE2;
        return mysqrt((uint32_t)result);
    }
    return 120;
    
    
}

uint16_t mysqrt(uint32_t n){
    uint32_t xn = n;
    uint8_t i;
    for (i=0; i<20; i++){
        xn = (xn + n/xn)/2;
    }
    return (uint16_t)xn;
}

