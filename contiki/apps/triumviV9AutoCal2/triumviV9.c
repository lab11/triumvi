
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
#include "simple_network_driver.h"
#include "dev/rom-util.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef RSENSE_LOW
#define I_TRANSFORM 97 // 45.3 ohm sensing resistor
#else
#define I_TRANSFORM 48.34468 // 90.9 ohm sensing resistor
#endif

#define FIRSTSAMPLE_STATUSREG  0x0100
#define EXTERNALVOLT_STATUSREG 0x0080
#define BATTERYPACK_STATUSREG  0x0040
#define FRAMWRITE_STATUSREG    0x0008
#define POWERFACTOR_STATUSREG  0x0004
#define TIMESTAMP_STATUSREG    0x0002

#define FLASH_BASE 0x200000
#define FLASH_SIZE 0x80000 //rom_util_get_flash_size()
#define FLASH_ERASE_SIZE 0x800

// Adjusted (DC removal) ADC sample thresholds
#define UPPERTHRESHOLD0  400 // upper threshold for gain == 17
#define UPPERTHRESHOLD1  500 // upper threshold for gain == 3, 5, 9
#define UPPERTHRESHOLD2  700 // upper threshold for gain == 2
#define LOWERTHRESHOLD  185  // lower threshold

// number of samples per cycle
#define BUF_SIZE 120        // sample current only, 11-bit resolution, 1 cycles
#define BUF_SIZE2 228       // sample both current and voltage, 10-bit resolution, 2 cycles
#define MAX_BUF_SIZE 228    // max(BUF_SIZE, BUF_SIZE2)

// number of calibration cycles
#define CALIBRATION_CYCLES 512
#define AMP_CALIBRATION_CYCLE 64

#define VOLTAGE_NOMINAL 120

// maximum different current setting per gain
#define MAX_CURRENT_SETTING_PER_GAIN 16

// maximum achievable setting for APS3B12, unit is mA
#define MAX_CURRENT_SETTING 8750

// phase lock threshold
#define PHASE_VARIANCE_THRESHOLD 15

// INA gain indices
#define MAX_INA_GAIN_IDX 5
#define MIN_INA_GAIN_IDX 1

// voltage isolation filter offset
#define VOLTAGE_SAMPLE_OFFSET 0
// voltage scaling constant
#define VOLTAGE_SCALING 720

// APS3B12 control macro
#define APS3B12_PACKET_ID 31
#define APS3B12_ENABLE 1
#define APS3B12_SET_CURRENT 2
#define APS3B12_READ 3
#define APS3B12_READ_CURRENT 0
#define APS3B12_CURRENT_INFO 3
#define APS3B12_TRIALS 10

#define DIFF_THRESHOLD 10

#ifdef RTC_ENABLE
#define TRIUMVI_RTC 0xac
#define TRIUMVI_RTC_SET 0xff
#define TRIUMVI_RTC_REQ 0xfe
static rv3049_time_t rtcTime;
#endif

#include "calibration_coef.h"

const uint8_t inaGainArr[6] = {1, 2, 3, 5, 9, 17};

typedef enum {
    MODE_PHASE_CALIBRATION,
    MODE_AMPLITUDE_CALIBRATION,
    MODE_NORMAL
} triumvi_mode_t;

typedef enum {
    GAIN_TOO_LOW,
    GAIN_OK,
    GAIN_TOO_HIGH
} gainSetting_t;

typedef enum {
    #ifdef RTC_ENABLE
    STATE_READ_RTC_TIME,
    #endif
    STATE_INIT,
    STATE_WAITING_VOLTAGE_STABLE,
    STATE_WAITING_COMPARATOR_STABLE,
    STATE_BATTERYPACK_LEDBLINK,
    STATE_TRIUMVI_LEDBLINK
} triumvi_state_t;

typedef enum {
    STATE_STD_LOAD_ACTIVATATION,
    STATE_STD_LOAD_SET,
    STATE_CALIBRATION_IN_PROGRESS,
    #ifdef RTC_ENABLE
    STATE_CALIBRATION_SET_RTC,
    #endif
    STATE_CALIBRATION_COMPLETED,
    STATE_CALIBRATION_NULL
} triumvi_state_calibration_t;

typedef enum{
    STATE_AMP_STD_LOAD_SET,
    STATE_AMP_CALIBRATION_IN_PROGRESS,
    STATE_AMP_STD_LOAD_RECEIVE,
    STATE_AMP_STD_LOAD_VERIFY,
    STATE_AMP_NULL
} triumvi_state_amp_calibration_t;


/* global variables */

volatile static triumvi_state_t myState;
const uint32_t flash_addr = FLASH_BASE - 2*FLASH_ERASE_SIZE + FLASH_SIZE;
volatile uint32_t flash_data;

uint32_t timerVal[2];
uint16_t currentADCVal[MAX_BUF_SIZE];
int adjustedCurrSamples[MAX_BUF_SIZE];
int voltADCVal[BUF_SIZE2]; 

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
volatile uint8_t rfReceivedInt;
#ifdef RTC_ENABLE
volatile uint8_t rtc_packet_received;
#endif

static uint8_t aps_trials;
volatile int aps3b12_current_value; 

/* End of global variables */

#include "sineTable.h"

/* function prototypes */

/* functions for calibration process only */
// return product of voltage and current for a cycle
int cycleProduct(uint16_t* adcSamples, uint16_t offset, uint16_t currentRef);
// return the phase has maximum correlation
uint16_t phaseMatchFilter(uint16_t* adcSamples, uint16_t* currentAVG);
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
void encryptAndTransmit(triumvi_record_t* thisSample, 
                        uint8_t* myNonce, uint32_t nonceCounter);

int currentDataTransform(int currentReading, uint8_t externalVolt);
int voltDataTransform(int voltReading, uint16_t voltReference);
// controlling aps3b12
void aps3b12_set_current(uint16_t cu);
void aps3b12_enable(uint8_t en);

#ifdef AMPLITUDE_CALIBRATION_EN
void aps3b12_read_current();
// find coefficient for 1st order linear regression
void linearFit(uint16_t* reading, uint16_t* setting, uint8_t length, 
                uint32_t* slope_n, uint32_t* slope_d, int* offset);
#endif

#if defined(AMPLITUDE_CALIBRATION_EN) || defined(RTC_ENABLE)
void rf_rx_handler();
#endif

// functions do not use in data dump mode
static void disable_all_ioc_override();


uint16_t currentRMS(uint16_t triumviStatusReg);
uint16_t voltageRMS(uint16_t triumviStatusReg);

// ISRs
static void rtimerEvent(struct rtimer *t, void *ptr);
static void referenceIntCallBack(uint8_t port, uint8_t pin);
static void unitReadyCallBack(uint8_t port, uint8_t pin);

/* End of function prototypes */

/*---------------------------------------------------------------------------*/
PROCESS(startupProcess, "Startup");
PROCESS(phaseCalibrationProcess, "Phase Calibration");
#ifdef AMPLITUDE_CALIBRATION_EN
PROCESS(amplitudeCalibrationProcess, "Amplitude Calibration");
PROCESS(rf_received_process, "rf receive");
#endif
PROCESS(triumviProcess, "Triumvi");
AUTOSTART_PROCESSES(&startupProcess);
/*---------------------------------------------------------------------------*/

// Startup process, check flash
PROCESS_THREAD(startupProcess, ev, data) {
    PROCESS_BEGIN();

    CC2538_RF_CSP_ISRFOFF();

    random_init(0);
    fm25v02_sleep();
    simple_network_set_callback(&rf_rx_handler);
    process_start(&rf_received_process, NULL);

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
        process_start(&phaseCalibrationProcess, NULL);
        operation_mode = MODE_PHASE_CALIBRATION;
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

PROCESS_THREAD(phaseCalibrationProcess, ev, data) {
    PROCESS_BEGIN();

    static uint16_t cycleCnt = 0;
    uint16_t i;
    gainSetting_t gainSetting;
    static triumvi_state_calibration_t calibration_state = STATE_STD_LOAD_ACTIVATATION;
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
    #ifdef RTC_ENABLE
    static uint8_t rtc_pkt[2] = {TRIUMVI_RTC, TRIUMVI_RTC_REQ};
    rtc_packet_received = 0;
    #endif
    rfReceivedInt = 0;
    aps_trials = 0;

    rom_util_page_erase(flash_addr, FLASH_ERASE_SIZE);
    triumviLEDON();
    etimer_set(&calibration_timer, CLOCK_SECOND*10);

    // enable 5k ohm trickle charge resistor
    rv3049_write_register(RV3049_PAGE_ADDR_EEPROM_CTRL, 0x00, 0x22);

    // enable LDO, release power gating
    meterSenseVREn(SENSE_ENABLE);
    meterSenseConfig(VOLTAGE, SENSE_ENABLE);
    meterSenseConfig(CURRENT, SENSE_ENABLE);

    while (1){
        PROCESS_YIELD();
        switch (calibration_state){
            case STATE_STD_LOAD_ACTIVATATION:
                if (etimer_expired(&calibration_timer)){
                    triumviLEDOFF();
                    if (aps_trials < APS3B12_TRIALS){
                        aps3b12_enable(0x1);
                        etimer_set(&calibration_timer, CLOCK_SECOND*0.5);
                        aps_trials += 1;
                    }
                    else{
                        etimer_set(&calibration_timer, CLOCK_SECOND*3);
                        calibration_state = STATE_STD_LOAD_SET;
                        aps_trials = 0;
                    }
                }
            break;

            case STATE_STD_LOAD_SET:
                if (etimer_expired(&calibration_timer)){
                    if (aps_trials < APS3B12_TRIALS){
                        aps3b12_set_current(2000); // 2000 mA
                        etimer_set(&calibration_timer, CLOCK_SECOND*1);
                        aps_trials += 1;
                    }
                    else{
                        etimer_set(&calibration_timer, CLOCK_SECOND*3);
                        calibration_state = STATE_CALIBRATION_IN_PROGRESS;
                        aps_trials = 0;
                    }
                }
            break;

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
                            #ifdef RTC_ENABLE
                            calibration_state = STATE_CALIBRATION_SET_RTC;
                            #else
                            calibration_state = STATE_CALIBRATION_COMPLETED;
                            #endif
                            etimer_set(&calibration_timer, CLOCK_SECOND*3);
                            #endif // end of DATADUMP
                        }
                    }
                    else{
                        triumviLEDOFF();
                        etimer_set(&calibration_timer, CLOCK_SECOND*1);
                    }
                }
            break;

            #ifdef RTC_ENABLE
            case STATE_CALIBRATION_SET_RTC:
                if (etimer_expired(&calibration_timer)){
                    if ((rtc_packet_received) && (rtcTime.year > 2000)){
                        calibration_state = STATE_CALIBRATION_COMPLETED;
                        rv3049_write_register(RV3049_PAGE_ADDR_CONTROL, 0x03, 0x00);
                    }
                    else{
                        packetbuf_copyfrom(rtc_pkt, 2);
                        cc2538_on_and_transmit();
                    }
                    rtc_packet_received = 0;
                    etimer_set(&calibration_timer, CLOCK_SECOND*0.5);
                }
            break;
            #endif

            case STATE_CALIBRATION_COMPLETED:
                if (etimer_expired(&calibration_timer)){
                    #ifdef AMPLITUDE_CALIBRATION_EN
                    process_start(&amplitudeCalibrationProcess, NULL);
                    operation_mode = MODE_AMPLITUDE_CALIBRATION;
                    #else
                    #ifndef DATADUMP2
                    GPIO_SET_OUTPUT(GPIO_A_BASE, 0x47);
                    GPIO_CLR_PIN(GPIO_A_BASE, 0x07);
                    #endif
                    process_start(&triumviProcess, NULL);
                    operation_mode = MODE_NORMAL;
                    // enable 80k trickle charge resistor
                    rv3049_write_register(RV3049_PAGE_ADDR_EEPROM_CTRL, 0x00, 0x82);
                    #endif
                    triumviLEDOFF();
                    if (batteryPackIsAttached()){
                        batteryPackLEDOff(BATTERY_PACK_LED_GREEN);
                        batteryPackVoltageEn(SENSE_DISABLE);
                    }
                    calibration_state = STATE_CALIBRATION_NULL;
                }
            break;

            case STATE_CALIBRATION_NULL:
            break;

            default:
            break;
        }
    }
    PROCESS_END();

}

#ifdef AMPLITUDE_CALIBRATION_EN
PROCESS_THREAD(amplitudeCalibrationProcess, ev, data){
    PROCESS_BEGIN();


    static uint16_t currentSetting = 1000; // starts with 1000 mA
    triumviLEDON();
    etimer_set(&calibration_timer, CLOCK_SECOND*5);
    static triumvi_state_amp_calibration_t amplitude_calibration_state = STATE_AMP_STD_LOAD_SET;
    static uint32_t timerExp, currentTime;
    uint16_t i, j;
    int tempPower;
    int energyCal;
    static uint32_t sum_power;
    static uint32_t sum_currentRMS;
    static uint8_t prevInaGainIdx = MAX_INA_GAIN_IDX+1;
    static uint16_t current_setting[MAX_CURRENT_SETTING_PER_GAIN];
    static uint16_t read_current[MAX_CURRENT_SETTING_PER_GAIN];
    static uint16_t read_power[MAX_CURRENT_SETTING_PER_GAIN];
    static uint8_t current_set_cnt;
    static uint8_t amp_cal_cnt;
    static uint8_t amp_cal_completed;
    static uint32_t slope_n, slope_d;
    static int offset;
    static uint8_t increase_current;
    static uint32_t diff;
    gainSetting_t gainSetting;
    aps_trials = 0;
    current_set_cnt = 0;
    amp_cal_completed = 0;
    increase_current = 0;

    // enable LDO, release power gating
    meterSenseVREn(SENSE_ENABLE);
    meterSenseConfig(VOLTAGE, SENSE_ENABLE);
    meterSenseConfig(CURRENT, SENSE_ENABLE);

    while (1){
        PROCESS_YIELD();
        switch (amplitude_calibration_state){
            case STATE_AMP_STD_LOAD_SET:
                if (etimer_expired(&calibration_timer)){
                    triumviLEDOFF();
                    if (aps_trials < APS3B12_TRIALS){
                        aps3b12_set_current(currentSetting);
                        etimer_set(&calibration_timer, CLOCK_SECOND*0.5);
                        aps_trials += 1;
                    }
                    else{
                        aps_trials = 0;
                        sum_currentRMS = 0;
                        sum_power = 0;
                        amp_cal_cnt = 0;
                        if (amp_cal_completed){
                            if (batteryPackIsAttached()){
                                batteryPackLEDOff(BATTERY_PACK_LED_GREEN);
                                batteryPackVoltageEn(SENSE_DISABLE);
                            }
                            triumviLEDOFF();
                            // start measurement process
                            process_start(&triumviProcess, NULL);
                            #ifndef DATADUMP2
                            GPIO_SET_OUTPUT(GPIO_A_BASE, 0x47);
                            GPIO_CLR_PIN(GPIO_A_BASE, 0x07);
                            #endif
                            operation_mode = MODE_NORMAL;
                            amplitude_calibration_state = STATE_AMP_NULL;
                            // enable 80k trickle charge resistor
                            rv3049_write_register(RV3049_PAGE_ADDR_EEPROM_CTRL, 0x00, 0x82);
                        }
                        else{
                            aps3b12_read_current();
                            etimer_set(&calibration_timer, CLOCK_SECOND*5);
                            amplitude_calibration_state = STATE_AMP_STD_LOAD_RECEIVE;
                        }
                        
                    }
                }
            break;

            case STATE_AMP_STD_LOAD_RECEIVE:
                // timer expired, did not received data
                if (etimer_expired(&calibration_timer)) {
                    amplitude_calibration_state = STATE_AMP_STD_LOAD_SET;
                    etimer_set(&calibration_timer, CLOCK_SECOND*1);
                }
                else{
                    CC2538_RF_CSP_ISRFOFF();
                    amplitude_calibration_state = STATE_AMP_STD_LOAD_VERIFY;
                }
            break;

            case STATE_AMP_STD_LOAD_VERIFY:
                if (etimer_expired(&calibration_timer)) {
                    if (aps3b12_current_value > currentSetting){
                        diff = aps3b12_current_value - currentSetting;
                    }
                    else if (aps3b12_current_value < currentSetting){
                        diff = currentSetting - aps3b12_current_value;
                    }
                    else{
                        diff = 0;
                    }
                    if (diff < DIFF_THRESHOLD){
                        currentSetting = aps3b12_current_value;
                        amplitude_calibration_state = STATE_AMP_CALIBRATION_IN_PROGRESS;
                    }
                    else{
                        amplitude_calibration_state = STATE_AMP_STD_LOAD_SET;
                    }
                    etimer_set(&calibration_timer, CLOCK_SECOND*1);
                }
            break;

            case STATE_AMP_CALIBRATION_IN_PROGRESS:
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
                        gainSetting = gainCtrl(currentADCVal, 0x0);
                        REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
                        gate_gpt(GPTIMER_1);
                    }

                    if (gainSetting==GAIN_OK){
                        
                        energyCal = 0;
                        for (i=0; i<BUF_SIZE; i++){
                            j = ((i*3+phaseOffset) >= 360)? i*3+phaseOffset-360 : i*3+phaseOffset;
                            adjustedCurrSamples[i] = currentDataTransform(adjustedCurrSamples[i], 0x0);
                            energyCal += (adjustedCurrSamples[i]*stdSineTable[j]);
                        }
                        tempPower = (energyCal/BUF_SIZE); // unit is mW
                        // Fix phase oppsite down
                        if (tempPower < 0)
                            tempPower = -1*tempPower;

                        // need to perform flash check, make sure if it's empty to proceed
                        flash_data = REG(flash_addr+(inaGainIdx*16)+4);

                        if (flash_data==0xffffffff){ 
                            sum_power += tempPower;
                            sum_currentRMS += currentRMS(0);
                            amp_cal_cnt += 1;
                            if (amp_cal_cnt == AMP_CALIBRATION_CYCLE){
                                current_setting[current_set_cnt] = currentSetting;
                                read_current[current_set_cnt] = sum_currentRMS/AMP_CALIBRATION_CYCLE;
                                read_power[current_set_cnt] = (sum_power/AMP_CALIBRATION_CYCLE/VOLTAGE_NOMINAL);
                                #ifdef DATADUMP3
                                printf("Source Setting: %u\r\n", current_setting[current_set_cnt]);
                                printf("IRMS Reading: %u\r\n", read_current[current_set_cnt]);
                                #endif

                                if (((current_set_cnt == MAX_CURRENT_SETTING_PER_GAIN) || (currentSetting == MAX_CURRENT_SETTING)) && (current_set_cnt > 1)){
                                    linearFit(read_current, current_setting, current_set_cnt, &slope_n, &slope_d, &offset);
                                    #ifdef DATADUMP3
                                    printf("B\r\n");
                                    printf("current correction\r\n");
                                    printf("Number of points: %u\r\n", current_set_cnt);
                                    printf("Numerator: %lu\r\n", slope_n);
                                    printf("Denumerator: %lu\r\n", slope_d);
                                    printf("Offset: %d\r\n", offset);
                                    #endif
                                    rom_util_program_flash((uint32_t*)&slope_n, flash_addr+(inaGainIdx*16)+4, 4);
                                    rom_util_program_flash((uint32_t*)&slope_d, flash_addr+(inaGainIdx*16)+8, 4);
                                    rom_util_program_flash((uint32_t*)&offset, flash_addr+(inaGainIdx*16)+12, 4);
                                    // power correction
                                    linearFit(read_power, current_setting, current_set_cnt, &slope_n, &slope_d, &offset);
                                    offset *= VOLTAGE_NOMINAL;
                                    #ifdef DATADUMP3
                                    printf("power correction\r\n");
                                    printf("Numerator: %lu\r\n", slope_n);
                                    printf("Denumerator: %lu\r\n", slope_d);
                                    printf("Offset: %d\r\n", offset);
                                    #endif
                                    rom_util_program_flash((uint32_t*)&slope_n, flash_addr+(inaGainIdx*16)+4+(MAX_INA_GAIN_IDX+1)*16, 4);
                                    rom_util_program_flash((uint32_t*)&slope_d, flash_addr+(inaGainIdx*16)+8+(MAX_INA_GAIN_IDX+1)*16, 4);
                                    rom_util_program_flash((uint32_t*)&offset, flash_addr+(inaGainIdx*16)+12+(MAX_INA_GAIN_IDX+1)*16, 4);
                                    current_set_cnt = 0;
                                }
                                else{
                                    current_set_cnt += 1;
                                }
                                increase_current = 1;
                                amp_cal_cnt = 0;
                            }
                            else{
                                if ((prevInaGainIdx<=MAX_INA_GAIN_IDX) && (amp_cal_cnt==1) && (inaGainIdx != prevInaGainIdx) && (current_set_cnt > 1)){
                                    linearFit(read_current, current_setting, current_set_cnt, &slope_n, &slope_d, &offset);
                                    #ifdef DATADUMP3
                                    printf("A\r\n");
                                    printf("current correction\r\n");
                                    printf("Number of points: %u\r\n", current_set_cnt);
                                    printf("Numerator: %lu\r\n", slope_n);
                                    printf("Denumerator: %lu\r\n", slope_d);
                                    printf("Offset: %d\r\n", offset);
                                    #endif
                                    rom_util_program_flash((uint32_t*)&slope_n, flash_addr+(prevInaGainIdx*16)+4, 4);
                                    rom_util_program_flash((uint32_t*)&slope_d, flash_addr+(prevInaGainIdx*16)+8, 4);
                                    rom_util_program_flash((uint32_t*)&offset, flash_addr+(prevInaGainIdx*16)+12, 4);
                                    // power correction
                                    linearFit(read_power, current_setting, current_set_cnt, &slope_n, &slope_d, &offset);
                                    offset *= VOLTAGE_NOMINAL;
                                    #ifdef DATADUMP3
                                    printf("power correction\r\n");
                                    printf("Numerator: %lu\r\n", slope_n);
                                    printf("Denumerator: %lu\r\n", slope_d);
                                    printf("Offset: %d\r\n", offset);
                                    #endif
                                    rom_util_program_flash((uint32_t*)&slope_n, flash_addr+(prevInaGainIdx*16)+4+(MAX_INA_GAIN_IDX+1)*16, 4);
                                    rom_util_program_flash((uint32_t*)&slope_d, flash_addr+(prevInaGainIdx*16)+8+(MAX_INA_GAIN_IDX+1)*16, 4);
                                    rom_util_program_flash((uint32_t*)&offset, flash_addr+(prevInaGainIdx*16)+12+(MAX_INA_GAIN_IDX+1)*16, 4);
                                    current_set_cnt = 0;
                                }
                                etimer_set(&calibration_timer, CLOCK_SECOND*0.1);
                            }
                            prevInaGainIdx = inaGainIdx;
                        }
                        // it has been alreay written, advances current setting
                        else{
                            increase_current = 1;
                        }
                        if (increase_current){
                            currentSetting += 250;
                            // calibration completed
                            if (currentSetting > MAX_CURRENT_SETTING){
                                currentSetting = 1000;
                                amp_cal_completed = 1;
                                if (batteryPackIsAttached()){
                                    batteryPackVoltageEn(SENSE_ENABLE);
                                    batteryPackInit();
                                    batteryPackLEDDriverInit();
                                    batteryPackLEDOn(BATTERY_PACK_LED_GREEN);
                                    i2c_disable(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN); 
                                }
                                triumviLEDON();
                            }
                            etimer_set(&calibration_timer, CLOCK_SECOND*1);
                            amplitude_calibration_state = STATE_AMP_STD_LOAD_SET;
                            increase_current = 0;
                        }
                        if (amp_cal_completed==0){
                            triumviLEDOFF();
                        }
                        
                    }
                    else{
                        triumviLEDOFF();
                        etimer_set(&calibration_timer, CLOCK_SECOND*0.1);
                    }
                }
            break;

            case STATE_AMP_NULL:
            break;

            default:
            break;
        }
    }
    PROCESS_END();

}
#endif

#if defined(AMPLITUDE_CALIBRATION_EN) || defined(RTC_ENABLE)
PROCESS_THREAD(rf_received_process, ev, data) {
    PROCESS_BEGIN();

    //uint8_t *header_ptr;
    uint8_t *data_ptr;
    uint8_t data_length;
    //uint8_t header_length;

    while(1){
        PROCESS_YIELD();
        //triumviLEDToggle();
        // Get data from radio buffer and parse it
        if (rfReceivedInt==1){ 
            rfReceivedInt = 0;
            //header_length = packetbuf_hdrlen();
            //header_ptr = packetbuf_hdrptr();                                   
            data_length = packetbuf_datalen();                               
            data_ptr = packetbuf_dataptr();                                  

            #ifdef RTC_ENABLE
            if ((data_ptr[0] == TRIUMVI_RTC) && (data_ptr[1] == TRIUMVI_RTC_SET)&&(data_length==8)){
                rtcTime.year    = data_ptr[2] + 2000;
                rtcTime.month   = data_ptr[3];
                rtcTime.days    = data_ptr[4];
                rtcTime.hours   = data_ptr[5];
                rtcTime.minutes = data_ptr[6];
                rtcTime.seconds = data_ptr[7];
                rv3049_set_time(&rtcTime);
                CC2538_RF_CSP_ISRFOFF();
                rtc_packet_received = 1;
            }
            #endif
            if (data_ptr[0] == APS3B12_PACKET_ID){
                if (data_ptr[1] == APS3B12_CURRENT_INFO){
                    aps3b12_current_value = ((data_ptr[2] << 24) | (data_ptr[3] << 16) 
                            | (data_ptr[4] << 8) | data_ptr[5]);
                    process_poll(&amplitudeCalibrationProcess);
                }
            }
        }
    }
    PROCESS_END();
}
#endif

PROCESS_THREAD(triumviProcess, ev, data) {
    PROCESS_BEGIN();

    static triumvi_record_t triumvi_record;
    // packet preprocessing
    static uint8_t extAddr[8];
    NETSTACK_RADIO.get_object(RADIO_PARAM_64BIT_ADDR, extAddr, 8);

    // AES Preprocessing
    static uint8_t aesKey[] = AES_KEY;
    static uint8_t myNonce[13] = {0};
    memcpy(myNonce, extAddr, 8);
    static uint32_t nonceCounter = 0;
    aes_load_keys(aesKey, AES_KEY_STORE_SIZE_KEY_SIZE_128, 1, 0);
    uint16_t rand0, rand1;

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
    uint32_t numerator, denumerator;
    int offset;

    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND, 1, &rtimerEvent, NULL);

    #ifdef RTC_ENABLE
    static uint8_t rtc_pkt[2] = {TRIUMVI_RTC, TRIUMVI_RTC_REQ};
    myState = STATE_READ_RTC_TIME;
    static uint8_t spi_buf;
    rtc_packet_received = 0;
    #else
    myState = STATE_INIT;
    #endif

    while(1){
        PROCESS_YIELD();
        switch (myState){
            #ifdef RTC_ENABLE
            case STATE_READ_RTC_TIME:
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                    rv3049_read_time(&rtcTime);
                    spi_buf = rv3049_read_register(RV3049_PAGE_ADDR_CONTROL, 0x03);
                    // PON Bit in control_status register. if this bit is set,
                    // Time is corrupted, ask gateway for correct time
                    if (spi_buf & 0x20){
                        packetbuf_copyfrom(rtc_pkt, 2);
                        cc2538_on_and_transmit();

                        // stays on for 2.5 ms to receive correct time
                        ungate_gpt(GPTIMER_1);
                        REG(SYSTICK_STCTRL) &= (~SYSTICK_STCTRL_INTEN);
                        timerExp = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A) - 40000;
                        do {
                            currentTime = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
                        }
                        while ((currentTime > timerExp) && (rtc_packet_received==0));
                        REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
                        gate_gpt(GPTIMER_1);
                        CC2538_RF_CSP_ISRFOFF();
                        // if time is received from gateway, clear PON bit
                        if (rtc_packet_received==1){
                            rtc_packet_received = 0;
                            rv3049_write_register(RV3049_PAGE_ADDR_CONTROL, 0x03, 0x00);
                        }

                    }
                    myState = STATE_INIT;
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.5, 1, &rtimerEvent, NULL);
                }
            break;
            #endif

            // initialization state, check READYn is low before moving forward
            case STATE_INIT:
                rdy = 0;
                unitReady();
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
                    unitClrReady();
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
                    // Bit 2: RTC Enable
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

                    triumviStatusReg |= POWERFACTOR_STATUSREG;

                    #ifdef RTC_ENABLE
                    triumviStatusReg |= TIMESTAMP_STATUSREG;
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
                        #ifdef POLYFIT
                        numerator   = REG(flash_addr+(inaGainIdx*16)+4+(MAX_INA_GAIN_IDX+1)*16);
                        denumerator = REG(flash_addr+(inaGainIdx*16)+8+(MAX_INA_GAIN_IDX+1)*16);
                        offset      = REG(flash_addr+(inaGainIdx*16)+12+(MAX_INA_GAIN_IDX+1)*16);
                        if (offset != 0xffffffff){
                            if (avgPower > 50){
                                avgPower = (int)(((int64_t)avgPower)*numerator/denumerator + offset);
                            }
                        }
                        else{
                            switch (inaGain){
                                case 2:
                                    avgPower = (int)((float)avgPower*PGAIN2_D1 + PGAIN2_D0);
                                break;
                                case 3:
                                    avgPower = (int)((float)avgPower*PGAIN3_D1 + PGAIN3_D0);
                                break;
                                case 5:
                                    avgPower = (int)((float)avgPower*PGAIN5_D1 + PGAIN5_D0);
                                break;
                                case 9:
                                    avgPower = (int)((float)avgPower*PGAIN9_D1 + PGAIN9_D0);
                                break;
                                case 17:
                                    if (avgPower>50)
                                        avgPower = (int)((float)avgPower*PGAIN17_D1 + PGAIN17_D0);
                                break;
                                default:
                                break;
                            }
                        }
                        #endif
                        IRMS = currentRMS(triumviStatusReg);
                        VRMS = voltageRMS(triumviStatusReg);
                        #ifdef POLYFIT
                        numerator   = REG(flash_addr+(inaGainIdx*16)+4);
                        denumerator = REG(flash_addr+(inaGainIdx*16)+8);
                        offset      = REG(flash_addr+(inaGainIdx*16)+12);
                        if (offset != 0xffffffff){
                            if (IRMS > 0.4){
                                IRMS = (uint16_t)((((uint64_t)IRMS)*numerator/denumerator) + offset);
                            }
                        }
                        // use default calibration coef
                        else{
                            switch (inaGain){
                                case 2:
                                    IRMS = (int)((float)IRMS*IGAIN2_D1 + IGAIN2_D0);
                                break;
                                case 3:
                                    IRMS = (int)((float)IRMS*IGAIN3_D1 + IGAIN3_D0);
                                break;
                                case 5:
                                    IRMS = (int)((float)IRMS*IGAIN5_D1 + IGAIN5_D0);
                                break;
                                case 9:
                                    IRMS = (int)((float)IRMS*IGAIN9_D1 + IGAIN9_D0);
                                break;
                                case 17:
                                    if (IRMS>0.4)
                                        IRMS = (int)((float)IRMS*IGAIN17_D1 + IGAIN17_D0);
                                break;
                                default:
                                break;
                            }
                        }
                        #endif
                        if ((IRMS==0) || (avgPower==0))
                            pf = 0;
                        else
                            pf = (float)avgPower/(VRMS*IRMS);
                        if (pf > 1)
                            pf = 1;
                        #ifdef DATADUMP2
                        uint8_t i;
                        //printf("ADC reference: %u\r\n", dcOffset);
                        //printf("Time difference: %lu\r\n", (timerVal[0]-timerVal[1]));
                        //printf("INA Gain: %u\r\n", inaGain);
                        //for (i=0; i<BUF_SIZE; i+=1)
                        //    printf("Current reading: %d\r\n", currentADCVal[i]);
                        printf("INA Gain: %u\r\n", inaGain);
                        printf("IRMS: %u\r\n", IRMS);
                        printf("Average Power: %u\r\n", avgPower);
                        #else
                        rand0 = random_rand();
                        rand1 = random_rand();
                        nonceCounter = (rand0<<16) | rand1;
                        
                        // Battery pack attached, turn it on ans samples switches
                        if (triumviStatusReg & BATTERYPACK_STATUSREG){
                            batteryPackVoltageEn(SENSE_ENABLE);
                            sx1509b_init();
                            sx1509b_high_voltage_input_enable(SX1509B_PORTA, 0x1, SX1509B_HIGH_INPUT_ENABLE);
                            triumvi_record.panelID = batteryPackReadPanelID();
                            triumvi_record.circuitID = batteryPackReadCircuitID();
                            batteryPackVoltageEn(SENSE_DISABLE);
                            i2c_disable(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, 
                                        I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN); 
                        }
                        triumvi_record.avgPower = avgPower;
                        triumvi_record.triumviStatusReg = (uint8_t)(triumviStatusReg & 0xff);
                        triumvi_record.IRMS = IRMS;
                        triumvi_record.VRMS = (inaGain<<8)|VRMS;
                        triumvi_record.pf = pf;
                        // Write data into FRAM
                        #ifdef FRAM_WRITE
                        triumviFramWrite(triumvi_record, rtctime);
                        #endif
                        encryptAndTransmit(&triumvi_record, myNonce, nonceCounter);
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
                    #ifdef RTC_ENABLE
                    myState = STATE_READ_RTC_TIME;
                    #else
                    myState = STATE_INIT;
                    #endif
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
                    #ifdef RTC_ENABLE
                    myState = STATE_READ_RTC_TIME;
                    #else
                    myState = STATE_INIT;
                    #endif
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

// return phase offset has max product, only be called in calibration mode
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

// Fine tune to 3.002 degree / sample
void sampleCurrentWaveform(){
	uint16_t sampleCnt = 0;
	uint16_t temp;
    #ifdef FIFTYHZ
    uint8_t i;
    #endif
    timerVal[0] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
	while (sampleCnt < BUF_SIZE){
        #ifdef FIFTYHZ
        for (i=0; i<59; i++)
            asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        #else
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
        #endif
		temp = adc_get(I_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_512);
		currentADCVal[sampleCnt] = ((temp>>4)>2047)? 0 : (temp>>4);
		sampleCnt++;
	}
    timerVal[1] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
}

void meterInit(){

    // GPIO default Input
    // Set all un-used pins to output and clear output
    #if defined(DATADUMP) || defined(DATADUMP2) || defined(DEBUG_ON) || defined(DATADUMP3)
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
    #if !defined(DATADUMP) && !defined(DATADUMP2) && !defined(DEBUG_ON) && !defined(DATADUMP3)
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
    #ifdef RSENSE_LOW
    inaGainIdx = 4; // G = 9
    #else
    inaGainIdx = 3; // G = 5
    #endif

	backOffTime = 4;
	backOffHistory = 0;

}

// this function check if the ADC samples are within a proper range
gainSetting_t gainCtrl(uint16_t* adcSamples, uint8_t externalVolt){
    uint16_t i;
    uint16_t upperThreshold = (inaGainIdx==MAX_INA_GAIN_IDX)? UPPERTHRESHOLD0 : 
                              (inaGainIdx==1)? UPPERTHRESHOLD2 : UPPERTHRESHOLD1;
    uint16_t lowerThreshold = LOWERTHRESHOLD;
    uint16_t length = BUF_SIZE;
    uint16_t currentRef;
    int currentCal;
    int maxVal = 0;
    gainSetting_t res = GAIN_OK;
    
    // in calibration mode, use ADC sample as reference
    if (operation_mode==MODE_PHASE_CALIBRATION){
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
            upperThreshold = (upperThreshold>>1);
            lowerThreshold = (lowerThreshold>>1);
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

void encryptAndTransmit(triumvi_record_t* thisSample, 
                        uint8_t* myNonce, uint32_t nonceCounter){
	// 1 byte Identifier, 4 bytes nonce, 5~19 bytes payload, 4 byte MIC
	static uint8_t packetData[28];
	// 4 bytes reading, 1 byte status reg, 
    // (1 bytes panel ID, 1 bytes circuit ID, 2 bytes PF, 2 bytes VRMS, 2 bytes IRMS optional)
    // (6 bytes time stamp (yy, mm, dd, hh, mm, ss) optional)
	static uint8_t readingBuf[19];
	uint8_t* aData = myNonce;
	uint8_t* pData = readingBuf;
	uint8_t myMic[8] = {0x0};
	uint8_t myPDATA_LEN = PDATA_LEN;
	uint8_t packetLen = 14; // minimum length, 1 byte ID, 4 bytes nonce, 5 bytes payload, 4 byte MIC
	uint16_t randBackOff;

	packetData[0] = TRIUMVI_PKT_IDENTIFIER;
	if (thisSample->triumviStatusReg & BATTERYPACK_STATUSREG){
        readingBuf[myPDATA_LEN] = thisSample->panelID;
        readingBuf[myPDATA_LEN+1] = thisSample->circuitID;
        myPDATA_LEN += 2;
        packetLen += 2;
	}
	readingBuf[4] = thisSample->triumviStatusReg;
	packData(&packetData[1], nonceCounter, 4);
	packData(&myNonce[9], nonceCounter, 4);
	packData(readingBuf, thisSample->avgPower, 4);
    packData(&readingBuf[myPDATA_LEN], (uint16_t)(thisSample->pf*1000), 2);
    packData(&readingBuf[myPDATA_LEN+2], thisSample->VRMS, 2);
    packData(&readingBuf[myPDATA_LEN+4], thisSample->IRMS, 2);
    myPDATA_LEN += 6;
    packetLen += 6;

    #ifdef RTC_ENABLE
    if (thisSample->triumviStatusReg & TIMESTAMP_STATUSREG){
        readingBuf[myPDATA_LEN] = (rtcTime.year - 2000);
        readingBuf[myPDATA_LEN+1] = rtcTime.month;
        readingBuf[myPDATA_LEN+2] = rtcTime.days;
        readingBuf[myPDATA_LEN+3] = rtcTime.hours;
        readingBuf[myPDATA_LEN+4] = rtcTime.minutes;
        readingBuf[myPDATA_LEN+5] = rtcTime.seconds;
        myPDATA_LEN += 6;
        packetLen += 6;
    }
    #endif

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
                voltRef = getAverage32(voltADCVal, BUF_SIZE2);
                energyCal = 0;
                for (j=0; j<BUF_SIZE2; j++){
                    k = ((j + VOLTAGE_SAMPLE_OFFSET)>=BUF_SIZE2)? 
                        (j+VOLTAGE_SAMPLE_OFFSET-BUF_SIZE2) : 
                        j+VOLTAGE_SAMPLE_OFFSET;
                    adjustedCurrSamples[j] = currentDataTransform(adjustedCurrSamples[j], 0x1);
                    voltADCVal[k] = voltDataTransform(voltADCVal[k], voltRef);
                    energyCal += (adjustedCurrSamples[j]*voltADCVal[k])/1000;
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

int voltDataTransform(int voltReading, uint16_t voltReference){
    return (voltReading - voltReference)*VOLTAGE_SCALING;
}



void sampleCurrentVoltageWaveform(){
	uint16_t sampleCnt = 0;
	uint16_t temp;
    #ifdef FIFTYHZ
    uint8_t i;
    #endif
	timerVal[0] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
	while (sampleCnt < BUF_SIZE2){
        #ifdef FIFTYHZ
        for (i=0; i<59; i++)
            asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        #else
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        #endif
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
}

uint16_t currentRMS(uint16_t triumviStatusReg){
    uint16_t i;
    uint32_t result = 0;
    uint64_t result64 = 0;
    uint16_t length = (triumviStatusReg & EXTERNALVOLT_STATUSREG)? BUF_SIZE2 : BUF_SIZE;
    uint8_t gain = inaGainArr[inaGainIdx];

    // if IRMS > 4.34 A, 32 bit will overflow with external voltage (228 samples)
    // For lower gain setting (higher current), use 64 bits
    #ifdef RSENSE_LOW
    #define I_OVERFLOW_GAIN_THRESHOLD 5
    #else
    #define I_OVERFLOW_GAIN_THRESHOLD 3
    #endif
    if (gain <= I_OVERFLOW_GAIN_THRESHOLD){
        for (i=0; i<length; i++){
            result64 += (adjustedCurrSamples[i]*adjustedCurrSamples[i]);
        }
        result = (uint32_t)(result64/length);
    }
    else{
        for (i=0; i<length; i++){
            result += (adjustedCurrSamples[i]*adjustedCurrSamples[i]);
        }
        result /= length;
    }
    return mysqrt(result);
}

// unit is V
uint16_t voltageRMS(uint16_t triumviStatusReg){
    uint16_t i;
    float tmp;
    float result = 0;
    if (triumviStatusReg & EXTERNALVOLT_STATUSREG){
        for (i=0; i<BUF_SIZE2; i++){
            tmp = voltADCVal[i]/1000;
            result += tmp*tmp;
        }
        result /= BUF_SIZE2;
        return mysqrt((uint32_t)result);
    }
    return VOLTAGE_NOMINAL;
    
    
}

void aps3b12_set_current(uint16_t cu){
    uint8_t pkt[4] = {APS3B12_PACKET_ID, APS3B12_SET_CURRENT, 
        (cu&0xff00)>>8, (cu&0xff)};
    packetbuf_copyfrom(pkt, 4);
    cc2538_on_and_transmit();
    CC2538_RF_CSP_ISRFOFF();
}

void aps3b12_enable(uint8_t en){
    uint8_t pkt[4] = {APS3B12_PACKET_ID, APS3B12_ENABLE, (en&0x1), 0x0};
    packetbuf_copyfrom(pkt, 4);
    cc2538_on_and_transmit();
    CC2538_RF_CSP_ISRFOFF();
}

#if defined(AMPLITUDE_CALIBRATION_EN) || defined(RTC_ENABLE)
void rf_rx_handler(){
    process_poll(&rf_received_process);
    rfReceivedInt = 1;
}   
#endif

#ifdef AMPLITUDE_CALIBRATION_EN
void aps3b12_read_current(){
    uint8_t pkt[4] = {APS3B12_PACKET_ID, APS3B12_READ, APS3B12_READ_CURRENT, 0x0};
    packetbuf_copyfrom(pkt, 4);
    cc2538_on_and_transmit();
}

void linearFit(uint16_t* reading, uint16_t* setting, uint8_t length, 
                uint32_t* slope_n, uint32_t* slope_d, int* offset){
    uint8_t i;
    int32_t readingAvg = getAverage(reading, length);
    int32_t settingAvg = getAverage(setting, length);
    int32_t tmp0 = 0;
    uint32_t tmp1 = 0;
    for (i=0; i<length; i++){
        tmp0 += ((reading[i] - readingAvg)*(setting[i] - settingAvg));
        tmp1 += ((reading[i] - readingAvg)*(reading[i] - readingAvg));
    }
    *slope_n = tmp0;
    *slope_d = tmp1;
    *offset = settingAvg - (((uint64_t)readingAvg)*tmp0/tmp1);
}
#endif
