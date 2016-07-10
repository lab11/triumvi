
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
#define THREEPHASE_STATUSREG   0x0030
#define FRAMWRITE_STATUSREG    0x0008

#define FLASH_BASE 0x200000
#define FLASH_SIZE 0x80000 //rom_util_get_flash_size()
#define FLASH_ERASE_SIZE 0x800

// Adjusted (DC removal) ADC sample thresholds
#define UPPERTHRESHOLD  430 // any value above this, gain is too large
#define LOWERTHRESHOLD  185 // max value below this, gain is too small

// number of samples per cycle
#define BUF_SIZE 120    // sample current only, 11-bit resolution
#define BUF_SIZE2 113   // sample both current and voltage, 10-bit resolution

// number of calibration cycles
#define CALIBRATION_CYCLES 255

// phase lock threshold
#define PHASE_VARIANCE_THRESHOLD 15

// INA gain indices
#define MAX_INA_GAIN_IDX 4
#define MIN_INA_GAIN_IDX 1

//#define DATADUMP

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

volatile static triumvi_state_t myState;

/* global variables */

volatile uint32_t flash_calibration = FLASH_BASE - 2*FLASH_ERASE_SIZE + FLASH_SIZE;
volatile uint32_t flash_data;

uint32_t timerVal[BUF_SIZE];
uint16_t currentADCVal[BUF_SIZE];
int adjustedADCSamples[BUF_SIZE];
uint16_t voltADCVal[BUF_SIZE2+2]; // compensate the voltage sample offset

// only for test purpose
//uint16_t adcSamples[] = {
//886, 868, 851, 834, 818, 802, 786, 772, 757, 744, 731, 719, 
//708, 697, 688, 679, 671, 664, 658, 653, 649, 646, 643, 642, 
//642, 643, 645, 647, 651, 656, 661, 668, 675, 684, 693, 703, 
//714, 725, 738, 751, 765, 779, 794, 810, 826, 843, 860, 878, 
//896, 914, 932, 951, 969, 988, 1007, 1026, 1045, 1063, 1082, 1100, 
//1118, 1136, 1153, 1170, 1186, 1202, 1218, 1232, 1247, 1260, 1273, 1285, 
//1296, 1307, 1316, 1325, 1333, 1340, 1346, 1351, 1355, 1358, 1361, 1362, 
//1362, 1361, 1359, 1357, 1353, 1348, 1343, 1336, 1329, 1320, 1311, 1301, 
//1290, 1279, 1266, 1253, 1239, 1225, 1210, 1194, 1178, 1161, 1144, 1126, 
//1108, 1090, 1072, 1053, 1035, 1016, 997, 978, 959, 941, 922, 904};

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


/* End of global variables */

#include "sineTable.h"

/* function prototypes */
uint16_t getAverage(uint16_t* data, uint8_t length);
int cycleProduct(uint16_t* adcSamples, uint16_t offset, uint16_t currentRef);
uint16_t phaseMatchFilter(uint16_t* adcSamples, uint16_t* currentAVG);
gainSetting_t gainCtrl(uint16_t* adcSamples, uint8_t externalVolt);
void meterInit();
void sampleCurrentWaveform();
// functions do not use in data dump mode
#ifndef DATADUMP
static void disable_all_ioc_override();
#endif

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
    disableAll();

    // read internal flash
    flash_data = REG(flash_calibration);

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
    #ifdef DATADUMP
    uint8_t inaGain;
    #else
    uint16_t currentRef;
    uint16_t calculatedPhase;
    static uint32_t phaseOffset_accum = 0;
    static uint16_t phaseOffset_array[CALIBRATION_CYCLES];
    static uint32_t dcOffset_accum = 0;
    int sqr;
    static uint32_t variance = 0;
    uint32_t tmp = 0;
    #endif

    rom_util_page_erase(flash_calibration, FLASH_ERASE_SIZE);
    triumviLEDON();
    etimer_set(&calibration_timer, CLOCK_SECOND*10);

    // enable LDO, release power gating
    meterSenseVREn(SENSE_ENABLE);
    meterSenseConfig(VOLTAGE, SENSE_ENABLE);
    meterSenseConfig(CURRENT, SENSE_ENABLE);

    while (1){
        PROCESS_YIELD();
        if (etimer_expired(&calibration_timer)){
            triumviLEDON();

            // Enable digital pot
            ad5274_init();
            ad5274_ctrl_reg_write(AD5274_REG_RDAC_RP);
            setINAGain(inaGainArr[inaGainIdx]);

            // Enable comparator interrupt
            GPIO_DETECT_RISING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
            meterVoltageComparator(SENSE_ENABLE);
            ungate_gpt(GPTIMER_1);
            REG(SYSTICK_STCTRL) &= (~SYSTICK_STCTRL_INTEN);

            // waiting for comparator interrupt
            while (referenceInt==0){}
            sampleCurrentWaveform();

            // resume systick
            REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
            gate_gpt(GPTIMER_1);

            // check INA gain setting
            gainSetting = gainCtrl(currentADCVal, 0x0);

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
                phaseOffset_accum += calculatedPhase;
                phaseOffset_array[cycleCnt] = calculatedPhase;
                dcOffset_accum += currentRef;
                #endif
                cycleCnt += 1;
                
                if (cycleCnt < CALIBRATION_CYCLES){
                    etimer_set(&calibration_timer, CLOCK_SECOND*0.1);
                }
                else{
                    #ifdef DATADUMP
                    triumviLEDToggle();
                    etimer_set(&calibration_timer, CLOCK_SECOND*0.1);
                    #else
                    meterSenseVREn(SENSE_DISABLE);
                    meterSenseConfig(VOLTAGE, SENSE_DISABLE);
                    meterSenseConfig(CURRENT, SENSE_DISABLE);

                    phaseOffset = phaseOffset_accum/CALIBRATION_CYCLES;
                    dcOffset = (dcOffset_accum/CALIBRATION_CYCLES);
                    
                    // calculate variance of phase offset array 
                    for (i=0; i<CALIBRATION_CYCLES; i++){
                        sqr = phaseOffset_array[i] - phaseOffset;
                        variance += (sqr*sqr);
                    }
                    variance /= CALIBRATION_CYCLES;

                    // cannot lock phase
                    if (variance > PHASE_VARIANCE_THRESHOLD){
                        while(1){}
                    }

                    // write to flash
                    tmp = (dcOffset<<16) | phaseOffset;
                    rom_util_program_flash(&tmp, flash_calibration, 4);
                    operation_mode = MODE_NORMAL;
                    process_start(&triumviProcess, NULL);
                    triumviLEDOFF();
                    break;
                    #endif
                }
            }
            else{
                etimer_set(&calibration_timer, CLOCK_SECOND*0.1);
            }
            triumviLEDOFF();
        }
        /*
        if (etimer_expired(&calibration_timer)){
            triumviLEDOFF();
            phaseOffset = phaseMatchFilter(adcSamples, &currentRef);
            dcOffset = currentRef;
            rom_util_program_flash(&tmp, flash_calibration, 4);
            process_start(&triumviProcess, NULL);
            triumviLEDON();
            break;
        }
        */
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
	static int powerValid;
	static uint16_t currentRef, voltRef;

	// Keep a counter of the number of samples we have taken
	// since we have been on. This will obviously get reset if we lose power.
	// We would have to store this counter in FRAM for it to be persistent.
	static uint32_t sampleCount = 0;

    unitReady();
    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND, 1, &rtimerEvent, NULL);
    myState = init;

    while(1){
        PROCESS_YIELD();
        switch (myState){
            // initialization state, check READYn is low before moving forward
            case STATE_INIT:
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                }
                if (allUnitsReady()){
                    meterSenseVREn(SENSE_ENABLE);
                    meterSenseConfig(VOLTAGE, SENSE_ENABLE);
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.4, 1, &rtimerEvent, NULL);
                    myState = STATE_WAITING_VOLTAGE_STABLE;
                    // consecutive 4 samples, decreases sampling interval
                    if (((backOffHistory&0x0f)==0x0f)&&(backOffTime>2))
                        backOffTime = (backOffTime>>1);
                    backOffHistory = (backOffHistory<<1) | 0x01;
                }
                // not all units are ready, go back to sleep
                else{
                    GPIO_DETECT_FALLING(TRIUMVI_RDYn_IN_GPIO_BASE, 0x1<<TRIUMVI_RDYn_IN_GPIO_PIN);
                    GPIO_ENABLE_INTERRUPT(TRIUMVI_RDYn_IN_GPIO_BASE, 0x1<<TRIUMVI_RDYn_IN_GPIO_PIN);
                    nvic_interrupt_enable(TRIUMVI_RDYn_IN_INT_NVIC_PORT);
                }
            break;

            // voltage is sattled, enable current measurement
            case STATE_WAITING_VOLTAGE_STABLE:
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                    meterSenseConfig(CURRENT, SENSE_ENABLE);
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.02, 1, &rtimerEvent, NULL);
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
                    uint16_t triumviStatusReg = 0x0000;
                    if (externalVoltSel())
                        triumviStatusReg |= EXTERNALVOLT_STATUSREG;
                    if (sampleCount == 0)
                        triumviStatusReg |= FIRSTSAMPLE_STATUSREG;
                    if (vcapLoopBack())
                        triumviStatusReg |= ((THREEPHASE_ID & 0x3)<<4);
                    #ifdef FRAM_WRITE
                    triumviStatusReg |= FRAMWRITE_STATUSREG;
                    #endif

                    // Check if configuration board is attached
                    if (batteryPackIsAttached())
                        triumviStatusReg |= BATTERYPACK_STATUSREG;

                    // Enable digital pot
                    ad5274_init();
                    ad5274_ctrl_reg_write(AD5274_REG_RDAC_RP);
                    setINAGain(inaGainArr[inaGainIdx]);

                    // Enable comparator interrupt
                    GPIO_DETECT_RISING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
                    meterVoltageComparator(SENSE_ENABLE);
                    ungate_gpt(GPTIMER_1);
                    REG(SYSTICK_STCTRL) &= (~SYSTICK_STCTRL_INTEN);

                    // Interrupted
                    while (referenceInt==0){}

                    // voltage sensing and comparator interrupt can be disabled first (disableALL())
                    avgPower = sampleAndCalculate(triumviStatusReg);

                    sampleCount++;
                    referenceInt = 0;

                    if (avgPower>=0){
                        // Write data into FRAM
                        #ifdef FRAM_WRITE
                        writeFRAM((uint16_t)(avgPower/1000), &rtctime);
                        #endif // FRAM_WRITE
                        nonceCounter = random_rand();
                        // Battery pack attached, turn it on ans samples switches
                        if (triumviStatusReg & BATTERYPACK_STATUSREG){
                            batteryPackVoltageEn(SENSE_ENABLE);
                            sx1509b_init();
                            sx1509b_high_voltage_input_enable(SX1509B_PORTA, 0x1, SX1509B_HIGH_INPUT_ENABLE);
                        }
                        encryptAndTransmit(triumviStatusReg, avgPower, myNonce, nonceCounter);
                        #endif // End of CALIBRATE
                        // First sample, blinks battery pack blue LED
                        if (batteryPackIsUSBAttached() &&
                            (triumviStatusReg & FIRSTSAMPLE_STATUSREG)){
                            batteryPackVoltageEn(SENSE_ENABLE);
                            batteryPackInit();
                            batteryPackLEDDriverInit();
                            batteryPackLEDOn(BATTERY_PACK_LED_BLUE);
                            myState = batteryPackLEDBlink;
                        }
                        else{
                            #ifndef CALIBRATE
                            triumviLEDON();
                            #endif
                            myState = triumviLEDBlink;
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
                        myState = triumviLEDBlink;
                    }
                }
            break;

            case batteryPackLEDBlink:
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                    batteryPackVoltageEn(SENSE_DISABLE);
                    i2c_disable(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, 
                                I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN); 
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
                    myState = init;
                }
            break;

            case triumviLEDBlink:
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                    triumviLEDOFF();
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
                    myState = init;
                }
            break;

            default:
            break;
        }
    }
    PROCESS_END();
    /*
    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND, 1, &rtimerEvent, NULL);
    while(1){
        PROCESS_YIELD();
        if (rTimerExpired==1){
            //printf("Mem addr: %x, value: %x\r\n", flash_calibration, REG(flash_calibration));
            rTimerExpired = 0;
            triumviLEDToggle();
            rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND, 1, &rtimerEvent, NULL);
            //printf("calculated phase offset: %u\r\n", phaseOffset);
            //printf("calculated DC level: %u\r\n", dcOffset);
        }
    }
    PROCESS_END();
    */
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
        prod[i] = cycleProduct(adcSamples, i, currentRef, 0x0);
        if (prod[i] > maxVal){
            maxVal = prod[i];
            maxVal_phaseOffset = i;
        }
    }
    *currentAVG = currentRef;
    return maxVal_phaseOffset;
}

int cycleProduct(uint16_t* adcSamples, uint16_t offset, uint16_t currentRef){
    uint8_t i;
    uint16_t tmp;
    uint8_t length = BUF_SIZE;
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
uint16_t getAverage(uint16_t* data, uint8_t length){
    uint8_t i;
    uint32_t sum = 0;
    for (i=0; i<length; i++)
        sum += data[i];
    return (uint16_t)(sum/length);
}

// Don't add/remove any lines in this subroutine
void sampleCurrentWaveform(){
	uint16_t sampleCnt = 0;
	uint16_t temp;
	while (sampleCnt < BUF_SIZE){
		timerVal[sampleCnt] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
		temp = adc_get(I_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_512);
		currentADCVal[sampleCnt] = ((temp>>4)>2047)? 0 : (temp>>4);
		sampleCnt++;
	}
}

void meterInit(){

    // GPIO default Input
    // Set all un-used pins to output and clear output
    #ifdef DATADUMP
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

    // Port C
    GPIO_SET_OUTPUT(GPIO_C_BASE, 0xeb);
    GPIO_CLR_PIN(GPIO_C_BASE, 0xeb);

    // Port D
    GPIO_SET_OUTPUT(GPIO_D_BASE, 0x1f);
    GPIO_SET_PIN(GPIO_D_BASE, 0x0f);
    GPIO_CLR_PIN(GPIO_D_BASE, 0x10);
    #ifndef DATADUMP
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
    //inaGainIdx = MAX_INA_GAIN_IDX;
    inaGainIdx = 3; // use larger gain setting

	backOffTime = 4;
	backOffHistory = 0;

}

// this function check if the ADC samples are within a proper range
gainSetting_t gainCtrl(uint16_t* adcSamples, uint8_t externalVolt){
    uint8_t i;
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
            currentRef = (dcOffset>>1);
            upperThreshold = (UPPERTHRESHOLD>>1);
            lowerThreshold= (LOWERTHRESHOLD>>1);
            length = BUF_SIZE2;
        }
        else{
            currentRef = dcOffset;
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
        adjustedADCSamples[i] = currentCal;
    }
    
    if ((maxVal < lowerThreshold) && (inaGainIdx<MAX_INA_GAIN_IDX)){
        res = GAIN_TOO_LOW;
        inaGainIdx += 1;
        setINAGain(inaGainArr[inaGainIdx]);
    }

    return res;
}

void encryptAndTransmit(uint8_t triumviStatusReg, int avgPower, uint8_t* myNonce, uint32_t nonceCounter){
	// 1 byte Identifier, 4 bytes nonce, 5~7 bytes payload, 4 byte MIC
	static uint8_t packetData[16];
	// 4 bytes reading, 1 byte status reg, (1 byte panel ID, 1 byte circuit ID optional)
	static uint8_t readingBuf[7];
	uint8_t* aData = myNonce;
	uint8_t* pData = readingBuf;
	uint8_t myMic[8] = {0x0};
	uint8_t myPDATA_LEN = PDATA_LEN;
	uint8_t packetLen = 14;
	uint16_t randBackOff;

	packetData[0] = TRIUMVI_PKT_IDENTIFIER;
	if (triumviStatusReg & BATTERYPACK_STATUSREG){
		readingBuf[PDATA_LEN] = batteryPackReadPanelID();
		readingBuf[PDATA_LEN+1] = batteryPackReadCircuitID();
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

    REG(RFCORE_XREG_TXPOWER) = 0xb6;    // set TX power to 0 dBm
	cc2538_on_and_transmit();
	CC2538_RF_CSP_ISRFOFF();
}

int sampleAndCalculate(uint8_t triumviStatusReg){
    int tempPower;
    int tempPower2 = 0;
    uint8_t i, j;
    uint8_t numOfCycles = 16;
    uint8_t numOfBitShift = 4; // log2(numOfCycles)
    uint16_t voltRef;
    int energyCal = 0;
    gainSetting_t gainSetting;
    if (triumviStatusReg & EXTERNALVOLT_STATUSREG){
        for (i=0; i<numOfCycles; i++){
            sampleCurrentVoltageWaveform();
            gainSetting = gainCtrl(currentADCVal, 0x0);
            if (gainSetting == GAIN_OK){
                // shift two samples
                voltADCVal[BUF_SIZE2] = voltADCVal[0];
                voltADCVal[BUF_SIZE2+1] = voltADCVal[1];
                voltRef = getAverage(voltADCVal, BUF_SIZE2);
                for (j=0; j<BUF_SIZE2; j++){
                    energyCal += currentDataTransform(adjustedADCSamples[i], 0x1)*voltDataTransform(voltData[i], voltRef);
                }
                tempPower = energyCal/BUF_SIZE2; // unit is mW
                // Fix phase oppsite down
                if (tempPower < 0)
                    tempPower = -1*tempPower;
                tempPower2 += tempPower;
            }
            else{
                disableAll();
                disablePOT();
                return -1
            }
        }
        disableAll();
        disablePOT();
        return (tempPower2>>numOfBitShift);
    }
    else{
        sampleCurrentWaveform();
        disableAll();
        gainSetting = gainCtrl(currentADCVal, 0x0);
        disablePOT();
        if (gainSetting==GAIN_OK){
            for (i=0; i<BUF_SIZE; i++){
                tmp = i*3 + phaseOffset;
                if (tmp>=360)
                    tmp -= 360;
                energyCal += currentDataTransform(adjustedADCSamples[i], 0x0)*stdSineTable[tmp];
            }
            tempPower = energyCal/BUF_SIZE; // unit is mW
            // Fix phase oppsite down
            if (tempPower < 0)
                tempPower = -1*tempPower;
            return tempPower
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
    float tmp;
    // 10-bit ADC
    if (externalVolt){
        tmp = currentReading*I_TRANSFORM*2/inaGain;
        return (int)tmp;
    }
    // 11-Bit ADC
    else{
        tmp = currentReading*I_TRANSFORM/inaGain; // ideal, completely linear
        return (int)tmp;
    }
}


#ifndef DATADUMP
static void disable_all_ioc_override() {
	uint8_t portnum = 0;
	uint8_t pinnum = 0;
	for(portnum = 0; portnum < 4; portnum++) {
		for(pinnum = 0; pinnum < 8; pinnum++) {
			ioc_set_over(portnum, pinnum, IOC_OVERRIDE_DIS);
		}
	}
}
#endif

// interrupt when all units are ready
static void unitReadyCallBack(uint8_t port, uint8_t pin){
	GPIO_DISABLE_INTERRUPT(TRIUMVI_RDYn_IN_GPIO_BASE, 0x1<<TRIUMVI_RDYn_IN_GPIO_PIN);
	nvic_interrupt_disable(TRIUMVI_RDYn_IN_INT_NVIC_PORT);
	process_poll(&triumviProcess);
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

