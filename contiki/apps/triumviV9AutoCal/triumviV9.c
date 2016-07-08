
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


#define FLASH_BASE 0x200000
#define FLASH_SIZE 0x80000 //rom_util_get_flash_size()
#define FLASH_ERASE_SIZE 0x800

// Adjusted (DC removal) ADC sample thresholds
#define UPPERTHRESHOLD  430 // any value above this, gain is too large
#define LOWERTHRESHOLD  185 // max value below this, gain is too small

// number of samples per cycle
#define BUF_SIZE 120    // sample current only, 11-bit resolution
#define BUF_SIZE2 113   // sample both current and voltage, 10-bit resolution

#define CALIBRATION_CYCLES 500


typedef enum {
    MODE_CALIBRATION,
    MODE_NORMAL
} triumvi_mode_t;

typedef enum {
    GAIN_TOO_LOW,
    GAIN_OK,
    GAIN_TOO_HIGH
} gainSetting_t;

/* global variables */

volatile uint32_t flash_calibration = FLASH_BASE - 2*FLASH_ERASE_SIZE + FLASH_SIZE;
volatile uint32_t flash_data;

int adjustedADCSamples[BUF_SIZE];
//uint16_t adcSamples[BUF_SIZE];

// only for test purpose
uint16_t adcSamples[] = {
886, 868, 851, 834, 818, 802, 786, 772, 757, 744, 731, 719, 
708, 697, 688, 679, 671, 664, 658, 653, 649, 646, 643, 642, 
642, 643, 645, 647, 651, 656, 661, 668, 675, 684, 693, 703, 
714, 725, 738, 751, 765, 779, 794, 810, 826, 843, 860, 878, 
896, 914, 932, 951, 969, 988, 1007, 1026, 1045, 1063, 1082, 1100, 
1118, 1136, 1153, 1170, 1186, 1202, 1218, 1232, 1247, 1260, 1273, 1285, 
1296, 1307, 1316, 1325, 1333, 1340, 1346, 1351, 1355, 1358, 1361, 1362, 
1362, 1361, 1359, 1357, 1353, 1348, 1343, 1336, 1329, 1320, 1311, 1301, 
1290, 1279, 1266, 1253, 1239, 1225, 1210, 1194, 1178, 1161, 1144, 1126, 
1108, 1090, 1072, 1053, 1035, 1016, 997, 978, 959, 941, 922, 904};

volatile uint16_t phaseOffset;
volatile uint16_t dcOffset;


volatile static triumvi_mode_t operation_mode;

static struct etimer calibration_timer;
static struct rtimer myRTimer;
volatile uint8_t rTimerExpired;
volatile uint8_t externalVolt = 0;


/* End of global variables */

#include "sineTable.h"

/* function prototypes */
uint16_t getAverage(uint16_t* data, uint8_t length);
int cycleProduct(uint16_t* adcSamples, uint16_t offset, uint16_t currentRef);
uint16_t phaseMatchFilter(uint16_t* adcSamples, uint16_t* currentAVG);
static void rtimerEvent(struct rtimer *t, void *ptr);

/*---------------------------------------------------------------------------*/
PROCESS(startupProcess, "Startup");
PROCESS(calibrationProcess, "Calibration");
PROCESS(triumviProcess, "Triumvi");
AUTOSTART_PROCESSES(&startupProcess);
/*---------------------------------------------------------------------------*/

// Startup process, check flash
PROCESS_THREAD(startupProcess, ev, data) {
    PROCESS_BEGIN();

    flash_data = REG(flash_calibration);

    if (flash_data==0x0){
        process_start(&triumviProcess, NULL);
        operation_mode = MODE_NORMAL;
    }
    else{
        process_start(&calibrationProcess, NULL);
        operation_mode = MODE_CALIBRATION;
    }
    PROCESS_END();
}

PROCESS_THREAD(calibrationProcess, ev, data) {
    PROCESS_BEGIN();

    rom_util_page_erase(flash_calibration, FLASH_ERASE_SIZE);
    triumviLEDON();
    etimer_set(&calibration_timer, CLOCK_SECOND*10);
    static uint32_t tmp = 0;
    static uint16_t currentRef;
    static uint16_t calibratePhaseOffsets[CALIBRATION_CYCLES];
    static uint16_t calibrateDCOffsets[CALIBRATION_CYCLES];
    uint16_t cycleCnt = 0;


    while (1){
        PROCESS_YIELD();
        if (etimer_expired(&calibration_timer)){
            triumviLEDOFF();
            phaseOffset = phaseMatchFilter(adcSamples, &currentRef);
            dcOffset = currentRef;
            rom_util_program_flash(&tmp, flash_calibration, 4);
            process_start(&triumviProcess, NULL);
            triumviLEDON();
            break;
        }
    }

    PROCESS_END();
}

PROCESS_THREAD(triumviProcess, ev, data) {
    PROCESS_BEGIN();
    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND, 1, &rtimerEvent, NULL);
    while(1){
        PROCESS_YIELD();
        if (rTimerExpired==1){
            //printf("Mem addr: %x, value: %x\r\n", flash_calibration, REG(flash_calibration));
            rTimerExpired = 0;
            triumviLEDToggle();
            rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND, 1, &rtimerEvent, NULL);
            printf("calculated phase offset: %u\r\n", phaseOffset);
            printf("calculated DC level: %u\r\n", dcOffset);
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
    uint8_t i;
    uint16_t tmp;
    uint8_t length = BUF_SIZE;
    int product = 0;
    if (operation_mode==MODE_CALIBRATION){
        for (i=0; i<length; i++){
            tmp = i*3 + offset;
            if (tmp>=360)
                tmp -= 360;
            product += (adcSamples[i] - currentRef)*stdSineTable[tmp];
        }
    }
    // use data from adjustedADCSamples (DC removal)
    else if (externalVolt==0){
        for (i=0; i<length; i++){
            tmp = i*3 + offset;
            if (tmp>=360)
                tmp -= 360;
            product += adjustedADCSamples[i]*stdSineTable[tmp];
        }
    }
    // use external voltage waveform
    else{
        length = BUF_SIZE2;
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

/*
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
            currentRef = (HARD_REF>>1);
            upperThreshold = (UPPERTHRESHOLD>>1);
            lowerThreshold= (LOWERTHRESHOLD>>1);
            length = BUF_SIZE2;
        }
        else{
            currentRef = HARD_REF;
        }
    }

    // loop the entire samples, substract offset and update max ADC value
    for (i=0; i<length; i++){
        currentCal = adcSamples[i] - currentRef;
        if ((currentCal>upperThreshold)&&(inaGainIdx>MIN_INA_GAIN_IDX)){
            res = GAIN_TOO_HIGH;
            return res;
        }
        if (currentCal > maxVal){
            maxVal = currentcal;
        }
        adjustedADCSamples[i] = currentCal;
    }
    
    if ((maxVal < lowerThreshold) && (inaGainIdx<MAX_INA_GAIN_IDX)){
        res = GAIN_TOO_LOW;
    }

    return res;
}
*/

