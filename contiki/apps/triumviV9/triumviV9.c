
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
#include "cc2538-rf.h"
#include "fm25v02.h"
#include "triumvi.h"
#include "sx1509b.h"
#include "ad5274.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


static struct rtimer myRTimer;

PROCESS(triumviProcess, "Triumvi");
AUTOSTART_PROCESSES(&triumviProcess);


volatile uint8_t rTimerExpired;
volatile uint8_t referenceInt;
volatile uint8_t backOffTime;
volatile uint8_t backOffHistory;
volatile uint8_t inaGainIdx;

#define MAX_INA_GAIN_IDX 4
#define MIN_INA_GAIN_IDX 1
const uint8_t inaGainArr[6] = {1, 2, 3, 5, 9, 17};

rv3049_time_t rtctime;
#ifdef CALIBRATE
static uint8_t calibrate_cnt = 0;
#endif

#define BUF_SIZE 120
#define BUF_SIZE2 113 // for external voltage input
uint16_t currentADCVal[BUF_SIZE];
uint16_t voltADCVal[BUF_SIZE2+2]; // compensate the voltage sample offset
uint32_t timerVal[BUF_SIZE];

#include "calibrateData.h"



/*
current can be calculated as following:
  Ip                           ADC val(adjusted)
------ x R(shunt) x G(INA) = -------------------- x ADC Ref
 G(CT)                        MAX_ADC_VAL

G(CT) = 3000
R(shunt) = 90.9
MAX_ADC_VAL = 2048 (11 bits)
ADC Ref = 3

Ip = 48.34468 / G(INA) unit is mA
*/

#define I_TRANSFORM 48.34468
#define I_TRANSFORM_WO_CT 0.016115

#define POLYFIT_NEG_COEF1 3122
#define POLYFIT_NEG_COEF0 (-27)
#define POLYFIT_POS_COEF1 3032
#define POLYFIT_POS_COEF0 74

#define FIRSTSAMPLE_STATUSREG  0x0100
#define EXTERNALVOLT_STATUSREG 0x0080
#define BATTERYPACK_STATUSREG  0x0040
#define THREEPHASE_STATUSREG   0x0030
#define FRAMWRITE_STATUSREG    0x0008

#define UPPERTHRESHOLD  430
#define LOWERTHRESHOLD  185


// Function prototypes
static void referenceIntCallBack(uint8_t port, uint8_t pin);
void sampleCurrentWaveform();
void sampleCurrentVoltageWaveform();
static void rtimerEvent(struct rtimer *t, void *ptr);
void disableAll();
void meterInit();
int currentVoltProcess(uint16_t *currentData, uint16_t* voltData,
						uint16_t currentRef, uint16_t voltRef,
						int *power, uint8_t externalVolt);
void packData(uint8_t* dest, int reading);
uint16_t voltDataAverge(uint16_t* voltData);
int voltDataTransform(uint16_t voltReading, uint16_t voltReference);
int currentDataTransform(int currentReading, uint8_t externalVolt);
int sampleAndCalculate(uint8_t triumviStatusReg, int* avgPower, uint16_t* currentRef, uint16_t* voltRef);
void encryptAndTransmit(uint8_t triumviStatusReg, int avgPower, uint8_t* myNonce, uint32_t nonceCounter);

#ifndef CALIBRATE
static void disable_all_ioc_override();
#else
void printCalibrationValuse(uint8_t triumviStatusReg, uint16_t currentRef, uint16_t voltRef);
#endif
#ifdef FRAM_WRITE
inline void writeFRAM(uint16_t powerRead, rv3049_time_t* rtctime);
#endif
// End of prototypes



typedef enum state{
	init,
	waitingVoltageStable,
	waitingComparatorStable,
	batteryPackLEDBlink,
    triumviLEDBlink
} state_t;

volatile static state_t myState;


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(triumviProcess, ev, data)
{
	PROCESS_BEGIN();
	random_init(0);
	CC2538_RF_CSP_ISRFOFF();
	fm25v02_sleep();
	disableSPI();

	#ifdef ERASE_FRAM
	fm25v02_eraseAll();
	#endif

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

    meterInit();
    if (isButtonPressed()){
        triumviFramPtrClear();
        triumviLEDON();
    }
    disableAll();

    unitReady();
    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.1, 1, &rtimerEvent, NULL);
    myState = init;

    while(1){
        PROCESS_YIELD();
        switch (myState){
            // initialization state, check READYn is low before moving forward
            case init:
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                }
                if (allUnitsReady()){
                    meterSenseVREn(SENSE_ENABLE);
                    meterSenseConfig(VOLTAGE, SENSE_ENABLE);
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.4, 1, &rtimerEvent, NULL);
                    myState = waitingVoltageStable;
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
            case waitingVoltageStable:
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                    meterSenseConfig(CURRENT, SENSE_ENABLE);
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.04, 1, &rtimerEvent, NULL);
                    myState = waitingComparatorStable;
                }
            break;

            case waitingComparatorStable:
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

                    // Turn on the battery pack's power
                    if (batteryPackIsAttached())
                        triumviStatusReg |= BATTERYPACK_STATUSREG;

                    // Enable digital pot
                    ad5274_init();
                    ad5274_ctrl_reg_write(AD5274_REG_RDAC_RP);
                    setINAGain(inaGainArr[inaGainIdx]);

                    //// Enable comparator interrupt
                    GPIO_DETECT_RISING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
                    meterVoltageComparator(SENSE_ENABLE);
                    ungate_gpt(GPTIMER_1);
                    REG(SYSTICK_STCTRL) &= (~SYSTICK_STCTRL_INTEN);

                    // Interrupted
                    while (referenceInt==0){}

                    // voltage sensing and comparator interrupt can be disabled first (disableALL())
                    powerValid = sampleAndCalculate(triumviStatusReg, &avgPower, &currentRef, &voltRef);

                    // current sensing can be disabled after adjusting the digital POT
                    meterSenseConfig(CURRENT, SENSE_DISABLE);
                    meterSenseVREn(SENSE_DISABLE);

                    sampleCount++;
                    referenceInt = 0;

                    if (powerValid>0){
                        #ifdef CALIBRATE
                        printCalibrationValuse(triumviStatusReg, currentRef, voltRef);
                        #else // not define calibrate
                        // Write data into FRAM
                        #ifdef FRAM_WRITE
                        writeFRAM((uint16_t)(avgPower/1000), &rtctime);
                        #endif // FRAM_WRITE
                        nonceCounter = random_rand();
                        // Battery pack attached, turn it on ans samples switches
                        if (triumviStatusReg & BATTERYPACK_STATUSREG){
                            batteryPackVoltageEn(SENSE_ENABLE);
                            batteryPackInit();
                        }
                        encryptAndTransmit(triumviStatusReg, avgPower, myNonce, nonceCounter);
                        #endif // End of CALIBRATE
                        // First sample, blinks battery pack blue LED
                        if ((triumviStatusReg & BATTERYPACK_STATUSREG) &&
                            (triumviStatusReg & FIRSTSAMPLE_STATUSREG) && 
                            (batteryPackIsUSBAttached())){
                            batteryPackLEDDriverInit();
                            batteryPackLEDOn(BATTERY_PACK_LED_BLUE);
                            myState = batteryPackLEDBlink;
                        }
                        else{
                            if (triumviStatusReg & BATTERYPACK_STATUSREG){
                                batteryPackVoltageEn(SENSE_DISABLE);
                                disableI2C();
                            }
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
                            disableI2C();
                        }
                        rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.1, 1, &rtimerEvent, NULL);
                        myState = triumviLEDBlink;
                    }
                }
            break;

            case batteryPackLEDBlink:
                if (rTimerExpired==1){
                    rTimerExpired = 0;
                    sx1509b_init();
                    batteryPackLEDOff(BATTERY_PACK_LED_BLUE);
                    batteryPackLEDDriverDisable();
                    rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
                    batteryPackVoltageEn(SENSE_DISABLE);
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
}

// Don't add/remove any lines in this subroutine
static void referenceIntCallBack(uint8_t port, uint8_t pin){
	meterVoltageComparator(SENSE_DISABLE);
	referenceInt = 1;
	process_poll(&triumviProcess);
}

// interrupt when all units are ready
static void unitReadyCallBack(uint8_t port, uint8_t pin){
	GPIO_DISABLE_INTERRUPT(TRIUMVI_RDYn_IN_GPIO_BASE, 0x1<<TRIUMVI_RDYn_IN_GPIO_PIN);
	nvic_interrupt_disable(TRIUMVI_RDYn_IN_INT_NVIC_PORT);
	process_poll(&triumviProcess);
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

void sampleCurrentVoltageWaveform(){
	uint16_t sampleCnt = 0;
	uint16_t temp;
	while (sampleCnt < BUF_SIZE2){
		timerVal[sampleCnt] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
		temp = adc_get(I_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_256);
		currentADCVal[sampleCnt] = ((temp>>5)>1023)? 0 : (temp>>5);
		temp = adc_get(EXT_VOLT_IN_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_256);
		voltADCVal[sampleCnt] = ((temp>>5)>1023)? 0 : (temp>>5);
		sampleCnt++;
	}
}

static void rtimerEvent(struct rtimer *t, void *ptr){
	rTimerExpired = 1;
	process_poll(&triumviProcess);
}


void disableAll(){
	REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
	meterSenseConfig(VOLTAGE, SENSE_DISABLE);
	meterVoltageComparator(SENSE_DISABLE);
	gate_gpt(GPTIMER_1);
}

void meterInit(){

    // GPIO default Input
    // Set all un-used pins to output and clear output
    #ifdef CALIBRATE
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
    #ifndef CALIBRATE
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
    inaGainIdx = MAX_INA_GAIN_IDX;

	#ifdef CALIBRATE
	backOffTime = 1;
	#else
	backOffTime = 4;
	#endif
	backOffHistory = 0;

}

int currentVoltProcess(uint16_t *currentData, uint16_t* voltData,
						uint16_t currentRef, uint16_t voltRef,
						int *power, uint8_t externalVolt){
	uint16_t i;
	int currentCal;
	int energyCal = 0;
	float avgPower;
	int maxADCValue = 0;
	uint8_t bufSize;
	int upperThreshold;
	uint8_t lowerThreshold;

	if (externalVolt){
		upperThreshold = (UPPERTHRESHOLD>>1);
		lowerThreshold= (LOWERTHRESHOLD>>1);
		bufSize = BUF_SIZE2;
	}
	else{
		upperThreshold = UPPERTHRESHOLD;
		lowerThreshold = LOWERTHRESHOLD;
		bufSize = BUF_SIZE;
	}

	for (i=0;i<bufSize;i++){
		currentCal = currentData[i] - currentRef;
		if (currentCal > maxADCValue)
			maxADCValue = currentCal;
		if ((currentCal>upperThreshold)&&(inaGainIdx>MIN_INA_GAIN_IDX)){
            inaGainIdx -= 1;
            setINAGain(inaGainArr[inaGainIdx]);
			return -1;
		}
		if (externalVolt)
			energyCal += currentDataTransform(currentCal, 0x1)*voltDataTransform(voltData[i], voltRef);
		else
			energyCal += currentDataTransform(currentCal, 0x0)*sineTable[i];
	}
	if ((maxADCValue < lowerThreshold) && (inaGainIdx<MAX_INA_GAIN_IDX)){
        inaGainIdx += 1;
        setINAGain(inaGainArr[inaGainIdx]);
		return -1;
	}
	else{
		avgPower = energyCal/bufSize; // unit is mW
		// Fix phase oppsite down
		if (avgPower < 0)
			avgPower = -1*avgPower;
		return 1;
	}
}

void packData(uint8_t* dest, int reading){
	uint8_t i;
	for (i=0; i<4; i++){
		dest[i] = (reading&(0xff<<(i<<3)))>>(i<<3);
	}
}

uint16_t voltDataAverge(uint16_t* voltData){
	uint32_t sum = 0;
	uint8_t i;
	for (i=0; i<BUF_SIZE2; i++)
		sum += voltData[i];
	return (uint16_t)(sum/BUF_SIZE2);
}

int voltDataTransform(uint16_t voltReading, uint16_t voltReference){
	float voltageScaling = 1.262;
	return (int)((voltReading - voltReference)*voltageScaling);
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
        if (currentReading < 0)
            tmp = (currentReading*POLYFIT_NEG_COEF1 + POLYFIT_NEG_COEF0)*I_TRANSFORM_WO_CT/inaGain;
        else
            tmp = (currentReading*POLYFIT_POS_COEF1 + POLYFIT_POS_COEF0)*I_TRANSFORM_WO_CT/inaGain;
        //float tmp = currentReading*I_TRANSFORM/inaGain;
        return (int)tmp;
    }
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

	cc2538_on_and_transmit();
	CC2538_RF_CSP_ISRFOFF();
}

int sampleAndCalculate(uint8_t triumviStatusReg, int* avgPower, uint16_t* currentRef, uint16_t* voltRef){
	int tempPower;
	int tempPower2 = 0;
	uint8_t i;
	uint8_t numOfCycles = 16;
	uint8_t numOfBitShift = 4; // log2(numOfCycles)
	uint16_t tempADCReading;
	if (triumviStatusReg & EXTERNALVOLT_STATUSREG){
		#ifdef CALIBRATE
		numOfCycles = 1;
		numOfBitShift = 0; // log2(numOfCycles)
		#endif
		for (i=0; i<numOfCycles; i++){
			sampleCurrentVoltageWaveform();
			tempADCReading = adc_get(V_REF_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_256);
			*currentRef = ((tempADCReading>>5)>1023)? 0 : (tempADCReading>>5);
			*voltRef = voltDataAverge(voltADCVal);
			// shift two samples
			voltADCVal[BUF_SIZE2] = voltADCVal[0];
			voltADCVal[BUF_SIZE2+1] = voltADCVal[1];
			if (currentVoltProcess(currentADCVal, voltADCVal+2, *currentRef, *voltRef, &tempPower, 0x01)<0)
				return -1;
			else
				tempPower2 += tempPower;
		}
		*avgPower = tempPower2>>numOfBitShift;
		return 1;
	}
	else{
		sampleCurrentWaveform();
		tempADCReading = adc_get(V_REF_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_512);
		*currentRef = ((tempADCReading>>4)>2047)? 0 : (tempADCReading>>4);
		disableAll();
		return currentVoltProcess(currentADCVal, NULL, *currentRef, 0, avgPower, 0x00);
	}
}

#ifdef FRAM_WRITE
inline void writeFRAM(uint16_t powerRead, rv3049_time_t* rtctime){
	reenableSPI();
	rv3049_read_time(rtctime);
	triumviFramWrite(powerRead, rtctime);
	disableSPI();
}
#endif


#ifndef CALIBRATE
static void disable_all_ioc_override() {
	uint8_t portnum = 0;
	uint8_t pinnum = 0;
	for(portnum = 0; portnum < 4; portnum++) {
		for(pinnum = 0; pinnum < 8; pinnum++) {
			ioc_set_over(portnum, pinnum, IOC_OVERRIDE_DIS);
		}
	}
}
#else
void printCalibrationValuse(uint8_t triumviStatusReg, uint16_t currentRef, uint16_t voltRef){
	uint8_t i;
    uint8_t inaGain = inaGainArr[inaGainIdx];
	if (calibrate_cnt<255)
		calibrate_cnt++;
	else
		triumviLEDToggle();
	printf("ADC reference: %u\r\n", currentRef);
	printf("Time difference: %lu\r\n", (timerVal[0]-timerVal[1]));
	printf("INA Gain: %u\r\n", inaGain);
	if (triumviStatusReg & EXTERNALVOLT_STATUSREG){
		for (i=0; i<BUF_SIZE2; i+=1){
			printf("Current reading: %d (mA) Voltage Reading: %d (V)\r\n",
			currentDataTransform(currentADCVal[i]-currentRef, 0x1), voltDataTransform(voltADCVal[i+2], voltRef));
			//currentADCVal[i]-currentRef, voltDataTransform(voltADCVal[i], voltRef));
		}
		//printf("Calculated Power: %d\r\n", avgPower);
	}
	else{
		for (i=0; i<BUF_SIZE; i+=1){
			//printf("Current reading: %d\r\n", currentDataTransform(currentADCVal[i]-currentRef, inaGain, 0x0));
			//printf("Current reading: %d\r\n", currentADCVal[i]-currentRef);
			printf("Current reading: %d\r\n", currentADCVal[i]);
		}
	}

}
#endif


/*---------------------------------------------------------------------------*/
