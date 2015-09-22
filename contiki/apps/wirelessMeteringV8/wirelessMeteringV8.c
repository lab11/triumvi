
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


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


static struct rtimer myRTimer;

PROCESS(wirelessMeterProcessing, "Wireless Metering Process");
AUTOSTART_PROCESSES(&wirelessMeterProcessing);


volatile uint8_t rTimerExpired;
volatile uint8_t referenceInt;
volatile uint8_t backOffTime;
volatile uint8_t backOffHistory;
rv3049_time_t rtctime;

#define BUF_SIZE 120
#define BUF_SIZE2 113 // for external voltage input
uint16_t currentADCVal[BUF_SIZE];
uint16_t voltADCVal[BUF_SIZE2];
uint32_t timerVal[BUF_SIZE];

#include "calibrateData.h"


#define CURVE_FIT

/*
// 3.0078 degree / sample, DC = 170
#define ADC_MAX_VAL 2047
#define ADC_REF	3
#define INA_GAIN 2
#define CT_GAIN 0.00033333
#define SHUNT_RESISTOR 90.9

current can be calculated as following:
I = (data - (vRef/adcRef)*adcMax)/adcMax*adcRef/inaGain/shuntResistor/ctGain
unit is A, multiply by 1000 gets mA
*/

#define I_TRANSFORM 48.36830 // 1000/ADC_MAX_VAL*ADC_REF/SHUNT_RESISTOR/CT_GAIN, unit is mA
#define P_TRANSFORM 0.403069 // I_TRANSFORM/TABLE_SIZE, unit is mW
//#define I_TRANSFORM 16.11489 // 1000/ADC_MAX_VAL*ADC_REF/SHUNT_RESISTOR/CT_GAIN, unit is mA
//#define P_TRANSFORM 0.134291 // I_TRANSFORM/TABLE_SIZE, unit is mW

#define P_TRANSFORM2 0.855658

// The following data is accuired from 98% tile
// y = POLY_NEG_OR1*x + POLY_NEG_OR0
#define POLY_NEG_OR1 1.01
#define POLY_NEG_OR0 2.42

// y = POLY_POS_OR1*x + POLY_POS_OR0
#define POLY_POS_OR1 0.93
#define POLY_POS_OR0 2.32



// Function prototypes
static void referenceIntCallBack(uint8_t port, uint8_t pin);
void sampleCurrentWaveform();
void sampleCurrentVoltageWaveform();
static void rtimerEvent(struct rtimer *t, void *ptr);
void disableAll();
void meterInit();
//int currentProcess(uint16_t *data, uint16_t vRefADCVal, int *power);
int currentVoltProcess(uint16_t *currentData, uint16_t* voltData, 
						uint16_t currentRef, uint16_t voltRef, 
						int *power, uint8_t externalVoltPresent);
inline int getThreshold(uint8_t inaGain);
void packData(uint8_t* dest, int reading);
uint16_t voltDataAverge(uint16_t* voltData);
int voltDataTransform(uint16_t voltReading, uint16_t voltReference);
static void disable_all_ioc_override();
// End of prototypes



typedef enum state{
	init,
	waitingVoltageStable,
	#ifdef COMPARATOR_NEGEDGE
	waitingComparatorStable2,
	waitingComparatorStable1,
	#endif
	waitingComparatorStable,
	waitingRadioInt,
	#ifdef BLINK_LED
	ledBlink,
	#endif
	nullState
} state_t;

volatile static state_t myState;


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wirelessMeterProcessing, ev, data)
{
	PROCESS_BEGIN();
	CC2538_RF_CSP_ISRFOFF();
	fm25v02_sleep();
	disableSPI();

	#ifdef CLEAR_FRAM
	//fm25v02_eraseAll();
	triumviFramClear();
	#endif

	#ifdef CALIBRATE
	static uint8_t calibrate_cnt = 0;
	#else
	static uint8_t meterData[13];
	#ifdef AES_ENABLE
	// packet preprocessing
	static uint8_t readingBuf[4];
	static uint8_t extAddr[8];
    NETSTACK_RADIO.get_object(RADIO_PARAM_64BIT_ADDR, extAddr, 8);
	
	// AES Preprocessing
	static uint8_t aesKey[] = AES_KEY;
	static uint8_t myMic[8] = {0x0};
	static uint8_t myNonce[13] = {0};
	memcpy(myNonce, extAddr, 8);
	static uint32_t nonceCounter = 0;
	aes_load_keys(aesKey, AES_KEY_STORE_SIZE_KEY_SIZE_128, 1, 0);

	static uint8_t* aData = myNonce;
	static uint8_t* pData = readingBuf;
	#endif
	#endif
	static int avgPower;
	static int powerValid;
	static uint8_t inaGain;
	static uint16_t currentRef, voltRef;

	
	meterInit();
	inaGain = getINAGain();
	setINAGain(inaGainArr[0]);
	disableAll();
	#ifdef START_IMMEDIATELY
	meterSenseConfig(VOLTAGE, SENSE_ENABLE);
	rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.4, 1, &rtimerEvent, NULL);
	myState = waitingVoltageStable;
	#endif

	while(1){
		PROCESS_YIELD();
		switch (myState){
			// initialization state, release voltage measurement gating
			case init:
				if (rTimerExpired==1){
					rTimerExpired = 0;
					meterSenseConfig(VOLTAGE, SENSE_ENABLE);
					rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.4, 1, &rtimerEvent, NULL);
					myState = waitingVoltageStable;
					// consecutive 4 samples, decreases sampling interval
					if (((backOffHistory&0x0f)==0x0f)&&(backOffTime>2))
						backOffTime = (backOffTime>>1);
					backOffHistory = (backOffHistory<<1) | 0x01;
				}
			break;

			// voltage is sattled, enable current measurement
			case waitingVoltageStable:
				if (rTimerExpired==1){
					rTimerExpired = 0;
					meterSenseConfig(CURRENT, SENSE_ENABLE);
					meterMUXConfig(SENSE_ENABLE);
					setINAGain(inaGain);
					rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.04, 1, &rtimerEvent, NULL);
					#ifdef COMPARATOR_NEGEDGE
					myState = waitingComparatorStable2;
					#else
					myState = waitingComparatorStable;
					#endif
				}
			break;

			#ifdef COMPARATOR_NEGEDGE
			case waitingComparatorStable2:
				if (rTimerExpired==1){
					rTimerExpired = 0;
					GPIO_DETECT_FALLING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
					meterVoltageComparator(SENSE_ENABLE);
					myState = waitingComparatorStable1;
				}
			break;

			case waitingComparatorStable1:
				if (referenceInt==1){
					referenceInt = 0;
					rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.007, 1, &rtimerEvent, NULL);
					myState = waitingComparatorStable;
				}
			break;
			#endif

			case waitingComparatorStable:
				if (rTimerExpired==1){
					rTimerExpired = 0;
					GPIO_DETECT_RISING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
					meterVoltageComparator(SENSE_ENABLE);
					ungate_gpt(GPTIMER_1);
					REG(SYSTICK_STCTRL) &= (~SYSTICK_STCTRL_INTEN);
					while (referenceInt==0){}
					if (externalVoltSel()){
						uint8_t i = 0;
						int tempPower;
						avgPower = 0;
						while (i<16){
							sampleCurrentVoltageWaveform();
							currentRef = adc_get(V_REF_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_256);
							currentRef = ((currentRef>>5)>1023)? 0 : (currentRef>>5);
							voltRef = voltDataAverge(voltADCVal);
							powerValid = currentVoltProcess(currentADCVal, voltADCVal, currentRef, voltRef, &tempPower, 0x01);
							if (powerValid>0){
								i++;
								avgPower += tempPower;
							}
						}
						avgPower = (avgPower>>4);
					}
					else{
						sampleCurrentWaveform();
						currentRef = adc_get(V_REF_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_512);
						currentRef = ((currentRef>>4)>2047)? 0 : (currentRef>>4);
						disableAll();
						powerValid = currentVoltProcess(currentADCVal, NULL, currentRef, 0, &avgPower, 0x00);
					}
					referenceInt = 0;
					inaGain = getINAGain();
					setINAGain(inaGainArr[0]);
					if (powerValid>0){
						#ifdef CALIBRATE
						uint8_t i;
						if (calibrate_cnt<255)
							calibrate_cnt++;
						else
							triumviLEDToggle();
						printf("ADC reference: %u\r\n", currentADCVal);
						printf("Time difference: %lu\r\n", (timerVal[0]-timerVal[1]));
						printf("INA Gain: %u\r\n", inaGain);
						if (externalVoltSel()){
							for (i=0; i<BUF_SIZE2; i+=1){
								printf("ADC reading: %u Voltage Reading: %d\r\n", currentADCVal[i], voltDataTransform(voltADCVal[i], voltRef));
							}
						}
						else{
							for (i=0; i<BUF_SIZE; i+=1){
								printf("ADC reading: %u\r\n", currentADCVal[i]);
							}
						}
						#else
						// Write data into FRAM
						#ifdef FRAM_WRITE
						reenableSPI();
						uint16_t powerRead = avgPower/1000;
						rv3049_read_time(&rtctime);
						triumviFramWrite(powerRead, &rtctime);
						disableSPI();
						// Send data over RF
						#else
						meterData[0] = AES_PKT_IDENTIFIER;
						packData(&myNonce[9], nonceCounter);
						packData(readingBuf, avgPower);
						packData(&meterData[1], nonceCounter);
						ccm_auth_encrypt_start(LEN_LEN, 0, myNonce, aData, ADATA_LEN, 
										pData, PDATA_LEN, MIC_LEN, NULL);
						nonceCounter += 1;
						while(ccm_auth_encrypt_check_status()!=AES_CTRL_INT_STAT_RESULT_AV){}
						ccm_auth_encrypt_get_result(myMic, MIC_LEN);
						memcpy(&meterData[5], readingBuf, PDATA_LEN);
						memcpy(&meterData[9], myMic, MIC_LEN);
						packetbuf_copyfrom(meterData, 13);
						cc2538_on_and_transmit();
						CC2538_RF_CSP_ISRFOFF();
						#endif
						#endif
						#ifdef BLINK_LED
						triumviLEDON();
						rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.001, 1, &rtimerEvent, NULL);
						myState = ledBlink;
						#else
						rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
						myState = init;
						#endif
					}
					// improper gain setting, try after 4 s
					else{
						rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*4, 1, &rtimerEvent, NULL);
						myState = init;
					}
				}
			break;

			#ifdef BLINK_LED
			case ledBlink:
				if (rTimerExpired==1){
					rTimerExpired = 0;
					triumviLEDOFF();
					rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
					myState = init;
				}
			break;
			#endif

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
	process_poll(&wirelessMeterProcessing);
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
	process_poll(&wirelessMeterProcessing);
}


void disableAll(){
	REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
	meterSenseConfig(CURRENT, SENSE_DISABLE);
	meterSenseConfig(VOLTAGE, SENSE_DISABLE);
	meterVoltageComparator(SENSE_DISABLE);
	meterMUXConfig(SENSE_DISABLE);
	gate_gpt(GPTIMER_1);
}

void meterInit(){

	// Set all un-used pins to output and clear output
	#ifndef CALIBRATE
	GPIO_SET_OUTPUT(GPIO_A_BASE, 0x47);
	GPIO_CLR_PIN(GPIO_A_BASE, 0x47);
	GPIO_SET_OUTPUT(GPIO_B_BASE, 0x86);
	GPIO_CLR_PIN(GPIO_B_BASE, 0x86);
	GPIO_SET_OUTPUT(GPIO_C_BASE, 0x08);
	GPIO_CLR_PIN(GPIO_C_BASE, 0x08);
	GPIO_SET_OUTPUT(GPIO_D_BASE, 0x1f);
	GPIO_SET_PIN(GPIO_D_BASE, 0x0f);
	GPIO_CLR_PIN(GPIO_D_BASE, 0x10);
	// disable all pull-up resistors
	disable_all_ioc_override();
	#endif

	// ADC for current sense
	adc_init();

	ioc_set_over(I_ADC_GPIO_NUM, I_ADC_GPIO_PIN, IOC_OVERRIDE_ANA);
	ioc_set_over(V_REF_ADC_GPIO_NUM, V_REF_ADC_GPIO_PIN, IOC_OVERRIDE_ANA);

	// External voltage waveform inputs
	ioc_set_over(EXT_VOLT_IN_GPIO_NUM, EXT_VOLT_IN_GPIO_PIN, IOC_OVERRIDE_ANA);
	GPIO_SET_INPUT(EXT_VOLT_IN_SEL_GPIO_BASE, 0x1<<EXT_VOLT_IN_SEL_GPIO_PIN);

	// GPIO for Voltage/Current measurement
	GPIO_SET_OUTPUT(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
	GPIO_CLR_PIN(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
	GPIO_SET_OUTPUT(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);
	GPIO_CLR_PIN(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);

	// GPIO for MUX (select INA gain)
	GPIO_SET_OUTPUT(MUX_IO_GPIO_BASE, 0x1<<MUX_EN_GPIO_PIN);
	GPIO_SET_OUTPUT(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN);
	GPIO_SET_OUTPUT(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN);
	GPIO_CLR_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_EN_GPIO_PIN);
	setINAGain(inaGainArr[MAX_INA_GAIN_IDX]);

	// Set voltage reference crossing as input
	// Disable pull up/down resistor
	// Enable interrupt on rising edge
	GPIO_SET_INPUT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	ioc_set_over(V_REF_CROSS_INT_GPIO_NUM, V_REF_CROSS_INT_GPIO_PIN, IOC_OVERRIDE_DIS);
	GPIO_DETECT_EDGE(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	GPIO_TRIGGER_SINGLE_EDGE(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	// interrupt
	gpio_register_callback(referenceIntCallBack, V_REF_CROSS_INT_GPIO_NUM, V_REF_CROSS_INT_GPIO_PIN);
	GPIO_DISABLE_INTERRUPT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	nvic_interrupt_disable(V_REF_CROSS_INT_NVIC_PORT);

	// timer1, used for counting samples
	ungate_gpt(GPTIMER_1);
	gpt_set_mode(GPTIMER_1, GPTIMER_SUBTIMER_A, GPTIMER_TAMR_TAMR_PERIODIC);
	gpt_set_count_dir(GPTIMER_1, GPTIMER_SUBTIMER_A, GPTIMER_TnMR_TnCDIR_COUNT_DOWN);
	gpt_set_interval_value(GPTIMER_1, GPTIMER_SUBTIMER_A, 0xffffffff);
	gpt_enable_event(GPTIMER_1, GPTIMER_SUBTIMER_A);
	gate_gpt(GPTIMER_1);

	rTimerExpired = 0;
	referenceInt = 0;

	#ifdef CALIBRATE
	backOffTime = 1;
	#else
	backOffTime = 16;
	#endif
	backOffHistory = 0;

	#ifndef START_IMMEDIATELY
	myState = init;
	rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*1, 1, &rtimerEvent, NULL);
	#endif
}

inline int getThreshold(uint8_t inaGain){
	if (inaGain==10)
		return 500;
	else
		return 650;
}

int currentVoltProcess(uint16_t *currentData, uint16_t* voltData, 
						uint16_t currentRef, uint16_t voltRef, 
						int *power, uint8_t externalVoltPresent){
	uint16_t i;
	int currentCal;
	int energyCal = 0;
	float avgPower;
	int maxADCValue = 0;
	uint8_t inaGain = getINAGain();
	int upperThreshold; 
	uint8_t bufSize;
	uint8_t currentADCLowerBound;


	if (externalVoltPresent){
		upperThreshold = getThreshold(inaGain)>>1;
		bufSize = BUF_SIZE2;
		currentADCLowerBound = 90;
	}
	else{
		upperThreshold = getThreshold(inaGain);
		bufSize = BUF_SIZE;
		currentADCLowerBound = 180;
	}

	for (i=0;i<bufSize;i++){
		currentCal = currentData[i] - currentRef;
		if (currentCal > maxADCValue)
			maxADCValue = currentCal;
		if ((currentCal>upperThreshold)&&(inaGain>MIN_INA_GAIN)){
			decreaseINAGain();
			return -1;
		}
		// Use linear regression
		#ifdef CURVE_FIT
		float temp;
		if (currentCal<0)
			temp = currentCal*POLY_NEG_OR1 + POLY_NEG_OR0;
		else
			temp = currentCal*POLY_POS_OR1 + POLY_POS_OR0;
		currentCal = (int)temp;
		#endif
		if (externalVoltPresent)
			energyCal += currentCal*voltDataTransform(voltData[i], voltRef);
		else
			energyCal += currentCal*sineTable[i];

	}
	if ((maxADCValue < currentADCLowerBound)&&(inaGain<MAX_INA_GAIN)){
		increaseINAGain();
		return -1;
	}
	else{
		if (externalVoltPresent)
			avgPower = energyCal*P_TRANSFORM2/inaGain; // Unit is mW
		else
			avgPower = energyCal*P_TRANSFORM/inaGain; // Unit is mW
		// Fix phase oppsite down
		if (avgPower < 0)
			avgPower = -1*avgPower;

		if (inaGain==10)
			*power = (int)(avgPower*1.05); // manually adjust this
		else
			*power = (int)avgPower;
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
	float voltageScaling = 1.2;
	return (int)((voltReading - voltReference)*voltageScaling);
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

/*---------------------------------------------------------------------------*/
