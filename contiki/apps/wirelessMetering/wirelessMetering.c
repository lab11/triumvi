#include "contiki.h"
#include "cpu.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "dev/leds.h"
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
static const uint8_t inaGainArr[4] = {1, 2, 5, 10};

#define BUF_SIZE 120
uint16_t adcVal[BUF_SIZE];
uint32_t timerVal[BUF_SIZE];

#include "calibrateData.h"


#define PGOOD_GPIO_NUM	GPIO_B_NUM
#define PGOOD_GPIO_BASE	GPIO_B_BASE
#define PGOOD_GPIO_PIN	3
#define PGOOD_INT_NVIC_PORT NVIC_INT_GPIO_PORT_B

#define I_ADC_GPIO_NUM	GPIO_A_NUM
#define I_ADC_GPIO_PIN	3
#define I_ADC_CHANNEL	SOC_ADC_ADCCON_CH_AIN3

#define V_REF_ADC_GPIO_NUM	GPIO_A_NUM
#define V_REF_ADC_GPIO_PIN	4
#define V_REF_ADC_CHANNEL	SOC_ADC_ADCCON_CH_AIN4

#define V_MEAS_EN_GPIO_BASE	GPIO_C_BASE
#define V_MEAS_EN_GPIO_PIN	0

#define I_MEAS_EN_GPIO_BASE	GPIO_C_BASE
#define I_MEAS_EN_GPIO_PIN	1

#define V_REF_CROSS_INT_GPIO_BASE GPIO_C_BASE
#define V_REF_CROSS_INT_GPIO_NUM GPIO_C_NUM
#define V_REF_CROSS_INT_GPIO_PIN 2
#define V_REF_CROSS_INT_NVIC_PORT NVIC_INT_GPIO_PORT_C

#define MUX_IO_GPIO_BASE GPIO_C_BASE
#define MUX_A1_GPIO_PIN 5
#define MUX_A0_GPIO_PIN 6
#define MUX_EN_GPIO_PIN 7


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

// The following data is accuired from 98% tile
// y = POLY_NEG_OR1*x + POLY_NEG_OR0
#define POLY_NEG_OR1 1.01
#define POLY_NEG_OR0 2.42

// y = POLY_POS_OR1*x + POLY_POS_OR0
#define POLY_POS_OR1 0.93
#define POLY_POS_OR0 2.32

#define VOLTAGE 0x0
#define CURRENT 0x1
#define SENSE_ENABLE 0x1
#define SENSE_DISABLE 0x0

#define MAX_INA_GAIN_IDX 3
#define MAX_INA_GAIN 10
#define MIN_INA_GAIN 1

// Function prototypes
inline void meterSenseConfig(uint8_t type, uint8_t en);
inline void meterVoltageComparator(uint8_t en);
static void pGOODIntCallBack(uint8_t port, uint8_t pin);
static void referenceIntCallBack(uint8_t port, uint8_t pin);
void sampleCurrentWaveform();
static void rtimerEvent(struct rtimer *t, void *ptr);
void disableAll();
void meterInit();
void setINAGain(uint8_t gain);
inline uint8_t getINAIDX();
inline uint8_t getINAGain();
inline void meterMUXConfig(uint8_t en);
int currentProcess(uint16_t *data, uint16_t vRefADCVal, int *power);
void increaseINAGain();
void decreaseINAGain();
inline int getThreshold(uint8_t inaGain);
static void disable_all_ioc_override();
void packData(uint8_t* dest, int reading);
// End of prototypes



typedef enum state{
	init,
	waitingVoltageStable,
	waitingComparatorStable,
	waitingVoltageInt,
	waitingRadioInt,
	nullState
} state_t;

volatile static state_t myState;


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wirelessMeterProcessing, ev, data)
{
	PROCESS_BEGIN();
	CC2538_RF_CSP_ISRFOFF();

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
	uint16_t vRefADCVal;

	
	meterInit();

	inaGain = getINAGain();
	setINAGain(1);
	disableAll();
	#ifdef START_IMMEDIATELY
	while (GPIO_READ_PIN(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN)==0){}
	meterSenseConfig(VOLTAGE, SENSE_ENABLE);
	rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.3, 1, &rtimerEvent, NULL);
	myState = waitingVoltageStable;
	#endif
	while(1){
		PROCESS_YIELD();
		switch (myState){
			// initialization state, release voltage measurement gating
			case init:
				if (rTimerExpired==1){
					rTimerExpired = 0;
					if (GPIO_READ_PIN(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN)){
						meterSenseConfig(VOLTAGE, SENSE_ENABLE);
						rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.3, 1, &rtimerEvent, NULL);
						myState = waitingVoltageStable;
						// consecutive 4 samples, decreases sampling interval
						if (((backOffHistory&0x0f)==0x0f)&&(backOffTime>2))
							backOffTime = (backOffTime>>1);
						backOffHistory = (backOffHistory<<1) | 0x01;
					}
					else{
						#ifndef CALIBRATE
						if (backOffTime<16)
							backOffTime = (backOffTime<<2);
						backOffHistory = (backOffHistory<<1) & (~0x01);
						#endif
						rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
					}
				}
			break;

			// voltage is sattled, enable current measurement
			case waitingVoltageStable:
				if (rTimerExpired==1){
					rTimerExpired = 0;
					meterSenseConfig(CURRENT, SENSE_ENABLE);
					meterMUXConfig(SENSE_ENABLE);
					setINAGain(inaGain);
					rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.05, 1, &rtimerEvent, NULL);
					myState = waitingComparatorStable;
				}
			break;

			case waitingComparatorStable:
				if (rTimerExpired==1){
					rTimerExpired = 0;
					GPIO_DETECT_RISING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
					meterVoltageComparator(SENSE_ENABLE);
					ungate_gpt(GPTIMER_1);
					REG(SYSTICK_STCTRL) &= (~SYSTICK_STCTRL_INTEN);
					myState = waitingVoltageInt;
				}
			break;


			// Start measure current
			case waitingVoltageInt:
				if (referenceInt==1){
					sampleCurrentWaveform();
					vRefADCVal = adc_get(V_REF_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_512);
					vRefADCVal = ((vRefADCVal>>4)>2047)? 0 : (vRefADCVal>>4);
					disableAll();
					referenceInt = 0;
					powerValid = currentProcess(adcVal, vRefADCVal, &avgPower);
					inaGain = getINAGain();
					setINAGain(1);
					if (powerValid>0){
						#ifdef CALIBRATE
						uint8_t i;
						if (calibrate_cnt<255)
							calibrate_cnt++;
						else
							leds_toggle(LEDS_RED);
						printf("ADC reference: %u\r\n", vRefADCVal);
						printf("Time difference: %lu\r\n", (timerVal[0]-timerVal[1]));
						printf("INA Gain: %u\r\n", inaGain);
						for (i=0; i<BUF_SIZE; i+=1){
							printf("ADC reading: %u\r\n", adcVal[i]);
						}
						#else
						#ifdef AES_ENABLE
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
						#else
						uint8_t i;
						packData(&meterData[METER_DATA_OFFSET], avgPower);
						meterData[0] = METER_TYPE_ENERGY;
						meterData[1] = METER_ENERGY_UNIT_mW;
						packetbuf_copyfrom(meterData, (METER_DATA_OFFSET+METER_DATA_LENGTH));
						#endif
						cc2538_on_and_transmit();
						CC2538_RF_CSP_ISRFOFF();
						#endif
					}
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

static void disable_all_ioc_override() {
	uint8_t portnum = 0;
	uint8_t pinnum = 0;
	for(portnum = 0; portnum < 4; portnum++) {
		for(pinnum = 0; pinnum < 8; pinnum++) {
			ioc_set_over(portnum, pinnum, IOC_OVERRIDE_DIS);
		}
	}
}

inline void meterMUXConfig(uint8_t en){
	if (en==SENSE_ENABLE)
		GPIO_SET_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_EN_GPIO_PIN);
	else
		GPIO_CLR_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_EN_GPIO_PIN);
}

inline void meterSenseConfig(uint8_t type, uint8_t en){
	if (type==VOLTAGE){
		if (en==SENSE_ENABLE)
			GPIO_SET_PIN(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
		else
			GPIO_CLR_PIN(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
	}
	else{
		if (en==SENSE_ENABLE)
			GPIO_SET_PIN(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);
		else
			GPIO_CLR_PIN(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);
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

// Power is dead, disable the system, go to sleep
static void pGOODIntCallBack(uint8_t port, uint8_t pin){
	GPIO_CLEAR_INTERRUPT(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN);
	#ifndef CALIBRATE
	backOffTime = 16;
	backOffHistory = (backOffHistory<<1) & (~0x01);
	#endif
	disableAll();
	rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
	myState = init;
	process_poll(&wirelessMeterProcessing);
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
		adcVal[sampleCnt] = ((temp>>4)>2047)? 0 : (temp>>4);
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
	GPIO_SET_OUTPUT(GPIO_A_BASE, 0x63);
	GPIO_CLR_PIN(GPIO_A_BASE, 0x63);
	GPIO_SET_OUTPUT(GPIO_B_BASE, 0xf7);
	GPIO_CLR_PIN(GPIO_B_BASE, 0xf7);
	GPIO_SET_OUTPUT(GPIO_C_BASE, 0x18);
	GPIO_CLR_PIN(GPIO_C_BASE, 0x18);
	GPIO_SET_OUTPUT(GPIO_D_BASE, 0x3f);
	GPIO_CLR_PIN(GPIO_D_BASE, 0x37);
	GPIO_SET_PIN(GPIO_D_BASE, 0x08);


	// disable all pull-up resistors
	disable_all_ioc_override();
	#endif

	// ADC for current sense
	adc_init();

	ioc_set_over(I_ADC_GPIO_NUM, I_ADC_GPIO_PIN, IOC_OVERRIDE_ANA);
	ioc_set_over(V_REF_ADC_GPIO_NUM, V_REF_ADC_GPIO_PIN, IOC_OVERRIDE_ANA);

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
	setINAGain(10);

	// GPIO for PGOOD interrupt
	GPIO_SET_INPUT(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN);
	ioc_set_over(PGOOD_GPIO_NUM, PGOOD_GPIO_PIN, IOC_OVERRIDE_DIS);
	GPIO_DETECT_EDGE(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN);
	GPIO_TRIGGER_SINGLE_EDGE(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN);
	//GPIO_DETECT_FALLING(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN);
	// interrupt
	gpio_register_callback(pGOODIntCallBack, PGOOD_GPIO_NUM, PGOOD_GPIO_PIN);
	GPIO_ENABLE_INTERRUPT(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN);
	nvic_interrupt_enable(PGOOD_INT_NVIC_PORT);

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
	rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
	myState = init;
	#endif
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

void setINAGain(uint8_t gain){
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
}

inline uint8_t getINAIDX(){
	uint8_t mux_a0_sel = GPIO_READ_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN)>>MUX_A0_GPIO_PIN;
	uint8_t mux_a1_sel = GPIO_READ_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN)>>MUX_A1_GPIO_PIN;
	return (mux_a1_sel<<1 | mux_a0_sel);
}

inline uint8_t getINAGain(){
	return inaGainArr[getINAIDX()];
}

inline int getThreshold(uint8_t inaGain){
	if (inaGain==10)
		return 500;
	else
		return 650;
}

int currentProcess(uint16_t *data, uint16_t vRefADCVal, int *power){
	uint16_t i;
	int currentCal;
	int energyCal = 0;
	float avgPower;
	int maxADCValue = 0;
	uint8_t inaGain = getINAGain();
	int upperThreshold = getThreshold(inaGain);

	for (i=0;i<BUF_SIZE;i++){
		currentCal = data[i] - vRefADCVal;
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
		energyCal += currentCal*sineTable[i];
	}
	if ((maxADCValue < 180)&&(inaGain<MAX_INA_GAIN)){
		increaseINAGain();
		return -1;
	}
	else{
		avgPower = energyCal*P_TRANSFORM/inaGain; // Unit is mW
		if (avgPower < 180000)
			*power = (int)(avgPower + 2000); // manually adjust this
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


/*---------------------------------------------------------------------------*/
