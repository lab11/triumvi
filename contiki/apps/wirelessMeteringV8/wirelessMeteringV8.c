
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
#ifdef CALIBRATE
static uint8_t calibrate_cnt = 0;
#endif

#define BUF_SIZE 120
#define BUF_SIZE2 113 // for external voltage input
uint16_t currentADCVal[BUF_SIZE];
uint16_t voltADCVal[BUF_SIZE2+2]; // compensate the voltage sample offset
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

#define INAGAIN1_OFFSET 1
#define INAGAIN1_SCALE 47.14
#define INAGAIN2_OFFSET 61
#define INAGAIN2_SCALE 23.737
#define INAGAIN5_OFFSET -23
#define INAGAIN5_SCALE 9.339
#define INAGAIN10_OFFSET -35
#define INAGAIN10_SCALE 4.704

#define INAGAIN1_OFFSET2 1.5
#define INAGAIN1_SCALE2 95.073
#define INAGAIN2_OFFSET2 1.5
#define INAGAIN2_SCALE2 47.536
#define INAGAIN5_OFFSET2 6
#define INAGAIN5_SCALE2 18.965
#define INAGAIN10_OFFSET2 6
#define INAGAIN10_SCALE2 9.428

#define EXTERNALVOLT_STATUSREG 0x80
#define BATTERYPACK_STATUSREG 0x40
#define THREEPHASE_STATUSREG 0x30
#define FRAMWRITE_STATUSREG 0x08

#ifndef THREEPHASE_ID
#define THREEPHASE_ID 0
#endif


// Function prototypes
#ifndef THREEPHASE_UNIT
static void referenceIntCallBack(uint8_t port, uint8_t pin);
#endif
void sampleCurrentWaveform();
void sampleCurrentVoltageWaveform();
static void rtimerEvent(struct rtimer *t, void *ptr);
void disableAll();
void meterInit();
int currentVoltProcess(uint16_t *currentData, uint16_t* voltData, 
						uint16_t currentRef, uint16_t voltRef, 
						int *power, uint8_t externalVolt);
inline int getThreshold(uint8_t inaGain);
void packData(uint8_t* dest, int reading);
uint16_t voltDataAverge(uint16_t* voltData);
int voltDataTransform(uint16_t voltReading, uint16_t voltReference);
int currentDataTransform(int currentReading, uint8_t inaGain, uint8_t externalVolt);
int sampleAndCalculate(uint8_t triumviStatusReg, int* avgPower, uint16_t* currentRef, uint16_t* voltRef);
void encryptAndTransmit(uint8_t triumviStatusReg, int avgPower, uint8_t* myNonce, uint32_t nonceCounter);
#ifndef CALIBRATE
static void disable_all_ioc_override();
#else
void printCalibrationValuse(uint8_t triumviStatusReg, uint8_t inaGain, uint16_t currentRef, uint16_t voltRef);
#endif
#ifdef THREEPHASE_UNIT
#ifndef THREEPHASE_MASTER
static void threephaseStartMeasure(uint8_t port, uint8_t pin);
#endif
#endif
#ifdef FRAM_WRITE
inline void writeFRAM(uint16_t powerRead, rv3049_time_t* rtctime);
#endif
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
	batteryPackLEDBlink,
	nullState
} state_t;

volatile static state_t myState;


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wirelessMeterProcessing, ev, data)
{
	PROCESS_BEGIN();
	#ifdef THREEPHASE_UNIT
	random_init(0);
	#endif
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
	static uint8_t inaGain;
	static uint16_t currentRef, voltRef;

	
	meterInit();
	if (isButtonPressed()){
		triumviFramPtrClear();
		triumviLEDON();
	}
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
					triumviLEDOFF();
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
					#ifdef THREEPHASE_MASTER
					rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*5, 1, &rtimerEvent, NULL);
					#else
					rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.04, 1, &rtimerEvent, NULL);
					#endif
					#ifdef COMPARATOR_NEGEDGE
					myState = waitingComparatorStable2;
					#else
					myState = waitingComparatorStable;
					#endif
				}
			break;

			#ifndef THREEPHASE_UNIT
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
			#endif

			case waitingComparatorStable:
				if (rTimerExpired==1){
					rTimerExpired = 0;
					// Layout of Status Reg:
					// Bit 8: External Volt Selected
					// Bit 7: Battery Pack attached
					// Bit 6:5: Three Phase Slave Selected
					// 00 --> non, 01 --> Master, 10 --> slave 1, 11 --> slave2
					// Bit 4: FRAM WRITE Enabled
					// Note: bit 6 & 7 cannot be set simultaneously
					uint8_t triumviStatusReg = 0x00;
					if (externalVoltSel())
						triumviStatusReg |= EXTERNALVOLT_STATUSREG;
					#ifdef FRAM_WRITE
					triumviStatusReg |= FRAMWRITE_STATUSREG;
					#endif
					#ifdef THREEPHASE_UNIT
					triumviStatusReg |= ((THREEPHASE_ID&3)<<4);
					#ifdef THREEPHASE_MASTER
					GPIO_SET_PIN(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
					clock_delay_usec(13);
					GPIO_CLR_PIN(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
					#else
					GPIO_ENABLE_INTERRUPT(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
					nvic_interrupt_enable(I2C_SCL_NVIC_PORT);
					#endif // End of THREEPHASE_MASTER
					#else // non three phase unit
					if (batteryPackIsAttached()){
						triumviStatusReg |= BATTERYPACK_STATUSREG;
						triumviLEDON();
						batteryPackInit();
					}
					GPIO_DETECT_RISING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
					meterVoltageComparator(SENSE_ENABLE);
					ungate_gpt(GPTIMER_1);
					#endif
					REG(SYSTICK_STCTRL) &= (~SYSTICK_STCTRL_INTEN);
					#ifndef THREEPHASE_MASTER
					while (referenceInt==0){}
					#endif
					powerValid = sampleAndCalculate(triumviStatusReg, &avgPower, &currentRef, &voltRef);
					referenceInt = 0;
					inaGain = getINAGain();
					setINAGain(inaGainArr[0]);
					if (powerValid>0){
						#ifdef CALIBRATE
						printCalibrationValuse(triumviStatusReg, inaGain, currentRef, voltRef);
						#else
						// Write data into FRAM
						#ifdef FRAM_WRITE
						writeFRAM((uint16_t)(avgPower/1000), &rtctime);
						#endif // FRAM_WRITE
						encryptAndTransmit(triumviStatusReg, avgPower, myNonce, nonceCounter);
						nonceCounter += 1;
						#endif // CALIBRATE
						if (triumviStatusReg & THREEPHASE_STATUSREG){
							triumviLEDON();
							rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.1, 1, &rtimerEvent, NULL);
							myState = init;
						}
						else if (triumviStatusReg & BATTERYPACK_STATUSREG){
							batteryPackLEDDriverInit();
							batteryPackLEDOn(BATTERY_PACK_LED_BLUE);
							myState = batteryPackLEDBlink;
							rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.1, 1, &rtimerEvent, NULL);
						}
						else{
							rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
							myState = init;
						}
					}
					// improper gain setting, try after 4 s
					else{
						rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*4, 1, &rtimerEvent, NULL);
						myState = init;
					}
				}
			break;

			case batteryPackLEDBlink:
				if (rTimerExpired==1){
					rTimerExpired = 0;
					sx1509b_init();
					batteryPackLEDOff(BATTERY_PACK_LED_BLUE);
					batteryPackLEDDriverDisable();
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

#ifndef THREEPHASE_UNIT
// Don't add/remove any lines in this subroutine
static void referenceIntCallBack(uint8_t port, uint8_t pin){
	meterVoltageComparator(SENSE_DISABLE);
	referenceInt = 1;
	process_poll(&wirelessMeterProcessing);
}
#endif

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
	ioc_set_over(EXT_VOLT_IN_SEL_GPIO_NUM, EXT_VOLT_IN_SEL_GPIO_PIN, IOC_OVERRIDE_PDE);

	// tactile button
	GPIO_SET_INPUT(MEM_RST_GPIO_BASE, 0x1<<MEM_RST_GPIO_PIN);

	#ifdef THREEPHASE_UNIT
	#ifndef THREEPHASE_MASTER
	// 3 phase slave trigger, use SCL as trigger input
	GPIO_SET_INPUT(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
	ioc_set_over(I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN, IOC_OVERRIDE_PDE);
	GPIO_DETECT_EDGE(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
	GPIO_TRIGGER_SINGLE_EDGE(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
	GPIO_DETECT_RISING(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
	// interrupt
	gpio_register_callback(threephaseStartMeasure, I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN);
	GPIO_DISABLE_INTERRUPT(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
	nvic_interrupt_disable(I2C_SCL_NVIC_PORT);
	#endif
	#endif


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

	#ifndef THREEPHASE_UNIT
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
	#endif

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
	backOffTime = 4;
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
						int *power, uint8_t externalVolt){
	uint16_t i;
	int currentCal;
	int energyCal = 0;
	float avgPower;
	int maxADCValue = 0;
	uint8_t inaGain = getINAGain();
	int upperThreshold; 
	uint8_t bufSize;
	uint8_t currentADCLowerBound;


	if (externalVolt){
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
		/*
		// Use linear regression
		#ifdef CURVE_FIT
		float temp;
		if (currentCal<0)
			temp = currentCal*POLY_NEG_OR1 + POLY_NEG_OR0;
		else
			temp = currentCal*POLY_POS_OR1 + POLY_POS_OR0;
		currentCal = (int)temp;
		#endif
		if (externalVolt)
			energyCal += currentCal*voltDataTransform(voltData[i], voltRef);
		else
			energyCal += currentCal*sineTable[i];
		*/
		if (externalVolt)
			energyCal += currentDataTransform(currentCal, inaGain, 0x1)*voltDataTransform(voltData[i], voltRef);
		else
			energyCal += currentDataTransform(currentCal, inaGain, 0x0)*sineTable[i];

	}
	if ((maxADCValue < currentADCLowerBound)&&(inaGain<MAX_INA_GAIN)){
		increaseINAGain();
		return -1;
	}
	else{
		avgPower = energyCal/bufSize; // unit is mW
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

int currentDataTransform(int currentReading, uint8_t inaGain, uint8_t externalVolt){
	// 10-bit ADC
	if (externalVolt){
		switch (inaGain){
			case 1:
				return (int)((currentReading - INAGAIN1_OFFSET2)*INAGAIN1_SCALE2);
			break;
			case 2:
				return (int)((currentReading - INAGAIN2_OFFSET2)*INAGAIN2_SCALE2);
			break;
			case 5:
				return (int)((currentReading - INAGAIN5_OFFSET2)*INAGAIN5_SCALE2);
			break;
			case 10:
				return (int)((currentReading - INAGAIN10_OFFSET2)*INAGAIN10_SCALE2);
			break;

			default:
				return 0;
			break;
		}
	}
	// 11-Bit ADC
	else{
		switch (inaGain){
			case 1:
				return (int)((currentReading - INAGAIN1_OFFSET)*INAGAIN1_SCALE);
			break;
			case 2:
				return (int)((currentReading - INAGAIN2_OFFSET)*INAGAIN2_SCALE);
			break;
			case 5:
				return (int)((currentReading - INAGAIN5_OFFSET)*INAGAIN5_SCALE);
			break;
			case 10:
				return (int)((currentReading - INAGAIN10_OFFSET)*INAGAIN10_SCALE);
			break;

			default:
				return 0;
			break;
		}
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

	if (triumviStatusReg & THREEPHASE_STATUSREG){
		randBackOff = random_rand();
		clock_delay_usec(randBackOff);
		clock_delay_usec(randBackOff);
	}
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
		//GPIO_SET_PIN(I2C_SDA_GPIO_BASE, 1<<I2C_SDA_GPIO_PIN); // Test timing
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
		//GPIO_CLR_PIN(I2C_SDA_GPIO_BASE, 1<<I2C_SDA_GPIO_PIN); // Test timing
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
void printCalibrationValuse(uint8_t triumviStatusReg, uint8_t inaGain, uint16_t currentRef, uint16_t voltRef){
	uint8_t i;
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
			currentDataTransform(currentADCVal[i]-currentRef, inaGain, 0x1), voltDataTransform(voltADCVal[i+2], voltRef));
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

#ifdef THREEPHASE_UNIT
#ifndef THREEPHASE_MASTER
static void threephaseStartMeasure(uint8_t port, uint8_t pin){
	GPIO_DISABLE_INTERRUPT(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
	nvic_interrupt_disable(I2C_SCL_NVIC_PORT);
	referenceInt = 1;
	process_poll(&wirelessMeterProcessing);
}
#endif
#endif

/*---------------------------------------------------------------------------*/
