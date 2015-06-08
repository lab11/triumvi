#include "contiki.h"
#include "cpu.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "dev/leds.h"
#include "dev/uart.h"
#include "dev/button-sensor.h"
#include "dev/watchdog.h"
#include "dev/serial-line.h"
#include "dev/sys-ctrl.h"
#include "dev/gpio.h"
#include "dev/gptimer.h"
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

///*---------------------------------------------------------------------------*/
//#define LOOP_INTERVAL       CLOCK_SECOND
//#define LEDS_OFF_HYSTERISIS (RTIMER_SECOND >> 1)
//#define LEDS_PERIODIC       LEDS_YELLOW
//#define LEDS_BUTTON         LEDS_RED
//#define LEDS_SERIAL_IN      LEDS_ORANGE
//#define LEDS_REBOOT         LEDS_ALL
//#define LEDS_RF_RX          (LEDS_YELLOW | LEDS_ORANGE)
//#define BROADCAST_CHANNEL   129
///*---------------------------------------------------------------------------*/
//#define MACDEBUG 0
//
//#define DEBUG 1
//#if DEBUG
//#include <stdio.h>
//#define PRINTF(...) printf(__VA_ARGS__)
//#define PRINT6ADDR(addr) PRINTF(" %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x ", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
//#define PRINTLLADDR(lladdr) PRINTF(" %02x:%02x:%02x:%02x:%02x:%02x ",lladdr->addr[0], lladdr->addr[1], lladdr->addr[2], lladdr->addr[3],lladdr->addr[4], lladdr->addr[5])
//#else
//#define PRINTF(...)
//#define PRINT6ADDR(addr)
//#endif
//
//
//uint8_t spibyte = 0;
//uint16_t address = 0xF0;
//uint8_t spibuf[100];

static struct rtimer myRTimer;

PROCESS(wirelessMeterProcessing, "Wireless Metering Process");
AUTOSTART_PROCESSES(&wirelessMeterProcessing);


volatile uint8_t voltageCompInt;
volatile uint8_t referenceCrossInt;
volatile uint8_t timeoutInt;
volatile uint8_t rtimerExpired;
volatile uint8_t backOffTime;
volatile uint8_t backOffHistory;
static const uint8_t inaGainArr[4] = {1, 2, 5, 10};
volatile uint8_t inaGainIDX;
#define MAX_INA_GAIN_IDX 3

#define BUF_SIZE 120
uint16_t adcVal[BUF_SIZE];
uint32_t timerVal[BUF_SIZE];
volatile uint32_t referenceIntTime;

int sineTable[BUF_SIZE] = {
	-153, -148, -144, -139, -134, -128, -122, -116, -109, 
	-102, -95, -87, -79, -71, -63, -55, -46, 
	-37, -29, -20, -11, -2, 7, 16, 25, 
	33, 42, 51, 59, 67, 76, 83, 91, 
	99, 106, 112, 119, 125, 131, 137, 142, 
	146, 151, 155, 158, 161, 164, 166, 167, 
	169, 169, 170, 170, 169, 168, 166, 164, 
	162, 159, 155, 151, 147, 142, 137, 132, 
	126, 120, 114, 107, 100, 92, 85, 77, 
	69, 61, 52, 44, 35, 26, 17, 8, 
	-1, -9, -18, -27, -36, -45, -53, -62, 
	-70, -78, -86, -93, -101, -108, -114, -121, 
	-127, -133, -138, -143, -148, -152, -156, -159, 
	-162, -164, -166, -168, -169, -170, -170, -169, 
	-169, -167, -166, -163, -161, -158, -154};



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

// Packet structure:
// TYPE_ENERGY, ENERGY_UNIT, DATA
#define METER_TYPE_ENERGY 0xaa
#define METER_ENERGY_UNIT_mW 0x0
#define METER_DATA_OFFSET 2
#define METER_DATA_LENGTH 4

#define DEBUG_EN 1
//#define ADC_EXT_REF
#define CURVE_FIT
//#define CALIBRATE

/*
// 3.0078 degree / sample, DC = 170
#define ADC_MAX_VAL 2048
#define ADC_REF	3.3
#define INA_GAIN 2
#define CT_GAIN 0.00033333
#define SHUNT_RESISTOR 90.9

current can be calculated as following:
I = (data - (vRef/adcRef)*adcMax)/adcMax*adcRef/inaGain/shuntResistor/ctGain
unit is A, multiply by 1000 gets mA
*/

#define I_TRANSFORM 48.34468 // 1000/ADC_MAX_VAL*ADC_REF/SHUNT_RESISTOR/CT_GAIN, unit is mA
#define P_TRANSFORM 0.402872 // I_TRANSFORM/TABLE_SIZE, unit is mW

// The following data is accuired from 98% tile
// y = POLY_NEG_OR1*x + POLY_NEG_OR0
#define POLY_NEG_OR1 1.02
#define POLY_NEG_OR0 5.78

// y = POLY_POS_OR1*x - POLY_POS_OR0
#define POLY_POS_OR1 0.95
#define POLY_POS_OR0 3.98 // negative

#define VOLTAGE 0x0
#define CURRENT 0x1
#define SENSE_ENABLE 0x1
#define SENSE_DISABLE 0x0

// Function prototypes
inline void meterSenseConfig(uint8_t type, uint8_t en);
inline void meterVoltageComparator(uint8_t en);
static void pGOODIntCallBack(uint8_t port, uint8_t pin);
static void referenceIntCallBack(uint8_t port, uint8_t pin);
void sampleCurrentWaveform();
static void rtimerEvent(struct rtimer *t, void *ptr);
void stateReset();
void meterInit();
void setINAGain(uint8_t gain);
uint8_t getINAGain();
inline void meterMUXConfig(uint8_t en);
int currentProcess(uint32_t* timeStamp, uint16_t *data, int *avgPower);
void increaseINAGain();
void decreaseINAGain();
// End of prototypes



typedef enum state{
	init,
	waitingVoltageStable,
	waitingCurrentStable,
	waitingNegEdge,
	waitingVoltageInt,
	waitingRadioInt,
	nullState
} state_t;

volatile static state_t myState;


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wirelessMeterProcessing, ev, data)
{
	//static uint8_t meterData[METER_DATA_OFFSET+METER_DATA_LENGTH];
	static uint8_t meterData[BUF_SIZE];
	static int avgPower;
	uint8_t i;

	#ifdef DEBUG_EN
	// Debugging
	GPIO_SET_OUTPUT(GPIO_B_BASE, 0x01<<5);
	GPIO_CLR_PIN(GPIO_B_BASE, 0x01<<5); 
	GPIO_SET_OUTPUT(GPIO_B_BASE, 0x01<<6); 
	// End of Debugging
	#endif

	PROCESS_BEGIN();
	meterInit();


	stateReset();
	#ifdef DEBUG_EN
	GPIO_SET_PIN(GPIO_B_BASE, 0x01<<6);
	#endif
	
	while(1){
		PROCESS_YIELD();
		switch (myState){
			// initialization state, release voltage measurement gating
			case init:
				if (rtimerExpired){
					rtimerExpired = 0;
					if (GPIO_READ_PIN(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN)){
						#ifdef DEBUG_EN
						GPIO_SET_PIN(GPIO_B_BASE, 0x01<<5); 
						#endif
						meterSenseConfig(VOLTAGE, SENSE_ENABLE);
						rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.3, 1, &rtimerEvent, NULL);
						myState = waitingVoltageStable;
						backOffHistory = (backOffHistory<<1) | 0x01;
						// consecutive 4 samples, decreases sampling interval
						if (((backOffHistory&0x0f)==0x0f)&&(backOffTime>2))
							backOffTime = (backOffTime>>1);
					}
					else{
						if (backOffTime<32)
							backOffTime = (backOffTime<<1);
						backOffHistory = (backOffHistory<<1) & (~0x01);
						rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
					}
				}
			break;

			// voltage is sattled, enable current measurement
			case waitingVoltageStable:
				if (rtimerExpired){
					rtimerExpired = 0;
					meterSenseConfig(CURRENT, SENSE_ENABLE);
					rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.05, 1, &rtimerEvent, NULL);
					meterMUXConfig(SENSE_ENABLE);
					myState = waitingCurrentStable;
				}
			break;

			// enable comparator interrupt
			case waitingCurrentStable:
				if (rtimerExpired){
					rtimerExpired = 0;
					myState = waitingNegEdge;
					GPIO_DETECT_FALLING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
					meterVoltageComparator(SENSE_ENABLE);
				}
			break;

			case waitingNegEdge:
				if (voltageCompInt){
					voltageCompInt = 0;
					ungate_gpt(GPTIMER_1);
					REG(SYSTICK_STCTRL) &= (~SYSTICK_STCTRL_INTEN);
					GPIO_DETECT_RISING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
					meterVoltageComparator(SENSE_ENABLE);
					myState = waitingVoltageInt;
				}
			break;

			// Start measure current
			case waitingVoltageInt:
				if (voltageCompInt){
					sampleCurrentWaveform();
					stateReset();
					voltageCompInt = 0;

					#ifdef CALIBRATE
					meterData[0] = 0xbb;
					uint16_t vRefADCVal;
					vRefADCVal = adc_get(V_REF_ADC_CHANNEL, SOC_ADC_ADCCON_REF_AVDD5, SOC_ADC_ADCCON_DIV_512);
					vRefADCVal = ((vRefADCVal>>4)>2048)? 0 : (vRefADCVal>>4);
					meterData[1] = vRefADCVal&0xff;
					meterData[2] = (vRefADCVal&0xff00)>>8;
					meterData[3] = (referenceIntTime-timerVal[0])&0xff;
					meterData[4] = ((referenceIntTime-timerVal[0])&0xff00)>>8;
					meterData[5] = (timerVal[0]-timerVal[1])&0xff;
					meterData[6] = ((timerVal[0]-timerVal[1])&0xff00)>>8;
					uint8_t byteCnt = 6;
					for (i=0; i<BUF_SIZE; i+=3){
						meterData[byteCnt+1] = (adcVal[i]%0xff);
						meterData[byteCnt+2] = ((adcVal[i]&0xff00)>>8);
						byteCnt+=2;
					}
					packetbuf_copyfrom(meterData, 87);
					cc2538_on_and_transmit();
					CC2538_RF_CSP_ISRFOFF();
					#else
					if (currentProcess(timerVal, adcVal, &avgPower)>0){
						for (i=0; i<METER_DATA_LENGTH; i++){
							meterData[METER_DATA_OFFSET+i] = (avgPower&(0xff<<(i<<3)))>>(i<<3);
						}
						meterData[0] = METER_TYPE_ENERGY;
						meterData[1] = METER_ENERGY_UNIT_mW;
						packetbuf_copyfrom(meterData, (METER_DATA_OFFSET+METER_DATA_LENGTH));
						cc2538_on_and_transmit();
						CC2538_RF_CSP_ISRFOFF();
					}
					#endif

				}
			break;

			default:
			break;
		}
	}
	PROCESS_END();
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
	//watchdog_reboot();
	if (backOffTime<32)
		backOffTime = (backOffTime<<1);
	backOffHistory = (backOffHistory<<1) & (~0x01);
	stateReset();
	process_poll(&wirelessMeterProcessing);
}

// Don't add/remove any lines in this subroutine
static void referenceIntCallBack(uint8_t port, uint8_t pin){
	referenceIntTime = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
	meterVoltageComparator(SENSE_DISABLE);
	voltageCompInt = 1;
	process_poll(&wirelessMeterProcessing);
}

// Don't add/remove any lines in this subroutine
void sampleCurrentWaveform(){
	uint16_t sampleCnt = 0;
	uint16_t temp;
	while (sampleCnt < BUF_SIZE){
		timerVal[sampleCnt] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
		#ifdef ADC_EXT_REF
		temp = adc_get(I_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_512);
		#else
		temp = adc_get(I_ADC_CHANNEL, SOC_ADC_ADCCON_REF_AVDD5, SOC_ADC_ADCCON_DIV_512);
		#endif
		adcVal[sampleCnt] = ((temp>>4)>2048)? 0 : (temp>>4);
		sampleCnt++;
	}
}

static void rtimerEvent(struct rtimer *t, void *ptr){
	rtimerExpired = 1;
	process_poll(&wirelessMeterProcessing);
}


void stateReset(){
	REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
	meterSenseConfig(CURRENT, SENSE_DISABLE);
	meterSenseConfig(VOLTAGE, SENSE_DISABLE);
	meterVoltageComparator(SENSE_DISABLE);
	meterMUXConfig(SENSE_DISABLE);
	gate_gpt(GPTIMER_1);
	#ifdef DEBUG_EN
	GPIO_CLR_PIN(GPIO_B_BASE, 0x01<<5); 
	#endif
	rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*backOffTime, 1, &rtimerEvent, NULL);
	myState = init;
}

void meterInit(){
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
	setINAGain(1);

	// GPIO for PGOOD interrupt
	GPIO_SET_INPUT(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN);
	ioc_set_over(PGOOD_GPIO_NUM, PGOOD_GPIO_PIN, IOC_OVERRIDE_DIS);
	GPIO_DETECT_EDGE(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN);
	GPIO_TRIGGER_SINGLE_EDGE(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN);
	GPIO_DETECT_FALLING(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN);
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

	backOffTime = 4;
	backOffHistory = 0;
}

void increaseINAGain(){
	if (inaGainIDX<MAX_INA_GAIN_IDX){
		setINAGain(inaGainArr[(inaGainIDX+1)]);
	}
}

void decreaseINAGain(){
	if (inaGainIDX>0){
		setINAGain(inaGainArr[(inaGainIDX-1)]);
	}
}

void setINAGain(uint8_t gain){
	switch (gain){
		case 1: // Select S1
			GPIO_CLR_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN);
			GPIO_CLR_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN);
			inaGainIDX = 0;
		break;
		case 2: // Select S2
			GPIO_SET_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN);
			GPIO_CLR_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN);
			inaGainIDX = 1;
		break;
		case 5: // Select S3
			GPIO_CLR_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN);
			GPIO_SET_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN);
			inaGainIDX = 2;
		break;
		case 10: // Select S4
			GPIO_SET_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN);
			GPIO_SET_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN);
			inaGainIDX = 3;
		break;
		default:
		break;
	}
}

uint8_t getINAGain(){
	uint8_t mux_a0_sel = GPIO_READ_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN)>>MUX_A0_GPIO_PIN;
	uint8_t mux_a1_sel = GPIO_READ_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN)>>MUX_A1_GPIO_PIN;
	return inaGainArr[(mux_a1_sel<<1 | mux_a0_sel)];
}

int currentProcess(uint32_t* timeStamp, uint16_t *data, int *power){
	uint16_t i;
	int currentCal;
	uint16_t vRefADCVal;
	int energyCal = 0;
	float avgPower;
	uint8_t inaGain;
	uint16_t maxADCValue = 0;

	// Read voltage reference
	#ifdef ADC_EXT_REF
	vRefADCVal = adc_get(V_REF_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_512);
	#else
	vRefADCVal = adc_get(V_REF_ADC_CHANNEL, SOC_ADC_ADCCON_REF_AVDD5, SOC_ADC_ADCCON_DIV_512);
	#endif
	vRefADCVal = ((vRefADCVal>>4)>2048)? 0 : (vRefADCVal>>4);
	#ifdef DEBUG_PRINTF
	printf("Reference reading: %u\r\n", vRefADCVal);
	#endif

	// The sine table is calibrated as following:
	// First time stamp: 330
	// Time diff: 2228
	// If the following numbers don't match, the sine table needs to be recalculated
	#ifdef DEBUG_PRINTF
	printf("First time stamp: %lu\r\n", referenceIntTime-timeStamp[0]);
	printf("Time difference between current samples: %lu\r\n", timeStamp[0]-timeStamp[1]);
	#endif

	for (i=0;i<BUF_SIZE;i++){
		currentCal = data[i] - vRefADCVal;
		if (data[i] > maxADCValue)
			maxADCValue = data[i];
		// The valid range is from 0.66~2.45 V
		if ((data[i]>1700)||(data[i]<450)){
			decreaseINAGain();
			return -1;
		}
		// Use linear regression
		// for negative: y = 1.04*x + 1.28
		// for positive: y = 0.98*x - 1.67
		#ifdef CURVE_FIT
		float temp;
		if (currentCal<0)
			temp = currentCal*POLY_NEG_OR1 + POLY_NEG_OR0;
		else
			temp = currentCal*POLY_POS_OR1 - POLY_POS_OR0;
		currentCal = (int)temp;
		#endif
		//
		#ifdef DEBUG_PRINTF
		printf("current: %d\r\n", (int)(currentCal/inaGain));
		#endif
		energyCal += currentCal*sineTable[i];
	}
	if (maxADCValue < 1200){
		increaseINAGain();
		return -1;
	}
	else{
		inaGain = getINAGain();
		avgPower = energyCal*P_TRANSFORM/inaGain; // Unit is mW
		*power = (int)avgPower;
		return 1;
	}
}

/*---------------------------------------------------------------------------*/
