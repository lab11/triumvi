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

static struct etimer myTimer;

PROCESS(wirelessMeterProcessing, "Wireless Metering Process");
AUTOSTART_PROCESSES(&wirelessMeterProcessing);


volatile uint8_t voltageCompInt;
volatile uint8_t referenceCrossInt;
volatile uint8_t timeoutInt;

#define BUF_SIZE 120
uint16_t adcVal[BUF_SIZE];
uint32_t timerVal[BUF_SIZE];
volatile uint32_t referenceIntTime;


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

#define V_MEAS_EN_GPIO_NUM	GPIO_C_NUM
#define V_MEAS_EN_GPIO_BASE	GPIO_C_BASE
#define V_MEAS_EN_GPIO_PIN	0

#define I_MEAS_EN_GPIO_NUM	GPIO_C_NUM
#define I_MEAS_EN_GPIO_BASE	GPIO_C_BASE
#define I_MEAS_EN_GPIO_PIN	1

#define V_REF_CROSS_INT_GPIO_BASE GPIO_C_BASE
#define V_REF_CROSS_INT_GPIO_NUM GPIO_C_NUM
#define V_REF_CROSS_INT_GPIO_PIN 2
#define V_REF_CROSS_INT_NVIC_PORT NVIC_INT_GPIO_PORT_C

// Packet structure:
// TYPE_ENERGY, ENERGY_UNIT, DATA
#define METER_TYPE_ENERGY 0xaa
#define METER_ENERGY_UNIT_mW 0x0
#define METER_DATA_OFFSET 2
#define METER_DATA_LENGTH 4

#define DEBUG_EN 1

/*
// 3.0078 degree / sample, DC = 170
#define ADC_MAX_VAL 2048
#define ADC_REF	3.3
#define INA_GAIN 2
#define CT_GAIN 0.00033333
#define SHUNT_RESISTOR 180

vRef = 1.53;
adcRef = 3;
adcMAX = 2048;
ctGain = 0.00033333; 1/3*10^-3
shuntResistor = 180;

current can be calculated as following:
I = (data - (vRef/adcRef)*adcMax)/adcMax*adcRef/inaGain/shuntResistor/ctGain
unit is A, multiply by 1000 gets mA
*/

#define I_TRANSFORM 24.41406 // 1000/ADC_MAX_VAL*ADC_REF/SHUNT_RESISTOR/CT_GAIN, unit is mA
#define P_TRANSFORM 0.203451 // I_TRANSFORM/TABLE_SIZE, unit is mW

#define POLY_NEG_OR0 1.28
#define POLY_NEG_OR1 1.04
#define POLY_POS_OR0 1.67 // negative
#define POLY_POS_OR1 0.98

#define VOLTAGE 0x0
#define CURRENT 0x1
#define SENSE_ENABLE 0x1
#define SENSE_DISABLE 0x0

// Function prototypes
void meterSenseConfig(uint8_t type, uint8_t en);
inline void meterVoltageComparator(uint8_t en);
static void pGOODIntCallBack(uint8_t port, uint8_t pin);
static void referenceIntCallBack(uint8_t port, uint8_t pin);
void sampleCurrentWaveform();
static void timeoutCallBack(uint32_t gpt_time);
void stateReset();
void meterInit();
int currentProcess(uint32_t* timeStamp, uint16_t *data);



typedef enum state{
	init,
	waitingVoltageStable,
	waitingCurrentStable,
	waitingVoltageInt,
	waitingRadioInt,
	nullState
} state_t;

volatile static state_t myState;


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wirelessMeterProcessing, ev, data)
{
	static uint8_t meterData[METER_DATA_OFFSET+METER_DATA_LENGTH];
	static int avgPower;
	uint8_t i;

	PROCESS_BEGIN();
	meterInit();

	#ifdef DEBUG_EN
	// Debugging
	GPIO_SET_OUTPUT(GPIO_B_BASE, 0x01<<5);
	GPIO_CLR_PIN(GPIO_B_BASE, 0x01<<5); 
	GPIO_SET_OUTPUT(GPIO_B_BASE, 0x01<<6); 
	GPIO_CLR_PIN(GPIO_B_BASE, 0x01<<6);
	// End of Debugging
	#endif

	// Timeout
	ungate_gpt(GPTIMER_2);
	gpt_configure_timer(GPTIMER_2, GPTIMER_CFG_GPTMCFG_32BIT_TIMER);
	gpt_set_mode(GPTIMER_2, GPTIMER_SUBTIMER_A, GPTIMER_TAMR_TAMR_PERIODIC);
	gpt_set_count_dir(GPTIMER_2, GPTIMER_SUBTIMER_A, GPTIMER_TnMR_TnCDIR_COUNT_DOWN);
	gpt_set_interval_value(GPTIMER_2, GPTIMER_SUBTIMER_A, 0x411ab); // 0x411ab = 16.67 ms
	gpt_register_callback(timeoutCallBack, GPTIMER_2, GPTIMER_SUBTIMER_A, GPTIMER_TIMEOUT_INT);
	gpt_enable_interrupt(GPTIMER_2, GPTIMER_SUBTIMER_A, GPTIMER_TIMEOUT_INT);
	nvic_interrupt_enable(NVIC_INT_GPTIMER_2A);

	stateReset();
	static uint8_t sampleCnt = 0;
	
	while(1){
		PROCESS_YIELD();
		switch (myState){
			// initialization state, release voltage measurement gating
			case init:
				if (etimer_expired(&myTimer)) {
					leds_off(LEDS_GREEN);
					if (GPIO_READ_PIN(PGOOD_GPIO_BASE, 0x1<<PGOOD_GPIO_PIN)){
						#ifdef DEBUG_EN
						GPIO_SET_PIN(GPIO_B_BASE, 0x01<<5); 
						#endif
						meterSenseConfig(VOLTAGE, SENSE_ENABLE);
						etimer_set(&myTimer, 0.3*CLOCK_SECOND);
						myState = waitingVoltageStable;
					}
					else{
						etimer_restart(&myTimer);
					}
				}
			break;

			// voltage is sattled, enable current measurement
			case waitingVoltageStable:
				if (etimer_expired(&myTimer)) {
					meterSenseConfig(CURRENT, SENSE_ENABLE);
					etimer_set(&myTimer, 0.05*CLOCK_SECOND);
					myState = waitingCurrentStable;
				}
			break;

			// enable comparator interrupt
			case waitingCurrentStable:
				if (etimer_expired(&myTimer)) {
					gpt_set_interval_value(GPTIMER_2, GPTIMER_SUBTIMER_A, 0x411ab); // 0x411ab = 16.67 ms
					gpt_enable_event(GPTIMER_2, GPTIMER_SUBTIMER_A);
					myState = waitingVoltageInt;
					REG(SYSTICK_STCTRL) &= (~SYSTICK_STCTRL_INTEN);
					meterVoltageComparator(SENSE_ENABLE);
				}
			break;

			// Start measure current
			case waitingVoltageInt:
				if (voltageCompInt){
					sampleCurrentWaveform();
					stateReset();
					voltageCompInt = 0;
					avgPower = currentProcess(timerVal, adcVal);
					printf("Power: %d\r\n", avgPower);
					if (sampleCnt<255)
						sampleCnt++;
					else
						leds_on(LEDS_GREEN);
					//for (i=0; i<METER_DATA_LENGTH; i++){
					//	meterData[METER_DATA_OFFSET+i] = (avgPower&(0xff<<(i<<3)))>>(i<<3);
					//}
					//meterData[0] = METER_TYPE_ENERGY;
					//meterData[1] = METER_ENERGY_UNIT_mW;
					//packetbuf_copyfrom(meterData, (METER_DATA_OFFSET+METER_DATA_LENGTH));
					//cc2538_on_and_transmit();
					//CC2538_RF_CSP_ISRFOFF();
				}
				// Didn't get interrupt within 16.67 ms, lost power
				else if (timeoutInt){
					timeoutInt = 0;
					stateReset();
				}
			break;

			default:
			break;
		}
	}
	PROCESS_END();
}

void meterSenseConfig(uint8_t type, uint8_t en){
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
	watchdog_reboot();
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
		temp = adc_get(I_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_512);
		temp = ((temp>>4)>2048)? 0 : (temp>>4);
		adcVal[sampleCnt] = temp;
		sampleCnt++;
	}
}

static void timeoutCallBack(uint32_t gpt_time){
	timeoutInt = 1;
	process_poll(&wirelessMeterProcessing);
}

void stateReset(){
	REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
	meterSenseConfig(CURRENT, SENSE_DISABLE);
	meterSenseConfig(VOLTAGE, SENSE_DISABLE);
	meterVoltageComparator(SENSE_DISABLE);
	gpt_disable_event(GPTIMER_2, GPTIMER_SUBTIMER_A);
	#ifdef DEBUG_EN
	GPIO_CLR_PIN(GPIO_B_BASE, 0x01<<5); 
	#endif
	etimer_set(&myTimer, 0.5*CLOCK_SECOND);
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
	GPIO_DETECT_RISING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
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

}

int currentProcess(uint32_t* timeStamp, uint16_t *data){
	uint16_t sineTable[BUF_SIZE] = {
	38, 44, 50, 56, 63, 70, 77, 85,
	93, 101, 109, 118, 126, 135, 143, 152,
	161, 170, 179, 188, 197, 205, 214, 223,
	231, 239, 247, 255, 263, 270, 277, 284,
	290, 296, 302, 307, 312, 317, 321, 325,
	328, 331, 334, 336, 337, 339, 339, 339,
	339, 338, 337, 335, 333, 331, 328, 324,
	320, 316, 311, 306, 301, 295, 289, 282,
	275, 268, 261, 253, 245, 237, 229, 221,
	212, 203, 195, 186, 177, 168, 159, 150,
	141, 133, 124, 115, 107, 99, 91, 83,
	76, 68, 61, 55, 48, 42, 36, 31,
	26, 22, 17, 14, 10, 8, 5, 3,
	2, 1, 0, 0, 0, 1, 3, 4,
	6, 9, 12, 16, 20, 24, 29, 34};

	uint16_t i;
	int currentCal, voltageCal;
	uint16_t vRefADCVal;
	uint16_t voltRefVal = 170;
	int energyCal = 0;
	float avgPower;
	float temp;
	uint8_t inaGain = 2;

	// Read voltage reference
	vRefADCVal = adc_get(V_REF_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_512);
	vRefADCVal = ((vRefADCVal>>4)>2048)? 0 : (vRefADCVal>>4);
	#ifdef DEBUG_PRINTF
	printf("Reference reading: %u\r\n", vRefADCVal);
	#endif

	// The sine table is calibrated as following:
	// First time stamp: 332
	// Time diff: 2233
	// If the following numbers don't match, the sine table needs to be recalculated
	#ifdef DEBUG_PRINTF
	printf("First time stamp: %lu\r\n", referenceIntTime-timeStamp[0]);
	printf("Time difference between current samples: %lu\r\n", timeStamp[0]-timeStamp[1]);
	#endif

	//printf("\r\n");
	for (i=0;i<BUF_SIZE;i++){
		currentCal = data[i] - vRefADCVal;
		// User linear regression
		// for negative: y = 1.04*x + 1.28
		// for positive: y = 0.98*x - 1.67
		if (currentCal<0)
			temp = currentCal*POLY_NEG_OR1 + POLY_NEG_OR0;
		else
			temp = currentCal*POLY_POS_OR1 - POLY_POS_OR0;
		currentCal = (int)(temp/inaGain);
		//
		#ifdef DEBUG_PRINTF
		printf("current: %d\r\n", currentCal);
		#endif
		voltageCal = sineTable[i] - voltRefVal;
		energyCal += currentCal*voltageCal;
	}
	avgPower = energyCal*P_TRANSFORM; // Unit is mW
	#ifdef DEBUG_PRINTF
	//printf("Average power: %d mW\r\n", (int)avgPower);
	#endif
	return (int)avgPower;
}

/*---------------------------------------------------------------------------*/
