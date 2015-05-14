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


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*---------------------------------------------------------------------------*/
#define LOOP_INTERVAL       CLOCK_SECOND
#define LEDS_OFF_HYSTERISIS (RTIMER_SECOND >> 1)
#define LEDS_PERIODIC       LEDS_YELLOW
#define LEDS_BUTTON         LEDS_RED
#define LEDS_SERIAL_IN      LEDS_ORANGE
#define LEDS_REBOOT         LEDS_ALL
#define LEDS_RF_RX          (LEDS_YELLOW | LEDS_ORANGE)
#define BROADCAST_CHANNEL   129
/*---------------------------------------------------------------------------*/
#define MACDEBUG 0

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF(" %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x ", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF(" %02x:%02x:%02x:%02x:%02x:%02x ",lladdr->addr[0], lladdr->addr[1], lladdr->addr[2], lladdr->addr[3],lladdr->addr[4], lladdr->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#endif


uint8_t spibyte = 0;
uint16_t address = 0xF0;

static struct etimer myTimer;

PROCESS(wirelessMeterProcessing, "Wireless Metering Process");
AUTOSTART_PROCESSES(&wirelessMeterProcessing);

uint8_t spibuf[100];

volatile uint8_t voltageCompInt;
volatile uint8_t referenceCrossInt;
volatile uint8_t timeoutInt;

#define BUF_SIZE 120
uint16_t adcVal[BUF_SIZE];
uint32_t timerVal[BUF_SIZE];
volatile uint32_t referenceIntTime;

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

#define DEBUG_EN 1

/*
// 3.0078 degree / sample, DC = 170
#define ADC_MAX_VAL 2048
#define ADC_REF	3.3
#define INA_GAIN 2
#define CT_GAIN 0.00033333
#define SHUNT_RESISTOR 180

vRef = 1.53;
adcRef = 3.3;
adcMAX = 2048;
inaGain = 2;
ctGain = 0.00033333;
shuntResistor = 180;

current can be calculated as following:
I = (data - (vRef/adcRef)*adcMax)/adcMax*adcRef/inaGain/shuntResistor/ctGain
unit is A, multiply by 1000 gets mA
*/

#define I_TRANSFORM 13.4277 // 1000/ADC_MAX_VAL*ADC_REF/INA_GAIN/SHUNT_RESISTOR/CT_GAIN, unit is mA
#define P_TRANSFORM 0.111898 // I_TRANSFORM/TABLE_SIZE, unit is mW


static void referenceIntCallBack(uint8_t port, uint8_t pin){
	referenceIntTime = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
	GPIO_DISABLE_INTERRUPT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	GPIO_CLEAR_INTERRUPT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	#ifdef DEBUG_EN
	GPIO_SET_PIN(GPIO_B_BASE, 0x1<<3);
	#endif
	voltageCompInt = 1;
	process_poll(&wirelessMeterProcessing);
}

typedef enum state{
	init,
	waitingVoltageStable,
	waitingCurrentStable,
	waitingVoltageInt,
	nullState
} state_t;

state_t myState;

void sampleCurrentWaveform(){
	uint16_t sampleCnt = 0;
	uint16_t temp;
	while (sampleCnt < BUF_SIZE){
		timerVal[sampleCnt] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
		temp = adc_get(I_ADC_CHANNEL, SOC_ADC_ADCCON_REF_AVDD5, SOC_ADC_ADCCON_DIV_512);
		temp = ((temp>>4)>2048)? 0 : (temp>>4);
		adcVal[sampleCnt] = temp;
		sampleCnt++;
	}
}

static void timeoutCallBack(uint32_t gpt_time){
	timeoutInt = 1;
	process_poll(&wirelessMeterProcessing);
}

void currentProcess(uint32_t* timeStamp, uint16_t *data){
	uint16_t sineTable[BUF_SIZE] = {
	29, 34, 40, 46, 52, 58, 65, 72,
	80, 88, 96, 104, 112, 120, 129, 138,
	146, 155, 164, 173, 182, 191, 200, 208,
	217, 226, 234, 242, 250, 258, 265, 272,
	279, 286, 292, 298, 304, 309, 314, 319,
	323, 326, 330, 332, 335, 336, 338, 339,
	339, 339, 339, 338, 337, 335, 332, 330,
	327, 323, 319, 314, 310, 304, 299, 293,
	287, 280, 273, 266, 258, 251, 243, 235,
	226, 218, 209, 200, 192, 183, 174, 165,
	156, 147, 138, 130, 121, 113, 104, 96,
	88, 81, 73, 66, 59, 52, 46, 40,
	35, 29, 25, 20, 16, 13, 9, 7,
	4, 3, 1, 0, 0, 0, 1, 2,
	3, 5, 7, 10, 13, 17, 21, 26};

	uint16_t i;
	int currentCal, voltageCal;
	uint16_t vRefADCVal;
	uint16_t voltRefVal = 170;
	int energyCal = 0;
	float avgPower;

	// Read voltage reference
	vRefADCVal = adc_get(V_REF_ADC_CHANNEL, SOC_ADC_ADCCON_REF_AVDD5, SOC_ADC_ADCCON_DIV_512);
	vRefADCVal = ((vRefADCVal>>4)>2048)? 0 : (vRefADCVal>>4);
/*
	uint32_t tComp;
	uint32_t firstTimeStamp;

	printf("\r\n");
	if (referenceIntTime < timeStamp[0])
		firstTimeStamp = referenceIntTime + (0xffffffff - timeStamp[0]);
	else
		firstTimeStamp = referenceIntTime - timeStamp[0];

	for (i=0; i<BUF_SIZE; i++){
		tComp = (referenceIntTime > timeStamp[i])? referenceIntTime - timeStamp[i] : referenceIntTime + (0xffffffff- timeStamp[i]);
		currentCal = (data[i] - vRefADCVal)*I_TRANSFORM; // unit is mA
		printf("Time Stamp: %lu\t Current (mA): %d\r\n", tComp, currentCal);
	}
	*/

	printf("\r\n");
	for (i=0;i<BUF_SIZE;i++){
		currentCal = data[i] - vRefADCVal;
		voltageCal = sineTable[i] - voltRefVal;
		energyCal += currentCal*voltageCal;
	}
	avgPower = energyCal*P_TRANSFORM; // Unit is mW
	printf("Average power: %d mW\r\n", (int)avgPower);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wirelessMeterProcessing, ev, data)
{


	PROCESS_BEGIN();
	// ADC for current sense
	adc_init();
	ioc_set_over(I_ADC_GPIO_NUM, I_ADC_GPIO_PIN, IOC_OVERRIDE_ANA);
	ioc_set_over(V_REF_ADC_GPIO_NUM, V_REF_ADC_GPIO_PIN, IOC_OVERRIDE_ANA);

	// GPIO for Voltage/Current measurement
	GPIO_SET_OUTPUT(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
	GPIO_CLR_PIN(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
	GPIO_SET_OUTPUT(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);
	GPIO_CLR_PIN(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);

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
	nvic_interrupt_enable(V_REF_CROSS_INT_NVIC_PORT);


	etimer_set(&myTimer, CLOCK_SECOND<<2);
	myState = init;

	#ifdef DEBUG_EN
	// Debugging
	GPIO_SET_OUTPUT(GPIO_B_BASE, 0x1<<3);
	GPIO_CLR_PIN(GPIO_B_BASE, 0x1<<3);
	GPIO_SET_OUTPUT(GPIO_B_BASE, 0x01<<6); 
	GPIO_CLR_PIN(GPIO_B_BASE, 0x01<<6);
	GPIO_SET_OUTPUT(GPIO_B_BASE, 0x01<<5);
	GPIO_CLR_PIN(GPIO_B_BASE, 0x01<<5); 
	// End of Debugging
	#endif


	// timer1, used for counting samples
	ungate_gpt(GPTIMER_1);
	gpt_set_mode(GPTIMER_1, GPTIMER_SUBTIMER_A, GPTIMER_TAMR_TAMR_PERIODIC);
	gpt_set_count_dir(GPTIMER_1, GPTIMER_SUBTIMER_A, GPTIMER_TnMR_TnCDIR_COUNT_DOWN);
	gpt_set_interval_value(GPTIMER_1, GPTIMER_SUBTIMER_A, 0xffffffff);
	gpt_enable_event(GPTIMER_1, GPTIMER_SUBTIMER_A);

	// Timeout
	ungate_gpt(GPTIMER_2);
	gpt_configure_timer(GPTIMER_2, GPTIMER_CFG_GPTMCFG_32BIT_TIMER);
	gpt_set_mode(GPTIMER_2, GPTIMER_SUBTIMER_A, GPTIMER_TAMR_TAMR_PERIODIC);
	gpt_set_count_dir(GPTIMER_2, GPTIMER_SUBTIMER_A, GPTIMER_TnMR_TnCDIR_COUNT_DOWN);
	gpt_set_interval_value(GPTIMER_2, GPTIMER_SUBTIMER_A, 0x411ab); // 0x411ab = 16.67 ms
	gpt_register_callback(timeoutCallBack, GPTIMER_2, GPTIMER_SUBTIMER_A, GPTIMER_TIMEOUT_INT);
	gpt_enable_interrupt(GPTIMER_2, GPTIMER_SUBTIMER_A, GPTIMER_TIMEOUT_INT);
	nvic_interrupt_enable(NVIC_INT_GPTIMER_2A);

	
	while(1){
		PROCESS_YIELD();
		switch (myState){
			// initialization state, release voltage measurement gating
			// and wait XXX ms for voltage to settle
			case init:
				if (etimer_expired(&myTimer)) {
					#ifdef DEBUG_EN
					GPIO_CLR_PIN(GPIO_B_BASE, 0x01<<5); 
					#endif
					GPIO_SET_PIN(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
					etimer_set(&myTimer, CLOCK_SECOND);
					myState = waitingVoltageStable;
				}
			break;

			// voltage is sattled, enable current measurement
			case waitingVoltageStable:
				if (etimer_expired(&myTimer)) {
					GPIO_SET_PIN(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);
					etimer_set(&myTimer, CLOCK_SECOND>>3);
					myState = waitingCurrentStable;
				}
			break;

			// enable comparator interrupt
			case waitingCurrentStable:
				if (etimer_expired(&myTimer)) {
					#ifdef DEBUG_EN
					GPIO_SET_PIN(GPIO_B_BASE, 0x01<<5); 
					#endif
					REG(SYSTICK_STCTRL) &= (~SYSTICK_STCTRL_INTEN);
					GPIO_CLEAR_INTERRUPT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
					GPIO_ENABLE_INTERRUPT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
					myState = waitingVoltageInt;
					gpt_enable_event(GPTIMER_2, GPTIMER_SUBTIMER_A);
				}
			break;

			// Start measure current
			case waitingVoltageInt:
				if (voltageCompInt){
					sampleCurrentWaveform();
					REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
					#ifdef DEBUG_EN
					GPIO_CLR_PIN(GPIO_B_BASE, 0x1<<3);
					#endif
					GPIO_CLR_PIN(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
					GPIO_CLR_PIN(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);
					voltageCompInt = 0;
					currentProcess(timerVal, adcVal);
					leds_toggle(LEDS_RED);
					myState = init;
					etimer_set(&myTimer, CLOCK_SECOND<<2);
				}
				// Didn't get interrupt within 16.67 ms, lost power
				else if (timeoutInt){
					timeoutInt = 0;
					REG(SYSTICK_STCTRL) |= SYSTICK_STCTRL_INTEN;
					gpt_disable_event(GPTIMER_2, GPTIMER_SUBTIMER_A);
					GPIO_CLR_PIN(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
					GPIO_CLR_PIN(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);
					GPIO_DISABLE_INTERRUPT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
					etimer_set(&myTimer, CLOCK_SECOND<<2);
					myState = init;
				}
			break;

			default:
			break;
		}
	}


  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
