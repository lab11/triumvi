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
#include "adc.h"
#include "soc-adc.h"
#include "ioc.h"
#include "nvic.h"
#include "triumvi.h"


#include <stdio.h>
#include <stdint.h>

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

#define P_TRANSFORM2 0.855658

// The following data is accuired from 98% tile
// y = POLY_NEG_OR1*x + POLY_NEG_OR0
#define POLY_NEG_OR1 1.01
#define POLY_NEG_OR0 2.42

// y = POLY_POS_OR1*x + POLY_POS_OR0
#define POLY_POS_OR1 0.93
#define POLY_POS_OR0 2.32

static struct etimer periodic_timer;

PROCESS(wirelessMeterProcessing, "Wireless Metering Process");
AUTOSTART_PROCESSES(&wirelessMeterProcessing);


#define BUF_SIZE 113
uint16_t currentADCVal[BUF_SIZE];
uint32_t timerVal[BUF_SIZE];
uint16_t voltADCVal[BUF_SIZE];
volatile uint8_t referenceInt = 0;

uint16_t voltDataAverge(uint16_t* voltData){
	uint32_t sum = 0;
	uint8_t i;
	for (i=0; i<BUF_SIZE; i++)
		sum += voltData[i];
	return (uint16_t)(sum/BUF_SIZE);
}

int voltDataTransform(uint16_t voltReading, uint16_t voltReference){
	float voltageScaling = 1.3;
	return (int)((voltReading - voltReference)*voltageScaling);
}

void sampleCurrentVoltageWaveform(){
	uint16_t sampleCnt = 0;
	uint16_t temp;
	while (sampleCnt < BUF_SIZE){
		timerVal[sampleCnt] = get_event_time(GPTIMER_1, GPTIMER_SUBTIMER_A);
		temp = adc_get(I_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_256);
		currentADCVal[sampleCnt] = ((temp>>5)>1023)? 0 : (temp>>5);
		temp = adc_get(EXT_VOLT_IN_ADC_CHANNEL, SOC_ADC_ADCCON_REF_AVDD5, SOC_ADC_ADCCON_DIV_256);
		voltADCVal[sampleCnt] = ((temp>>5)>1023)? 0 : (temp>>5);
		sampleCnt++;
	}
}

static void referenceIntCallBack(uint8_t port, uint8_t pin){
	meterVoltageComparator(SENSE_DISABLE);
	referenceInt = 1;
	process_poll(&wirelessMeterProcessing);
}

inline int getThreshold(uint8_t inaGain){
	if (inaGain==10)
		return 500;
	else
		return 650;
}


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wirelessMeterProcessing, ev, data)
{


	PROCESS_BEGIN();

	triumviLEDinit();

	// ADC for current sense
	adc_init();
	ioc_set_over(I_ADC_GPIO_NUM, I_ADC_GPIO_PIN, IOC_OVERRIDE_ANA);
	// ADC for voltage waveform
	ioc_set_over(EXT_VOLT_IN_GPIO_NUM, EXT_VOLT_IN_GPIO_PIN, IOC_OVERRIDE_ANA);

	// set gp timer
	ungate_gpt(GPTIMER_1);
	gpt_set_mode(GPTIMER_1, GPTIMER_SUBTIMER_A, GPTIMER_TAMR_TAMR_PERIODIC);
	gpt_set_count_dir(GPTIMER_1, GPTIMER_SUBTIMER_A, GPTIMER_TnMR_TnCDIR_COUNT_DOWN);
	gpt_set_interval_value(GPTIMER_1, GPTIMER_SUBTIMER_A, 0xffffffff);
	gpt_enable_event(GPTIMER_1, GPTIMER_SUBTIMER_A);

	etimer_set(&periodic_timer, 5*CLOCK_SECOND);

	// GPIO for voltage / current power gating
	GPIO_SET_OUTPUT(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
	GPIO_SET_OUTPUT(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);

	// GPIO for MUX (select INA gain)
	GPIO_SET_OUTPUT(MUX_IO_GPIO_BASE, 0x1<<MUX_EN_GPIO_PIN);
	GPIO_SET_OUTPUT(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN);
	GPIO_SET_OUTPUT(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN);
	GPIO_CLR_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_EN_GPIO_PIN);

	// Enable MUX
	GPIO_SET_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_EN_GPIO_PIN);
	// release voltage / current power gating
	GPIO_SET_PIN(V_MEAS_EN_GPIO_BASE, 0x1<<V_MEAS_EN_GPIO_PIN);
	GPIO_SET_PIN(I_MEAS_EN_GPIO_BASE, 0x1<<I_MEAS_EN_GPIO_PIN);


	// Set voltage reference crossing as input
	// Disable pull up/down resistor
	// Enable interrupt on rising edge
	GPIO_SET_INPUT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	GPIO_DETECT_RISING(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	GPIO_DETECT_EDGE(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	GPIO_TRIGGER_SINGLE_EDGE(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	// interrupt
	gpio_register_callback(referenceIntCallBack, V_REF_CROSS_INT_GPIO_NUM, V_REF_CROSS_INT_GPIO_PIN);
	GPIO_DISABLE_INTERRUPT(V_REF_CROSS_INT_GPIO_BASE, 0x1<<V_REF_CROSS_INT_GPIO_PIN);
	nvic_interrupt_disable(V_REF_CROSS_INT_NVIC_PORT);

	uint16_t adcValue;
	//static uint32_t* soc_adc_adccon1 = (uint32_t*)SOC_ADC_ADCCON1;
	//static uint32_t* soc_adc_adccon2 = (uint32_t*)SOC_ADC_ADCCON2;
	//static uint32_t* soc_adc_adccon3 = (uint32_t*)SOC_ADC_ADCCON3;
	//static uint32_t* soc_adc_adcl = (uint32_t*)SOC_ADC_ADCL;
	//static uint32_t* soc_adc_adch = (uint32_t*)SOC_ADC_ADCH;
	//printf("ADC Reading: %u\r\n", adcVal);
	//printf("SOC_ADC_ADCCON1: %lx\r\n", *soc_adc_adccon1);
	//printf("SOC_ADC_ADCCON2: %lx\r\n", *soc_adc_adccon2);
	//printf("SOC_ADC_ADCCON3: %lx\r\n", *soc_adc_adccon3);
	//printf("SOC_ADC_ADCL: %lx\r\n", *soc_adc_adcl); 
	//printf("SOC_ADC_ADCH: %lx\r\n", *soc_adc_adch);
	
	
	uint8_t i;
	static int avgPower;
	while(1){
		PROCESS_YIELD();
		if (etimer_expired(&periodic_timer)) {
			meterVoltageComparator(SENSE_ENABLE);
			while (referenceInt==0){} 
			sampleCurrentVoltageWaveform();
			uint16_t currentRef = adc_get(V_REF_ADC_CHANNEL, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_256);
			currentRef = ((currentRef>>5)>1023)? 0 : (currentRef>>5);
			uint16_t voltRef = voltDataAverge(voltADCVal);
			int powerValid = currentVoltProcess(currentADCVal, voltADCVal, currentRef, voltRef, &avgPower);
			if (powerValid>0)
				printf("Power: %d\r\n", avgPower);
			referenceInt = 0;
			etimer_restart(&periodic_timer);
			/*
			printf("\r\nInterrupted\r\n");
			for (i=0; i<BUF_SIZE; i+=1)
				printf("current value: %u voltage value: %d\r\n", 
				currentADCVal[i], voltDataTransform(voltADCVal[i], voltAvg));
			*/
			
		/*
			adcValue = adc_get(EXT_VOLT_IN_ADC_CHANNEL, SOC_ADC_ADCCON_REF_AVDD5, SOC_ADC_ADCCON_DIV_256);
			adcValue = ((adcValue>>5) > 1023)? 0 : (adcValue>>5);
			float voltIn = ((float)adcValue)/1024*3.3;
			printf("ADC reading: %u  External volt input: %f\r\n", adcValue, voltIn);
			triumviLEDToggle();
			etimer_restart(&periodic_timer);
			*/
			
			/*
			adcValue = (adc_get(I_ADC_CHANNEL, SOC_ADC_ADCCON_REF_AVDD5, SOC_ADC_ADCCON_DIV_512)>>4);
			adcValue = (adcValue > 2048)? 0 : adcValue;
			printf("ADC Reading: %u\r\n", adcValue);
			etimer_restart(&periodic_timer);
			*/
		}
	}

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
