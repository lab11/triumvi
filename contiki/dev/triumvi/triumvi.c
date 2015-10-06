
#include <stdint.h>
#include <stdio.h>

#include "contiki.h"
#include "fm25v02.h"
#include "rv3049.h"
#include "sx1509b.h"
#include "triumvi.h"


uint16_t getReadWritePtr(uint8_t ptrType){
	uint8_t readBuf[2];
	if (ptrType==READ_PTR_TYPE)
		fm25v02_read(FM25V02_READ_LOC_ADDR, 2, readBuf);
	else
		fm25v02_read(FM25V02_WRITE_LOC_ADDR, 2, readBuf);
	uint16_t myPtr = (readBuf[0]<<8 | readBuf[1]);
	return myPtr;
}

void triumviFramPtrClear(){
	// using Big Endianness
	uint8_t writeBuf[2];
	writeBuf[0] = (FM25V02_MIN_ADDR&0xff00)>>8;
	writeBuf[1] = FM25V02_MIN_ADDR&0xff;
	// Dummy read, wake up FRAM
	fm25v02_dummyWakeup();
	fm25v02_write(FM25V02_WRITE_LOC_ADDR, 2, writeBuf);
	fm25v02_write(FM25V02_READ_LOC_ADDR, 2, writeBuf);
	fm25v02_sleep();
}

void updatePtr(uint8_t ptrType, uint16_t readWritePtr){
	if (readWritePtr < FM25V02_MAX_ADDR)
		readWritePtr += TRIUMVI_RECORD_SIZE;
	else
		readWritePtr = FM25V02_MIN_ADDR;

	uint8_t writeBuf[2];
	writeBuf[0] = (readWritePtr & 0xff00)>>8;
	writeBuf[1] = readWritePtr & 0xff;

	if (ptrType==READ_PTR_TYPE){
		fm25v02_write(FM25V02_READ_LOC_ADDR, 2, writeBuf);
	}
	else{
		fm25v02_write(FM25V02_WRITE_LOC_ADDR, 2, writeBuf);
	}
}

// Write record into FRAM, return -1 if FRAM is full
// Otherwise, return 0 (success)
int triumviFramWrite(uint16_t powerReading, rv3049_time_t* rtctime){
	// Dummy read, wake up FRAM
	fm25v02_dummyWakeup();
	uint16_t readPtr = getReadWritePtr(READ_PTR_TYPE);
	uint16_t writePtr = getReadWritePtr(WRITE_PTR_TYPE);
	static uint8_t writeBuf[TRIUMVI_RECORD_SIZE] = {0xff};
	if (writePtr < readPtr){
		// FRAM is full
		if (readPtr == (writePtr + TRIUMVI_RECORD_SIZE))
			return -1;
	}
	else{
		// FRAM is full
		if ((readPtr==FM25V02_MIN_ADDR) && (writePtr==FM25V02_MAX_ADDR)) 
			return -1;
	}
	writeBuf[0] = (rtctime->year & 0xff0)>>4;
	writeBuf[1] = ((rtctime->year & 0xf)<<4) | ((uint8_t)rtctime->month & 0xf);
	writeBuf[2] = rtctime->days;
	writeBuf[3] = rtctime->hours;
	writeBuf[4] = rtctime->minutes;
	writeBuf[5] = rtctime->seconds;
	writeBuf[6] = (powerReading&0xff00)>>8;
	writeBuf[7] = powerReading&0xff;
	fm25v02_write(writePtr, TRIUMVI_RECORD_SIZE, writeBuf);
	updatePtr(WRITE_PTR_TYPE, writePtr);
	fm25v02_sleep();
	return 0;
}

int triumviFramRead(triumviData_t* record){
	// Dummy read, wake up FRAM
	fm25v02_dummyWakeup();
	uint16_t readPtr = getReadWritePtr(READ_PTR_TYPE);
	uint16_t writePtr = getReadWritePtr(WRITE_PTR_TYPE);
	static uint8_t readBuf[TRIUMVI_RECORD_SIZE];
	// FRAM is empty
	if (readPtr==writePtr){
		return -1;
	}
	fm25v02_read(readPtr, TRIUMVI_RECORD_SIZE, readBuf);
	record->year = readBuf[0]<<4 | ((readBuf[1]&0xf0)>>4);
	record->month = (readBuf[1] & 0xf);
	record->days = readBuf[2];
	record->hours = readBuf[3];
	record->minutes = readBuf[4];
	record->seconds = readBuf[5];
	record->powerReading = ((readBuf[6]<<8) | readBuf[7]);
	updatePtr(READ_PTR_TYPE, readPtr);
	fm25v02_sleep();
	return 0;
}

inline void triumviLEDinit(){
	GPIO_SET_OUTPUT(LED_RED_BASE, LED_RED_MASK);
	GPIO_SET_PIN(LED_RED_BASE, LED_RED_MASK);
}

inline void triumviLEDON(){
	GPIO_CLR_PIN(LED_RED_BASE, LED_RED_MASK);
}

inline void triumviLEDOFF(){
	GPIO_SET_PIN(LED_RED_BASE, LED_RED_MASK);
}

inline void triumviLEDToggle(){
	uint8_t portState = GPIO_READ_PIN(LED_RED_BASE, LED_RED_MASK);
	if (portState > 0)
		GPIO_CLR_PIN(LED_RED_BASE, LED_RED_MASK);
	else
		GPIO_SET_PIN(LED_RED_BASE, LED_RED_MASK);
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

inline uint8_t getINAIDX(){
	uint8_t mux_a0_sel = GPIO_READ_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A0_GPIO_PIN)>>MUX_A0_GPIO_PIN;
	uint8_t mux_a1_sel = GPIO_READ_PIN(MUX_IO_GPIO_BASE, 0x1<<MUX_A1_GPIO_PIN)>>MUX_A1_GPIO_PIN;
	return (mux_a1_sel<<1 | mux_a0_sel);
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

inline uint8_t getINAGain(){
	return inaGainArr[getINAIDX()];
}

void disableSPI(){
	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SPI_CLK_PORT), 0x1<<SPI_CLK_PIN);
	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SPI_MOSI_PORT), 0x1<<SPI_MOSI_PIN);
	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SPI_MISO_PORT), 0x1<<SPI_MISO_PIN);
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(SPI_CLK_PORT), 0x1<<SPI_CLK_PIN);
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(SPI_MOSI_PORT), 0x1<<SPI_MOSI_PIN);
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(SPI_MISO_PORT), 0x1<<SPI_MISO_PIN);
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(SPI_CLK_PORT), 0x1<<SPI_CLK_PIN);
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(SPI_MOSI_PORT), 0x1<<SPI_MOSI_PIN);
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(SPI_MISO_PORT), 0x1<<SPI_MISO_PIN);
}

void reenableSPI(){
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(SPI_CLK_PORT), 0x1<<SPI_CLK_PIN);
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(SPI_MOSI_PORT), 0x1<<SPI_MOSI_PIN);
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(SPI_MISO_PORT), 0x1<<SPI_MISO_PIN);
}

inline uint8_t externalVoltSel(){
	return GPIO_READ_PIN(EXT_VOLT_IN_SEL_GPIO_BASE, 0x1<<EXT_VOLT_IN_SEL_GPIO_PIN)>>EXT_VOLT_IN_SEL_GPIO_PIN;
}

inline uint8_t isButtonPressed(){
	if (GPIO_READ_PIN(MEM_RST_GPIO_BASE, 0x1<<MEM_RST_GPIO_PIN)>>MEM_RST_GPIO_PIN)
		return 0;
	else
		return 1;
}

uint8_t batteryPackIsAttached(){
	// set input, disable internal pull-up
	// If external battery pack is attached, I2C should be hold high
	GPIO_SET_INPUT(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
	ioc_set_over(I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN, IOC_OVERRIDE_DIS);
	uint8_t result = GPIO_READ_PIN(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN)>>I2C_SCL_GPIO_PIN;
	// If pin is high, set both pins input and returns. 
	if (result>0){
		GPIO_SET_INPUT(I2C_SDA_GPIO_BASE, 0x1<<I2C_SDA_GPIO_PIN);
		ioc_set_over(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, IOC_OVERRIDE_DIS);
		return 1;
	}
	// Otherwise, set it to output and drive it low
	GPIO_SET_OUTPUT(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
	GPIO_CLR_PIN(I2C_SCL_GPIO_BASE, 0x1<<I2C_SCL_GPIO_PIN);
	return 0;

}

void batteryPackInit(){
	// I2C init
	sx1509b_init();
	// Port A pin 0 is connected to VUSB directly
	sx1509b_high_voltage_input_enable(SX1509B_PORTA, 0x1, SX1509B_HIGH_INPUT_ENABLE);
	// Setup RGB led IOs
	sx1509b_gpio_pullup_cfg(SX1509B_PORTA, 0x0e, SX1509B_OUTPUT_RESISTOR_DISABLE);
	sx1509b_gpio_set_output(SX1509B_PORTA, 0x0e);
	sx1509b_gpio_output_type(SX1509B_PORTA, 0x0e, SX1509B_OUTPUT_TYPE_OPENDRAIN);
	sx1509b_gpio_set_pin(SX1509B_PORTA, 0x0e);
	// Disable all pull-up and pull-down resistors
	sx1509b_gpio_pullup_cfg(SX1509B_PORTA, 0xff, SX1509B_OUTPUT_RESISTOR_DISABLE);
	sx1509b_gpio_pullup_cfg(SX1509B_PORTB, 0xff, SX1509B_OUTPUT_RESISTOR_DISABLE);
	sx1509b_gpio_pulldown_cfg(SX1509B_PORTA, 0xff, SX1509B_OUTPUT_RESISTOR_DISABLE);
	sx1509b_gpio_pulldown_cfg(SX1509B_PORTB, 0xff, SX1509B_OUTPUT_RESISTOR_DISABLE);
}

uint8_t batteryPackReadPanelID(){
	// enable pull-up resistor before read IO state
	sx1509b_gpio_pullup_cfg(SX1509B_PORTA, 0xf0, SX1509B_OUTPUT_RESISTOR_ENABLE);
	uint8_t panelID = 15 - (sx1509b_gpio_read_port(SX1509B_PORTA)>>4);
	sx1509b_gpio_pullup_cfg(SX1509B_PORTA, 0xf0, SX1509B_OUTPUT_RESISTOR_DISABLE);
	return panelID;
}

uint8_t batteryPackReadCircuitID(){
	sx1509b_gpio_pullup_cfg(SX1509B_PORTB, 0xff, SX1509B_OUTPUT_RESISTOR_ENABLE);
	uint8_t portBReg = sx1509b_gpio_read_port(SX1509B_PORTB);
	uint8_t circuitID = (15 - (portBReg&0x0f)) + (15 - (portBReg>>4))*10;
	sx1509b_gpio_pullup_cfg(SX1509B_PORTB, 0xff, SX1509B_OUTPUT_RESISTOR_DISABLE);
	return circuitID;
}

void batteryPackLEDOn(uint8_t leds){
	uint8_t myLED = (leds&0x0e);
	sx1509b_gpio_clr_pin(SX1509B_PORTA, myLED);
}

void batteryPackLEDOff(uint8_t leds){
	uint8_t myLED = (leds&0x0e);
	sx1509b_gpio_set_pin(SX1509B_PORTA, myLED);
}

void batteryPackLEDToggle(uint8_t leds){
	uint8_t ledState = (sx1509b_gpio_read_port(SX1509B_PORTA) & 0x0e);
	switch (leds){
		case BATTERY_PACK_LED_RED: 
			if (ledState & 0x08)
				sx1509b_gpio_clr_pin(SX1509B_PORTA, 0x08);
			else
				sx1509b_gpio_set_pin(SX1509B_PORTA, 0x08);
		break;
		case BATTERY_PACK_LED_GREEN:
			if (ledState & 0x04)
				sx1509b_gpio_clr_pin(SX1509B_PORTA, 0x04);
			else
				sx1509b_gpio_set_pin(SX1509B_PORTA, 0x04);
		break;
		case BATTERY_PACK_LED_BLUE:
			if (ledState & 0x02)
				sx1509b_gpio_clr_pin(SX1509B_PORTA, 0x02);
			else
				sx1509b_gpio_set_pin(SX1509B_PORTA, 0x02);
		break;
		default:
		break;
	}
}

uint8_t batteryPackIsUSBAttached(){
	uint8_t portAReg = sx1509b_gpio_read_port(SX1509B_PORTA);
	if (portAReg & 0x1)
		return 1;
	else
		return 0;
}

void batteryPackLEDDriverConfig(){
	// select internal 2 MHz oscillator
	sx1509b_oscillator_source_select(SX1509B_OSCI_SOURCE_INT);
	// set fOSCOUT = 2 MHz / 2 ^ (12-1) = 977 Hz
	sx1509b_oscillator_freq_divider(12);
	// set CLKx to 977 / 2 ^ (1-1) = 977 Hz
	sx1509b_led_driver_freq_divider(1);
	// Enable LED driver for RGB LEDs
	sx1509b_led_driver_enable(SX1509B_PORTA, 0x0e, SX1509B_LED_DRIVER_ENABLE);
}

void batteryPackLEDIntensityDecrease(uint8_t leds){

}

void batteryPackLEDIntensityIncrease(uint8_t leds){
}


