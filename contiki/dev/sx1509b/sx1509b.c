

#include <stdio.h>
#include "i2c.h"
#include "sx1509b.h"

// helper functions
uint8_t sx1509b_read_register_single(uint8_t regAddr){
	uint8_t myData;
	i2c_single_send(SX1509B_CHIP_ADDR, regAddr);
	i2c_single_receive(SX1509B_CHIP_ADDR, &myData);
	return myData;
}

void sx1509b_write_register_single(uint8_t regAddr, uint8_t regData){
	uint8_t i2cOutGoingData[2] = {regAddr, regData};
	i2c_burst_send(SX1509B_CHIP_ADDR, i2cOutGoingData, 2);
}

void sx1509b_gpio_set_input_output(uint8_t port, uint8_t pin_mask, uint8_t input_output){
	// Read direction register
	// 0 --> output, 1 --> input
	uint8_t gpio_direction = sx1509b_read_register_single(SX1509B_RegDirB+port);
	// Read input buffer
	// 0 --> input buffer enabled, 1 --> input buffer disabled
	uint8_t gpio_inputBuffer = sx1509b_read_register_single(SX1509B_RegInputDisableB+port);

	uint8_t i2cOutGoingData[4] = {(SX1509B_RegInputDisableB+port), 0,
	  (SX1509B_RegDirB+port), 0};

	// configure input buffer
	i2cOutGoingData[1] = (input_output==SX1509B_GPIO_OUTPUT)? 
	  (gpio_inputBuffer | pin_mask) : (gpio_inputBuffer & (~pin_mask));

	// configure direction
	i2cOutGoingData[3] = (input_output==SX1509B_GPIO_OUTPUT)? 
	  (gpio_direction & (~pin_mask)) : (gpio_direction | pin_mask);
	i2c_burst_send(SX1509B_CHIP_ADDR, i2cOutGoingData, 4);
}

void sx1509b_gpio_set_clr_pin(uint8_t port, uint8_t pin_mask, uint8_t set_clr){
	uint8_t gpio_port = sx1509b_read_register_single(SX1509B_RegDataB+port);
	uint8_t temp = (set_clr==SX1509B_GPIO_PIN_SET)?
	  (gpio_port | pin_mask) : (gpio_port & (~pin_mask));
	sx1509b_write_register_single((SX1509B_RegDataB+port), temp);
}
// End of helper functions

// user space APIs
void sx1509b_init(){
	i2c_init(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, 
	I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN,I2C_SCL_NORMAL_BUS_SPEED); 
}

inline uint8_t sx1509b_gpio_read_port(uint8_t port){
	return sx1509b_read_register_single(SX1509B_RegDataB+port);
}

inline void sx1509b_gpio_set_input(uint8_t port, uint8_t pin_mask){
	if (port <= 1)
		sx1509b_gpio_set_input_output(port, pin_mask, SX1509B_GPIO_INPUT);
}

inline void sx1509b_gpio_set_output(uint8_t port, uint8_t pin_mask){
	if (port <= 1)
		sx1509b_gpio_set_input_output(port, pin_mask, SX1509B_GPIO_OUTPUT);
}

inline void sx1509b_gpio_set_pin(uint8_t port, uint8_t pin_mask){
	if (port <= 1)
		sx1509b_gpio_set_clr_pin(port, pin_mask, SX1509B_GPIO_PIN_SET);
}

inline void sx1509b_gpio_clr_pin(uint8_t port, uint8_t pin_mask){
	if (port <= 1)
		sx1509b_gpio_set_clr_pin(port, pin_mask, SX1509B_GPIO_PIN_CLR);
}

void sx1509b_gpio_output_type(uint8_t port, uint8_t pin_mask, uint8_t type){
	if ((port <= 1)&&(type <= 1)){
		// 0 --> push pull output, 1 --> open drain output
		uint8_t out_type = sx1509b_read_register_single(SX1509B_RegOpenDrainB+port);
		uint8_t temp = (type == SX1509B_OUTPUT_TYPE_PUSHPULL)? 
		  (out_type & (~pin_mask)) : (out_type | pin_mask);
		sx1509b_write_register_single((SX1509B_RegOpenDrainB+port), temp);
	}
}
void sx1509b_gpio_pullup_cfg(uint8_t port, uint8_t pin_mask, uint8_t cfg){
	if ((port <= 1)&&(cfg <= 1)){
		// 0 --> disable, 1 --> enable
		uint8_t pullup_cfg = sx1509b_read_register_single(SX1509B_RegPullUpB+port);
		uint8_t temp = (cfg == SX1509B_OUTPUT_RESISTOR_DISABLE)? 
		  (pullup_cfg & ~(pin_mask)) : (pullup_cfg | pin_mask);
		sx1509b_write_register_single((SX1509B_RegPullUpB+port), temp);
	}
}
void sx1509b_gpio_pulldown_cfg(uint8_t port, uint8_t pin_mask, uint8_t cfg){
	if ((port <= 1)&&(cfg <= 1)){
		// 0 --> disable, 1 --> enable
		uint8_t pulldown_cfg = sx1509b_read_register_single(SX1509B_RegPullDownB+port);
		uint8_t temp = (cfg == SX1509B_OUTPUT_RESISTOR_DISABLE)? 
		  (pulldown_cfg & ~(pin_mask)) : (pulldown_cfg | pin_mask);
		sx1509b_write_register_single((SX1509B_RegPullDownB+port), temp);
	}
}

void sx1509b_oscillator_source_select(uint8_t clk_source){
	if (clk_source <= 2){
		uint8_t reg_clk = sx1509b_read_register_single(SX1509B_RegClock);
		sx1509b_write_register_single(SX1509B_RegClock, ((reg_clk & 0x1f)|(clk_source<<5)));
	}
}

void sx1509b_oscillator_freq_divider(uint8_t divider){
	if (divider<=0xf){
		uint8_t reg_clk = sx1509b_read_register_single(SX1509B_RegClock);
		sx1509b_write_register_single(SX1509B_RegClock, ((reg_clk & 0xf0) | divider));
	}
}

void sx1509b_led_driver_freq_divider(uint8_t divider){
	if (divider<=7){
		uint8_t reg_misc = sx1509b_read_register_single(SX1509B_RegMisc);
		sx1509b_write_register_single(SX1509B_RegMisc, ((reg_misc & 0x8f) | (divider<<4)));
	}
}

void sx1509b_led_driver_cfg(uint8_t port, uint8_t pin_mask, uint8_t cfg){
	if ((port <= 1) && (cfg<=1)){
		uint8_t led_en_reg = sx1509b_read_register_single(SX1509B_RegLEDDriverEnableB+port);
		uint8_t temp = (cfg==SX1509B_LED_DRIVER_DISABLE)? 
		  (led_en_reg & (~pin_mask)) : (led_en_reg | pin_mask);
		sx1509b_write_register_single(SX1509B_RegLEDDriverEnableB+port, temp);
	}
}


