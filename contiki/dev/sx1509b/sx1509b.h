
#ifndef _SX1509B_H_
#define _SX1509B_H_

#define SX1509B_PORTA 0x1
#define SX1509B_PORTB 0x0

#define SX1509B_GPIO_OUTPUT 0x0
#define SX1509B_GPIO_INPUT 0x1

#define SX1509B_GPIO_PIN_SET 0x0
#define SX1509B_GPIO_PIN_CLR 0x1

#define SX1509B_OUTPUT_TYPE_PUSHPULL 0x0
#define SX1509B_OUTPUT_TYPE_OPENDRAIN 0x1

#define SX1509B_OUTPUT_RESISTOR_DISABLE 0x0
#define SX1509B_OUTPUT_RESISTOR_ENABLE 0x1

#define SX1509B_OSCI_SOURCE_OFF 0x0
#define SX1509B_OSCI_SOURCE_EXT 0x1
#define SX1509B_OSCI_SOURCE_INT 0x2

#define SX1509B_LED_DRIVER_DISABLE 0x0
#define SX1509B_LED_DRIVER_ENABLE 0x1

// I2C address
#define SX1509B_CHIP_ADDR 0x3e

// Register address
#define SX1509B_RegInputDisableB        0x00
#define SX1509B_RegInputDisableA        0x01
#define SX1509B_RegLongSlewB Output     0x02
#define SX1509B_RegLongSlewA Output     0x03
#define SX1509B_RegLowDriveB Output     0x04
#define SX1509B_RegLowDriveA Output     0x05
#define SX1509B_RegPullUpB              0x06
#define SX1509B_RegPullUpA              0x07
#define SX1509B_RegPullDownB            0x08
#define SX1509B_RegPullDownA            0x09
#define SX1509B_RegOpenDrainB           0x0A
#define SX1509B_RegOpenDrainA           0x0B
#define SX1509B_RegPolarityB            0x0C
#define SX1509B_RegPolarityA            0x0D
#define SX1509B_RegDirB                 0x0E
#define SX1509B_RegDirA                 0x0F
#define SX1509B_RegDataB                0x10
#define SX1509B_RegDataA                0x11
#define SX1509B_RegInterruptMaskB       0x12
#define SX1509B_RegInterruptMaskA       0x13
#define SX1509B_RegSenseHighB           0x14
#define SX1509B_RegSenseLowB            0x15
#define SX1509B_RegSenseHighA           0x16
#define SX1509B_RegSenseLowA            0x17
#define SX1509B_RegInterruptSourceB     0x18
#define SX1509B_RegInterruptSourceA     0x19
#define SX1509B_RegEventStatusB         0x1A
#define SX1509B_RegEventStatusA         0x1B
#define SX1509B_RegLevelShifter1        0x1C
#define SX1509B_RegLevelShifter2        0x1D
#define SX1509B_RegClock                0x1E
#define SX1509B_RegMisc                 0x1F
#define SX1509B_RegLEDDriverEnableB     0x20
#define SX1509B_RegLEDDriverEnableA     0x21

void sx1509b_init();
inline uint8_t sx1509b_gpio_read_port(uint8_t port);
inline void sx1509b_gpio_set_input(uint8_t port, uint8_t pin_mask);
inline void sx1509b_gpio_set_output(uint8_t port, uint8_t pin_mask);

inline void sx1509b_gpio_set_pin(uint8_t port, uint8_t pin_mask);
inline void sx1509b_gpio_clr_pin(uint8_t port, uint8_t pin_mask);
void sx1509b_gpio_output_type(uint8_t port, uint8_t pin_mask, uint8_t type);

void sx1509b_gpio_pullup_cfg(uint8_t port, uint8_t pin_mask, uint8_t cfg);
void sx1509b_gpio_pulldown_cfg(uint8_t port, uint8_t pin_mask, uint8_t cfg);

// LED driver
// select source for OSC, internal freq = 2 MHz
void sx1509b_oscillator_source_select(uint8_t clk_source);
// fOSCOUT = fOSC/2^(divider-1), divider <= 0xf
// fOSCOUT = 0, if divider == 0, or 0xf (configured as additional output)
// if divider == 0, output low. 0xf, output high
void sx1509b_oscillator_freq_divider(uint8_t divider);
// CLKx = fOSC/2^(divider-1), divider <= 7
void sx1509b_led_driver_freq_divider(uint8_t divider);
void sx1509b_led_driver_cfg(uint8_t port, uint8_t pin_mask, uint8_t cfg);

#endif
