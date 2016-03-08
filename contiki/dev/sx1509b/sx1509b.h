
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
#define SX1509B_HIGH_INPUT_DISABLE 0x0
#define SX1509B_HIGH_INPUT_ENABLE 0x1

// I2C address
#define SX1509B_CHIP_ADDR 0x3e

// General Registers
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

// LED Driver registers
#define SX1509B_RegTOn0        0x29
#define SX1509B_RegIOn0        0x2A
#define SX1509B_RegOff0        0x2B
#define SX1509B_RegTOn1        0x2C
#define SX1509B_RegIOn1        0x2D
#define SX1509B_RegOff1        0x2E
#define SX1509B_RegTOn2        0x2F
#define SX1509B_RegIOn2        0x30
#define SX1509B_RegOff2        0x31
#define SX1509B_RegTOn3        0x32
#define SX1509B_RegIOn3        0x33
#define SX1509B_RegOff3        0x34
#define SX1509B_RegTOn4        0x35
#define SX1509B_RegIOn4        0x36
#define SX1509B_RegOff4        0x37
#define SX1509B_RegTRise4      0x38
#define SX1509B_RegTFall4      0x39
#define SX1509B_RegTOn5        0x3A
#define SX1509B_RegIOn5        0x3B
#define SX1509B_RegOff5        0x3C
#define SX1509B_RegTRise5      0x3D
#define SX1509B_RegTFall5      0x3E
#define SX1509B_RegTOn6        0x3F
#define SX1509B_RegIOn6        0x40
#define SX1509B_RegOff6        0x41
#define SX1509B_RegTRise6      0x42
#define SX1509B_RegTFall6      0x43
#define SX1509B_RegTOn7        0x44
#define SX1509B_RegIOn7        0x45
#define SX1509B_RegOff7        0x46
#define SX1509B_RegTRise7      0x47
#define SX1509B_RegTFall7      0x48
#define SX1509B_RegTOn8        0x49
#define SX1509B_RegIOn8        0x4A
#define SX1509B_RegOff8        0x4B
#define SX1509B_RegTOn9        0x4C
#define SX1509B_RegIOn9        0x4D
#define SX1509B_RegOff9        0x4E
#define SX1509B_RegTOn10       0x4F
#define SX1509B_RegIOn10       0x50
#define SX1509B_RegOff10       0x51
#define SX1509B_RegTOn11       0x52
#define SX1509B_RegIOn11       0x53
#define SX1509B_RegOff11       0x54
#define SX1509B_RegTOn12       0x55
#define SX1509B_RegIOn12       0x56
#define SX1509B_RegOff12       0x57
#define SX1509B_RegTRise12     0x58
#define SX1509B_RegTFall12     0x59
#define SX1509B_RegTOn13       0x5A
#define SX1509B_RegIOn13       0x5B
#define SX1509B_RegOff13       0x5C
#define SX1509B_RegTRise13     0x5D
#define SX1509B_RegTFall13     0x5E
#define SX1509B_RegTOn14       0x5F
#define SX1509B_RegIOn14       0x60
#define SX1509B_RegOff14       0x61
#define SX1509B_RegTRise14     0x62
#define SX1509B_RegTFall14     0x63
#define SX1509B_RegTOn15       0x64
#define SX1509B_RegIOn15       0x65
#define SX1509B_RegOff15       0x66
#define SX1509B_RegTRise15     0x67
#define SX1509B_RegTFall15     0x68

// Miscellaneous registers
#define SX1509B_RegHighInputB  0x69
#define SX1509B_RegHighInputA  0x6A
#define SX1509B_RegReset       0x7D

void sx1509b_init();
uint8_t sx1509b_gpio_read_port(uint8_t port);
void sx1509b_gpio_set_input(uint8_t port, uint8_t pin_mask);
void sx1509b_gpio_set_output(uint8_t port, uint8_t pin_mask);

void sx1509b_gpio_set_pin(uint8_t port, uint8_t pin_mask);
void sx1509b_gpio_clr_pin(uint8_t port, uint8_t pin_mask);
void sx1509b_gpio_write_port(uint8_t port, uint8_t pin_mask, uint8_t val);
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
void sx1509b_led_driver_enable(uint8_t port, uint8_t pin_mask, uint8_t cfg);

// val <= 31,
// val = 0 --> static mode
// 1  <= val <= 15 --> TOn =  64 * val * (255/CLKx)
// 16 <= val <= 31 --> Ton = 512 * val * (255/CLKx)
void sx1509b_led_driver_TON(uint8_t pin, uint8_t val);

uint8_t sx1509b_led_driver_get_ION(uint8_t pin);
void sx1509b_led_driver_set_ION(uint8_t pin, uint8_t val);

// val <= 31,
// val = 0 --> static mode
// 1  <= val <= 15 --> TOff =  64 * val * (255/CLKx)
// 16 <= val <= 31 --> Toff = 512 * val * (255/CLKx)
void sx1509b_led_driver_TOFF(uint8_t pin, uint8_t val);

// val <= 7
// IOff = 4 * val
void sx1509b_led_driver_IOFF(uint8_t pin, uint8_t val);

// val <= 31
// val = 0 --> Off
void sx1509b_led_driver_TRise(uint8_t pin, uint8_t val);
void sx1509b_led_driver_TFall(uint8_t pin, uint8_t val);

void sx1509b_high_voltage_input_enable(uint8_t port, uint8_t pin_mask, uint8_t cfg);

void sx1509b_software_reset();

#endif
