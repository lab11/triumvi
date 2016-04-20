
#ifndef _AD5274_H_
#define _AD5274_H_

// Define this if using AD5272
#define AD5272

// PIN ADDR
// GND --> 0x2f
// VDD --> 0x2c
// NC  --> 0x2e
#define AD5274_CHIP_ADDR 0x2e

#define AD5274_CMD_NOP          0x00
#define AD5274_CMD_RDAC_WRITE   0x01
#define AD5274_CMD_RDAC_READ    0x02
#define AD5274_CMD_RDAC_STORE   0x03
#define AD5274_CMD_RDAC_RST     0x04
#define AD5274_CMD_TP_READ      0x05
#define AD5274_CMD_TP_ADDR_READ 0x06
#define AD5274_CMD_REG_WRITE    0x07
#define AD5274_CMD_REG_READ     0x08
#define AD5274_CMD_SHUTDOWN     0x09

#define AD5274_REG_TP_PROG_EN   0x01
#define AD5274_REG_RDAC_RP      0x02

void ad5274_init();

void ad5274_ctrl_reg_write(uint8_t control_bits);

uint8_t ad5274_ctrl_reg_read();

void ad5274_rdac_write(uint16_t rdac_val);

uint16_t ad5274_rdac_read();

// 0 --> normal
// 1 --> shutdown
void ad5274_shutdown(uint8_t shdn);

void ad5274_nop();

// store RDAC setting to the latest TP location
void ad5274_rdac_store();

uint8_t ad5274_get_last_tp_location();

#endif
