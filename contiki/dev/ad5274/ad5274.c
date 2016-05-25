
#include <stdio.h>
#include "i2c.h"
#include "ad5274.h"

void ad5274_init(){
    i2c_init(AD527X_SDA_GPIO_NUM, AD527X_SDA_GPIO_PIN, 
             AD527X_SCL_GPIO_NUM, AD527X_SCL_GPIO_PIN, I2C_SCL_NORMAL_BUS_SPEED); 
}

void ad5274_nop(){
    uint8_t i2cOutGoingData[2] = {AD5274_CMD_NOP<<2, 0x00};
    i2c_burst_send(AD5274_CHIP_ADDR, i2cOutGoingData, 2);
}

void ad5274_software_reset(){
    uint8_t i2cOutGoingData[2] = {AD5274_CMD_RDAC_RST<<2, 0x00};
    i2c_burst_send(AD5274_CHIP_ADDR, i2cOutGoingData, 2);
}

void ad5274_ctrl_reg_write(uint8_t control_bits){
    uint8_t i2cOutGoingData[2] = {AD5274_CMD_REG_WRITE<<2, control_bits};
    i2c_burst_send(AD5274_CHIP_ADDR, i2cOutGoingData, 2);
}

uint8_t ad5274_ctrl_reg_read(){
    uint8_t i2cOutGoingData[2] = {AD5274_CMD_REG_READ<<2, 0x00};
    uint8_t i2cIncomingData[2];
    i2c_burst_send(AD5274_CHIP_ADDR, i2cOutGoingData, 2);
    i2c_burst_receive(AD5274_CHIP_ADDR, i2cIncomingData, 2);
    return i2cIncomingData[1] & 0x0f;
}

void ad5274_rdac_write(uint16_t rdac_val){
    uint8_t i2cOutGoingData[2] = {AD5274_CMD_RDAC_WRITE<<2, 0x00};
    #ifndef AD5272
    i2cOutGoingData[0] |= ((rdac_val & 0xc0)>>6);
    i2cOutGoingData[1] = ((rdac_val & 0x3f)<<2);
    #else
    i2cOutGoingData[0] |= ((rdac_val & 0x300)>>8);
    i2cOutGoingData[1] = (rdac_val & 0xff);
    #endif
    i2c_burst_send(AD5274_CHIP_ADDR, i2cOutGoingData, 2);
}

uint16_t ad5274_rdac_read(){
    uint8_t i2cOutGoingData[2] = {AD5274_CMD_RDAC_READ<<2, 0x00};
    uint8_t i2cIncomingData[2];
    i2c_burst_send(AD5274_CHIP_ADDR, i2cOutGoingData, 2);
    i2c_burst_receive(AD5274_CHIP_ADDR, i2cIncomingData, 2);
    return ((i2cIncomingData[0] & 0x03)<<8) | i2cIncomingData[1];
}

void ad5274_shutdown(uint8_t shdn){
    uint8_t i2cOutGoingData[2] = {AD5274_CMD_SHUTDOWN<<2, (shdn&0x01)};
    i2c_burst_send(AD5274_CHIP_ADDR, i2cOutGoingData, 2);
}

void ad5274_rdac_store(){
    uint8_t i2cOutGoingData[2] = {AD5274_CMD_RDAC_STORE<<2, 0x00};
    i2c_burst_send(AD5274_CHIP_ADDR, i2cOutGoingData, 2);
}

uint8_t ad5274_get_last_tp_location(){
    uint8_t i2cOutGoingData[2] = {AD5274_CMD_TP_ADDR_READ<<2, 0x00};
    uint8_t i2cIncomingData[2];
    i2c_burst_send(AD5274_CHIP_ADDR, i2cOutGoingData, 2);
    i2c_burst_receive(AD5274_CHIP_ADDR, i2cIncomingData, 2);
    return (i2cIncomingData[1]&0x7f);
}
