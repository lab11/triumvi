#ifndef FM25CL64B_H_
#define FM25CL64B_H_

#define FM25CL64B_WRITE_ENABLE_COMMAND  0x06
#define FM25CL64B_WRITE_DISABLE_COMMAND 0x04
#define FM25CL64B_READ_STATUS_COMMAND   0x05
#define FM25CL64B_WRITE_STATUS_COMMAND  0x01
#define FM25CL64B_READ_COMMAND          0x03
#define FM25CL64B_WRITE_COMMAND         0x02
#define FM25CL64B_ADDRESS_MASK          0x1fff

void fm25cl64b_init();
int fm25cl64b_read(uint16_t address, uint16_t len, uint8_t *buf);
int fm25cl64b_write(uint16_t address, uint16_t len, uint8_t *buf);
uint8_t fm25cl64b_readStatus();
int fm25cl64b_writeStatus(uint8_t statusReg);
void fm25cl64b_eraseAll();

#endif
