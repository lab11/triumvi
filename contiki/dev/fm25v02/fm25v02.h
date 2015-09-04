#ifndef FM25V02_H_
#define FM25V02_H_

#define FM25V02_WRITE_ENABLE_COMMAND  0x06
#define FM25V02_WRITE_DISABLE_COMMAND 0x04
#define FM25V02_READ_STATUS_COMMAND   0x05
#define FM25V02_WRITE_STATUS_COMMAND  0x01
#define FM25V02_READ_COMMAND          0x03
#define FM25V02_FASTREAD_COMMAND      0x0b
#define FM25V02_WRITE_COMMAND         0x02
#define FM25V02_SLEEP_COMMAND         0xb9

void fm25v02_init();
int fm25v02_read(uint16_t address, uint16_t len, uint8_t *buf);
int fm25v02_write(uint16_t address, uint16_t len, uint8_t *buf);
void fm25v02_sleep();

#endif
