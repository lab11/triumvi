#include <stdint.h>

#ifndef SST25VF_H

#define SST25VF_H
#define READ                  0x03
#define FAST_READ_DUAL_IO     0xbb
#define FAST_READ_DUAL_OUTPUT 0x3b
#define HIGH_SPEED_READ       0x0b
#define SECTOR_ERASE          0x20
#define SMALL_BLOCK_ERASE     0x52
#define LARGE_BLOCK_ERASE     0xD8
#define CHIP_ERASE            0x60
#define CHIP_ERASE_2          0xc7
#define PAGE_PROGRAM          0x02
#define DUAL_IN_PAGE_PROGRAM  0xa2
#define RDSR                  0x05
#define EWSR                  0x50
#define WRSR                  0x01
#define WREN                  0x06
#define WRDI                  0x04
#define RDID                  0x90
#define RDID_2                0xab
#define JEDEC_ID              0x9f
#define EHLD                  0xaa
#define READ_SID              0x88
#define PROGRAM_SID           0xa5
#define LOCKOUT_SID           0x85
#define T_PROGRAM_SID         2
#define T_PAGE_PROGRAM        3
#define T_CHIP_ERASE		  51
#define T_BLOCK_ERASE         26
#define T_SECTOR_ERASE		  26

#define STATUS_BUSY           0x01
#define STATUS_WEL            0x02


void sst25vf_turn_on();

void sst25vf_turn_off();

uint8_t sst25vf_read_status_register();

uint8_t sst25vf_read_page(uint32_t addr, uint8_t *rxBuffer, uint32_t rx_len);

void sst25vf_read_sid(uint8_t addr, uint8_t *rxBuffer, uint8_t rx_len);

void sst25vf_program_sid(uint8_t addr, uint8_t *txBuffer, uint8_t tx_len);

void sst25vf_write_enable();

void sst25vf_write_disable();

uint8_t sst25vf_program(uint32_t addr,
				        uint8_t *txBuffer,
					    uint32_t tx_len);

uint8_t sst25vf_chip_erase();

uint8_t sst25vf_4kb_erase(uint32_t addr);

uint8_t sst25vf_32kb_erase(uint32_t addr);

uint8_t sst25vf_64kb_erase(uint32_t addr);

int sst25vf_read_chipid (uint8_t* id, uint8_t* mtype, uint8_t* dev);

void sst25vf_ewsr();

void sst25vf_write_status_register(uint8_t status_data);

void sst25vf_init();

#endif