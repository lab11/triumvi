#include "contiki.h"
#include "sst25vf.h"
#include "spi-arch.h"
#include "spi.h"
#include "dev/ssi.h"
#include "dev/gpio.h"
#include "dev/ioc.h"
#include "dev/leds.h"
#include "sys/clock.h"
#include <stdint.h>
#include <stdbool.h>
#include "cpu.h"

static enum {CHIP_ERASE_DONE,
	  		 SMALL_BLOCK_ERASE_DONE,
	  		 LARGE_BLOCK_ERASE_DONE,
	  		 SECTOR_ERASE_DONE,
	  		 PROGRAM_SID_DONE,
	  		 IDLE} status = IDLE;

static uint32_t i = 0;
static uint8_t  m_addr[3];

static bool on = false;

void sst25vf_default_callback() {}


static inline void set_flash_cs() {
	SPI_CS_SET(SST25VF_CS_PORT_NUM, SST25VF_CS_PIN);
}

static inline void clr_flash_cs() {
	SPI_CS_CLR(SST25VF_CS_PORT_NUM, SST25VF_CS_PIN);
}

static inline void set_flash_power_pin() {
// not on torch platform
}

static inline void clr_flash_reset_pin() {
}

static inline void clr_flash_power_pin() {
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(SST25VF_HOLD_PORT_NUM), GPIO_PIN_MASK(SST25VF_HOLD_PIN));
}

static inline void set_flash_reset_pin() {
	GPIO_SET_PIN(GPIO_PORT_TO_BASE(SST25VF_HOLD_PORT_NUM), GPIO_PIN_MASK(SST25VF_HOLD_PIN));
}

static inline void enable_writes() {
	sst25vf_ewsr();
	sst25vf_write_status_register(0);
	sst25vf_write_enable();
}


static inline void runSpiByteRx(uint8_t *cmdBuffer, uint8_t *rxBuffer, uint32_t rx_len) {
	INTERRUPTS_DISABLE();
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);
	SPI_FLUSH();
	clr_flash_cs();
	for(i = 0; i < 4; i++) {
		SPI_WRITE(cmdBuffer[i]);
	}
	SPI_FLUSH();
	for(i = 0; i < rx_len; i++) {
		SPI_READ(rxBuffer[i]);
	}
	set_flash_cs();
	INTERRUPTS_ENABLE();
}
static inline void runSpiByteRxShort(uint8_t *cmdBuffer, uint8_t *rxBuffer, uint32_t rx_len) {
	INTERRUPTS_DISABLE();
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);
	SPI_FLUSH();
	clr_flash_cs();
	for(i = 0; i < 1; i++) {
		SPI_WRITE(cmdBuffer[i]);
	}
	SPI_FLUSH();
	for(i = 0; i < rx_len; i++) {
		SPI_READ(rxBuffer[i]);
	}
	set_flash_cs();
	INTERRUPTS_ENABLE();
}

static inline void runSpiByteTx(uint8_t *cmdBuffer, uint8_t *txBuffer, uint32_t tx_len) {
	INTERRUPTS_DISABLE();
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);
	SPI_FLUSH();
	clr_flash_cs();
	for(i = 0; i < 4; i++) {
		SPI_WRITE(cmdBuffer[i]);
	}
	for(i = 0; i < tx_len; i++) {
		SPI_WRITE(txBuffer[i]);
	}
	set_flash_cs();
	INTERRUPTS_ENABLE();
}

static inline void shiftPageAddr(uint32_t user_addr) {
	m_addr[0] = user_addr >> 16;
	m_addr[1] = user_addr >> 8;
	m_addr[2] = user_addr;
}

static inline void runSingleCommand(uint8_t cmd) {
	INTERRUPTS_DISABLE();
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);
	SPI_FLUSH();
	clr_flash_cs();
	SPI_WRITE(cmd);
	set_flash_cs();
	INTERRUPTS_ENABLE();
}

void sst25vf_turn_on() {
	clr_flash_reset_pin();
	set_flash_cs();
	clr_flash_power_pin();
	clock_delay_usec(150);
	set_flash_reset_pin();
	on = true;
}

void sst25vf_turn_off() {
	set_flash_power_pin();
	clr_flash_reset_pin();
	clr_flash_cs();
	on = false;
}

uint8_t sst25vf_read_page(uint32_t addr, uint8_t *rxBuffer, uint32_t rx_len) {
	uint8_t cmdBuffer[4];
	bool auton = false;

	if(!on) {
		auton = true;
		sst25vf_turn_on();
	}

	while(1) {
		if(!(sst25vf_read_status_register() & STATUS_BUSY)) {
			break;
		}
		clock_delay_usec(5000);
	}

	shiftPageAddr(addr);
	cmdBuffer[0] = READ;
	cmdBuffer[1] = m_addr[0];
	cmdBuffer[2] = m_addr[1];
	cmdBuffer[3] = m_addr[2];

	runSpiByteRx(&cmdBuffer[0], rxBuffer, rx_len);

	while(1) {
		clock_delay_usec(2600);
		if(!(sst25vf_read_status_register() & STATUS_BUSY)) {
			break;
		}
	}

	if(auton) {
		sst25vf_turn_off();
	}

	return 1;
}

int sst25vf_read_chipid (uint8_t* id, uint8_t* mtype, uint8_t* dev) {
	uint8_t cmdBuffer[4];
	bool auton = false;
	uint8_t buf[3];

	if(!on) {
		auton = true;
		sst25vf_turn_on();
	}

	while(1) {
		if(!(sst25vf_read_status_register() & STATUS_BUSY)) {
			break;
		}
		clock_delay_usec(5000);
	}

	cmdBuffer[0] = JEDEC_ID;

	runSpiByteRxShort(&cmdBuffer[0], buf, 3);

	while(1) {
		clock_delay_usec(2600);
		if(!(sst25vf_read_status_register() & STATUS_BUSY)) {
			break;
		}
	}

	if(auton) {
		sst25vf_turn_off();
	}

	*id = buf[0];
	*mtype = buf[1];
	*dev = buf[2];

	return 0;
}

void sst25vf_read_sid(uint8_t addr, uint8_t *rxBuffer, uint8_t rx_len) {
	uint8_t cmdBuffer[3];
	cmdBuffer[0] = READ_SID;
	cmdBuffer[1] = addr;
	cmdBuffer[2] = 0;
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);
	SPI_FLUSH();
	clr_flash_cs();
	SPI_WRITE(cmdBuffer[0]);
	SPI_WRITE(cmdBuffer[1]);
	SPI_WRITE(cmdBuffer[2]);
	SPI_FLUSH();
	for(i = 0; i < rx_len; i++) {
		SPI_READ(((uint8_t *)rxBuffer)[i]);
	}
	set_flash_cs();
}

void sst25vf_program_sid(uint8_t addr, uint8_t *txBuffer, uint8_t tx_len) {
	// DON'T USE THIS. UNTESTED
	uint8_t cmdBuffer[2];
	cmdBuffer[0] = PROGRAM_SID;
	cmdBuffer[1] = addr;

	enable_writes();
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);
	SPI_FLUSH();
	clr_flash_cs();
	SPI_WRITE(cmdBuffer[0]);
	SPI_WRITE(cmdBuffer[1]);
	for(i = 0; i < tx_len; i++) {
		SPI_WRITE(((uint8_t *)txBuffer)[i]);
	}
	set_flash_cs();

	status = PROGRAM_SID_DONE;
}

uint8_t sst25vf_read_status_register() {
	INTERRUPTS_DISABLE();
	uint8_t status_buffer = 0;
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);
	SPI_FLUSH();
	clr_flash_cs();
	SPI_WRITE(RDSR);
	SPI_FLUSH();
	SPI_READ(status_buffer);
	set_flash_cs();
	INTERRUPTS_ENABLE();
	return status_buffer;

}

void sst25vf_write_enable() {
	runSingleCommand(WREN);
}

void sst25vf_write_disable() {
	runSingleCommand(WRDI);
}


uint8_t sst25vf_program(uint32_t addr,
				        uint8_t *txBuffer,
				        uint32_t tx_len) {
	uint8_t cmdBuffer[4];
	bool auton = false;

	if(!on) {
		auton = true;
		sst25vf_turn_on();
	}

	while(1) {
		if(!(sst25vf_read_status_register() & STATUS_BUSY)) {
			break;
		}
		clock_delay_usec(5000);
	}

	shiftPageAddr(addr);
	cmdBuffer[0] = PAGE_PROGRAM;
	cmdBuffer[1] = m_addr[0];
	cmdBuffer[2] = m_addr[1];
	cmdBuffer[3] = m_addr[2];

	enable_writes();
	runSpiByteTx(&cmdBuffer[0], txBuffer, tx_len);

	while(1) {
		clock_delay_usec(2600);
		if(!(sst25vf_read_status_register() & STATUS_BUSY)) {
			break;
		}
	}

	if(auton) {
		sst25vf_turn_off();
	}

	return 1;
}

uint8_t sst25vf_chip_erase() {
	uint8_t cmd = CHIP_ERASE;
	bool auton = false;

	if(!on) {
		auton = true;
		sst25vf_turn_on();
	}

	while(1) {
		if(!(sst25vf_read_status_register() & STATUS_BUSY)) {
			break;
		}
		clock_delay_usec(5000);
	}

	enable_writes();
	runSingleCommand(cmd);

	while(1) {
		clock_delay_usec(26000);
		if(!(sst25vf_read_status_register() & STATUS_BUSY)) {
			break;
		}
	}
	if(auton) {
		sst25vf_turn_off();
	}

	return 1;
}

void sst25vf_ewsr() {
	runSingleCommand(EWSR);
}

void sst25vf_write_status_register(uint8_t status_data) {
	sst25vf_ewsr();
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);
	SPI_FLUSH();
	clr_flash_cs();
	SPI_WRITE(WRSR);
	SPI_WRITE(status_data);
	set_flash_cs();
}

void sst25vf_init() {
	spi_cs_init(SST25VF_CS_PORT_NUM, SST25VF_CS_PIN);
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);

	// init pins
	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SST25VF_HOLD_PORT_NUM), GPIO_PIN_MASK(SST25VF_HOLD_PIN));
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(SST25VF_HOLD_PORT_NUM), GPIO_PIN_MASK(SST25VF_HOLD_PIN));
	GPIO_SET_PIN(GPIO_PORT_TO_BASE(SST25VF_HOLD_PORT_NUM), GPIO_PIN_MASK(SST25VF_HOLD_PIN));

	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SST25VF_WP_PORT_NUM), GPIO_PIN_MASK(SST25VF_WP_PIN));
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(SST25VF_WP_PORT_NUM), GPIO_PIN_MASK(SST25VF_WP_PIN));
	GPIO_SET_PIN(GPIO_PORT_TO_BASE(SST25VF_WP_PORT_NUM), GPIO_PIN_MASK(SST25VF_WP_PIN));

	set_flash_power_pin();
	clr_flash_cs();
	clr_flash_reset_pin();
}

uint8_t sst25vf_4kb_erase(uint32_t addr) {
	uint8_t cmdBuffer[4];

	if(sst25vf_read_status_register() & STATUS_BUSY) {
		return 0;
	}

	shiftPageAddr(addr);
	cmdBuffer[0] = SECTOR_ERASE;
	cmdBuffer[1] = m_addr[0];
	cmdBuffer[2] = m_addr[1];
	cmdBuffer[3] = m_addr[2];

	enable_writes();
	runSpiByteTx(&cmdBuffer[0], NULL, 0);

	status = SECTOR_ERASE_DONE;
	return 1;
}

uint8_t sst25vf_32kb_erase(uint32_t addr) {
	uint8_t cmdBuffer[4];

	if(sst25vf_read_status_register() & STATUS_BUSY) {
		return 0;
	}

	shiftPageAddr(addr);
	cmdBuffer[0] = SMALL_BLOCK_ERASE;
	cmdBuffer[1] = m_addr[0];
	cmdBuffer[2] = m_addr[1];
	cmdBuffer[3] = m_addr[2];

	enable_writes();
	runSpiByteTx(&cmdBuffer[0], NULL, 0);

	status = SMALL_BLOCK_ERASE_DONE;
	return 1;
}

uint8_t sst25vf_64kb_erase(uint32_t addr) {
	uint8_t cmdBuffer[4];

	if(sst25vf_read_status_register() & STATUS_BUSY) {
		return 0;
	}

	shiftPageAddr(addr);
	cmdBuffer[0] = LARGE_BLOCK_ERASE;
	cmdBuffer[1] = m_addr[0];
	cmdBuffer[2] = m_addr[1];
	cmdBuffer[3] = m_addr[2];

	enable_writes();
	runSpiByteTx(&cmdBuffer[0], NULL, 0);

	status = LARGE_BLOCK_ERASE_DONE;
	return 1;
}
