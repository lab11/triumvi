/**
* \defgroup fm25cl64b
* @{
*/

#include "contiki.h"
#include "fm25cl64b.h"
#include "spi-arch.h"
#include "spi.h"
#include "dev/ssi.h"

/**
* \file   Driver for the FM25CL64B series of flash chips
* \author Ye-Sheng Kuo <samkuo@umich.edu>
*/

/**
 * \brief Initialize the FM25CL64B.
 */
void
fm25cl64b_init()
{
  #if defined(VERSION9) || defined(VERSION10) || defined(VERSION11)
  /* Set the HOLD_N and WP_N pins to outputs and high */
  GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(FM25CL64B_HOLD_N_PORT_NUM),
                  GPIO_PIN_MASK(FM25CL64B_HOLD_N_PIN));
  GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(FM25CL64B_WP_N_PORT_NUM),
                  GPIO_PIN_MASK(FM25CL64B_WP_N_PIN));
  GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(FM25CL64B_HOLD_N_PORT_NUM),
                        GPIO_PIN_MASK(FM25CL64B_HOLD_N_PIN));
  GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(FM25CL64B_WP_N_PORT_NUM),
                        GPIO_PIN_MASK(FM25CL64B_WP_N_PIN));
  GPIO_SET_PIN(GPIO_PORT_TO_BASE(FM25CL64B_HOLD_N_PORT_NUM),
               GPIO_PIN_MASK(FM25CL64B_HOLD_N_PIN));
  GPIO_SET_PIN(GPIO_PORT_TO_BASE(FM25CL64B_WP_N_PORT_NUM),
               GPIO_PIN_MASK(FM25CL64B_WP_N_PIN));
  #endif

  spi_cs_init(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);
  SPI_CS_SET(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);
}

/**
 * \brief         Read from the FRAM chip.
 * \param address The index of the byte to start reading from.
 * \param len     The number of bytes to read.
 * \param buf     A buffer to put the return data in.
 * \return        0 on success, -1 on error
 *
 *                Reads len bytes from the FRAM chip starting at address.
 */
int
fm25cl64b_read(uint16_t address, uint16_t len, uint8_t *buf)
{
  uint16_t i;

  spi_set_mode(SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);

  SPI_CS_CLR(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);

  /* Send the READ command and the address to the FRAM */
  SPI_WRITE(FM25CL64B_READ_COMMAND);
  SPI_WRITE((address&FM25CL64B_ADDRESS_MASK)>>8);
  SPI_WRITE((address&0xff));

  SPI_FLUSH();

  for (i=0; i<len; i++) {
    SPI_READ(buf[i]);
  }

  SPI_CS_SET(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);

  return 0;
}

/**
 * \brief         Write to the FRAM chip.
 * \param address The index of the byte to start writing to.
 * \param len     The number of bytes to write.
 * \param buf     A buffer of values to write.
 * \return        0 on success, -1 on error
 *
 *                Writes len bytes to the FRAM chip starting at address.
 */
int
fm25cl64b_write(uint16_t address, uint16_t len, uint8_t *buf)
{
  uint16_t i;

  spi_set_mode(SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);


  SPI_CS_CLR(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);

  /* Send the WRITE ENABLE command to allow writing to the FRAM */
  SPI_WRITE(FM25CL64B_WRITE_ENABLE_COMMAND);

  SPI_CS_SET(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);
  SPI_CS_CLR(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);

  /* Send the WRITE command and the address to the FRAM */
  SPI_WRITE(FM25CL64B_WRITE_COMMAND);
  SPI_WRITE((address&FM25CL64B_ADDRESS_MASK)>>8);
  SPI_WRITE((address&0xff));

  /* Send the data to write */
  for(i=0; i<len; i++) {
    SPI_WRITE(buf[i]);
  }

  SPI_CS_SET(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);

  return 0;
}


uint8_t fm25cl64b_readStatus(){
	uint8_t statusReg;
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);
	SPI_CS_CLR(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);
	SPI_WRITE(FM25CL64B_READ_STATUS_COMMAND);
	SPI_FLUSH();
	SPI_READ(statusReg);
	SPI_CS_SET(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);
	return statusReg;
}

int fm25cl64b_writeStatus(uint8_t statusReg){
	// Set WEL bit in status register
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);
	SPI_CS_CLR(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);
	SPI_WRITE(FM25CL64B_WRITE_ENABLE_COMMAND);
	SPI_CS_SET(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);

	SPI_CS_CLR(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);
	SPI_WRITE(FM25CL64B_WRITE_STATUS_COMMAND);
    SPI_WRITE(statusReg);
	SPI_CS_SET(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);
	return 0;
}

void fm25cl64b_eraseAll(){
	uint16_t i;
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);
	SPI_CS_CLR(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);
	SPI_WRITE(FM25CL64B_WRITE_ENABLE_COMMAND);
	SPI_CS_SET(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);

	SPI_CS_CLR(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);
	SPI_WRITE(FM25CL64B_WRITE_COMMAND);
	// Address
	SPI_WRITE(0x00);
	SPI_WRITE(0x00);

  /* Send the data to write */
	for(i=0; i<FM25CL64B_ADDRESS_MASK; i++) {
		SPI_WRITE(0x00);
	}

	SPI_CS_SET(FM25CL64B_CS_N_PORT_NUM, FM25CL64B_CS_N_PIN);
}

