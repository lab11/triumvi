
#include "contiki.h"
#include "rv3049.h"
#include "spi-arch.h"
#include "spi.h"
#include "dev/ssi.h"

/**
* \file   Contiki driver for the SPI based Micro Crystal RV-3049 RTC.
* \author Brad Campbell <bradjc@umich.edu>
*/


// Check that the application was compiled with the RTC constants for
// initialization
#ifndef RTC_SECONDS
#error "To use the RTC you must compile with RTC_ initial values."
#endif

uint8_t rv3049_binary_to_bcd (uint8_t binary) {
  uint8_t out = 0;

  if (binary >= 40) {
    out |= 0x40;
    binary -= 40;
  }
  if (binary >= 20) {
    out |= 0x20;
    binary -= 20;
  }
  if (binary >= 10) {
    out |= 0x10;
    binary -= 10;
  }
  out |= binary;
  return out;
}

void
rv3049_init()
{
  
  // Triumvi V9 doesn't connect to this pin
  #ifdef VERSION8
  GPIO_SET_INPUT(GPIO_PORT_TO_BASE(RV3049_INT_N_PORT_NUM),
                 GPIO_PIN_MASK(RV3049_INT_N_PIN));
  #endif

  spi_cs_init(RV3049_CS_PORT_NUM, RV3049_CS_PIN);
  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);


  // Write the initial values
  #ifdef RTC_SET
  {
    rv3049_time_t start_time = {RTC_SECONDS, RTC_MINUTES, RTC_HOURS,
                                RTC_DAYS,    RTC_WEEKDAY, RTC_MONTH,
                                RTC_YEAR};
    rv3049_set_time(&start_time);
  }
  #endif
}

int
rv3049_read_time(rv3049_time_t* time)
{
  uint8_t buf[8];
  int i;

  spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, SSI_CR0_SPH, 8);

  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Tell the RTC we want to read the clock
  SPI_WRITE(RV3049_SET_READ_BIT(RV3049_PAGE_ADDR_CLOCK));

  SPI_FLUSH();

  // Read a null byte here. Not exactly sure why.
  // only VERSION8 doesn't need this
  #ifndef VERSION8
  SPI_READ(buf[0]);
  #endif

  // Then actually read the clock
  for (i=0; i<RV3049_READ_LEN_TIME; i++) {
    SPI_READ(buf[i]);
  }

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Convert the values
  time->seconds = BCD_TO_BINARY(buf[0]);
  time->minutes = BCD_TO_BINARY(buf[1]);
  time->hours   = BCD_TO_BINARY((buf[2])&0x3F);
  time->days    = BCD_TO_BINARY(buf[3]);
  time->weekday = buf[4];
  time->month   = buf[5];
  time->year    = BCD_TO_BINARY(buf[6])+2000;

  return 0;
}

int
rv3049_set_time(rv3049_time_t* time)
{
  uint8_t buf[8];
  int i;

  buf[0] = rv3049_binary_to_bcd(time->seconds);
  buf[1] = rv3049_binary_to_bcd(time->minutes);
  buf[2] = rv3049_binary_to_bcd(time->hours); // 24 hour mode
  buf[3] = rv3049_binary_to_bcd(time->days);
  buf[4] = time->weekday;
  buf[5] = time->month;
  buf[6] = rv3049_binary_to_bcd(time->year - 2000);

  spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, SSI_CR0_SPH, 8);

  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Signal a write to the clock
  SPI_WRITE(RV3049_SET_WRITE_BIT(RV3049_PAGE_ADDR_CLOCK));

  // Write the clock values
  for (i=0; i<RV3049_WRITE_LEN_TIME; i++) {
    SPI_WRITE(buf[i]);
  }

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  return 0;
}


uint8_t rv3049_read_register(uint8_t page, uint8_t addr){
    uint8_t spi_buf;
    spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, SSI_CR0_SPH, 8);
    SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);
    SPI_WRITE(RV3049_SET_READ_BIT(page+addr));
    SPI_FLUSH();

    // 1st byte is the same as last byte from previous read
    // I guess it might be a silicon bug
    SPI_READ(spi_buf);

    // this read returns the correct value
    SPI_READ(spi_buf);

    SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);
    return spi_buf;
}

void rv3049_write_register(uint8_t page, uint8_t addr, uint8_t val){
    spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, SSI_CR0_SPH, 8);

    SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

    SPI_WRITE(RV3049_SET_WRITE_BIT(page+addr));

    SPI_WRITE(val);

    SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);
}


void rv3049_set_trickle_charge_resistor(reickle_charge_resistor_e resistor){
    uint8_t eeprom_ctrl = rv3049_read_register(RV3049_PAGE_ADDR_EEPROM_CTRL, 0x00);
    // clear top 4 bits
    eeprom_ctrl &= 0x0f;
    // set the corresponding bits
    eeprom_ctrl |= resistor;
    rv3049_write_register(RV3049_PAGE_ADDR_EEPROM_CTRL, 0x00, eeprom_ctrl);
}
