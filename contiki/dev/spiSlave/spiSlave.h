
#ifndef __SPI_SLAVE_H__
#define __SPI_SLAVE_H__

#include "contiki.h"
#include "reg.h"
#include "dev/ioc.h"
#include "dev/sys-ctrl.h"
#include "dev/ssi.h"
#include "dev/gpio.h"

#ifndef SPI0_CLK_PORT
#define SPI0_CLK_PORT             (-1)
#endif
#ifndef SPI0_MOSI_PORT
#define SPI0_MOSI_PORT            (-1)
#endif
#ifndef SPI0_MISO_PORT
#define SPI0_MISO_PORT            (-1)
#endif
#ifndef SPI0_CS_PORT
#define SPI0_CS_PORT              (-1)
#endif

#ifndef SPI0_CLK_PIN
#define SPI0_CLK_PIN              (-1)
#endif
#ifndef SPI0_MOSI_PIN
#define SPI0_MOSI_PIN             (-1)
#endif
#ifndef SPI0_MISO_PIN
#define SPI0_MISO_PIN             (-1)
#endif
#ifndef SPI0_CS_PIN
#define SPI0_CS_PIN               (-1)
#endif

#ifndef SPI1_CLK_PORT
#define SPI1_CLK_PORT             (-1)
#endif
#ifndef SPI1_MOSI_PORT
#define SPI1_MOSI_PORT            (-1)
#endif
#ifndef SPI1_MISO_PORT
#define SPI1_MISO_PORT            (-1)
#endif
#ifndef SPI1_CS_PORT
#define SPI1_CS_PORT              (-1)
#endif

#ifndef SPI1_CLK_PIN
#define SPI1_CLK_PIN              (-1)
#endif
#ifndef SPI1_MOSI_PIN
#define SPI1_MOSI_PIN             (-1)
#endif
#ifndef SPI1_MISO_PIN
#define SPI1_MISO_PIN             (-1)
#endif
#ifndef SPI1_CS_PIN
#define SPI1_CS_PIN               (-1)
#endif

#define SSI_INSTANCE_COUNT 2

typedef struct{
  int8_t port;
  int8_t pin;
} spi_pad_t;

typedef struct{
  uint32_t base;
  uint32_t ioc_pxx_sel_ssi_txd;
  uint32_t ioc_clk_ssiin_ssi;
  uint32_t ioc_ssirxd_ssi;
  uint32_t ioc_ssifssin_ssi;
  spi_pad_t clk;
  spi_pad_t mosi;
  spi_pad_t miso;
  spi_pad_t cs;
} spi_slave_reg_t;

static const spi_slave_reg_t spi_slave_regs[SSI_INSTANCE_COUNT] = {
  {
    .base = SSI0_BASE,
    .ioc_pxx_sel_ssi_txd = IOC_PXX_SEL_SSI0_TXD,
    .ioc_clk_ssiin_ssi = IOC_CLK_SSIIN_SSI0,
    .ioc_ssirxd_ssi = IOC_SSIRXD_SSI0,
    .ioc_ssifssin_ssi = IOC_SSIFSSIN_SSI0,
    .clk = {SPI0_CLK_PORT, SPI0_CLK_PIN},
    .mosi = {SPI0_MOSI_PORT, SPI0_MOSI_PIN},
    .miso = {SPI0_MISO_PORT, SPI0_MISO_PIN},
    .cs = {SPI0_CS_PORT, SPI0_CS_PIN}
  },
  {
    .base = SSI1_BASE,
    .ioc_pxx_sel_ssi_txd = IOC_PXX_SEL_SSI1_TXD,
    .ioc_clk_ssiin_ssi = IOC_CLK_SSIIN_SSI1,
    .ioc_ssirxd_ssi = IOC_SSIRXD_SSI1,
    .ioc_ssifssin_ssi = IOC_SSIFSSIN_SSI1,
    .clk = {SPI1_CLK_PORT, SPI1_CLK_PIN},
    .mosi = {SPI1_MOSI_PORT, SPI1_MOSI_PIN},
    .miso = {SPI1_MISO_PORT, SPI1_MISO_PIN},
    .cs = {SPI1_CS_PORT, SPI1_CS_PIN}
  }
};


void spix_slave_init(uint8_t spi);
void spix_enable(uint8_t spi);
inline int spix_check_rx_fifo_empty(uint8_t spi);
inline int spix_check_tx_fifo_full(uint8_t spi);
uint16_t spix_get_data(uint8_t spi);
void spix_put_data(uint8_t spi, uint8_t data);

#endif


