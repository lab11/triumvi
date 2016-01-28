
#include "spiSlave.h"

void spix_slave_init(uint8_t spi){
  if (spi > SSI_INSTANCE_COUNT)
    return;

  const spi_slave_reg_t* regs = &spi_slave_regs[spi];

  if (regs->clk.port < 0)
    return;

  /* Enable the clock for the SSI peripheral */
  spix_enable(spi);

  /* Start by disabling the peripheral before configuring it */
  REG(regs->base + SSI_CR1) = 0;

  /* Set to Spi Slave */
  REG(regs->base + SSI_CR1) |= SSI_CR1_MS_M;

  /* Set the IO clock as the SSI clock */
  REG(regs->base + SSI_CC) = 1;

  /* Set the mux correctly to connect the SSI pins to the correct GPIO pins */
  ioc_set_sel(regs->miso.port, regs->miso.pin, regs->ioc_pxx_sel_ssi_txd);
  REG(regs->ioc_clk_ssiin_ssi) = (regs->clk.port<<3) + regs->clk.pin;
  REG(regs->ioc_ssirxd_ssi) = (regs->mosi.port<<3) + regs->mosi.pin;
  REG(regs->ioc_ssifssin_ssi) = (regs->cs.port<<3) + regs->cs.pin;

  /* Put all the SSI gpios into peripheral mode */
  GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(regs->clk.port), 
                          GPIO_PIN_MASK(regs->clk.pin));
  GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(regs->mosi.port), 
                          GPIO_PIN_MASK(regs->mosi.pin));
  GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(regs->miso.port), 
                          GPIO_PIN_MASK(regs->miso.pin));
  GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(regs->cs.port), 
                          GPIO_PIN_MASK(regs->cs.pin));

  /* Disable any pull ups or the like */
  ioc_set_over(regs->clk.port, regs->clk.pin, IOC_OVERRIDE_DIS);
  ioc_set_over(regs->mosi.port, regs->mosi.pin, IOC_OVERRIDE_DIS);
  ioc_set_over(regs->cs.port, regs->cs.pin, IOC_OVERRIDE_DIS);

  /* Configure the clock */
  REG(regs->base + SSI_CPSR) = 8;

  /* Configure the default SPI options.
   *   mode:  Motorola frame format
   *   clock: High when idle
   *   data:  Valid on rising edges of the clock
   *   bits:  8 byte data
   */
  //REG(regs->base + SSI_CR0) = SSI_CR0_SPH | SSI_CR0_SPO | (0x07);
  // Set it to 9 bit per data, for edison extra bit
  REG(regs->base + SSI_CR0) = SSI_CR0_SPH | SSI_CR0_SPO | (0x08);

  /* Enable the SSI */
  REG(regs->base + SSI_CR1) |= SSI_CR1_SSE;
}

// release clock gating
void spix_enable(uint8_t spi){
  REG(SYS_CTRL_RCGCSSI) |= (1<<spi);
  REG(SYS_CTRL_DCGCSSI) |= (1<<spi);
  //REG(SYS_CTRL_SCGCSSI) |= (1<<spi);
}

// return 1 if rx fifo is empty
inline int spix_check_rx_fifo_empty(uint8_t spi){
  const spi_slave_reg_t* regs = &spi_slave_regs[spi];
  if ((REG(regs->base + SSI_SR) & SSI_SR_RNE_M) > 0)
    return 0;
  else
    return 1;
}

// return 1 if tx fifo is full
inline int spix_check_tx_fifo_full(uint8_t spi){
  const spi_slave_reg_t* regs = &spi_slave_regs[spi];
  if ((REG(regs->base + SSI_SR) & SSI_SR_TNF_M) > 0)
    return 0;
  else
    return 1;
  
}

uint16_t spix_get_data(uint8_t spi){
  const spi_slave_reg_t* regs = &spi_slave_regs[spi];
  uint16_t rx_data = REG(regs->base + SSI_DR) & SSI_DR_DATA_M;
  return rx_data;
}

void spix_put_data(uint8_t spi, uint8_t data){
  const spi_slave_reg_t* regs = &spi_slave_regs[spi];
  while (spix_check_tx_fifo_full(spi)){}
  REG(regs->base + SSI_DR) = data;
}

