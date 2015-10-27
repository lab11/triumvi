
/* CC2538 SPI Slave driver
 *
 * Author: Ye-Sheng Kuo <samkuo@umich.edu>
 *
 */

#include "contiki.h"
#include "sys/energest.h"
#include "dev/nvic.h"
#include "lpm.h"
#include "i2c.h"
#include "i2cs.h"

static i2c_callback_t i2c_callback;

void
i2cs_init(uint8_t port_sda, uint8_t pin_sda, uint8_t port_scl, uint8_t pin_scl,
         uint32_t bus_speed)
{
  /* Enable I2C clock in different modes */
  REG(SYS_CTRL_RCGCI2C) |= 1; /* Run mode */

  /* Reset I2C peripheral */
  REG(SYS_CTRL_SRI2C) |= 1; /* Reset position */

  /* Delay for a little bit */
  clock_delay_usec(50);

  REG(SYS_CTRL_SRI2C) &= ~1;  /* Normal position */

  /* Set pins in input */
  GPIO_SET_INPUT(GPIO_PORT_TO_BASE(port_sda), GPIO_PIN_MASK(pin_sda));
  GPIO_SET_INPUT(GPIO_PORT_TO_BASE(port_scl), GPIO_PIN_MASK(pin_scl));

  /* Set peripheral control for the pins */
  GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(port_sda), GPIO_PIN_MASK(pin_sda));
  GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(port_scl), GPIO_PIN_MASK(pin_scl));

  /* Set pins as peripheral inputs */
  REG(IOC_I2CMSSDA) = ioc_input_sel(port_sda, pin_sda);
  REG(IOC_I2CMSSCL) = ioc_input_sel(port_scl, pin_scl);

  /* Set pins as peripheral outputs */
  ioc_set_sel(port_sda, pin_sda, IOC_PXX_SEL_I2C_CMSSDA);
  ioc_set_sel(port_scl, pin_scl, IOC_PXX_SEL_I2C_CMSSCL);

}

void
i2c_register_callback(i2c_callback_t f)
{
  i2c_callback = f;
}

void
i2c_slave_enable(void)
{
  REG(I2CM_CR) |= 0x00000020;
  REG(I2CS_CTRL) |= I2CS_CTRL_DA;
}

void
i2c_slave_init(uint8_t slaveAddr)
{
  i2c_slave_enable();
  REG(I2CS_OAR) = slaveAddr;
}

void
i2c_slave_disable(void)
{
  REG(I2CS_CTRL) |= 0;
  REG(I2CM_CR) &= (~0x00000020);
}

void
i2c_slave_int_enable(void)
{
  REG(I2CS_IMR) |= I2C_SLAVE_INT_DATA;
}

void
i2c_slave_int_disable(void)
{
  REG(I2CS_IMR) &= (~I2C_SLAVE_INT_DATA);
}

// interrupt Flags could be following:
// I2C_SLAVE_INT_STOP
// I2C_SLAVE_INT_START
// I2C_SLAVE_INT_DATA
void
i2c_slave_int_enable_ex(uint32_t interruptFlags)
{
  REG(I2CS_IMR) |= interruptFlags;
}

void
i2c_slave_int_disable_ex(uint32_t interruptFlags)
{
  REG(I2CS_IMR) &= (~interruptFlags);
}

void
i2c_slave_int_clear(void)
{
  REG(I2CS_ICR) = I2CS_ICR_DATAIC;
}

void
i2c_slave_int_clear_ex(uint32_t interruptFlags)
{
  REG(I2CS_ICR) = interruptFlags;
}

void
i2c_slave_data_put(uint8_t data)
{
  REG(I2CS_DR) = data;
}

uint32_t
i2c_slave_data_get(void)
{
  return REG(I2CS_DR);
}

uint32_t
i2c_slave_status(void)
{
  return REG(I2CS_STAT);
}

void i2c_isr()
{
  lpm_exit();
  (*i2c_callback)();
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
