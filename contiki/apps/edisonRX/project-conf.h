#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * @author: Brad Campbell <bradjc@umich.edu>
 */

// No need for UART
#define STARTUP_CONF_VERBOSE 0
#define UART_CONF_ENABLE 1

#define NETSTACK_CONF_NETWORK	simple_network_driver
#define CC2538_RF_CONF_SNIFFER  1

#define TRIUMVI_PKT_IDENTIFIER 0xa0
#define AES_PKT_IDENTIFIER 0xc0
#define LEN_LEN 2 // LVal
#define MIC_LEN 4 // MVal
#define ADATA_LEN 0x08
#define PDATA_LEN 0x05

// From Edison to CC2538
#define SPI0_CS_PORT_BASE               GPIO_PORT_TO_BASE(SPI0_CS_PORT)
#define SPI0_CS_PIN_MASK                GPIO_PIN_MASK(SPI0_CS_PIN)

#define RESET_PORT                      GPIO_C_NUM
#define RESET_PORT_BASE                 GPIO_PORT_TO_BASE(RESET_PORT)
#define RESET_PIN                       0
#define RESET_PIN_MASK                  GPIO_PIN_MASK(RESET_PIN)
#define RESET_NVIC_PORT                 NVIC_INT_GPIO_PORT_C

// From CC2538 to Edison
#define TRIUMVI_DATA_READY_PORT         GPIO_C_NUM
#define TRIUMVI_DATA_READY_PORT_BASE    GPIO_PORT_TO_BASE(TRIUMVI_DATA_READY_PORT)
#define TRIUMVI_DATA_READY_PIN          1
#define TRIUMVI_DATA_READY_MASK         GPIO_PIN_MASK(TRIUMVI_DATA_READY_PIN)

#define WATCHDOG_CONF_ENABLE            1

#endif /* PROJECT_CONF_H_ */

/** @} */
