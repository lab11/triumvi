
#include "contiki.h"
#include "sys/etimer.h"
#include "dev/gpio.h"
#include "cc2538-rf.h"
#include "i2c.h"
#include "dev/ssi.h"
#include "ioc.h"
#include "triumvi.h"
#include "sys/rtimer.h"
#include "nvic.h"
#include <stdio.h>
#include "i2cs.h"


/* Function prototype */
//static void rtimerEvent(struct rtimer *t, void *ptr);
//static void disable_all_ioc_override();
static void i2c_slave_callback();

/* Global variable */
//static struct rtimer myRTimer;


/*---------------------------------------------------------------------------*/
PROCESS(i2ctest_process, "I2C");
AUTOSTART_PROCESSES(&i2ctest_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(i2ctest_process, ev, data) {

	PROCESS_BEGIN();
	CC2538_RF_CSP_ISRFOFF();
	/*
	GPIO_SET_OUTPUT(GPIO_A_BASE, 0x63);
	GPIO_CLR_PIN(GPIO_A_BASE, 0x63);
	GPIO_SET_OUTPUT(GPIO_B_BASE, 0xf7);
	GPIO_CLR_PIN(GPIO_B_BASE, 0xf7);
	GPIO_SET_OUTPUT(GPIO_C_BASE, 0x18);
	GPIO_CLR_PIN(GPIO_C_BASE, 0x18);
	GPIO_SET_OUTPUT(GPIO_D_BASE, 0x3f);
	GPIO_CLR_PIN(GPIO_D_BASE, 0x37);
	GPIO_SET_PIN(GPIO_D_BASE, 0x08);
	disable_all_ioc_override();
	*/
	triumviLEDinit();
	i2cs_init(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, 
	          I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN);
	ioc_set_over(I2C_SCL_GPIO_NUM, I2C_SCL_GPIO_PIN, IOC_OVERRIDE_PUE);
	ioc_set_over(I2C_SDA_GPIO_NUM, I2C_SDA_GPIO_PIN, IOC_OVERRIDE_PUE);
	i2c_register_callback(i2c_slave_callback);
	i2c_slave_init(0x3e);
	i2c_slave_int_enable_ex(I2C_SLAVE_INT_DATA);
	nvic_interrupt_enable(NVIC_INT_I2C);
	uint32_t status;
	static uint8_t txCnt = 0;
	while (1){
		PROCESS_YIELD();
		status = i2c_slave_status();
		if (status & I2CS_STAT_RREQ){
			printf("DATA: %x\r\n", i2c_slave_data_get());
		}
		else if (status & I2CS_STAT_TREQ){
			i2c_slave_data_put(txCnt);
			txCnt++;
		}
		//printf("I2C Slave status register: %x\r\n", status);
		//status = i2c_slave_status();
		//printf("I2C Slave status register: %x\r\n", status);
	}
	PROCESS_END();
}

static void i2c_slave_callback(){
	i2c_slave_int_clear_ex(I2C_SLAVE_INT_DATA);
	triumviLEDToggle();
	process_poll(&i2ctest_process);
}

/*

static void rtimerEvent(struct rtimer *t, void *ptr){
	process_poll(&i2ctest_process);
}

static void disable_all_ioc_override() {
	uint8_t portnum = 0;
	uint8_t pinnum = 0;
	for(portnum = 0; portnum < 4; portnum++) {
		for(pinnum = 0; pinnum < 8; pinnum++) {
			ioc_set_over(portnum, pinnum, IOC_OVERRIDE_DIS);
		}
	}
}
*/
