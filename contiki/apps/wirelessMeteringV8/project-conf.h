#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * @author: Brad Campbell <bradjc@umich.edu>
 */

// No need for UART
#define STARTUP_CONF_VERBOSE 0
#define START_IMMEDIATELY
//#define COMPARATOR_NEGEDGE

//#define CALIBRATE
//#ifdef ERASE_FRAM
//#define RTC_SET
//#define FRAM_WRITE
//#define BLINK_LED
//#define THREEPHASE_SLAVE

#ifdef CALIBRATE
#define UART_CONF_ENABLE 1
#else
#define AES_ENABLE
#endif


#ifdef AES_ENABLE
#define AES_PKT_IDENTIFIER 0xc0
#define LEN_LEN 2 // LVal
#define MIC_LEN 4 // MVal
#define ADATA_LEN 0x08
#define PDATA_LEN 0x04
#else

// Packet structure:
// TYPE_ENERGY, ENERGY_UNIT, DATA
#define METER_TYPE_ENERGY 0xaa
#define METER_ENERGY_UNIT_mW 0x0
#define METER_DATA_OFFSET 2
#define METER_DATA_LENGTH 4
#endif

#endif /* PROJECT_CONF_H_ */

/** @} */
