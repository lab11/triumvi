#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * @author: Brad Campbell <bradjc@umich.edu>
 */

// No need for UART
#define STARTUP_CONF_VERBOSE 0

#define NETSTACK_CONF_NETWORK	simple_network_driver
#define CC2538_RF_CONF_SNIFFER  1

//#define FRAM_WRITE

//#define DATADUMP
//#define DATADUMP2
//#define DEBUG_ON

//#define AVG_VREF      // calculate the reference by averaging a cycle

#if defined(DATADUMP) || defined(DATADUMP2) || defined(DEBUG_ON) || defined(DATADUMP3)
#define UART_CONF_ENABLE 1
#endif

#define TRIUMVI_PKT_IDENTIFIER 0xa0
#define LEN_LEN 2 // LVal
#define MIC_LEN 4 // MVal
#define ADATA_LEN 0x08
#define PDATA_LEN 0x05

#define TRIUMVI_PKT_WAVE_IDENTIFIER0 0xa1
#define TRIUMVI_PKT_WAVE_IDENTIFIER1 0x5e

#define TRIUMVI_PKT_CALIBRATIONCOEF_IDENTIFIER0 0xa1
#define TRIUMVI_PKT_CALIBRATIONCOEF_IDENTIFIER1 0x5d


// Version options
#define VERSION10
//#define VERSION11
//#define VERSION12

// FRAM options
#define FM25CL64B
//#define FM25V02

// Other options
//#define FIFTYHZ
#define POLYFIT
#define RTC_ENABLE
#define COUNTER_ENABLE
#define AMPLITUDE_CALIBRATION_EN
//#define TRANSMIT_WAVEFORM
//#define CHARGING_ENABLE
//#define THREEPHASE_DELTA_CONFIG


// Use custom address for this address
// make ID=c0:98:e5:54:52:a0:00:01
#define IEEE_ADDR_CONF_USE_SECONDARY_LOCATION 1

#endif /* PROJECT_CONF_H_ */

/** @} */
