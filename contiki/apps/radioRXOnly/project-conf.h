#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * @author: Brad Campbell <bradjc@umich.edu>
 */

#define STARTUP_CONF_VERBOSE 0
#define NETSTACK_CONF_NETWORK	simple_network_driver
#define CC2538_RF_CONF_SNIFFER  1
#define AES_ENABLE

#ifdef AES_ENABLE
#define AES_PKT_IDENTIFIER 0xc0
#define LEN_LEN 2 // LVal
#define MIC_LEN 4 // MVal
#define ADATA_LEN 0x08
#define PDATA_LEN 0x04
#endif

#endif /* PROJECT_CONF_H_ */

/** @} */
