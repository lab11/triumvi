
#include "contiki.h"
#include "cpu.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "dev/gpio.h"
#include "ioc.h"
#include "nvic.h"
#include "dev/crypto.h"
#include "dev/ccm.h"
#include "dev/nvic.h"
#include "lpm.h"
#include "cc2538-rf.h"
#include "net/netstack.h"
#include "net/packetbuf.h"


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


static struct etimer myTimer;

PROCESS(radioProcess, "Radio test");
AUTOSTART_PROCESSES(&radioProcess);

void packData(uint8_t* dest, int reading){
	uint8_t i;
	for (i=0; i<4; i++){
		dest[i] = (reading&(0xff<<(i<<3)))>>(i<<3);
	}
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(radioProcess, ev, data)
{
	PROCESS_BEGIN();

	// packet preprocessing
	static int energyData = 0;
	static uint8_t packetPayload[13] = {0};
	static uint8_t readingBuf[4];
	packetPayload[0] = 0xc0;
	static uint8_t extAddr[8];
    NETSTACK_RADIO.get_object(RADIO_PARAM_64BIT_ADDR, extAddr, 8);
	
	// AES Preprocessing
	crypto_init();
	REG(AES_CTRL_ALG_SEL) = 0x00000000;
	static uint8_t aesKey[] = AES_KEY;
	static uint8_t myMic[8] = {0x0};
	static uint8_t myNonce[13] = {0};
	memcpy(myNonce, extAddr, 8);
	static uint32_t nonceCounter = 0;
	static uint8_t len_len = 2; // LVal
	static uint8_t mic_len = 4; // MVal
	static uint8_t adata_len = 0x08;
	static uint8_t pdata_len = 0x04;
	aes_load_keys(aesKey, AES_KEY_STORE_SIZE_KEY_SIZE_128, 1, 0);

	static uint8_t* aData = myNonce;
	static uint8_t* pData = readingBuf;
	
	etimer_set(&myTimer, CLOCK_SECOND*5);
	while(1){
		PROCESS_YIELD();
		// process poll by etimer
		if (etimer_expired(&myTimer)) {
			packData(&myNonce[9], nonceCounter);
			packData(readingBuf, energyData);
			packData(&packetPayload[1], nonceCounter);
			ccm_auth_encrypt_start(len_len, 0, myNonce, aData, adata_len, 
								pData, pdata_len, mic_len, &radioProcess);
			etimer_restart(&myTimer);
			nonceCounter += 1;
			energyData += 100;
		}
		// process poll by AES
		else if (ccm_auth_encrypt_check_status()==AES_CTRL_INT_STAT_RESULT_AV){
			ccm_auth_encrypt_get_result(myMic, mic_len);
			memcpy(&packetPayload[5], readingBuf, pdata_len);
			memcpy(&packetPayload[9], myMic, mic_len);
			packetbuf_copyfrom(packetPayload, 13);
			cc2538_on_and_transmit();
			CC2538_RF_CSP_ISRFOFF();
		}
	}


  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
