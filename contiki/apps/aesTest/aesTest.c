
#include "contiki.h"
#include "dev/leds.h"
#include "dev/crypto.h"
#include "dev/ccm.h"
#include "dev/nvic.h"
#include "dev/gpio.h"
#include <stdio.h>
#include <string.h>

static uint8_t rtimerExpired;
static void rtimerEvent(struct rtimer *t, void *ptr);
static struct rtimer myRTimer;

/*---------------------------------------------------------------------------*/
PROCESS(aes_process, "aesTest");
AUTOSTART_PROCESSES(&aes_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(aes_process, ev, data) {

	PROCESS_BEGIN();
	crypto_init();
	REG(AES_CTRL_ALG_SEL) = 0x00000000;

	uint8_t enc_res, key_res, dec_res, auth_res;
	static uint8_t aesKey[] = {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static uint8_t myNonce[] = {0x00, 0x00, 0xf0, 0xe0, 0xd0, 0xc0, 0xb0, 0xa0,
						0x00, 0x00, 0x00, 0x00, 0x05 };
	static uint8_t aData[] = {0x0, 0x12, 0x4b, 0x0, 0x4, 0x33, 0xf0, 0xd8};
	static uint8_t myMessage[] = { 0x14, 0xaa, 0xbb, 0x00, 0x00, 0x01, 0x02, 0x03};
	static uint8_t myMic[8] = {0x0};
	static uint8_t state = 0;
	static uint8_t len_len = 2; // LVal
	static uint8_t mic_len = 4; // MVal
	static uint8_t adata_len = 0x08;
	static uint8_t pdata_len = 0x08;

	// Decryption
	static uint8_t cDataDec[24];
	static uint8_t decMic[8] = {0x0};

	// test gpio
	GPIO_SET_OUTPUT(GPIO_C_BASE, 0x1<<2);
	GPIO_CLR_PIN(GPIO_C_BASE, 0x1<<2);

	rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*10, 1, &rtimerEvent, NULL);
	uint8_t i;
	while (1){
		PROCESS_YIELD();
		switch (state){
			case 0:
				if (rtimerExpired){
					rtimerExpired = 0;
					printf("Original Message: ");
					for (i=0; i<pdata_len; i++){
						printf("0x%x ", myMessage[i]);
					}
					printf("\r\n");
					key_res = aes_load_keys(aesKey, AES_KEY_STORE_SIZE_KEY_SIZE_128, 1, 0);
					if (key_res==CRYPTO_SUCCESS)
						printf("Key load success\r\n");
					else
						printf("Key load failed, error code: %u\r\n", key_res);
					printf("\r\n");
					state = 1;
					rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*5, 1, &rtimerEvent, NULL);
				}
			break;

			case 1:
				if (rtimerExpired){
					rtimerExpired = 0;
					GPIO_SET_PIN(GPIO_C_BASE, 0x1<<2);
					enc_res = ccm_auth_encrypt_start(len_len, 0, myNonce, aData, adata_len, myMessage, pdata_len, mic_len, &aes_process);
					if (enc_res==CRYPTO_SUCCESS){
						state = 2;
					}
					else{
						printf("Encryption failed, error code: %u\r\n", enc_res);
						break;
					}
					//rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.005, 1, &rtimerEvent, NULL);
				}
			break;

			case 2:
				if (ccm_auth_encrypt_check_status()==AES_CTRL_INT_STAT_RESULT_AV){
					ccm_auth_encrypt_get_result(myMic, mic_len);
					rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.005, 1, &rtimerEvent, NULL);
					//printf("Encryption Success\r\n");
					//printf("Encrypted Message: ");
					//for (i=0; i<pdata_len; i++){
					//	printf("0x%x ", myMessage[i]);
					//}
					//printf("\r\n");
					//printf("Tag: ");
					//for (i=0; i<mic_len; i++){
					//	printf("0x%x ", myMic[i]);
					//}
					//printf("\r\n");
					//printf("\r\n");
					memcpy(cDataDec, myMessage, pdata_len);
					memcpy(cDataDec+pdata_len, myMic, mic_len);
					state = 3;
					GPIO_CLR_PIN(GPIO_C_BASE, 0x1<<2);
				}
			break;

			case 3:
				if (rtimerExpired){
					rtimerExpired = 0;
					GPIO_SET_PIN(GPIO_C_BASE, 0x1<<2);
					//printf("Encrypted messages: ");
					//for (i=0; i<pdata_len+mic_len; i++){
					//	printf("0x%x ", cDataDec[i]);
					//}
					//printf("\r\n");
					//printf("\r\n");
					dec_res = ccm_auth_decrypt_start(len_len, 0, myNonce, aData, adata_len, cDataDec, (pdata_len+mic_len), mic_len, &aes_process);
					if (dec_res==CRYPTO_SUCCESS){
						state = 4;
						nvic_interrupt_enable(NVIC_INT_AES);
					}
					else{
						printf("Decryption failed\r\n");
						break;
					}
				}
			break;

			case 4:
				if (ccm_auth_decrypt_check_status()==AES_CTRL_INT_STAT_RESULT_AV){
					//printf("Decryption successful\r\n");
					auth_res = ccm_auth_decrypt_get_result(cDataDec, pdata_len+mic_len, decMic, mic_len);
					if (auth_res==CRYPTO_SUCCESS){
						//printf("Authentication success\r\n");
						//printf("\r\n");
						//printf("Decrypted tag: ");
						//for (i=0; i<mic_len; i++){
						//	printf("0x%x ", decMic[i]);
						//}
						//printf("\r\n");
						//printf("\r\n");
						//printf("Decrypted message: ");
						//for (i=0; i<pdata_len+mic_len; i++){
						//	printf("0x%x ", cDataDec[i]);
						//}
						//printf("\r\n");
						state = 5;
						rtimer_set(&myRTimer, RTIMER_NOW()+RTIMER_SECOND*0.005, 1, &rtimerEvent, NULL);
						GPIO_CLR_PIN(GPIO_C_BASE, 0x1<<2);
					}
					else{
						printf("Authentication failed\r\n");
						printf("Decrypted tag: ");
						for (i=0; i<mic_len; i++){
							printf("0x%x ", decMic[i]);
						}
						printf("\r\n");
						printf("\r\n");
						break;
					}
				}
			break;

			case 5:
				leds_on(LEDS_RED);
			break;
		}
	}
	PROCESS_END();
}
 
static void rtimerEvent(struct rtimer *t, void *ptr){
	rtimerExpired = 1;
	process_poll(&aes_process);
}

