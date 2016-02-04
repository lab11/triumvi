
#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "dev/watchdog.c"
#include "ioc.h"
#include "spiSlave.h"
#include "stdio.h"
#include "spiDataProc.c"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include "nvic.h"
#include "cc2538-rf.h"
#include "simple_network_driver.h"
#include "rf-header-parse.h"
#include "ieee-addr.h"
#include "dev/crypto.h"
#include "dev/ccm.h"

volatile uint8_t spiCSINT = 0;
volatile static uint8_t triumviPacketReceived = 0;

static uint8_t triumviRXData[16];
static uint8_t triumviRXLen;
packet_header_t rx_pkt_header;

static void spiCScallBack(uint8_t port, uint8_t pin);
static void resetcallBack(uint8_t port, uint8_t pin);
void rf_rx_handler();

#define FIFOSIZE 8
#define SPIDEV 0


/*---------------------------------------------------------------------------*/
PROCESS(edisonIntProcess, "Blink");
PROCESS(radioRXOnlyProcess, "Radio RX Process");
AUTOSTART_PROCESSES(&edisonIntProcess);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(edisonIntProcess, ev, data) {

    PROCESS_BEGIN();
    GPIO_SET_OUTPUT(GPIO_A_BASE, 0xf8);
    GPIO_SET_OUTPUT(GPIO_B_BASE, 0x0f);
    GPIO_SET_OUTPUT(GPIO_C_BASE, 0x02);
    GPIO_CLR_PIN(GPIO_A_BASE, 0xf8);
    GPIO_CLR_PIN(GPIO_B_BASE, 0x07);
    GPIO_SET_PIN(EDISON_WAKEUP_BASE, EDISON_WAKEUP_PIN_MASK); // edison WAKEUP
    GPIO_CLR_PIN(TRIUMVI_DATA_READY_PORT_BASE, TRIUMVI_DATA_READY_MASK);

    GPIO_SET_INPUT(RESET_PORT_BASE, RESET_PIN_MASK); // reset
    ioc_set_over(RESET_PORT, RESET_PIN, IOC_OVERRIDE_PUE);

    spix_slave_init(SPIDEV);
    
    leds_off(LEDS_RED);
    leds_off(LEDS_GREEN);
    leds_off(LEDS_BLUE);

    // SPI CS interrupt
    GPIO_DETECT_EDGE(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
    GPIO_DETECT_RISING(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
    GPIO_TRIGGER_SINGLE_EDGE(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
    gpio_register_callback(spiCScallBack, SPI0_CS_PORT, SPI0_CS_PIN);
    GPIO_ENABLE_INTERRUPT(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
    nvic_interrupt_enable(SPI0_CS_NVIC_PORT);

    // RESET interrupt, active low
    GPIO_DETECT_EDGE(RESET_PORT_BASE, RESET_PIN_MASK);
    GPIO_DETECT_FALLING(RESET_PORT_BASE, RESET_PIN_MASK);
    GPIO_TRIGGER_SINGLE_EDGE(RESET_PORT_BASE, RESET_PIN_MASK);
    gpio_register_callback(resetcallBack, RESET_PORT, RESET_PIN);
    GPIO_ENABLE_INTERRUPT(RESET_PORT_BASE, RESET_PIN_MASK);
    nvic_interrupt_enable(RESET_NVIC_PORT);

    uint8_t spi_data_fifo[FIFOSIZE];

    spi_packet_t spi_rx_pkt;

    process_start(&radioRXOnlyProcess, NULL);
    while(1) {
        PROCESS_YIELD();
        if (spiCSINT){
            leds_on(LEDS_BLUE);
            spiCSINT = 0;
            if (!spix_check_rx_fifo_empty(SPIDEV)){
                spix_get_data(SPIDEV, spi_data_fifo);
                spi_packet_parse(&spi_rx_pkt, spi_data_fifo);
                switch (spi_rx_pkt.cmd){
                    // write length into tx fifo
                    case SPI_MASTER_REQ_DATA:
                        spix_put_data_single(SPIDEV, triumviRXLen);
                    break;

                    // write data into tx fifo
                    case SPI_MASTER_DUMMY:
                        spix_put_data(SPIDEV, triumviRXData, triumviRXLen);
                        GPIO_CLR_PIN(TRIUMVI_DATA_READY_PORT_BASE, TRIUMVI_DATA_READY_MASK);
                    break;

                    // do nothing...
                    case SPI_MASTER_GET_DATA:
                        //leds_on(LEDS_GREEN);
                    break;

                    default:
                    break;
                }
            }
            leds_off(LEDS_BLUE);
            nvic_interrupt_enable(SPI0_CS_NVIC_PORT);
            GPIO_ENABLE_INTERRUPT(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
        }
        else if (triumviPacketReceived==1){
            triumviPacketReceived = 0;
            GPIO_SET_PIN(TRIUMVI_DATA_READY_PORT_BASE, TRIUMVI_DATA_READY_MASK);
        }

  }
  PROCESS_END();
}

static void spiCScallBack(uint8_t port, uint8_t pin){
	GPIO_DISABLE_INTERRUPT(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
    GPIO_CLEAR_INTERRUPT(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
	nvic_interrupt_disable(SPI0_CS_NVIC_PORT);
	spiCSINT = 1;
	process_poll(&edisonIntProcess);
}

static void resetcallBack(uint8_t port, uint8_t pin){
    leds_on(LEDS_RED);
    watchdog_reboot();
}

void rf_rx_handler(){
	process_poll(&radioRXOnlyProcess);
}   

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(radioRXOnlyProcess, ev, data)
{
	PROCESS_BEGIN();
	// AES
	static uint8_t myMic[8] = {0x0};
	static uint8_t myNonce[13] = {0};
	static uint8_t aes_key[] = AES_KEY;
	aes_load_keys(aes_key, AES_KEY_STORE_SIZE_KEY_SIZE_128, 1, 0);

	simple_network_set_callback(&rf_rx_handler);
	NETSTACK_RADIO.on();
    static uint8_t srcExtAddr[8];
	static uint8_t packetPayload[128];
	static uint8_t packetDuplicated[123];
	static uint8_t* cData = packetDuplicated;
	static uint8_t* aData = srcExtAddr;

	uint16_t i = 0;                                                             

	while(1){
		PROCESS_YIELD();
		leds_on(LEDS_RED);
		uint8_t *packet_hdr = packetbuf_hdrptr();                                   
		uint8_t *packet_ptr = packetbuf_dataptr();                                  
		uint16_t packet_length = packetbuf_datalen();                               
		process_packet_header(&rx_pkt_header, packet_hdr);

		memcpy(packetPayload, packet_ptr, packet_length);
		uint8_t src_addr_len = rx_pkt_header.pkt_src_addr_len;
        triumviRXLen = 1; 
		if (src_addr_len>0){
			for (i=0; i<src_addr_len; i++){
				srcExtAddr[i] = rx_pkt_header.pkt_src_addr[src_addr_len - 1 - i];
                triumviRXData[triumviRXLen] = srcExtAddr[i];
                triumviRXLen += 1;
			}
		}

		if ((packet_ptr[0]==AES_PKT_IDENTIFIER) || (packet_ptr[0]==AES_PKT_IDENTIFIER+1) || packet_ptr[0]==TRIUMVI_PKT_IDENTIFIER){
			memcpy(myNonce, srcExtAddr, 8);
			memcpy(&myNonce[9], &packetPayload[1], 4); // Nonce[8] should be 0
			uint8_t auth_res;
			uint8_t myPDATA_LEN = PDATA_LEN;
			// Old triumvi format, without status register
			switch (packet_ptr[0]){
				case AES_PKT_IDENTIFIER:
					myPDATA_LEN -= 1;
				break;
				case AES_PKT_IDENTIFIER+1:
					myPDATA_LEN += 1;
				break;
			}
			
			uint8_t i;
			uint8_t numOfTrial;
			numOfTrial = (packet_ptr[0]==TRIUMVI_PKT_IDENTIFIER)? 2 : 1;
			for (i=0; i<numOfTrial; i++){
				memcpy(packetDuplicated, &packetPayload[5], packet_length-5);
				ccm_auth_decrypt_start(LEN_LEN, 0, myNonce, aData, ADATA_LEN, 
					cData, (myPDATA_LEN+MIC_LEN), MIC_LEN, NULL);
				while (ccm_auth_decrypt_check_status()!=AES_CTRL_INT_STAT_RESULT_AV){}
				auth_res = ccm_auth_decrypt_get_result(cData, myPDATA_LEN+MIC_LEN, myMic, MIC_LEN);
				if (auth_res==CRYPTO_SUCCESS)
					break;
				// Only for Triumvi packet identidier, since it chould have 
				// two different length
				else
					myPDATA_LEN += 2;
			}

			if (auth_res==CRYPTO_SUCCESS){
                triumviRXData[0] = packet_ptr[0];
                for (i=0; i<4; i++){
                    triumviRXData[triumviRXLen] = cData[i];
                    triumviRXLen += 1;
                }
				if (packet_ptr[0]==AES_PKT_IDENTIFIER+1){
                    triumviRXData[triumviRXLen] = cData[4]; // Panel ID
                    triumviRXData[triumviRXLen+1] = cData[5]; // Circuit ID
                    triumviRXLen += 2;
				}
				else if (packet_ptr[0]==TRIUMVI_PKT_IDENTIFIER){
                    triumviRXData[triumviRXLen] = cData[4]; // Status register
					if (cData[4]&BATTERYPACK_STATUSREG){
                        triumviRXData[triumviRXLen+1] = cData[5]; // Panel ID
                        triumviRXData[triumviRXLen+2] = cData[6]; // Circuit ID
                        triumviRXLen += 2;
					}
                    triumviRXLen += 1;
				}
                triumviPacketReceived = 1;
	            process_poll(&edisonIntProcess);
			}
		}
		leds_off(LEDS_RED);
	}


  PROCESS_END();
}
