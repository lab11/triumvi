
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
#include "dev/udma.h"

#include <stdio.h>


/* Triumvi received packet buffer */
typedef struct{
    uint8_t payload[16];
    uint8_t length;
} triumviPacket_t;

#define TRIUMVI_PACKET_BUF_LEN 32

static triumviPacket_t triumviRXPackets[TRIUMVI_PACKET_BUF_LEN];
static uint8_t triumviAvailIDX = 0;
static uint8_t triumviFullIDX = 0;
static uint8_t triumviRXBufFull = 0;

static uint8_t radio_packet_received = 0;

/* spi interface */
#define SPIDEV          0
#define SPIFIFOSIZE     8
#define SPI0DR          (SSI0_BASE+SSI_DR)

#define SPI0TX_DMA_FLAG (UDMA_CHCTL_DSTINC_NONE | UDMA_CHCTL_DSTSIZE_8 | UDMA_CHCTL_SRCINC_8 | UDMA_CHCTL_SRCSIZE_8 | UDMA_CHCTL_ARBSIZE_4 | UDMA_CHCTL_XFERMODE_BASIC)

typedef enum{
    INT_EDISON,
    SPI_WAIT
}spiState_t;

static spiState_t spiState = INT_EDISON;


/*---------------------------------------------------------------------------*/
/* Function prototypes */
void rf_rx_handler();
static void spiCScallBack(uint8_t port, uint8_t pin);
static void resetcallBack(uint8_t port, uint8_t pin);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(mainProcess, "Main Process");
PROCESS(decryptProcess, "Decrypt Process");
PROCESS(spiProcess, "SPI Process");
AUTOSTART_PROCESSES(&mainProcess);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mainProcess, ev, data) {
    PROCESS_BEGIN();

    GPIO_SET_OUTPUT(GPIO_A_BASE, 0xf8);
    GPIO_SET_OUTPUT(GPIO_B_BASE, 0x0f);
    GPIO_SET_OUTPUT(GPIO_C_BASE, 0x02);
    GPIO_CLR_PIN(GPIO_A_BASE, 0xf8);
    GPIO_CLR_PIN(GPIO_B_BASE, 0x07);
    GPIO_SET_PIN(EDISON_WAKEUP_BASE, EDISON_WAKEUP_PIN_MASK); // edison WAKEUP

    GPIO_SET_INPUT(RESET_PORT_BASE, RESET_PIN_MASK); // reset
    ioc_set_over(RESET_PORT, RESET_PIN, IOC_OVERRIDE_PUE);

    leds_off(LEDS_RED);
    leds_off(LEDS_GREEN);
    leds_off(LEDS_BLUE);

    // RESET interrupt, active low
    GPIO_DETECT_EDGE(RESET_PORT_BASE, RESET_PIN_MASK);
    GPIO_DETECT_FALLING(RESET_PORT_BASE, RESET_PIN_MASK);
    GPIO_TRIGGER_SINGLE_EDGE(RESET_PORT_BASE, RESET_PIN_MASK);
    gpio_register_callback(resetcallBack, RESET_PORT, RESET_PIN);
    GPIO_ENABLE_INTERRUPT(RESET_PORT_BASE, RESET_PIN_MASK);
    nvic_interrupt_enable(RESET_NVIC_PORT);

    GPIO_CLR_PIN(TRIUMVI_DATA_READY_PORT_BASE, TRIUMVI_DATA_READY_MASK);

    // SPI CS interrupt
    GPIO_DETECT_EDGE(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
    GPIO_DETECT_RISING(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
    GPIO_TRIGGER_SINGLE_EDGE(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
    gpio_register_callback(spiCScallBack, SPI0_CS_PORT, SPI0_CS_PIN);

    // SPI interface
    spix_slave_init(SPIDEV);
    spix_txdma_enable(SPIDEV);

    // uDMA
    udma_channel_disable(CC2538_SPI0_TX_DMA_CHAN);
    udma_channel_prio_set_default(CC2538_SPI0_TX_DMA_CHAN);
    udma_channel_use_primary(CC2538_SPI0_TX_DMA_CHAN);
    udma_channel_use_single(CC2538_SPI0_TX_DMA_CHAN);
    udma_channel_mask_clr(CC2538_SPI0_TX_DMA_CHAN);
    udma_set_channel_dst(CC2538_SPI0_TX_DMA_CHAN, SPI0DR);
    udma_set_channel_assignment(CC2538_SPI0_TX_DMA_CHAN, UDMA_CH11_SSI0TX);


    simple_network_set_callback(&rf_rx_handler);
    NETSTACK_RADIO.on();
    
    process_start(&decryptProcess, NULL);
    process_start(&spiProcess, NULL);


    while (1){
        PROCESS_YIELD();
        if (radio_packet_received==1){
            radio_packet_received = 0;
            process_poll(&decryptProcess);
        }
        // buffer is not empty
        if ((spiState==INT_EDISON) && ((triumviAvailIDX!=triumviFullIDX) || (triumviRXBufFull==1))){
            process_poll(&spiProcess);
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(spiProcess, ev, data) {
    PROCESS_BEGIN();
    uint8_t spi_data_fifo[SPIFIFOSIZE];
    uint8_t packetLen;
    spi_packet_t spi_rx_pkt;
    uint8_t* dma_src_end_addr;


    while (1){
        PROCESS_YIELD();
        switch (spiState){
            case INT_EDISON:
                GPIO_SET_PIN(TRIUMVI_DATA_READY_PORT_BASE, TRIUMVI_DATA_READY_MASK);
                GPIO_ENABLE_INTERRUPT(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
                nvic_interrupt_enable(SPI0_CS_NVIC_PORT);
                spiState = SPI_WAIT;
                leds_on(LEDS_BLUE);
            break;

            case SPI_WAIT:
                if (!spix_check_rx_fifo_empty(SPIDEV)){
                    spix_get_data(SPIDEV, spi_data_fifo);
                    spi_packet_parse(&spi_rx_pkt, spi_data_fifo);
                    switch (spi_rx_pkt.cmd){
                        // write length into tx fifo
                        case SPI_MASTER_REQ_DATA:
                            spix_put_data_single(SPIDEV, triumviRXPackets[triumviFullIDX].length);
                            GPIO_CLR_PIN(TRIUMVI_DATA_READY_PORT_BASE, TRIUMVI_DATA_READY_MASK);
                            leds_off(LEDS_BLUE);
                        break;

                        // write data into tx fifo
                        case SPI_MASTER_DUMMY:
                            packetLen = triumviRXPackets[triumviFullIDX].length;
                            dma_src_end_addr = triumviRXPackets[triumviFullIDX].payload + packetLen - 1;
                            udma_set_channel_src(CC2538_SPI0_TX_DMA_CHAN, (uint32_t)(dma_src_end_addr));
                            udma_set_channel_control_word(CC2538_SPI0_TX_DMA_CHAN, 
                                (SPI0TX_DMA_FLAG | udma_xfer_size(packetLen)));
                            udma_channel_enable(CC2538_SPI0_TX_DMA_CHAN);
                        break;

                        // do nothing...
                        case SPI_MASTER_GET_DATA:
                            triumviRXBufFull = 0;
                            if (triumviFullIDX == TRIUMVI_PACKET_BUF_LEN-1)
                                triumviFullIDX = 0;
                            else
                                triumviFullIDX += 1;
                            leds_off(LEDS_GREEN);
                            spiState = INT_EDISON;
                            process_poll(&mainProcess);
                        break;

                        default:
                        break;
                    }
                }
                nvic_interrupt_enable(SPI0_CS_NVIC_PORT);
                GPIO_ENABLE_INTERRUPT(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
            break;

            default:
            break;
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(decryptProcess, ev, data) {
    PROCESS_BEGIN();
    // AES
    static uint8_t myMic[8] = {0x0};
    static uint8_t myNonce[13] = {0};
    static uint8_t aes_key[] = AES_KEY;
    aes_load_keys(aes_key, AES_KEY_STORE_SIZE_KEY_SIZE_128, 1, 0);
    
    static uint8_t srcExtAddr[8];
    static uint8_t packetPayload[128];
    static uint8_t packetDuplicated[123];
    static uint8_t* cData = packetDuplicated;
    static uint8_t* aData = srcExtAddr;

    uint8_t *packet_hdr;
    uint8_t *packet_ptr;
    uint16_t packet_length;
    uint8_t src_addr_len;

    uint8_t auth_res;
    uint8_t myPDATA_LEN;

    uint16_t i;
    uint8_t tmp;

    packet_header_t rx_pkt_header;

    while(1){
        PROCESS_YIELD();
        leds_on(LEDS_RED);
        // Get data from radio buffer and parse it
        packet_hdr = packetbuf_hdrptr();                                   
        packet_ptr = packetbuf_dataptr();                                  
        packet_length = packetbuf_datalen();                               
        process_packet_header(&rx_pkt_header, packet_hdr);
        memcpy(packetPayload, packet_ptr, packet_length);
        src_addr_len = rx_pkt_header.pkt_src_addr_len;

        // data format: 
        // 1 byte ID, 
        // 8 byte source addr, 
        // 4 byte power, 
        // 1 byte status reg
        // 2 byte panel/circuit ID (optional)

        // RX buffer is not full
        if (triumviRXBufFull==0){
            triumviRXPackets[triumviAvailIDX].length = 1;
            if (src_addr_len>0){
                for (i=0; i<src_addr_len; i++){
                    srcExtAddr[i] = rx_pkt_header.pkt_src_addr[src_addr_len - 1 - i];
                    triumviRXPackets[triumviAvailIDX].payload[1+i] = srcExtAddr[i];
                }
                triumviRXPackets[triumviAvailIDX].length += src_addr_len;
            }
            // Decrypt packet
            if (packet_ptr[0]==TRIUMVI_PKT_IDENTIFIER){
                memcpy(myNonce, srcExtAddr, 8);
                memcpy(&myNonce[9], &packetPayload[1], 4); // Nonce[8] should be 0
                myPDATA_LEN = PDATA_LEN;
                for (i=0; i<2; i++){
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
                // succefully decoded the packet, pack the data into buffer
                if (auth_res==CRYPTO_SUCCESS){
                    triumviRXPackets[triumviAvailIDX].payload[0] = packet_ptr[0];
                    tmp = triumviRXPackets[triumviAvailIDX].length;
                    for (i=0; i<4; i++)
                        triumviRXPackets[triumviAvailIDX].payload[tmp+i] = cData[i];
                    triumviRXPackets[triumviAvailIDX].payload[tmp+4] = cData[4]; // Status register
                    if (cData[4]&BATTERYPACK_STATUSREG){
                        triumviRXPackets[triumviAvailIDX].payload[tmp+5] = cData[5]; // Panel ID
                        triumviRXPackets[triumviAvailIDX].payload[tmp+6] = cData[6]; // Circuit ID
                        triumviRXPackets[triumviAvailIDX].length += 2;
                    }
                    triumviRXPackets[triumviAvailIDX].length += 5;
                    
                    // check if fifo is full
                    if (((triumviAvailIDX == TRIUMVI_PACKET_BUF_LEN-1) && (triumviFullIDX == 0)) || 
                        ((triumviAvailIDX != TRIUMVI_PACKET_BUF_LEN-1) && (triumviFullIDX == triumviAvailIDX+1))){
                        triumviRXBufFull = 1;
                        leds_on(LEDS_GREEN);
                    }
                    // advance pointer
                    if (triumviAvailIDX == TRIUMVI_PACKET_BUF_LEN-1)
                        triumviAvailIDX = 0;
                    else
                        triumviAvailIDX += 1;
                }
            }
        }
        leds_off(LEDS_RED);
        process_poll(&mainProcess);
    }
    PROCESS_END();
}

void rf_rx_handler(){
    radio_packet_received = 1;
    process_poll(&mainProcess);
}   

static void spiCScallBack(uint8_t port, uint8_t pin){
    GPIO_DISABLE_INTERRUPT(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
    GPIO_CLEAR_INTERRUPT(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
    nvic_interrupt_disable(SPI0_CS_NVIC_PORT);
    process_poll(&spiProcess);
}

static void resetcallBack(uint8_t port, uint8_t pin){
    leds_on(LEDS_RED);
    watchdog_reboot();
}
