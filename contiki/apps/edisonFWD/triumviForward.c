
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
#include "softwareRTC.c"


/* spi interface */
#define SPIDEV          0
#define SPIFIFOSIZE     128                 // maximum receiving buffer size per spi transaction
#define SPI0DR          (SSI0_BASE+SSI_DR)  // SPIDEV fifo address
#define SPI_DELAY       CLOCK_SECOND*0.02   // delay between asserting next interrupt

#define SPI0TX_DMA_FLAG (UDMA_CHCTL_DSTINC_NONE | \
                        UDMA_CHCTL_DSTSIZE_8 | \
                        UDMA_CHCTL_SRCINC_8 | \
                        UDMA_CHCTL_SRCSIZE_8 | \
                        UDMA_CHCTL_ARBSIZE_4 | \
                        UDMA_CHCTL_XFERMODE_BASIC)

#define TRIUMVI_PACKET_BUF_LEN 32
#define RESET_THRESHOLD 32  // if edison did not respond within 32 packets, reset CC2538

/* Triumvi received packet buffer */
typedef struct{
    uint8_t payload[SPIFIFOSIZE];
    uint8_t length;
} triumviPacket_t;

typedef enum{
    SPI_RESET,
    SPI_WAIT,
}spiState_t;


static triumviPacket_t triumviRXPackets[TRIUMVI_PACKET_BUF_LEN];
static uint8_t triumviAvailIDX = 0;
static uint8_t triumviFullIDX = 0;
static uint8_t triumviRXBufFull = 0;

static uint8_t spi_rxfifo_halffull = 0;
static uint8_t spi_cs_int = 0;
static uint8_t spiInUse = 0;

static uint8_t resetCnt = 0;

//#define LED_DEBUG

static spiState_t spiState = SPI_WAIT;

static struct etimer software_RTC_timer;
#define SOFTWARE_RTC_TICK 1
#define TRIUMVI_RTC 172
#define TRIUMVI_RTC_SET 255
#define TRIUMVI_RTC_REQ 254

/*---------------------------------------------------------------------------*/
/* Function prototypes */
void rf_rx_handler();
static void spiCScallBack(uint8_t port, uint8_t pin);
static void resetcallBack(uint8_t port, uint8_t pin);
static void spiFIFOcallBack();
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(mainProcess, "Main Process");
PROCESS(decryptProcess, "Decrypt Process");
PROCESS(spiProcess, "SPI Process");
PROCESS(rtcProcess, "Software RTC Process");
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
    GPIO_ENABLE_INTERRUPT(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);

    // SPI interface
    spix_slave_init(SPIDEV);
    spix_txdma_enable(SPIDEV);
    spi_register_callback(spiFIFOcallBack);
    spix_interrupt_enable(SPIDEV, SSI_IM_RXIM_M); // RX FIFO half full
    nvic_interrupt_enable(NVIC_INT_SSI0);

    // uDMA SPI0 TX
    udma_channel_disable(CC2538_SPI0_TX_DMA_CHAN);
    udma_channel_prio_set_default(CC2538_SPI0_TX_DMA_CHAN);
    udma_channel_use_primary(CC2538_SPI0_TX_DMA_CHAN);
    udma_channel_use_single(CC2538_SPI0_TX_DMA_CHAN);
    udma_channel_mask_clr(CC2538_SPI0_TX_DMA_CHAN);
    udma_set_channel_dst(CC2538_SPI0_TX_DMA_CHAN, SPI0DR);
    udma_set_channel_assignment(CC2538_SPI0_TX_DMA_CHAN, UDMA_CH11_SSI0TX);


    simple_network_set_callback(&rf_rx_handler);
    //NETSTACK_RADIO.off();
    
    process_start(&decryptProcess, NULL);
    process_start(&spiProcess, NULL);
    process_start(&rtcProcess, NULL);


    while (1){
        PROCESS_YIELD();
        // buffer is not empty, spi is not in use
        //if ((spiInUse==0) && (spix_busy(SPIDEV)==0) && ((triumviAvailIDX!=triumviFullIDX) || (triumviRXBufFull==1))){
        if ((spiInUse==0) && ((triumviAvailIDX!=triumviFullIDX) || (triumviRXBufFull==1))){
            GPIO_SET_PIN(TRIUMVI_DATA_READY_PORT_BASE, TRIUMVI_DATA_READY_MASK);
            if (triumviRXBufFull==1){
                resetCnt += 1;
                if (resetCnt==RESET_THRESHOLD){
                    watchdog_reboot();
                }
            }
            #ifdef LED_DEBUG
            leds_off(LEDS_RED);
            leds_off(LEDS_GREEN);
            leds_off(LEDS_BLUE);
            #endif
        }
        // Fail safe, CC2538 missing some SPI commands, reset spi state
        else if (triumviRXBufFull==1){
            spiState = SPI_RESET;
            process_poll(&spiProcess);
            #ifdef LED_DEBUG
            leds_off(LEDS_RED);
            leds_off(LEDS_GREEN);
            leds_on(LEDS_BLUE);
            #endif
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(spiProcess, ev, data) {
    PROCESS_BEGIN();
    uint8_t spi_data_fifo[SPIFIFOSIZE];
    uint8_t packetLen;
    static spi_packet_t spi_rx_pkt;
    uint8_t* dma_src_end_addr;
    static uint8_t spi_data_ptr = 0;
    uint8_t proc_idx;
    static uint8_t rf_pkt_payload[128] = {0};
    static uint8_t rf_pkt_len = 0;
    static myTime_t receivedTime;


    while (1){
        PROCESS_YIELD();
        switch (spiState){
            case SPI_RESET:
                spi_cs_int = 0;
                spi_rxfifo_halffull = 0; 
                spiInUse = 0;
                spi_data_ptr = 0;
                spix_interrupt_enable(SPIDEV, SSI_IM_RXIM_M);
                spiState = SPI_WAIT;
                process_poll(&mainProcess);
                #ifdef LED_DEBUG
                leds_off(LEDS_RED);
                leds_on(LEDS_GREEN);
                leds_on(LEDS_BLUE);
                #endif
            break;

            /* Wait SPI from Edison*/
            case SPI_WAIT:
                if (spi_rxfifo_halffull==1){
                    spi_rxfifo_halffull = 0; 
                    spi_data_ptr += spix_get_data(SPIDEV, spi_data_fifo+spi_data_ptr);
                    spix_interrupt_enable(SPIDEV, SSI_IM_RXIM_M);
                    spiInUse = 1;
                }
                if (spi_cs_int==1){
                    #ifndef LED_DEBUG
                    leds_on(LEDS_BLUE);
                    #endif
                    spi_data_ptr += spix_get_data(SPIDEV, spi_data_fifo+spi_data_ptr);
                    spi_cs_int = 0;
                    spiInUse = 1;
                    proc_idx = 0;
                    while (proc_idx < spi_data_ptr){
                        proc_idx += spi_packet_parse(&spi_rx_pkt, spi_data_fifo+proc_idx);
                        switch (spi_rx_pkt.cmd){
                            // write length and data into tx fifo
                            case SPI_MASTER_REQ_DATA:
                                packetLen = triumviRXPackets[triumviFullIDX].length;
                                spix_put_data_single(SPIDEV, packetLen);
                                dma_src_end_addr = triumviRXPackets[triumviFullIDX].payload + packetLen - 1;
                                udma_set_channel_src(CC2538_SPI0_TX_DMA_CHAN, (uint32_t)(dma_src_end_addr));
                                udma_set_channel_control_word(CC2538_SPI0_TX_DMA_CHAN, 
                                    (SPI0TX_DMA_FLAG | udma_xfer_size(packetLen)));
                                udma_channel_enable(CC2538_SPI0_TX_DMA_CHAN);
                                GPIO_CLR_PIN(TRIUMVI_DATA_READY_PORT_BASE, TRIUMVI_DATA_READY_MASK);
                            break;

                            // do nothing...
                            case SPI_MASTER_DUMMY:
                            break;

                            // spi transmission is completed, advances pointer
                            case SPI_MASTER_GET_DATA:
                                if ((triumviAvailIDX!=triumviFullIDX) || (triumviRXBufFull==1)){
                                    triumviRXBufFull = 0;
                                    if (triumviFullIDX == TRIUMVI_PACKET_BUF_LEN-1)
                                        triumviFullIDX = 0;
                                    else
                                        triumviFullIDX += 1;
                                    #ifndef LED_DEBUG
                                    leds_off(LEDS_GREEN);
                                    #endif
                                }
                                resetCnt  = 0;
                                spiInUse = 0;
                            break;

                            case SPI_MASTER_RADIO_ON:
                                NETSTACK_RADIO.on();
                                spiInUse = 0;
                            break;

                            case SPI_MASTER_RADIO_OFF:
                                NETSTACK_RADIO.off();
                                spiInUse = 0;
                            break;

                            case SPI_RF_PACKET_SEND:
                                rf_pkt_len = spi_rx_pkt.spi_payload_length;
                                memcpy(rf_pkt_payload, spi_rx_pkt.spi_payload, rf_pkt_len);
                                packetbuf_copyfrom(rf_pkt_payload, rf_pkt_len);
                                cc2538_on_and_transmit();
                                spiInUse = 0;
                            break;

                            case SPI_MASTER_SET_TIME:
                                receivedTime.year   = spi_rx_pkt.spi_payload[0] + 2000;
                                receivedTime.month  = spi_rx_pkt.spi_payload[1];
                                receivedTime.day    = spi_rx_pkt.spi_payload[2];
                                receivedTime.hours  = spi_rx_pkt.spi_payload[3];
                                receivedTime.minutes= spi_rx_pkt.spi_payload[4];
                                receivedTime.seconds= spi_rx_pkt.spi_payload[5];
                                setTime(&receivedTime);
                                spiInUse = 0;
                            break;

                            case SPI_MASTER_RST_RF_FIFO:
                                triumviAvailIDX = 0;
                                triumviFullIDX = 0;
                                triumviRXBufFull = 0;
                                spiInUse = 0;
                            break;

                            default:
                                spiInUse = 0;
                            break;
                        }
                    }
                    spi_data_ptr = 0;
                    #ifndef LED_DEBUG
                    leds_off(LEDS_BLUE);
                    #else
                    leds_on(LEDS_RED);
                    leds_off(LEDS_GREEN);
                    leds_on(LEDS_BLUE);
                    #endif
                }
                process_poll(&mainProcess);
            break;
            
            default:
            break;
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(decryptProcess, ev, data) {
    PROCESS_BEGIN();

    uint8_t *header_ptr;
    uint8_t *data_ptr;
    uint8_t data_length;
    uint8_t header_length;
    static uint8_t rtc_data_pkt[8];

    while(1){
        PROCESS_YIELD();
        #ifndef LED_DEBUG
        leds_on(LEDS_RED);
        #else
        leds_on(LEDS_RED);
        leds_on(LEDS_GREEN);
        leds_on(LEDS_BLUE);
        #endif
        // Get data from radio buffer and parse it
        header_length = packetbuf_hdrlen();
        header_ptr = packetbuf_hdrptr();                                   
        data_length = packetbuf_datalen();                               
        data_ptr = packetbuf_dataptr();                                  

        // check for time request packet
        if ((data_length==2) && (data_ptr[0]==TRIUMVI_RTC) && (data_ptr[1]==TRIUMVI_RTC_REQ)){
            rtc_data_pkt[0] = TRIUMVI_RTC;
            rtc_data_pkt[1] = TRIUMVI_RTC_SET;
            rtc_data_pkt[2] = (currentTime.year - 2000);
            rtc_data_pkt[3] = currentTime.month;
            rtc_data_pkt[4] = currentTime.day;
            rtc_data_pkt[5] = currentTime.hours;
            rtc_data_pkt[6] = currentTime.minutes;
            rtc_data_pkt[7] = currentTime.seconds;
            packetbuf_copyfrom(rtc_data_pkt, 8);
            cc2538_on_and_transmit();
        }
        // RX buffer is not full
        else if (triumviRXBufFull==0){
            triumviRXPackets[triumviAvailIDX].length = data_length + header_length;
            memcpy(triumviRXPackets[triumviAvailIDX].payload, header_ptr, header_length);
            memcpy(triumviRXPackets[triumviAvailIDX].payload+header_length, data_ptr, data_length);
            // check if fifo is full
            if (((triumviAvailIDX == TRIUMVI_PACKET_BUF_LEN-1) && (triumviFullIDX == 0)) || 
                ((triumviAvailIDX != TRIUMVI_PACKET_BUF_LEN-1) && (triumviFullIDX == triumviAvailIDX+1))){
                triumviRXBufFull = 1;
                #ifndef LED_DEBUG
                leds_on(LEDS_GREEN);
                #endif
            }
            // advance pointer
            if (triumviAvailIDX == TRIUMVI_PACKET_BUF_LEN-1)
                triumviAvailIDX = 0;
            else
                triumviAvailIDX += 1;
        }
        #ifndef LED_DEBUG
        leds_off(LEDS_RED);
        #endif
        process_poll(&mainProcess);
    }
    PROCESS_END();
}

PROCESS_THREAD(rtcProcess, ev, data) {
    PROCESS_BEGIN();
    etimer_set(&software_RTC_timer, CLOCK_SECOND*SOFTWARE_RTC_TICK);
    currentTime.year = 2000;
    currentTime.month = 1;
    currentTime.day = 1;
    currentTime.hours = 0;
    currentTime.minutes = 0;
    currentTime.seconds = 0;
    while(1) {
        PROCESS_YIELD();
        if (etimer_expired(&software_RTC_timer)){
            advanceTime(SOFTWARE_RTC_TICK);
            etimer_restart(&software_RTC_timer);
        }
    }
    PROCESS_END();
}

void rf_rx_handler(){
    process_poll(&decryptProcess);
}   

static void spiCScallBack(uint8_t port, uint8_t pin){
    GPIO_CLEAR_INTERRUPT(SPI0_CS_PORT_BASE, SPI0_CS_PIN_MASK);
    spi_cs_int = 1;
    process_poll(&spiProcess);
}

static void resetcallBack(uint8_t port, uint8_t pin){
    watchdog_reboot();
}

static void spiFIFOcallBack(){
    spix_interrupt_disable(SPIDEV, SSI_IM_RXIM_M);
    spi_rxfifo_halffull = 1;
    process_poll(&spiProcess);
}
