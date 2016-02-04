
/* packet structure */
// cmd + length (optional, length field counts) + payload (optional)
// REQ_DATA doesn't need length
// GET_DATA requires length

#define SPI_MASTER_REQ_DATA 0
#define SPI_MASTER_DUMMY 1
#define SPI_MASTER_GET_DATA 2

#define MAX_SPI_LENGTH 128

typedef struct{
  uint8_t cmd;
  uint8_t spi_payload_length;
  uint8_t spi_payload[MAX_SPI_LENGTH];
} spi_packet_t;

// return number of bytes are processed
uint16_t spi_packet_parse(spi_packet_t* rx_packet, uint8_t* data_ptr){
  rx_packet->cmd = data_ptr[0];
  uint8_t i;
  switch (rx_packet->cmd){
    case SPI_MASTER_REQ_DATA:
      rx_packet->spi_payload_length = 0;
      return 1;
    break;

    // do nothing
    case SPI_MASTER_DUMMY:
      rx_packet->spi_payload_length = 0;
      return 1;
    break;

    case SPI_MASTER_GET_DATA:
      rx_packet->spi_payload_length = 0;
    break;

    default:
      rx_packet->spi_payload_length = data_ptr[1]-1;
      for (i=0; i<rx_packet->spi_payload_length; i++)
        rx_packet->spi_payload[i] = data_ptr[2+i];
    break;
  }
  return data_ptr[1]+1;
}
