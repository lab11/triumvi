
#ifndef _RF_HEADER_PARSE_H_
#define _RF_HEADER_PARSE_H_
#include <stdint.h>
typedef enum frame_type{
	BEACON = 0,
	DATA,
	ACK,
	MAC_CMD
}frame_type_t;

typedef struct packet_header{
	frame_type_t pkt_frame_type;
	uint8_t pkt_security_en;
	uint8_t pkt_frame_pending;
	uint8_t pkt_ack_req;
	uint8_t pkt_seq_number;
	uint8_t pkt_dest_panID_len;
	uint8_t* pkt_dest_panID;
	uint8_t pkt_dest_addr_len;
	uint8_t* pkt_dest_addr;
	uint8_t pkt_src_panID_len;
	uint8_t* pkt_src_panID;
	uint8_t pkt_src_addr_len;
	uint8_t* pkt_src_addr;
} packet_header_t;

int process_packet_header(packet_header_t* header_ptr, uint8_t* packetbuf_header_ptr);

#endif
