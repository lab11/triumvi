
#include "rf-header-parse.h"

int process_packet_header(packet_header_t* header_ptr, uint8_t* packetbuf_header_ptr){
	header_ptr->pkt_frame_type = (packetbuf_header_ptr[0] & 0x03);
	header_ptr->pkt_security_en = (packetbuf_header_ptr[0] & 0x08)>>3;
	header_ptr->pkt_frame_pending = (packetbuf_header_ptr[0] & 0x10)>>4;
	header_ptr->pkt_ack_req = (packetbuf_header_ptr[0] & 0x20)>>5;
	uint8_t panIDCompression = (packetbuf_header_ptr[0] & 0x40)>>6;
	uint8_t destAddrMode = (packetbuf_header_ptr[1] & 0x0c)>>2;
	uint8_t srcAddrMode = (packetbuf_header_ptr[1] & 0xc0)>>6;
	header_ptr->pkt_seq_number = packetbuf_header_ptr[2];
	switch (destAddrMode){
		case 0:
			header_ptr->pkt_dest_panID_len = 0;
			header_ptr->pkt_dest_addr_len = 0;
		break;
		case 2:
			header_ptr->pkt_dest_panID_len = 2;
			header_ptr->pkt_dest_addr_len = 2;
		break;
		case 3:
			header_ptr->pkt_dest_panID_len = 2;
			header_ptr->pkt_dest_addr_len = 8;
		break;
		default:
			return -1;
		break;
	}
	switch (srcAddrMode){
		case 0:
			header_ptr->pkt_src_panID_len = 0;
			header_ptr->pkt_src_addr_len = 0;
		break;
		case 2:
			header_ptr->pkt_src_panID_len = 2;
			header_ptr->pkt_src_addr_len = 2;
		break;
		case 3:
			header_ptr->pkt_src_panID_len = 2;
			header_ptr->pkt_src_addr_len = 8;
		break;
		default:
			return -1;
		break;
	}
	uint8_t* tempPtr = (packetbuf_header_ptr + 3);
	if (((header_ptr->pkt_src_addr_len)>0) && ((header_ptr->pkt_dest_addr_len)>0) && (panIDCompression==1))
		header_ptr->pkt_src_panID_len = 0;

	if (header_ptr->pkt_dest_panID_len > 0){
		header_ptr->pkt_dest_panID = tempPtr;
		tempPtr += 2;
	}
	if (header_ptr->pkt_dest_addr_len > 0){
		header_ptr->pkt_dest_addr = tempPtr;
		tempPtr += (header_ptr->pkt_dest_addr_len);
	}

	if (header_ptr->pkt_src_panID_len > 0){
		header_ptr->pkt_src_panID = tempPtr;
		tempPtr += 2;
	}
	if (header_ptr->pkt_src_addr_len > 0){
		header_ptr->pkt_src_addr = tempPtr;
		tempPtr += (header_ptr->pkt_src_addr_len);
	}
	return  header_ptr->pkt_dest_panID_len + 
            header_ptr->pkt_dest_addr_len + 
            header_ptr->pkt_src_panID_len + 
            header_ptr->pkt_src_addr_len + 3;
}
