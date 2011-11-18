#ifndef WIREFORMAT_H
#define WIREFORMAT_H

#include <stdint.h>

#define MAX_PAYLOAD 5

#define TYPE_STATUS_HEARTBEAT	0x00
#define TYPE_STATUS_ARM_POS	0x01
#define TYPE_STATUS_HAND_POS	0x02
#define TYPE_STATUS_ARM_LLIM	0x03
#define TYPE_STATUS_ARM_ULIM	0x04
#define TYPE_STATUS_GYRO_RAW	0x05
#define TYPE_CMD_ARM_POS 	0x80
#define TYPE_CMD_HAND_POS	0x81

#define HEADER_BYTE0 0x68
#define HEADER_BYTE1 0x65
#define HEADER_BYTE2 0x61
#define HEADER_BYTE3 0x64

typedef struct _packet_t packet_t;
struct _packet_t
{
uint8_t     header[4];		 // header used for seeking
uint8_t     len;			 // length of entire packet in bytes
uint8_t     seq;			 // sequence number (for error tracking)
uint8_t     type;		 // packet id 
uint8_t     payload[MAX_PAYLOAD]; // payload data
uint8_t     ck_high;		 // first byte of checksum
uint8_t     ck_low;		 // second byte of checksum
};


#define STATUS_COMPLETE 1 
#define STATUS_INCOMPLETE 0	
#define STATUS_INVALID -1	

typedef struct _status_t status_t;
struct _status_t
{
uint8_t recvd;
uint8_t state;
};

// call for each byte in serial stream,
// returns 1 if valid packet is built from stream, 0 otherwise
int packet_decoded(uint8_t byte, packet_t* pkt, status_t* status);

// builds a packet given a payload, type and seq #
packet_t* create_packet(uint8_t type, uint8_t seq, uint8_t* payload, uint8_t len);

// turns a packet into an array of bytes
uint8_t packet_to_buffer(packet_t* pkt, uint8_t* buffer);

uint8_t valid_checksum(packet_t* pkt);

#endif //WIREFORMAT_H
