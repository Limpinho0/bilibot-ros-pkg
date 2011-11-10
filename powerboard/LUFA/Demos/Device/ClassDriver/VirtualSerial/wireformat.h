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
int8_t     header[4];		 // header used for seeking
int8_t     len;			 // length of entire packet in bytes
int8_t     seq;			 // sequence number (for error tracking)
int8_t     type;		 // packet id 
int8_t     payload[MAX_PAYLOAD]; // payload data
int8_t     ck_high;		 // first byte of checksum
int8_t     ck_low;		 // second byte of checksum
};


#define STATUS_COMPLETE 1 
#define STATUS_INCOMPLETE 0	
#define STATUS_INVALID -1	

typedef struct _status_t status_t;
struct _status_t
{
int8_t recvd;
int8_t state;
};

// call for each byte in serial stream,
// returns 1 if valid packet is built from stream, 0 otherwise
int packet_decoded(int8_t byte, packet_t* pkt, status_t* status);

// builds a packet given a payload, type and seq #
packet_t* create_packet(int8_t type, int8_t seq, int8_t* payload, int8_t len);

// turns a packet into an array of bytes
int8_t packet_to_buffer(packet_t* pkt, int8_t* buffer);

int8_t valid_checksum(packet_t* pkt);

#endif //WIREFORMAT_H
