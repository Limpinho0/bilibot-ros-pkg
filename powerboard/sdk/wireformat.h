#ifndef WIREFORMAT_H
#define WIREFORMAT_H

#include <stdint.h>

#define MAX_PAYLOAD 255

#define PKTYPE_STATUS_HEARTBEAT      0x00
#define PKTYPE_STATUS_ARM_STATE      0x01
#define PKTYPE_STATUS_GYRO_RAW	     0x02
#define PKTYPE_STATUS_BATT_RAW	     0x03
#define PKTYPE_CMD_SET_ARM_POS       0x80
#define PKTYPE_CMD_SET_HAND_POS      0x81
#define PKTYPE_CMD_SET_PWR_STATE     0x82
#define PKTYPE_CMD_ZERO_GYRO         0x83
#define PKTYPE_CMD_TOGGLE_KINECT     0x84
#define PKTYPE_CMD_TOGGLE_CREATE     0x85
#define PKTYPE_CMD_TOGGLE_HAND_STATE 0x86

#define HEADER_BYTE0 0x68 // 'h'
#define HEADER_BYTE1 0x65 // 'e'
#define HEADER_BYTE2 0x61 // 'a'
#define HEADER_BYTE3 0x64 // 'd'

typedef struct _packet_t packet_t;
struct _packet_t
{
    uint8_t header[4];		 // header used for seeking
    uint8_t len;			 // length of entire packet in bytes
    uint8_t seq;			 // sequence number (for error tracking)
    uint8_t type;		 // packet id 
    uint8_t payload[MAX_PAYLOAD]; // payload data
    uint8_t ck_high;		 // first byte of checksum
    uint8_t ck_low;		 // second byte of checksum
};

#define DECODE_STATUS_COMPLETE 1 
#define DECODE_STATUS_INCOMPLETE 0	
#define DECODE_STATUS_INVALID 2

typedef struct _status_t status_t;
struct _status_t
{
    uint8_t recvd;
    uint8_t state;
};

// call for each byte in serial stream,
// returns 1 if valid packet is built from stream, 0 otherwise
int PKT_Decoded(uint8_t byte, packet_t* pkt, status_t* status);

// builds a packet given a payload, type and seq #
packet_t* PKT_Create(uint8_t type, uint8_t seq, uint8_t* payload, uint8_t len);

// turns a packet into an array of bytes
uint8_t PKT_ToBuffer(packet_t* pkt, uint8_t* buffer);

uint8_t PKT_ValidChecksum(packet_t* pkt);

#endif //WIREFORMAT_H
