#include "wireformat.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

int PKT_Decoded(uint8_t byte, packet_t* pkt, status_t* status)
{
    if (status->recvd == 0 && byte == HEADER_BYTE0) 
    {
        pkt->header[0] = byte;
        status->state = DECODE_STATUS_INCOMPLETE;
    }
    else if (status->recvd == 0 && byte != HEADER_BYTE0)
    {
        status->state = DECODE_STATUS_INCOMPLETE;
        return DECODE_STATUS_INCOMPLETE;
    }
    else if (status->recvd == 1 && byte == HEADER_BYTE1) 
    {
        pkt->header[1] = byte;
    }
    else if (status->recvd == 2 && byte == HEADER_BYTE2) 
    {
        pkt->header[2] = byte;
    }
    else if (status->recvd == 3 && byte == HEADER_BYTE3) 
    {
        pkt->header[3] = byte;
    }
    else if (status->recvd == 4) 
    { 
        pkt->len = byte;
    }
    else if (status->recvd == 5) 
    { 
        pkt->seq = byte;
    }
    else if (status->recvd == 6) 
    { 
        pkt->type = byte;
    }
    else if (status->recvd < pkt->len - 2) 
    {
        pkt->payload[status->recvd - 7] = byte;
    }
    else if (status->recvd == pkt->len - 2) 
    {
        pkt->ck_high = byte;
    }
    else if (status->recvd == pkt->len - 1) 
    {
        pkt->ck_low = byte;
        if (PKT_ValidChecksum(pkt) == 1) {
            status->state = DECODE_STATUS_COMPLETE;
            return DECODE_STATUS_COMPLETE;
        }
        else 
        {
            status->state = DECODE_STATUS_INVALID;
            return DECODE_STATUS_INVALID;
        }
    } else 
    {
        status->state = DECODE_STATUS_INVALID;
        return DECODE_STATUS_INVALID;
    }
        
    status->recvd++;
    return DECODE_STATUS_INCOMPLETE;
}

packet_t* PKT_Create(uint8_t type, uint8_t seq, uint8_t* payload, uint8_t len)
{
    int i;
    packet_t* pkt = (packet_t*)malloc(sizeof(packet_t));
    pkt->header[0] = HEADER_BYTE0; // 'h'
    pkt->header[1] = HEADER_BYTE1; // 'e' 
    pkt->header[2] = HEADER_BYTE2; // 'a' 
    pkt->header[3] = HEADER_BYTE3; // 'd'
    pkt->type = type;
    pkt->seq = seq;
    pkt->len = len + 9;
    uint16_t ck = HEADER_BYTE0 + 
             HEADER_BYTE1 + 
             HEADER_BYTE2 +  
             HEADER_BYTE3 +
             pkt->type + pkt->seq + pkt->len;
    for(i = 0; i < len; i++)
        ck += payload[i];

    pkt->ck_high = ck >> 8 & 0x00ff; // first byte of checksum
    pkt->ck_low = ck & 0x00ff; // second byte of checksum
    memcpy(pkt->payload, payload, len);
    return pkt;
}

uint8_t PKT_ValidChecksum(packet_t* pkt)
{
    int i;
    uint16_t ck = HEADER_BYTE0 + 
             HEADER_BYTE1 + 
             HEADER_BYTE2 +  
             HEADER_BYTE3 +
             pkt->type + pkt->seq + pkt->len;
    for(i = 0; i < pkt->len - 9; i++)
        ck += pkt->payload[i];

    uint16_t pkt_ck = (pkt->ck_high << 8) + pkt->ck_low;
    if (ck != pkt_ck)
        return -1;
    return 1;
}

uint8_t PKT_ToBuffer(packet_t* pkt, uint8_t* buffer)
{
    int i = 0;
    for(; i < 4; i++)
        buffer[i] = pkt->header[i];
    buffer[i++] = pkt->len;
    buffer[i++] = pkt->seq;
    buffer[i++] = pkt->type;
    int j = i;
    for(; i < pkt->len - 2; i++) {
        buffer[i] = pkt->payload[i-j];
    }
    buffer[i++] = pkt->ck_high;
    buffer[i++] = pkt->ck_low;
    return i;
}
