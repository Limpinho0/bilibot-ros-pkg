#include "wireformat.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void) {
    // random placeholder data
    int8_t position[5] = { 0x01, 0x02, 0x03, 0x04, 0x05 };

    // create a new packet (again, random sequence number & data)
    packet_t* pkt = create_packet(TYPE_STATUS_ARM_POS, 123, position, 5);

    // create a buffer to stream over a serial port
    int8_t* buffer = (int8_t*)malloc(sizeof(packet_t));
    int8_t* unalignedbuffer = (int8_t*)malloc(sizeof(packet_t));

    // pack the buffer with pkt data
    int size = packet_to_buffer(pkt, buffer);

    packet_t p; // packet to build from buffer 
    status_t s;
    s.recvd = 0; // init received count to 0

    // fill in new pkt from buffer
    for(int i = 0; i < size; i++) {
        printf("Hex: 0x%x\tDec: %d\n", buffer[i], buffer[i]);
        if(packet_decoded(buffer[i], &p, &s) == STATUS_COMPLETE)
            printf("built valid packet\n");
    } 
    printf("size: %d\n", size);

    size = packet_to_buffer(&p, buffer);
    unalignedbuffer[0] = 42;
    for(int i = 0; i < size; i++) {
        unalignedbuffer[i+1] = buffer[i];
        printf("Hex: 0x%x\tDec: %d\n", buffer[i], buffer[i]);
    }
    printf("size: %d\n", size);
    size++;
    for(int i = 0; i < size; i++) {
        printf("Hex: 0x%x\tDec: %d\n", unalignedbuffer[i], unalignedbuffer[i]);
        if(packet_decoded(unalignedbuffer[i], &p, &s) == STATUS_COMPLETE)
            printf("built valid packet\n");
    } 
    printf("size: %d\n", size);
}
