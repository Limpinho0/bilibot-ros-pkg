
#ifndef MD_GYRO_H
#define MD_GYRO_H

#include "utility/twi.h"

//this header contains everything needed to interface to the IDG-3200 Gyro, which is interfacesd with I2C bus.

//The address of the gyro is: 1101000  unless someone connects the solder jumper, then it is: 1101001
//so we use adress == 104  
//the twi library shifts it for us.

#define GYRO_ADDR 104


// uint8_t gyrodata[8]; //data that the gyro gives



void InitGyro(){
  twi_init();
  uint8_t tdata[3];
//   twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t sendStop)
  //set sample rate divider:
  tdata[0] = 0x15;  //sample rate divider register
  tdata[1] = 0x00;  //set divider to 1 -> full rate
  //don't have to set a new reg - just continue on to reg 0x16
  tdata[2] = 0x18; //set rate to 'on', and 8Khz sample rate
  twi_writeTo(GYRO_ADDR, tdata, 3, 1, 1);
  //TODO: decide whether we should modify the clock?
}

void ReadGyro(uint8_t *data){
  
  //we have 8 registers to read, starting at register 0x1b
  uint8_t regaddr = 0x1b;
  twi_writeTo(GYRO_ADDR, &regaddr, 1, 1, 1); //send the address we are reading from
  twi_readFrom(GYRO_ADDR, data, 8, 1); //read 8 registers
}


























#endif