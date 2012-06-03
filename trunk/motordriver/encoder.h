//has all the info necessary to read, decifer the encoders

#ifndef ENCODER_H
#define ENCODER_H


//Wheel encoder pins:
//Wheel encoder LA - PE6 - int6
//Wheel encoder LB - PE7 - int7
//Wheel encoder RA - PD2 - int2
//Wheel encoder RB - PD3 - int3

// for the interrupt registers:
// 0 0 The low level of INTn generates an interrupt request.
// 0 1 Any edge of INTn generates asynchronously an interrupt request.
// 1 0 The falling edge of INTn generates asynchronously an interrupt request.
// 1 1 The rising edge of INTn generates asynchronously an interrupt request.

#define WELA 0x01
#define WELB 0x02
#define WERA 0x04
#define WERB 0x08
//partner at high level:
#define WEPART 0x10




void initEncoders(){
  //we'll do rising edge for now...
  EICRA |= 0xf0;  //set int 2 and int3 to rising edge
  EICRB |= 0xf0;  //set int6 and int7 to rising edge
  
  //set up pins: pins should be input, and pulled high
  PIND &= 0xf3;
  PINE &= 0x3f;
  PORTD |= 0x0c;
  PORTE |= 0xc0;
  
  
  //TODO: enable these vectors in a special way - see datasheet
  EIMSK |=0xcc;  //enable int7,6,3,2 interrupt vectors

  tickbit=0;
  tickbyte=0;
  
  
}

void readEncoders(){}


//used to figure out how far the encoders are apart, and if they need to 
void calibrateEncoders(){}

//each tick is 1.8 mm, so 1000 is 1m
//if the robot can go up to 2.7 m/s
//and we report at 10hz  we need to store 270 bits, or 34 bytes
uint8_t tickwheel[40]; //stores one bit for each tick - the level indicates which wheel ticked. (left high)
uint8_t tickdir[40];   //stores one bit for each tick - the high level indicates forward
uint8_t tickbit;  //where we are in the tickstorage
uint8_t tickbyte;  //which byte we are on

//TODO: this is not correct. we need to discover this with calibration
uint8_t getTickDir(uint8_t tickinfo){
  if(tickinfo && WELA || tickinfo && WERA){
    if(tickinfo && WEPART) return 1;
    return 0;    
  }
  if(tickinfo && WELB || tickinfo && WERB){
    if(tickinfo && WEPART) return 0;
    return 1;    
  }
}

void recordTick(uint8_t tickinfo){
  uint8_t td = getTickDir(tickinfo);
  uint8_t tw=0;
  if(tickinfo && WELA || tickinfo && WELB)
    tw=1;
  tw << tickbit;
  td << tickbit;
  tickdir[tickbyte] |= td;
  tickwheel[tickbyte] |= tw;
  //now incriment tick placement:
  tickbit++;
  if(tickbit>=8){
    tickbit=0;
    tickbyte++;
    tickbyte = tickbyte%40;
    //clear the new bytes
    tickwheel[tickbyte]=0;
    tickdir[tickbyte]=0;
    
  }
}

//when a pin goes high, figure out which state its partner is in
//these interrups happen a lot, so best to get out of there as fast as possible
SIGNAL(INT2_vect){ //RA
  if(PORTD & 0x08)
    recordTick(WERA & WEPART);
  else
    recordTick(WERA);
    
  
}

SIGNAL(INT3_vect){ //RB 
  if(PORTD & 0x04)
    recordTick(WERB & WEPART);
  else
    recordTick(WERB);
}

SIGNAL(INT6_vect){ //LA
  if(PORTE & 0x80)
    recordTick(WELA & WEPART);
  else
    recordTick(WELA);
  
}

SIGNAL(INT7_vect){ //LB
  if(PORTE & 0x40)
    recordTick(WELB & WEPART);
  else
    recordTick(WELB);
  
}


#endif