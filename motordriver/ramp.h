

//ramps allow us to increase and decrease speeds linearly

#ifndef MD_RAMP_H
#define MD_RAMP_H

//Ramps use the overflow vectors for the oscillators that the motors are using

#include "motordriver.h"

//motor connection:
// Motor 1:
// AHI   PB7 (OC1C)
// ALO   PA6 
// BHI   PB6  (OC1B)
// BLO    PA5
// RESET  PA7
// FAULT  PA4
// 
// Motor 2:
// AHI   PC4 (OC3C)
// ALO   PC5 
// BHI   PC6  (OC3A)
// BLO    PC7
// RESET  PC0
// FAULT  PC3




typedef struct {
  int16_t target;
  int16_t current;
  int16_t stepsize;  //number by which to increase/decrease
  uint8_t stop_when_over; //tells us which direction to check
  uint8_t enabled;
  uint8_t updatecounter; //used to count up to updatenum
  uint8_t updatesperstep; //allows fractional step size
} Ramp;

Ramp m1_ramp, m2_ramp;


void makeM1Ramp(int16_t target,int16_t step){
  if(step<0){
    m1_ramp.updatesperstep=(-1*step);
    step=1;
  }
  else
    m1_ramp.updatesperstep=0;
  //either way, the first update should happen right away
  m1_ramp.updatecounter = 0;
  
  m1_ramp.target=target;
  m1_ramp.current=0;
  if(TCCR1A & 0x08) //C is pwming
    m1_ramp.current=OCR1C;
  else if (TCCR1A & 0x20)
    m1_ramp.current=-1*OCR1B;
  m1_ramp.stepsize=step;
  m1_ramp.stop_when_over=1;
  if(m1_ramp.target<m1_ramp.current){
    m1_ramp.stop_when_over=0;
    m1_ramp.stepsize=step*-1;
  }
  m1_ramp.enabled=1;
  
}

//TCCR1A
//  00xxxxxx     //Compare Output Mode for Channel A
//  xx00xxxx     //Compare Output Mode for Channel B
//  xxxx00xx     //Compare Output Mode for Channel C
//check high bit to determine if we are pwming




void makeM2Ramp(int16_t target,int16_t step){
    if(step<0){
    m2_ramp.updatesperstep=(-1*step);
    step=1;
  }
  else
    m2_ramp.updatesperstep=0; //counts down to 0, so this way there is one update per step
  //either way, the first update should happen right away
  m2_ramp.updatecounter = 0;
  
  m2_ramp.target=target;
  m2_ramp.current=0;
  if(TCCR3A & 0x08) //C is pwming
    m2_ramp.current=OCR3C;
  else if (TCCR3A & 0x80)
    m2_ramp.current=-1*OCR3A;
  m2_ramp.stepsize=step;
  m2_ramp.stop_when_over=1;
  if(m2_ramp.target<m2_ramp.current){
    m2_ramp.stop_when_over=0;
    m2_ramp.stepsize=step*-1;
  }
  m2_ramp.enabled=1;
  
}

void UpdateM1Ramp(){
  if(!m1_ramp.enabled)
    return;

  if(m1_ramp.updatecounter){ //still counting down until update
    m1_ramp.updatecounter--;
    return;
  }
  else{
    //reset counter, allow rest of code to run
    m1_ramp.updatecounter = m1_ramp.updatesperstep;
  }
  
  m1_ramp.current=m1_ramp.current+m1_ramp.stepsize;
  if((m1_ramp.stop_when_over && m1_ramp.target<m1_ramp.current) ||
     (m1_ramp.stop_when_over==0 && m1_ramp.target>m1_ramp.current)) {
    m1_ramp.enabled=0;
    return;
  }
  setAbsSpeedM1(m1_ramp.current);  
}

void UpdateM2Ramp(){
  if(!m2_ramp.enabled)
    return;
  
  if(m2_ramp.updatecounter){ //still counting down until update
    m2_ramp.updatecounter--;
    return;
  }
  else{
    //reset counter, allow rest of code to run
    m2_ramp.updatecounter = m2_ramp.updatesperstep;
  }
  
  m2_ramp.current=m2_ramp.current+m2_ramp.stepsize;
  if((m2_ramp.stop_when_over && m2_ramp.target<m2_ramp.current) ||
     (m2_ramp.stop_when_over==0 && m2_ramp.target>m2_ramp.current)) {
    m2_ramp.enabled=0;
    return;
  }
  setAbsSpeedM2(m2_ramp.current);  
}


//TODO: check for highest speed command!

//high level funcs:
void setRamps(int16_t right, int16_t left){
   makeM2Ramp(scaleCmd(-1*left),-20);
 makeM1Ramp(scaleCmd(right),-20);
  
}



void setSpeedRamps(uint8_t h_right, uint8_t l_right, uint8_t h_left, uint8_t l_left){
 //convert into ints:
 int16_t right,left;
 right = l_right+(h_right<<8);
 left = l_left+(h_left<<8);
 setRamps(right,-1*left);
 
  
}

SIGNAL(TIMER1_OVF_vect){
  UpdateM1Ramp();
}

SIGNAL(TIMER3_OVF_vect){
  UpdateM2Ramp();
}

#endif