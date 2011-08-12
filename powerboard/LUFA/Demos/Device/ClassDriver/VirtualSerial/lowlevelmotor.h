
#ifndef LOWLEVELMOTOR_H
#define LOWLEVELMOTOR_H
#include "pinmapping.h"


void setupMotorOsc(){
     DDRC |= 0xfc;
   DDRF = 0;
  
//TCCR3A
//////////  10xxxxxx     //COM3A  OC3A clear on compare
//  00xxxxxx     //COM3A  going to manually toggle pin
//  xx0000xx     //COM1B/C  not toggling pins
//  xxxxxx10      //fast PWM top= ICR3
TCCR3A=0x82;
  
 //TCCR3B:
//  0xxxxxxx     //input capture noise canelation off
//  x1xxxxxx     //input capture edge select = rising
//  xx0xxxxx     // N/C
//  xxx11xxx      //waveform generation: fast pwm
//  xxxxx001      //prescaler = 1
//0x59
TCCR3B=0x59;

//lets set osc freq to 1Khz:
ICR3=16000;
//set both PWM to 50%:
OCR3A=8000;
OCR3B=8000;

//TIMSK3   - timer 3 interrupts
//  00xxxxxx     //N/C
//  xx0xxxxx     //input capture interrupt  
//  xxx0xxxx     // N/C
//  xxxx011x      //output compare B,A on, C off
//  xxxxxxx1      //timer overflow
TIMSK3 = 0x07;
  
}



struct LowLevelMotors{
  
  uint16_t handspeed;
  uint8_t handon;
  uint16_t basespeed;
  uint8_t baseon;

  
  
  void setBaseDown(){
	l_BASE_CTRL_B; 
	h_BASE_CTRL_A;
  }
  void setBaseUp(){
	h_BASE_CTRL_B; 
	l_BASE_CTRL_A;
    
  }
  void setBaseBrake(){
	l_BASE_CTRL_B; 
	l_BASE_CTRL_A;
  }
   void setHandClose(){
	h_HAND_CTRL_B; 
	l_HAND_CTRL_A;
  }
  void setHandOpen(){
	l_HAND_CTRL_B; 
	h_HAND_CTRL_A;
    
  }
  void setHandBrake(){
	l_HAND_CTRL_B; 
	l_HAND_CTRL_A;
  } 
	
    
  void Setup(){
    setupMotorOsc();
    baseon=0;
    handon=0;
    setHandBrake();
    setBaseBrake();
  }
  
  void setBaseSpeed(int16_t spd){
    if(spd==0){
      baseon=0;
      setBaseBrake();
      return;
    }
     uint16_t uspeed;
     uint8_t dir=1;
     if(spd<0){
        dir=0;
	int16_t nspeed=spd*-1;
	uspeed=nspeed*2;
	setBaseDown();
     }
     else{
	uspeed=spd;
	uspeed*=2;
	setBaseUp();
     }
     OCR3B=uspeed;
     baseon=1;
  }
  
  
};

// LowLevelMotors _motors;
// 
// 
// //We clear at the compare. this way, the enable stays on as long
// // as the OCR3A value
// SIGNAL(TIMER3_COMPA_vect){
//   if(_motors.handon==0)
//     return;
//   l_HAND_SPD;
// }
// 
// //We clear at the compare. this way, the enable stays on as long
// // as the OCR3B value
// SIGNAL(TIMER3_COMPB_vect){
//   if(_motors.baseon==0)
//     return;
//   l_BASE_SPD
// }
// 
// //at the overflow, if the hand or base should be on, we turn them on
// //otherwise we leave them off
// SIGNAL(TIMER3_OVF_vect){
//   if(_motors.handon==0)
//    l_HAND_SPD;
//   else{
//     //set at the top:
//     h_HAND_SPD;
//   }  
//   if(_motors.baseon==0)
//    l_BASE_SPD;
//   else{
//     //set at the top:
//     h_BASE_SPD;
//   }  
//   
// }









#endif