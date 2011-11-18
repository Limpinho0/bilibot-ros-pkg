#define SIMPLEMOTOR
#include "lowlevelmotor.h"
#include "adc.h"


int16_t lastspeed;

void setupMotors(){
   LowLevelSetup();
  
}

void simpleMotorCheck(){
    uint8_t pot=ADC_BASE_POT;
    if(lastspeed<0 && pot > 150)
      HL_BaseSpeed(0);
    if(lastspeed>0 && pot < 15)
      HL_BaseSpeed(0);
  
  
  
}

//a good start speed is 5000

void HL_BaseSpeed(int16_t spd){
  lastspeed=spd;
  setBaseSpeed(spd);
}

uint8_t goingtotarget=0;
uint8_t basetarget=80;

void sendtoMid(){
    if(goingtotarget==0) return;
    if(abs(basetarget-ADC_BASE_POT)<4){
      goingtotarget=0;
      HL_BaseSpeed(0);
      return;
    }
    int16_t svel= ADC_BASE_POT-basetarget; //could be 160->-160
    HL_BaseSpeed(173*svel + (svel>0?2000:-2000));

    
}

uint8_t testMotors(){
 //see if they are plugged in
 //turn on base motor, then stop.  see if the current spiked
  startProfile();
  HL_BaseSpeed(20000);
  _delay_ms(10);
  HL_BaseSpeed(0);
  uint8_t maxcurr= getProfileMax(ADC_BASE_CURR_CHANNEL);
  if(maxcurr < 2)
    return 1;
  return 0;
  //TODO: add hand motor
}



void setMotorMove(uint8_t c){
      
 /*     l_BASE_SPD;  //disable base motor
      l_HAND_SPD;  //disable hand motor
 */     
      if(c=='Y'){
	HL_BaseSpeed(32767);
	return;
      }
      if(c=='H'){
	HL_BaseSpeed(-32767);
	return;
      }
      if(c=='y'){
	HL_BaseSpeed(20000);
	return;
      }
      if(c=='h'){
	HL_BaseSpeed(-20000);
	return;
      }
      if(c=='T'){
	sendtoMid();
	goingtotarget=1;
	return;
      }
      if(c=='q'){
	HL_BaseSpeed(lastspeed+200);
	return;
      }
      if(c=='a'){
	HL_BaseSpeed(lastspeed-200);
	return;
      }
      HL_BaseSpeed(0);
      goingtotarget=0;
//       if(c=='T'){
// 	if(handon)
// 	  increaseHandSpeed();
// 	else{
// 	    handon=1;
// 	    handspeed=1000;
// 	    OCR3A=handspeed;
// 	}
// 	h_HAND_CTRL_B; 
// 	l_HAND_CTRL_A;
// 	return;
//       }
//       if(c=='G'){
// 	if(handon){
// 	  if(handspeed<16000)
// 	    handspeed+=100;
// 	    OCR3A=handspeed;
// 	}
// 	else{
// 	    handon=1;
// 	    handspeed=8000;
// 	    OCR3A=handspeed;
// 	}
// 	l_HAND_CTRL_B; 
// 	h_HAND_CTRL_A;
// 	return;
//       }
//       
//       if(c=='g'){
// 	if(handon)
// 	  decreaseHandSpeed();
// 	else{
// 	    handon=1;
// 	    handspeed=8000;
// 	    OCR3A=handspeed;
// 	}
// 	l_HAND_CTRL_B; 
// 	h_HAND_CTRL_A;
// 	return;
//       }
}