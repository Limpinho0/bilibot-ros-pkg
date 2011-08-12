#define SIMPLEMOTOR
#include "lowlevelmotor.h"




void setupMotors(){
   _motors.Setup();
  
}



void setMotorMove(uint8_t c){
      
 /*     l_BASE_SPD;  //disable base motor
      l_HAND_SPD;  //disable hand motor
 */     
      if(c=='Y'){
	_motors.setBaseSpeed(32767)
	return;
      }
      if(c=='H'){
	_motors.setBaseSpeed(-32767)
	return;
      }
	_motors.setBaseSpeed(0)
      
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