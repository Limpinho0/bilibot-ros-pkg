
#ifndef SIMPLEMOTOR_H
#define SIMPLEMOTOR_H
#define SIMPLEMOTOR


void setupMotors(){
   DDRC |= 0xfc;
   DDRF = 0;
  
}

uint16_t handspeed;
uint8_t handon;
uint16_t basespeed;
uint8_t baseon;


//We clear at the compare. this way, the enable stays on as long
// as the OCR3A value
SIGNAL(TIMER3_COMPA_vect){
  if(handon==0)
    return;
  l_HAND_SPD;
}

//We clear at the compare. this way, the enable stays on as long
// as the OCR3B value
SIGNAL(TIMER3_COMPB_vect){
  if(baseon==0)
    return;
  l_BASE_SPD
}

//at the overflow, if the hand or base should be on, we turn them on
//otherwise we leave them off
SIGNAL(TIMER3_OVF_vect){
  if(handon==0)
   l_HAND_SPD;
  else{
    //set at the top:
    h_HAND_SPD;
  }  
  if(baseon==0)
   l_BASE_SPD;
  else{
    //set at the top:
    h_BASE_SPD;
  }  
  
}

void increaseHandSpeed(){
 if(handspeed<16000)
   handspeed+=100;
 OCR3A=handspeed;
}

void decreaseHandSpeed(){
 if(handspeed>100)
   handspeed-=100;
 OCR3A=handspeed;
}
void increaseBaseSpeed(){
 if(handspeed<16000)
   handspeed+=100;
 OCR3B=handspeed;
}

void decreaseBaseSpeed(){
 if(handspeed>100)
   handspeed-=100;
 OCR3B=handspeed;
}


  void setMotorMove(uint8_t c){
      
 /*     l_BASE_SPD;  //disable base motor
      l_HAND_SPD;  //disable hand motor
 */     
      if(c=='Y'){
	h_BASE_CTRL_B; 
	l_BASE_CTRL_A;
      }
      if(c=='H'){
	l_BASE_CTRL_B; 
	h_BASE_CTRL_A;
      }
      
      if(c=='T'){
	if(handon)
	  increaseHandSpeed();
	else{
	    handon=1;
	    handspeed=1000;
	    OCR3A=handspeed;
	}
	h_HAND_CTRL_B; 
	l_HAND_CTRL_A;
	return;
      }
      if(c=='G'){
	if(handon){
	  if(handspeed<16000)
	    handspeed+=100;
	    OCR3A=handspeed;
	}
	else{
	    handon=1;
	    handspeed=8000;
	    OCR3A=handspeed;
	}
	l_HAND_CTRL_B; 
	h_HAND_CTRL_A;
	return;
      }
      
      if(c=='g'){
	if(handon)
	  decreaseHandSpeed();
	else{
	    handon=1;
	    handspeed=8000;
	    OCR3A=handspeed;
	}
	l_HAND_CTRL_B; 
	h_HAND_CTRL_A;
	return;
      }
//       if(c == 'Y' || c=='H' || c=='T' || c=='G'){
// 	h_BASE_SPD;  //enable base motor
// 	h_HAND_SPD;  //enable hand motor
//       }
      
	handon=0;
	l_BASE_CTRL_A;
	l_BASE_CTRL_B;
	l_HAND_CTRL_A;
	l_HAND_CTRL_B;
      
  }


void setupHandOsc(){
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



#endif