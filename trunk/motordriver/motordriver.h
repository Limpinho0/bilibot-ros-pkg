


#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

//Motor driver board driver interface:
#define MOTORMAX 500

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

// Table 1. Input Logic
// Pin Setting
// RESET xHI 	xLO 	GHx 	GLx 	Sx 	Mode of Operation
// H 	H 	L 	H 	L 	H 	High side MOSFET conducting
// H 	L 	H 	L 	H 	L 	Low Side MOSFET conducting
// H 	H 	H 	L 	H 	L 	Low Side MOSFET conducting – cross-conduction prevention
// H 	L 	L 	L 	L 	Z 	High side and low side off
// L 	x 	x 	Z 	Z 	Z 	All gate drives inactive, all MOSFETs off

//The pins we actually control are Reset, xHI and xLO
//Both outputs have the low side MOSFET connected to ground, so that will never be PWM'ed
//To control the speed, we set one output to ground, and the other is PWM'ed

uint16_t gabs(int16_t i){
 if(i>=0) return i;
 return i*-1;
}


void SetupMotors(){
  //set output pins:
  DDRA |= 0xef;  //PA5,PA6,PA7
  PORTA &= 0x1f; //control pins low, reset low
  DDRB |= 0xc0;  //PB6, PB7
  PORTB &= 0x3f; //control pins low
  DDRC |= 0xf1; //PC0, PC4, PC5, PC6, PC7
  PORTC &= 0x0e; //control pins low, reset low
  //fault pins:
  DDRC &= 0xf7;  //set PC3 to input
  PORTC |= 0x08; //set PC3 internal pullup
  DDRA &= 0xef;  //set PA4 to input
  PORTA |= 0x10; //set PA4 internal pullup
  
  //turn on timers 1 and 3:
  //our clock is 16Mhz
  //our PWM frequency will be ~16Khz - which we can achieve by setting the 
  //  TOP value to ICR1, and no prescaler
  //this means the acceptable pwm inputs are 1-ICR1
  //
//  Timer 3 is the same as 1
//	Fast PWM mode (7) - WGM: 1110
	//TCCR1A
	//  000000xx     //no output compareing - yet
	//  xxxxxx10      //waveform generation = Fast PWM top=0x3FF
	TCCR1A=0x02;
	TCCR3A=0x02;

	//TCCR1B
	//  0xxxxxxx     //input capture noise cancellation off
	//  x1xxxxxx     //input capture edge select = rising
	//  xx0xxxxx     // N/C
	//  xxx11xxx      //waveform generation: Fast PWM
	//  xxxxx001      //prescaler = 1
	TCCR1B=0x59;
	TCCR3B=0x59;
	ICR1 = MOTORMAX;  //up to 4351 -> 4khz
	ICR3 = MOTORMAX;
	

	//TIMSK1   - timer 1 interrupts
	//  00xxxxxx     //N/C
	//  xx0xxxxx     //input capture interrupt  (on PB0 or analog at  AC0)
	//  xxx00xxx     // N/C
	//  xxxxx00x      //output compare B,A (to OCR1B,A)
	//  xxxxxxx0      //timer overflow
	TIMSK1 = 0x01;
	TIMSK3 = 0x01;

}
  


void EnableMotors(){
  PORTA |= 0x80; //motor 1 reset high
  PORTC |= 0x01; //motor 1 reset
}

void DisableMotors(){
  PORTA &= 0x7f; //motor 1 reset low
  PORTC &= 0xfe; //motor 1 reset low
}

// Motor 1:
// AHI   PB7 (OC1C)
// ALO   PA6 
// BHI   PB6  (OC1B)
// BLO    PA5
void SetPWM_1A(uint16_t width){
  PORTA &= 0xbf;  //make sure ALO is off
  if(width>0 && width < MOTORMAX){
    TCCR1A |= 0x08;//set OC1C at top, clear on compare
    OCR1C = width;
    return;
  }
  TCCR1A &= 0xf3;  //disconnect timer controlled output
  if(width==0)     //turn mosfet full OFF - this means that this side is floating
    PORTB &=0x7f;
  if(width==MOTORMAX) //turn mosfet full ON - full power!
    PORTB |=0x80;
}

//grounding an output actually ties it to ground, openning the low side MOSFET
//I won't let you ground the output while it is PWM'ing, neither will the chip.
void Gnd1A(){
  TCCR1A &= 0xf3;   //disconnect timer controlled output
  PORTB &=0x7f;    //turn High mosfet full OFF
  PORTA |= 0x40;   //turn low mosfet full on
}


void SetPWM_1B(uint16_t width){
  PORTA &= 0xdf;  //make sure BLO is off
  if(width>0 && width < MOTORMAX){
    TCCR1A |= 0x20;//set OC1B at top, clear on compare
    OCR1B = width;
    return;
  }
  TCCR1A &= 0xcf;  //disconnect timer controlled output
  if(width==0)     //turn mosfet full OFF - this means that this side is floating
    PORTB &=0xbf;
  if(width==MOTORMAX) //turn mosfet full ON - full power!
    PORTB |=0x40;
}

//grounding an output actually ties it to ground, openning the low side MOSFET
//I won't let you ground the output while it is PWM'ing, neither will the chip.
void Gnd1B(){
  TCCR1A &= 0xcf;   //disconnect timer controlled output
  PORTB &=0xbf;    //turn High mosfet full OFF
  PORTA |= 0x20;   //turn low mosfet full on
}

// Motor 2:
// AHI   PC4 (OC3C)
// ALO   PC5 
// BHI   PC6  (OC3A)
// BLO    PC7
void SetPWM_2A(uint16_t width){
  PORTC &= 0xdf;  //make sure ALO is off
  if(width>0 && width < MOTORMAX){
    TCCR3A |= 0x08;//set OC1C at top, clear on compare
    OCR3C = width;
    return;
  }
  TCCR3A &= 0xf3;  //disconnect timer controlled output
  if(width==0)     //turn mosfet full OFF - this means that this side is floating
    PORTC &=0xef;
  if(width==MOTORMAX) //turn mosfet full ON - full power!
    PORTC |=0x10;
}

//grounding an output actually ties it to ground, openning the low side MOSFET
//I won't let you ground the output while it is PWM'ing, neither will the chip.
void Gnd2A(){
  TCCR3A &= 0xf3;   //disconnect timer controlled output
  PORTC &=0xef;    //turn High mosfet full OFF
  PORTC |= 0x20;   //turn low mosfet full on
}


void SetPWM_2B(uint16_t width){
  PORTC &= 0x7f;  //make sure BLO is off
  if(width>0 && width < MOTORMAX){
    TCCR3A |= 0x80;//set OC1C at top, clear on compare
    OCR3A = width;
    return;
  }
  TCCR3A &= 0x3f;  //disconnect timer controlled output
  if(width==0)     //turn mosfet full OFF - this means that this side is floating
    PORTC &=0xbf;
  if(width==MOTORMAX) //turn mosfet full ON - full power!
    PORTC |=0x40;
}

//grounding an output actually ties it to ground, openning the low side MOSFET
//I won't let you ground the output while it is PWM'ing, neither will the chip.
void Gnd2B(){
  TCCR3A &= 0x3f;   //disconnect timer controlled output
  PORTC &=0xbf;    //turn High mosfet full OFF
  PORTC |= 0x80;   //turn low mosfet full on
}



void setAbsSpeedM1(int16_t m1){
  if(m1>0){
    Gnd1B();
    SetPWM_1A(gabs(m1));    
  }
  if(m1<0){
    Gnd1A();
    SetPWM_1B(gabs(m1)); 
  }
  if(m1==0){ //brake - tie both sides of the motor to gnd
    Gnd1B();
    Gnd1A();
  }
}

void setAbsSpeedM2(int16_t m2){
  if(m2>0){
    Gnd2B();
    SetPWM_2A(gabs(m2));    
  }
  if(m2<0){
    Gnd2A();
    SetPWM_2B(gabs(m2)); 
  }  
  if(m2==0){ //brake - tie both sides of the motor to gnd
    Gnd2B();
    Gnd2A();
  }
  
}

int16_t scaleCmd(int16_t in){
  //expected input is between -1000 and 1000
  //need to scale it to -500 to 500
 return in/2; 
}


//low level: set the exact speed we tell it, immediatelly
//positive means A is PWM'ing, negative means B is PWM'ing
//this is one of the gateways between high and low level, so the command must be scaled here
void setAbsSpeed(int16_t m1, int16_t m2){
  setAbsSpeedM1(scaleCmd(m1));
  setAbsSpeedM2(scaleCmd(m2));
}




#define STEERMULT 5
#define VELMULT 5

void SetSpeed(int16_t pot1, int16_t pot2){
  
  PORTB &= 0xfe;
  if((PINB & 0x01) ==0){
    setAbsSpeed(0,0);  
    PORTD |= 0x10;
    return;
  }
   PORTD &= 0xef; 
 //pot1 is steering, pot2 is accel...
 //range is 0-255
 int16_t steerleft, steerright, straight;
 
 //50 wide dead zone
 if(pot1 >103 && pot1 <153){
     steerleft=0;
     steerright=0;
 }
  if( pot1 >=153){  //map to steer 0-1500
    steerleft=(pot1-153)*-STEERMULT;
    steerright=(pot1-153)*STEERMULT;
  }
  if( pot1 <=103){  //map to steer 0-1500
    steerleft=(103-pot1)*STEERMULT;
    steerright=(103-pot1)*-STEERMULT;
  }
  
  if(pot2 >103 && pot2 <153){
     straight=0;
 }
  if( pot2 >=153){  //map to steer 0-1500
    straight=(pot2-153)*-VELMULT;
  }
  if( pot2 <=103){  //map to steer 0-1500
    straight=(103-pot2)*VELMULT;
  }
  
  
  setAbsSpeed(steerleft+straight,-1.0*(steerright+straight));
  
}


void HL_setMotor(uint8_t h_right, uint8_t l_right, uint8_t h_left, uint8_t l_left){
 //convert into ints:
 int16_t right,left;
 right = l_right+(h_right<<8);
 left = l_left+(h_left<<8);
 
 //TODO: may want to do a ramp here, or atleast massage the input a little...
 setAbsSpeed(-1*left,right);
  
}

#endif