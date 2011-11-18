//Atmel pin 4, labeled PE2, is connected to the on led.  The suffix for this pin is: ON_LED.  It has the following defines: 	
   #define SETUP_ON_LED  DDRE |= 0x04	
   #define  h_ON_LED PORTE |= 0x04	
   #define  l_ON_LED PORTE &= 0xFB	
   #define READ_ON_LED  PORTE & 0x04
//Atmel pin 5, labeled PE3, is connected to the hand speed.  The suffix for this pin is: HAND_SPD.  It has the following defines: 	
   #define SETUP_HAND_SPD  DDRE |= 0x08	
   #define  h_HAND_SPD PORTE |= 0x08	
   #define  l_HAND_SPD PORTE &= 0xF7	
   #define READ_HAND_SPD  PORTE & 0x08
//Atmel pin 6, labeled PE4, is connected to the base speed.  The suffix for this pin is: BASE_SPD.  It has the following defines: 	
   #define SETUP_BASE_SPD  DDRE |= 0x10	
   #define  h_BASE_SPD PORTE |= 0x10	
   #define  l_BASE_SPD PORTE &= 0xEF	
   #define READ_BASE_SPD  PORTE & 0x10
//Atmel pin 7, labeled PE5, is connected to the on button.  The suffix for this pin is: ON_BTN.  It has the following defines: 	
   #define SETUP_ON_BTN  DDRE |= 0x20	
   #define  h_ON_BTN PORTE |= 0x20	
   #define  l_ON_BTN PORTE &= 0xDF	
   #define READ_ON_BTN  PORTE & 0x20
//Atmel pin 8, labeled PE6, is connected to the base lower limit.  The suffix for this pin is: LLIMIT.  It has the following defines: 	
   #define SETUP_LLIMIT  DDRE &= 0xBF	
   #define  h_LLIMIT PORTE |= 0x40	
   #define  l_LLIMIT PORTE &= 0xBF	
   #define READ_LLIMIT  PINE & 0x40
//Atmel pin 9, labeled PE7, is connected to the base upper limit.  The suffix for this pin is: ULIMIT.  It has the following defines: 	
   #define SETUP_ULIMIT  DDRE &= 0x7F	
   #define  h_ULIMIT PORTE |= 0x80	
   #define  l_ULIMIT PORTE &= 0x7F	
   #define READ_ULIMIT  PINE & 0x80
//Atmel pin 16, labeled PB6, is connected to the throb LED 1.  The suffix for this pin is: THROB1.  It has the following defines: 	
   #define SETUP_THROB1  DDRB |= 0x40	
   #define  h_THROB1 PORTB |= 0x40	
   #define  l_THROB1 PORTB &= 0xBF	
   #define READ_THROB1  PORTB & 0x40
//Atmel pin 17, labeled PB7, is connected to the Throb LED 2.  The suffix for this pin is: THROB2.  It has the following defines: 	
   #define SETUP_THROB2  DDRB |= 0x80	
   #define  h_THROB2 PORTB |= 0x80	
   #define  l_THROB2 PORTB &= 0x7F	
   #define READ_THROB2  PORTB & 0x80
//Atmel pin 33, labeled PG0, is connected to the General IO 0.  The suffix for this pin is: GIO_0.  It has the following defines: 	
   #define SETUP_GIO_0  DDRG |= 0x01	
   #define  h_GIO_0 PORTG |= 0x01	
   #define  l_GIO_0 PORTG &= 0xFE	
   #define READ_GIO_0  PORTG & 0x01
//Atmel pin 34, labeled PG1, is connected to the General IO 1.  The suffix for this pin is: GIO_1.  It has the following defines: 	
   #define SETUP_GIO_1  DDRG |= 0x02	
   #define  h_GIO_1 PORTG |= 0x02	
   #define  l_GIO_1 PORTG &= 0xFD	
   #define READ_GIO_1  PORTG & 0x02
//Atmel pin 35, labeled PC0, is connected to the General IO 2.  The suffix for this pin is: GIO_2.  It has the following defines: 	
   #define SETUP_GIO_2  DDRC |= 0x01	
   #define  h_GIO_2 PORTC |= 0x01	
   #define  l_GIO_2 PORTC &= 0xFE	
   #define READ_GIO_2  PORTC & 0x01
//Atmel pin 36, labeled PC1, is connected to the General IO 3.  The suffix for this pin is: GIO_3.  It has the following defines: 	
   #define SETUP_GIO_3  DDRC |= 0x02	
   #define  h_GIO_3 PORTC |= 0x02	
   #define  l_GIO_3 PORTC &= 0xFD	
   #define READ_GIO_3  PORTC & 0x02
//Atmel pin 37, labeled PC2, is connected to the General IO 4.  The suffix for this pin is: GIO_4.  It has the following defines: 	
   #define SETUP_GIO_4  DDRC |= 0x04	
   #define  h_GIO_4 PORTC |= 0x04	
   #define  l_GIO_4 PORTC &= 0xFB	
   #define READ_GIO_4  PORTC & 0x04
//Atmel pin 38, labeled PC3, is connected to the General IO 5.  The suffix for this pin is: GIO_5.  It has the following defines: 	
   #define SETUP_GIO_5  DDRC |= 0x08	
   #define  h_GIO_5 PORTC |= 0x08	
   #define  l_GIO_5 PORTC &= 0xF7	
   #define READ_GIO_5  PORTC & 0x08
//Atmel pin 39, labeled PC4, is connected to the General IO 6.  The suffix for this pin is: GIO_6.  It has the following defines: 	
   #define SETUP_GIO_6  DDRC |= 0x10	
   #define  h_GIO_6 PORTC |= 0x10	
   #define  l_GIO_6 PORTC &= 0xEF	
   #define READ_GIO_6  PORTC & 0x10
//Atmel pin 40, labeled PC5, is connected to the General IO 7.  The suffix for this pin is: GIO_7.  It has the following defines: 	
   #define SETUP_GIO_7  DDRC |= 0x20	
   #define  h_GIO_7 PORTC |= 0x20	
   #define  l_GIO_7 PORTC &= 0xDF	
   #define READ_GIO_7  PORTC & 0x20
//Atmel pin 42, labeled PC7, is connected to the Power indicator.  The suffix for this pin is: PWR_IND.  It has the following defines: 	
   #define SETUP_PWR_IND  DDRC |= 0x80	
   #define  h_PWR_IND PORTC |= 0x80	
   #define  l_PWR_IND PORTC &= 0x7F	
   #define READ_PWR_IND  PORTC & 0x80
//Atmel pin 43, labeled PG2, is connected to the Kinect enable.  The suffix for this pin is: KIN_EN.  It has the following defines: 	
   #define SETUP_KIN_EN  DDRG |= 0x04	
   #define  h_KIN_EN PORTG |= 0x04	
   #define  l_KIN_EN PORTG &= 0xFB	
   #define READ_KIN_EN  PORTG & 0x04
//Atmel pin 44, labeled PA7, is connected to the hand ctrl a.  The suffix for this pin is: HAND_CTRL_A.  It has the following defines: 	
   #define SETUP_HAND_CTRL_A  DDRA |= 0x80	
   #define  h_HAND_CTRL_A PORTA |= 0x80	
   #define  l_HAND_CTRL_A PORTA &= 0x7F	
   #define READ_HAND_CTRL_A  PORTA & 0x80
//Atmel pin 45, labeled PA6, is connected to the hand ctrl b.  The suffix for this pin is: HAND_CTRL_B.  It has the following defines: 	
   #define SETUP_HAND_CTRL_B  DDRA |= 0x40	
   #define  h_HAND_CTRL_B PORTA |= 0x40	
   #define  l_HAND_CTRL_B PORTA &= 0xBF	
   #define READ_HAND_CTRL_B  PORTA & 0x40
//Atmel pin 46, labeled PA5, is connected to the Base ctrl a.  The suffix for this pin is: BASE_CTRL_A.  It has the following defines: 	
   #define SETUP_BASE_CTRL_A  DDRA |= 0x20	
   #define  h_BASE_CTRL_A PORTA |= 0x20	
   #define  l_BASE_CTRL_A PORTA &= 0xDF	
   #define READ_BASE_CTRL_A  PORTA & 0x20
//Atmel pin 47, labeled PA4, is connected to the Base ctrl B.  The suffix for this pin is: BASE_CTRL_B.  It has the following defines: 	
   #define SETUP_BASE_CTRL_B  DDRA |= 0x10	
   #define  h_BASE_CTRL_B PORTA |= 0x10	
   #define  l_BASE_CTRL_B PORTA &= 0xEF	
   #define READ_BASE_CTRL_B  PORTA & 0x10
//Atmel pin 48, labeled PA3, is connected to the over charge.  The suffix for this pin is: OVR_CHRG.  It has the following defines: 	
   #define SETUP_OVR_CHRG  DDRA &= 0xF7	
   #define  h_OVR_CHRG PORTA |= 0x08	
   #define  l_OVR_CHRG PORTA &= 0xF7	
   #define READ_OVR_CHRG  PINA & 0x08
//Atmel pin 49, labeled PA2, is connected to the Create charge enable.  The suffix for this pin is: CREATE_PWR_EN.  It has the following defines: 	
   #define SETUP_CREATE_PWR_EN  DDRA |= 0x04	
   #define  h_CREATE_PWR_EN PORTA |= 0x04	
   #define  l_CREATE_PWR_EN PORTA &= 0xFB	
   #define READ_CREATE_PWR_EN  PORTA & 0x04
//Atmel pin 50, labeled PA1, is connected to the create on switch.  The suffix for this pin is: CREATE_ON.  It has the following defines: 	
   #define SETUP_CREATE_ON  DDRA |= 0x02	
   #define  h_CREATE_ON PORTA |= 0x02	
   #define  l_CREATE_ON PORTA &= 0xFD	
   #define READ_CREATE_ON  PORTA & 0x02
//Atmel pin 51, labeled PA0, is connected to the create charging ind.  The suffix for this pin is: CREATE_CHRG_IND.  It has the following defines: 	
   #define SETUP_CREATE_CHRG_IND  DDRA &= 0xFE	
   #define  h_CREATE_CHRG_IND PORTA |= 0x01	
   #define  l_CREATE_CHRG_IND PORTA &= 0xFE	
   #define READ_CREATE_CHRG_IND  PINA & 0x01
//Atmel pin 54, labeled PF7, is connected to the gyro rate.  The suffix for this pin is: GYRO_RATE.  It has the following defines: 	
   #define SETUP_GYRO_RATE  DDRF &= 0x7F	
   #define  h_GYRO_RATE PORTF |= 0x80	
   #define  l_GYRO_RATE PORTF &= 0x7F	
   #define READ_GYRO_RATE  PINF & 0x80
//Atmel pin 55, labeled PF6, is connected to the gyro temp.  The suffix for this pin is: GYRO_TEMP.  It has the following defines: 	
   #define SETUP_GYRO_TEMP  DDRF &= 0xBF	
   #define  h_GYRO_TEMP PORTF |= 0x40	
   #define  l_GYRO_TEMP PORTF &= 0xBF	
   #define READ_GYRO_TEMP  PINF & 0x40
//Atmel pin 56, labeled PF5, is connected to the hand current.  The suffix for this pin is: HAND_CURR.  It has the following defines: 	
   #define SETUP_HAND_CURR  DDRF &= 0xDF	
   #define  h_HAND_CURR PORTF |= 0x20	
   #define  l_HAND_CURR PORTF &= 0xDF	
   #define READ_HAND_CURR  PINF & 0x20
//Atmel pin 57, labeled PF4, is connected to the base current.  The suffix for this pin is: BASE_CURR.  It has the following defines: 	
   #define SETUP_BASE_CURR  DDRF &= 0xEF	
   #define  h_BASE_CURR PORTF |= 0x10	
   #define  l_BASE_CURR PORTF &= 0xEF	
   #define READ_BASE_CURR  PINF & 0x10
//Atmel pin 58, labeled PF3, is connected to the Base pot.  The suffix for this pin is: BASE_POT.  It has the following defines: 	
   #define SETUP_BASE_POT  DDRF &= 0xF7	
   #define  h_BASE_POT PORTF |= 0x08	
   #define  l_BASE_POT PORTF &= 0xF7	
   #define READ_BASE_POT  PINF & 0x08
//Atmel pin 59, labeled PF2, is connected to the Battery sense.  The suffix for this pin is: BATT_SENSE.  It has the following defines: 	
   #define SETUP_BATT_SENSE  DDRF &= 0xFB	
   #define  h_BATT_SENSE PORTF |= 0x04	
   #define  l_BATT_SENSE PORTF &= 0xFB	
   #define READ_BATT_SENSE  PINF & 0x04

void setupArm(){
  SETUP_HAND_SPD;  
  SETUP_BASE_SPD; 
  SETUP_HAND_CTRL_A;  
  SETUP_HAND_CTRL_B;  
  SETUP_BASE_CTRL_A;  
  SETUP_BASE_CTRL_B;  
  SETUP_HAND_CURR;  
  SETUP_BASE_CURR;  
  SETUP_BASE_POT;  

}

void chargeCreateOn(){
    h_CREATE_PWR_EN;
}

void chargeCreateOff(){
    l_CREATE_PWR_EN;
}
void setADCChannel(uint8_t channel){
    ADMUX &= 0xf8;
    ADMUX |= (channel & 0x07);
}

void ToggleThrob1(){
  if(READ_THROB1)
    l_THROB1;
  else
    h_THROB1;
}

void ToggleThrob2(){
  if(READ_THROB2)
    l_THROB2;
  else
    h_THROB2;
}

void setupADC(){
  //NOTE: make sure jtag interface is disabled in the fuses, or PF4-7 will be disabled.
  //ADMUX
  // 01xx xxxx    REFS: Volatge ref is AVCC
  // xx0x xxxx    ADLAR: ADC result is right aligned
  // xxx0 0xxx    MUX 4:3 -> always zero because we are not doing differential readings
  // xxxx x000    MUX 2:0 -> arbitrarily set the channel to 0.  it can be changed by calling setADCChannel
  // 0x40
  ADMUX=0x40;
  
  //  ADCSRA
  //  1xxx xxxx   ADEN - Enable ADC
  //  x0xx xxxx   ADSC - start conversion
  //  xx0x xxxx   ADFR - do not enable free running
  //  xxx0 xxxx   ADC interrup flag
  //  xxxx 1xxx   Enable ADC interrupts
  //  xxxx x111   ADC prescaler of 128, so adc freq is 125Khz
  ADCSRA=0x9f;
  DDRF = 0x00;
  
}



void setupIO(){
  SETUP_ON_LED;  l_ON_LED;
  //enable pullups for on button:
  SETUP_ON_BTN;  h_ON_BTN;
  //enable pullups for limit switches:
  SETUP_LLIMIT;  h_LLIMIT;
  SETUP_ULIMIT;  h_ULIMIT;
  SETUP_THROB1;  
  SETUP_THROB2;  
  //turn off all gio pins:
  SETUP_GIO_0;   h_GIO_0;  
  SETUP_GIO_1;   h_GIO_1;  
  SETUP_GIO_2;   h_GIO_2;  
  SETUP_GIO_3;   h_GIO_3;  
  SETUP_GIO_4;   h_GIO_4;  
  SETUP_GIO_5;   h_GIO_5;  
  SETUP_GIO_6;   h_GIO_6;  
  SETUP_GIO_7;   h_GIO_7;  
  
  //kinect on by default
  SETUP_KIN_EN;   h_KIN_EN;
  
  //enable pullup for battery indicators
  SETUP_OVR_CHRG;  h_OVR_CHRG;
  SETUP_PWR_IND;  h_PWR_IND;
  
  //create off by default:
  SETUP_CREATE_PWR_EN;  h_CREATE_PWR_EN;
  //keep create turnon pin low:
  SETUP_CREATE_ON;  l_CREATE_ON;

//ADC pins:
  SETUP_CREATE_CHRG_IND;  h_CREATE_CHRG_IND;
  SETUP_GYRO_RATE;  h_GYRO_RATE;
  SETUP_GYRO_TEMP; h_GYRO_TEMP;
  SETUP_BATT_SENSE;  h_BATT_SENSE;
}