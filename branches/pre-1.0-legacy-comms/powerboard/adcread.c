
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/signal.h>
#include "pinmapping.h"


/* at 10 MHz we get 1us per 10 instructions */
inline void delayus() { asm volatile("nop\nnop\nnop\nnop\nnop\n"
                                     "nop\nnop\nnop\nnop\nnop"); }
				     
void delayms(uint16_t millis) {
  uint16_t loop;
  while ( millis ) {
    loop = 100;
    while (loop) {
      /* 10us of delays */
      delayus(); delayus(); delayus(); delayus(); delayus();
      delayus(); delayus(); delayus(); delayus(); delayus();
      loop --;
    }
    millis--;
  }
}


#define stransmitf(args...)  {                   \
		  char tstring[50];					\
		  sprintf(tstring,args);			\
		  transmitstring(tstring,strlen(tstring));	\
                    } 


void transmit(unsigned char data){
	while(!(UCSR0A & 0x20));
//       while ( !( UCSR0A & (1<<UDRE0)) )
	UDR0=data;
} 

void transmitstring(char *tx, int len){
	int i;
	for(i=0;i<len;i++)
		transmit(tx[i]);
}

void togglePB7(){
	if(!(PORTB & 0x80))
		PORTB |= 0x80;
	else
		PORTB &= 0x7f;
}

void togglePB6(){
	if(!(PORTB & 0x40))
		PORTB |= 0x40;
	else
		PORTB &= 0xbf;
}


char charsread;

void parser(char c){
  if(c=='o'){
      chargeCreateOn();
   transmit('o');
      return;
  }
  if(c=='l'){
      chargeCreateOff();
   transmit('l');
      return;
  }
  
  
  
  
}





SIGNAL(SIG_USART0_RECV){
  
  while(!(UCSR0A & 0x80));
    charsread= UDR0;
  togglePB6();
  parser(charsread);
}




void setupSerial(){
  
  //UCSR0B:
  // 1xxx xxxx		Recieve complete enabled
  // x0xx xxxx		Transmit Complete disabled
  // xx0x xxxx		Data empmty interrupt disabled
  // xxx1 xxxx		Reciever enabled
  // xxxx 1xxx		Transmitter enabled
  // xxxx x0xx		Character size - 8 bits
  // xxxx xx00		unused (9th data bit)
  //UCSR0B - 0x98
  UCSR0B=0x98; 
 
 
   //UCSR0C:
  // 0xxx xxxx		reserved
  // x0xx xxxx		Asyncronous communication
  // xx00 xxxx		No Parity
  // xxxx 0xxx		one stop bit
  // xxxx x11x		8 bit character size
  // xxxx xxx0		unused (Clock polarity)
  //UCSR0c - 0x0x06
  UCSR0C=0x06;
  
  //UBBR = clockfreq/16*baud -1  
  // = 16,000,000/16*9600 -1 = 103
  UBRR0H=0x00;
  UBRR0L=0x67;
  
}

void sendBoardData(){
  //Data format:
  // $D<rate (2 bytes)><flags>%
  char tosend[6];
  tosend[0]='$';
  tosend[1]='D';
  tosend[2]=adc_readings_h[7];
  tosend[3]=adc_readings_l[7];
  tosend[4]=READ_ON_BTN;
    tosend[5]='%';
    transmitstring(&tosend, 6);
  
}


void sendBatteryData(){
  //Data format:
  // $B<battlevel (2 bytes)><flags>%
    //adc channel (0x04 -> adc_readings[2])
    uint8_t battlevell=adc_readings_l[2];  //TODO: protect this with buffer!
    uint8_t battlevelh=adc_readings_h[2];  //TODO: protect this with buffer!
    uint8_t isovercharge=READ_OVR_CHRG;  //indicate that the battery has reached max charge
    uint8_t ispowered=READ_PWR_IND;  //indicate that the battery has reached max charge
    char tosend[6];
    tosend[0]='$';
    tosend[1]='B';
    tosend[2]=battlevelh;
    tosend[3]=battlevell;
    tosend[4]=0x00;
    if(isovercharge) tosend[3] |=0x01;
    if(ispowered) tosend[3] |=0x02;
    tosend[5]='%';
    transmitstring(&tosend, 6);
    
}
void sendBoardDataTest(){
  //Data format:
  // $B<battlevel (2 bytes)><flags>%
    //adc channel (0x04 -> adc_readings[2])
    uint8_t battlevell=adc_readings_l[7];  //TODO: protect this with buffer!
    uint8_t battlevelh=adc_readings_h[7];  //TODO: protect this with buffer!
    uint8_t isovercharge=READ_OVR_CHRG;  //indicate that the battery has reached max charge
    uint8_t ispowered=READ_PWR_IND;  //indicate that the battery has reached max charge
    char tosend[6];
    transmit('$');
    transmit('d');
    
    char level[5];
    sprintf(level,"%3i\r\n",battlevell);
    transmitstring(&level, 5);
    
//     transmitstring(&tosend, 6);
//     tosend[2]=battlevelh;
//     tosend[3]=battlevell;
//     tosend[4]=0x00;
//     if(isovercharge) tosend[3] |=0x01;
//     if(ispowered) tosend[3] |=0x02;
    transmit('%');
//     transmitstring(&tosend, 6);
    
}
void sendBatteryDataTest(){
  //Data format:
  // $B<battlevel (2 bytes)><flags>%
    //adc channel (0x04 -> adc_readings[2])
    uint8_t battlevell=adc_readings_l[2];  //TODO: protect this with buffer!
    uint8_t battlevelh=adc_readings_h[2];  //TODO: protect this with buffer!
    uint8_t isovercharge=READ_OVR_CHRG;  //indicate that the battery has reached max charge
    uint8_t ispowered=READ_PWR_IND;  //indicate that the battery has reached max charge
    char tosend[6];
    transmit('$');
    transmit('B');
    
    char level[5];
    sprintf(level,"%3i\r\n",battlevell);
    transmitstring(&level, 5);
    
//     transmitstring(&tosend, 6);
//     tosend[2]=battlevelh;
//     tosend[3]=battlevell;
//     tosend[4]=0x00;
//     if(isovercharge) tosend[3] |=0x01;
//     if(ispowered) tosend[3] |=0x02;
    transmit('%');
//     transmitstring(&tosend, 6);
    
}

int main(void) {
  setupIO();
    //set up PD6 to blink
    DDRB |= 0xc0; /* set PD6 to output */
    PORTB &= 0x3f; /* LED on */
    DDRD &= 0xfe;
    PORTD |= 0x01;   
	//initialize serial port

	UBRR1H=0x00;
        UBRR1L=0x67;
	UCSR1B=0x18;
	UCSR1C=0x06;

	setupSerial();
	
	int8_t i,j;

 	togglePB6();
	delayms(500);
 	togglePB7();
	delayms(500);
 	togglePB6();
	delayms(500);
 	togglePB7();
  setupADC();
	sei();
	  //start next adc read:
//  	  ADCSRA |= 0x40;
  while(1){
	delayms(100);
	
//	for(i=0;i<100;i++){
//     if(PIND && 0x01){
// 	for(i=0;i<13;i++)
// 		transmit(string[i]);
// 	}
	sendBatteryData();
	sendBoardData();
	
 	togglePB6();
   	ADCSRA |= 0x40;
  }
  
  return 0;
}


