
#include <avr/interrupt.h>
#include <avr/io.h>

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

void transmit(unsigned char data){
	while(!(UCSR0A & 0x20));
//       while ( !( UCSR0A & (1<<UDRE0)) )
	UDR0=data;
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

SIGNAL(SIG_USART0_RECV){
  togglePB6();
}


int main(void) {
    //set up PD6 to blink
    DDRB |= 0xc0; /* set PD6 to output */
    PORTB &= 0x3f; /* LED on */
    DDRD &= 0xfe;
    PORTD |= 0x01;   
	//initialize serial port
	//UBBR = clockfreq/16*baud -1  
	// = 16,000,000/16*9600 -1 = 103
	UBRR1H=0x00;
        UBRR1L=0x67;
	UCSR1B=0x18;
	UCSR1C=0x06;

	UBRR0H=0x00;
        UBRR0L=0x67;
	UCSR0B=0x98;
	UCSR0C=0x06;
	char *string="hello world!\n";
	int8_t i,j;

	sei();
  while(1){
	delayms(5000);
//	for(i=0;i<100;i++){
    if(PIND && 0x01){
	for(i=0;i<13;i++)
		transmit(string[i]);
	}
	togglePB7();
  }
  
  return 0;
}


