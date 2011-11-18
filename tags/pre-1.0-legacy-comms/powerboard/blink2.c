
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/signal.h>

/* at 10 MHz we get 1us per 10 instructions 
inline void delayus() { asm volatile("nop\nnop\nnop\nnop\nnop\n"
                                     "nop\nnop\nnop\nnop\nnop"); }
*/

/* At 1 Mhz we get 1us per instruction */
inline void delayus() { asm volatile("nop\n");}


SIGNAL(SIG_OUTPUT_COMPARE0){ /* defined in avr/iom128.h */
	//		PORTB |= 0x80; /* LED on */
	
	
    volatile static int ggocount;
    ggocount=ggocount+1;
  
	if(ggocount >= 250){
		ggocount=0;  
		if (PORTB & 0x80)
			PORTB &= 0x7f; /* LED on */		
		else
			PORTB |= 0x80; /* LED off */
		
	} 

}

				     
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

int main(void) {
  TCCR0 = 0x1d; /* set Timer0 to be active with prescaler of 8 */
  TIMSK |= 0x02; /* set The Compare match interrupt Vector to be active */
  OCR0 = 124;    /*set copare at 124*/
  sei(); /* enable interrupts */
  
  DDRB |= 0x90; /* set PB7,PB4 to output */
  
  while(1);
  
  
  return 0;
}


