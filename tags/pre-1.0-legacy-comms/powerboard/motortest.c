#include <avr/io.h>
#include "motorcontrols.h"

/* at 10 MHz we get 1us per 10 instructions */
inline void delayus() { asm volatile("nop\nnop\nnop\nnop\nnop\n"
                                     "nop\nnop\nnop\nnop\nnop"); }

void delayms(uint16_t millis) {
  uint16_t loop;
  while ( millis ) {
    //do 10,000 cycles:
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

void togglePB7(){
	if(!(PORTB & (1<<PB7)))
		PORTB |= 1<<PB7;
	else
		PORTB &= !(1<<PB7);
}




int main(void) {
  DDRB |= 1<<PB7; /* set PB0 to output */
  initMotors();
  
  HandEnable();
  while(1) {
    togglePB7();
    HandUp();   
    delayms(1000);
    togglePB7();
    HandDown();
    delayms(1000);
  }
  return 0;
}

