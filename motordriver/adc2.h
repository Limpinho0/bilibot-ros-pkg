


#ifndef ADC_H
#define ADC_H
//ADC channels:
//ADC0 - pot1
//ADC1 - pot2
//ADC2 - pot3
//ADC3 - m1 current
//ADC4 - temp1
//ADC5 - n/c
//ADC6 - temp2
//ADC7 - m2 current







void setADCChannel(uint8_t channel){
    ADMUX &= 0xf8;
    ADMUX |= (channel & 0x07);
}



void setupADC(){
  //NOTE: make sure jtag interface is disabled in the fuses, or PF4-7 will be disabled.
  //ADMUX
  // 01xx xxxx    REFS: Volatge ref is AVCC
  // xx1x xxxx    ADLAR: ADC result is left aligned
  // xxx0 0xxx    MUX 4:3 -> always zero because we are not doing differential readings
  // xxxx x000    MUX 2:0 -> arbitrarily set the channel to 0.  it can be changed by calling setADCChannel
  // 0x40
  ADMUX=0x63;
  
  //  ADCSRA
  //  1xxx xxxx   ADEN - Enable ADC
  //  x0xx xxxx   ADSC - start conversion
  //  xx0x xxxx   ADATE - do not enable free running
  //  xxx0 xxxx   ADC interrup flag
  //  xxxx 1xxx   Enable ADC interrupts
  //  xxxx x110   ADC prescaler of 64, so adc freq is 125Khz
  ADCSRA=0x9e;
  DDRF = 0x00;
  
  //set up deadman switch
  DDRB &= 0xfe;
  PORTB &= 0xfe;
  
}

// void togglePD5(){
// 	if(PORTD & 0x20)
// 		PORTD &= 0xdf;
// 	else
// 		PORTD |= 0x20;
// }


uint8_t pot_1, pot_2, curr1,curr2,temp1,temp2;
// uint16_t adccount;

// SIGNAL(ADC_vect){ 
//   
// //     if(adccount>=1000){ 
// //     togglePD5();
// //     adccount=0;
// //   }
// //   adccount++;
//   uint8_t channel = ADMUX & 0x07;
//   //make reading
//   
//   //just making two readings: the two pots - ADC0 and ADC2
//   
//   switch (channel){
//     case 0:
//       pot_1=ADCH;
//       break;
//     case 1:
//       break;
//     case 2:
//       pot_2=ADCH;
//       break;
//     case 3:
//       curr1=ADCH;
//       break;
//     case 4:
//       temp1=ADCH;
//       break;
//     case 5:
//       break;
//     case 6:
//       temp2=ADCH;
//       break;
//     case 7:
//       curr2=ADCH;
//       break;
// 
//   }
//     setADCChannel(channel+1);
// 
//   //start next adc read:
//   ADCSRA |= 0x40;
//   
// }




#endif
