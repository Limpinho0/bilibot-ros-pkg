//this header has all the adc related stuff

#ifndef ADC_H
#define ADC_H
#define USING_ADC



// #define ADC_AD0_L adc_readings_l[0]
// #define ADC_AD1_L adc_readings_l[1]
// #define ADC_BATT_L adc_readings_l[2]
// #define ADC_GYRO_L adc_readings_l[3]
// #define ADC_BASE_CURR_L adc_readings_l[4]
// #define ADC_HAND_CURR_L adc_readings_l[5]
// #define ADC_TEMP_L adc_readings_l[6]
// #define ADC_BASE_POT_L adc_readings_l[7]
// 
// #define ADC_AD0_H adc_readings_h[0]
// #define ADC_AD1_H adc_readings_h[1]
// #define ADC_BATT_H adc_readings_h[2]
// #define ADC_GYRO_H adc_readings_h[3]
// #define ADC_BASE_CURR_H adc_readings_h[4]
// #define ADC_HAND_CURR_H adc_readings_h[5]
// #define ADC_TEMP_H adc_readings_h[6]
// #define ADC_BASE_POT_H adc_readings_h[7]

// #define ADC_AD0       (ADC_AD0_L + ADC_AD0_H<<8)
// #define ADC_AD1       (ADC_AD1_L + ADC_AD1_H<<8)
// #define ADC_BATT      (ADC_BATT_L +  ADC_BATT_H<<8)
// #define ADC_BASE_POT  (ADC_BASE_POT_L +  ADC_BASE_POT_H<<8)
// #define ADC_BASE_CURR (ADC_BASE_CURR_L +  ADC_BASE_CURR_H<<8)
// #define ADC_HAND_CURR (ADC_HAND_CURR_L +  ADC_HAND_CURR_H<<8)
// #define ADC_TEMP      (ADC_TEMP_L +  ADC_TEMP_H<<8)
// #define ADC_GYRO      (ADC_GYRO_L +  ADC_GYRO_H<<8)

#define ADC_AD0 adc_readings[0]
#define ADC_AD1 adc_readings[1]
#define ADC_BATT adc_readings[2]
#define ADC_GYRO adc_readings[3]
#define ADC_BASE_CURR adc_readings[4]
#define ADC_HAND_CURR adc_readings[5]
#define ADC_TEMP adc_readings[6]
#define ADC_BASE_POT adc_readings[7]



//uint8_t adc_readings_l[8];
//uint8_t adc_readings_h[8];
uint8_t adc_readings[8];

void setADCChannel(uint8_t channel){
    ADMUX &= 0xf8;
    ADMUX |= (channel & 0x07);
}
void setupADC(){
  //NOTE: make sure jtag interface is disabled in the fuses, or PF4-7 will be disabled.
  //ADMUX
  // 01xx xxxx    REFS: Volatge ref is AVCC
  // xx1x xxxx    ADLAR: ADC result is right aligned
  // xxx0 0xxx    MUX 4:3 -> always zero because we are not doing differential readings
  // xxxx x000    MUX 2:0 -> arbitrarily set the channel to 0.  it can be changed by calling setADCChannel
  // 0x40
  ADMUX=0x60;
  
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


SIGNAL(ADC_vect){ 
  uint8_t channel = ADMUX & 0x07;
  //make reading
//  adc_readings_l[channel] = ADCL;
  adc_readings[channel] = ADCH;
//  adc_readings[channel] = adc_readings_h[channel];
  //adc_readings[channel]=adc_readings[channel]*0x0100+ adc_readings_l[channel];
  
  
//   stransmitf(" r: %i ",channel);
//   uint8_t i=0;
//   for(i=0;i<8;i++)
//     stransmitf(" %04i ",adc_readings[i]);
//   transmit('\n');
//   transmit('\r');
//   transmit(10);
//   transmit(13);
  //update channel:
  channel++;
  setADCChannel(channel);
  
  //start next adc read:
  ADCSRA |= 0x40;
  
}


#endif
