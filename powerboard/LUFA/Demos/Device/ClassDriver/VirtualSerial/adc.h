//this header has all the adc related stuff




#define ADC_AD0_L adc_readings_l[0]
#define ADC_AD1_L adc_readings_l[1]
#define ADC_BATT_L adc_readings_l[2]
#define ADC_GYRO_L adc_readings_l[3]
#define ADC_BASE_CURR_L adc_readings_l[4]
#define ADC_HAND_CURR_L adc_readings_l[5]
#define ADC_TEMP_L adc_readings_l[6]
#define ADC_BASE_POT_L adc_readings_l[7]

#define ADC_AD0_H adc_readings_h[0]
#define ADC_AD1_H adc_readings_h[1]
#define ADC_BATT_H adc_readings_h[2]
#define ADC_GYRO_H adc_readings_h[3]
#define ADC_BASE_CURR_H adc_readings_h[4]
#define ADC_HAND_CURR_H adc_readings_h[5]
#define ADC_TEMP_H adc_readings_h[6]
#define ADC_BASE_POT_H adc_readings_h[7]

#define ADC_AD0       (ADC_AD0_L + ADC_AD0_H<<8)
#define ADC_AD1       (ADC_AD1_L + ADC_AD1_H<<8)
#define ADC_BATT      (ADC_BATT_L +  ADC_BATT_H<<8)
#define ADC_BASE_POT  (ADC_BASE_POT_L +  ADC_BASE_POT_H<<8)
#define ADC_BASE_CURR (ADC_BASE_CURR_L +  ADC_BASE_CURR_H<<8)
#define ADC_HAND_CURR (ADC_HAND_CURR_L +  ADC_HAND_CURR_H<<8)
#define ADC_TEMP      (ADC_TEMP_L +  ADC_TEMP_H<<8)
#define ADC_GYRO      (ADC_GYRO_L +  ADC_GYRO_H<<8)

uint8_t adc_readings_l[8];
uint8_t adc_readings_h[8];

void setADCChannel(uint8_t channel){
    ADMUX &= 0xf8;
    ADMUX |= (channel & 0x07);
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


SIGNAL(SIG_ADC){ 
 	togglePB7();
  uint8_t channel = ADMUX & 0x07;
  //make reading
  adc_readings_l[channel] = ADCL;
  adc_readings_h[channel] = ADCH;
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