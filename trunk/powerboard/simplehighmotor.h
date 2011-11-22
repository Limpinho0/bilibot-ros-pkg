#ifndef SIMPLEMOTOR
#define SIMPLEMOTOR
#include "lowlevelmotor.h"
#include "adc.h"


int16_t lastspeed;
int MOTOR_DIR_MSK;
// allowable directions
#define MOTOR_DIR_UP 0x1
#define MOTOR_DIR_DN 0x2

void setupMotors(){
    LowLevelSetup();
    startProfile();

    MOTOR_DIR_MSK = MOTOR_DIR_UP | MOTOR_DIR_DN;
    
    if (ULIMIT_VAL == 1)  
        MOTOR_DIR_MSK &= ~MOTOR_DIR_UP;
    if (LLIMIT_VAL == 1)  
        MOTOR_DIR_MSK &= ~MOTOR_DIR_DN;
}

ISR(INT7_vect)
{
    if (ULIMIT_VAL == 1) {
        MOTOR_DIR_MSK &= ~MOTOR_DIR_UP;
        HL_BaseSpeed(0);
    }
    if (ULIMIT_VAL == 0)
        MOTOR_DIR_MSK |= MOTOR_DIR_UP;
}

ISR(INT6_vect)
{
    if (LLIMIT_VAL == 1) {
        MOTOR_DIR_MSK &= ~MOTOR_DIR_DN;
        HL_BaseSpeed(0); 
    }
    if (LLIMIT_VAL == 0)
        MOTOR_DIR_MSK |= MOTOR_DIR_DN;
}     
    
void HL_BaseSpeed(int16_t spd){
    if ((spd > 0 && ((MOTOR_DIR_MSK & MOTOR_DIR_UP) == MOTOR_DIR_UP)) ||
        (spd < 0 && ((MOTOR_DIR_MSK & MOTOR_DIR_DN) == MOTOR_DIR_DN)) ||
         spd == 0)
    {
        lastspeed=spd;
        setBaseSpeed(spd);
    }
}

void simpleMotorCheck(){
    uint8_t pot=ADC_BASE_POT;
    if(lastspeed<0 && pot > 150)
      HL_BaseSpeed(0);
    if(lastspeed>0 && pot < 15)
      HL_BaseSpeed(0);
  
  
  
}

//a good start speed is 5000

uint8_t goingtotarget=0;
uint8_t basetarget=80;

void sendtoMid(){
    if(goingtotarget==0) return;
    if(abs(basetarget-ADC_BASE_POT)<4){
      goingtotarget=0;
      HL_BaseSpeed(0);
      return;
    }
    int16_t svel= ADC_BASE_POT-basetarget; //could be 160->-160
    HL_BaseSpeed(173*svel + (svel>0?2000:-2000));

    
}

uint8_t testMotors(){
 //see if they are plugged in
 //turn on base motor, then stop.  see if the current spiked
 /*
  startProfile();
  HL_BaseSpeed(20000);
  _delay_ms(10);
  HL_BaseSpeed(0);
  uint8_t maxcurr= getProfileMax(ADC_BASE_CURR_CHANNEL);
  if(maxcurr < 2)
    return 1;
    */
  return 0;
  //TODO: add hand motor
}

#endif
