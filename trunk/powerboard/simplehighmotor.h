#ifndef SIMPLEMOTOR
#define SIMPLEMOTOR
#include "lowlevelmotor.h"
#include "adc.h"

int16_t lastspeed;

int MOTOR_DIR_MSK;
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

#endif

