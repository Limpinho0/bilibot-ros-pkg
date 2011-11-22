#ifndef SIMPLEMOTOR
#define SIMPLEMOTOR
#include "lowlevelmotor.h"
#include "adc.h"

int16_t _lastSpeed;
uint8_t _targetPosition;

int MOTOR_DIR_MSK;
#define MOTOR_DIR_UP 0x1
#define MOTOR_DIR_DN 0x2

#define MOTOR_DEFAULT_SPD 32767
#define MOTOR_POS_ERR 5

void setupMotors(){
    LowLevelSetup();
    startProfile();

    _targetPosition = 0;

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
        _lastSpeed=spd;
        setBaseSpeed(spd);
    }
}

void HL_SetBasePosition(int8_t position)
{
    if (abs(ADC_BASE_POT-position) <= MOTOR_POS_ERR)  
        HL_BaseSpeed(0);

    if (ADC_BASE_POT > position)
        HL_BaseSpeed(MOTOR_DEFAULT_SPD);
    if (ADC_BASE_POT < position)
        HL_BaseSpeed(-MOTOR_DEFAULT_SPD);

    _targetPosition = position;
}

uint16_t HL_GetLastSpeed()
{
    return _lastSpeed;
}

void HL_CheckBasePosition()
{
    if (abs(ADC_BASE_POT-_targetPosition) <= MOTOR_POS_ERR)  
        HL_BaseSpeed(0);
}

#endif

