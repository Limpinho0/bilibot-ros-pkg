#ifndef SIMPLEMOTOR
#define SIMPLEMOTOR
#include "lowlevelmotor.h"
#include "adc.h"

int16_t _curSpeed;
uint8_t _targetPosition;

uint8_t _handTask;
uint32_t _count;

int MOTOR_DIR_MSK;
#define MOTOR_DIR_UP 0x1
#define MOTOR_DIR_DN 0x2

#define MOTOR_UP_SPD 32767
#define MOTOR_DN_SPD -32767
#define MOTOR_POS_ERR 5

void setupMotors(){
    LowLevelSetup();
    startProfile();

    _handTask = 0;
    _targetPosition = 0;
    _curSpeed = 0;

    MOTOR_DIR_MSK = MOTOR_DIR_UP | MOTOR_DIR_DN;
    
    if (ULIMIT_VAL == 1)  
        MOTOR_DIR_MSK &= ~MOTOR_DIR_UP;
    if (LLIMIT_VAL == 1)  
        MOTOR_DIR_MSK &= ~MOTOR_DIR_DN;
}

ISR(INT7_vect)
{
    if (ULIMIT_VAL == 1 && (_curSpeed == MOTOR_UP_SPD || _curSpeed == 0)) {
        MOTOR_DIR_MSK &= ~MOTOR_DIR_UP;
        HL_BaseSpeed(0);
    }
    if (ULIMIT_VAL == 0)
        MOTOR_DIR_MSK |= MOTOR_DIR_UP;
}

ISR(INT6_vect)
{
    if (LLIMIT_VAL == 1 && (_curSpeed == MOTOR_DN_SPD || _curSpeed == 0)) {
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
        _curSpeed=spd;
        setBaseSpeed(spd);
    }
}

void HL_SetBasePosition(uint8_t position)
{
    if (abs(ADC_BASE_POT-position) <= MOTOR_POS_ERR)  
        HL_BaseSpeed(0);

    if (ADC_BASE_POT < position)
        HL_BaseSpeed(MOTOR_UP_SPD);
    if (ADC_BASE_POT > position)
        HL_BaseSpeed(MOTOR_DN_SPD);

    _targetPosition = position;
}

uint16_t HL_GetBaseSpeed()
{
    return _curSpeed;
}

uint8_t HL_GetTargetPos()
{
    return _targetPosition;
}

uint8_t HL_GetLimitState()
{
    uint8_t state = 0;
    if (ULIMIT_VAL == 1) 
        state |= 0x2;
    if (LLIMIT_VAL == 1) 
        state |= 0x1;
    return state;
}

void HL_OpenHand()
{
    _handTask = 1;
    _count = 0;
    setHandOpen();
}

void HL_CloseHand()
{
    _handTask = 1;
    _count = 0;
    setHandClose();
}

void HL_UpdateState()
{
    if (abs(ADC_BASE_POT-_targetPosition) <= MOTOR_POS_ERR)  
        HL_BaseSpeed(0);

    if (_handTask)
        _count++;

    if (_count > 320000) {
        _count = 0;
        _handTask = 0;
        setHandBrake();
    }
}

#endif

