#include "lowlevelmotors.h"


LowLevelMotors _motors;


//We clear at the compare. this way, the enable stays on as long
// as the OCR3A value
SIGNAL(TIMER3_COMPA_vect){
  if(_motors.handon==0)
    return;
  l_HAND_SPD;
}

//We clear at the compare. this way, the enable stays on as long
// as the OCR3B value
SIGNAL(TIMER3_COMPB_vect){
  if(_motors.baseon==0)
    return;
  l_BASE_SPD
}

//at the overflow, if the hand or base should be on, we turn them on
//otherwise we leave them off
SIGNAL(TIMER3_OVF_vect){
  if(_motors.handon==0)
   l_HAND_SPD;
  else{
    //set at the top:
    h_HAND_SPD;
  }  
  if(_motors.baseon==0)
   l_BASE_SPD;
  else{
    //set at the top:
    h_BASE_SPD;
  }  
  
}