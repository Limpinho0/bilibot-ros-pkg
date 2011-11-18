

void HandBrake(){
 PORTA &= 0x3f;
}
void BaseBrake(){
 PORTA &= 0xcf;
}

void Motorsoff(){
  PORTE &= 0xe7;
}

void initMotors(){
//    PA7 - Hand Control A
//    PA6 - Hand Control B
//    PA4 - Base Control A
//    PA5 - Base Control B
//    PE3 - Hand Speed 
//    PE4 - Base Speed
  DDRE |= 0x18;
  DDRA |= 0xf0;
  BaseBrake();
  HandBrake();
  Motorsoff();  

//    PF3 - Base Pot  
//    PF4 - Base Sense
//    PF5 - Hand Sense
//    PE6 - Base_L_switch
//    PE7 - Base_U_switch
DDRF &= 0xc7;
DDRE &= 0x3f;


}

void HandUp(){  //A = 1 B = 0
  HandBrake();
  PORTA |= 0x80;  
}

void HandDown(){  //A = 1 B = 0
  HandBrake();
  PORTA |= 0x40;  
}

void BaseUp(){  //A = 1 B = 0
  BaseBrake();
  PORTA |= 0x10;  
}

void BaseDown(){  //A = 1 B = 0
  BaseBrake();
  PORTA |= 0x20;  
}

void BaseEnable(){
 PORTE |= 0x10;
}
void BaseDisable(){
 PORTE &= 0xef;
}

void HandEnable(){
 PORTE |= 0x08;
}
void HandDisable(){
 PORTE &= 0xf7;
}



