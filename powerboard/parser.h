//This header does the parsing for the serial command protocol

#include "adc.h"
#include "arm_base.h"


ArmBase arm_base;


uint8_t readstatus;
char cmd_in;
uint8_t data_in;
uint8_t cmd_verified;
#define READ0 0x11
#define READ1 0x12
#define READ2 0x13
#define READ3 0x14
#define READERROR 0x20


//Commands:
//syntax: #<command char><data char>;


// 'B'   base move to pos  (0-255)
// 'G'   Gripper move  (0 - open, 1 close)
// 'S'   Stop arm move  (1 - base, 2-gripper, 3-both)
// 'R'   arm cal 
// 'K'   turn kinect on/off   (1,0)
// 'C'   turn create on/off  (1,0)
// 'c'   charge create /disable charger  (1,0)





#define _GRIPPERON 0x02
#define _GRIPPERDIR 0x10
#define _GRIPPERPOS 0x20
//for gripper dir and pos, 0 is open, 1 is closed



// check limit switches, and base pot
//if not at goal, and moving,and supposed to move
void checkBase(){
  
  
  
}


//for gripper dir and pos, 0 is open, 1 is closed
void startGripper(uint8_t target){
  if(_GRIPPERPOS == _GRIPPERPOS*target) // if we are already at that position 
    return;
  if(target)
    _armstate |= _GRIPPERDIR;
  else
    _armstate &= (!_GRIPPERDIR);
  
  
}


void parseCmd(){
  switch(cmd_in){
    case 'B': 
      arm_base.start(data_in);
      break;
    case 'S': 
      if(data_in & 0x01)
	arm_base.stop();
      break;
     
    
	
    
    
    
    
    
  }
  
  
}


void parsechar(const char &c){
  if(!(readstatus & 0x10)){ //not currently reading a code
    if(c=="#")
      readstatus=READ0;
  }
  else{
	if(readstatus==READ0){//read cmd byte
	  cmd_in=c;
	  readstatus=READ1;
	  return;
	}
	if(readstatus==READ1){//read data byte
	  data_in=c;
	  readstatus=READ2;
	  return;
	}
	if(readstatus==READ2){//read last byte
	  if(c==';')
	    readstatus=READ3;
	  else
	    readstatus=READERROR;
	  return;
	}
  }
}


void handleInput(const char &c){
  parsechar(c);
  if(readstatus==READ3){ //done reading command
     parseCmd();
    
  }
  
  
}



