//This header does the parsing for the serial command protocol

uint8_t readstatus;
char cmd_in;
uint8_t data_in;
uint8_t cmd_verified;
#define READ0 0x11
#define READ1 0x12
#define READ2 0x13
#define READ3 0x14

//Commands:
//syntax: #<command char><data char>;


// 'M'   arm move to pos  (0-255)
// 'R'   arm cal 
// 'K'   turn kinect on/off   (1,0)
// 'C'   turn create on/off  (1,0)
// 'G'   charge create /disable charger  (1,0)





void parsechar(char c){
  if(!(readstatus & 0x10)){ //not currently reading a code
    if(c=="#")
      readstatus=READ0;
  }
  else{
	if(READ0){//read cmd byte
	  cmd_in=c;
	  readstatus=READ1;
	  return;
	}
	if(READ1){//read cmd byte
	  data_in=c;
	  readstatus=READ2;
	  return;
	}
	if(READ1){//read cmd byte
	  data_in=c;
	  readstatus=READ2;
	  return;
	}
    
    
    
  }
  
  
  
}