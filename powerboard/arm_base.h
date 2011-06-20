
//handles all the arm base movement









#define _BASETOLERANCE 10
#define _BASE_ULIMIT 0x40
#define _BASEON 0x80
#define _BASEDIR 0x10
#define _BASERDIR 0x08


#define _BASE_STATUS_CMD 0x80
#define _BASE_STATUS_UL 0x40
#define _BASE_STATUS_LL 0x20
#define _BASE_STATUS_DIR 0x10
#define _BASE_STATUS_RDIR 0x08
#define _BASE_STATUS_DONE 0x04
#define _BASE_STATUS_CMD 0x02
#define _BASE_STATUS_CMD 0x01


#define _BASE_ERROR_ERR 0x80
#define _BASE_ERROR_UL 0x40
#define _BASE_ERROR_LL 0x20
#define _BASE_ERROR_DIR 0x10
#define _BASE_ERROR_SLIM 0x08
#define _BASE_ERROR_NMOVE 0x04

//threshold to detrermine if arm is moving
//TODO: determine if this is a good threshold!
#define _BASE_MOVE_THRESH 0x08

//TODO: write softlimits into eeprom
struct ArmBase{
  uint8_t status;  
//bit breakdown:
// Commanded to run 0/1
// Upper Limit
// Lower Limit
// Direction    1->up 0->down
// Read Direction    1->up 0->down
// At Destination    
//state: 00 -> stopped 
//state: 01 -> down 
//state: 10 -> up 
//state: 11 -> stopped in error
//

  uint8_t error;
  //in error
  //Upper limit
  //lower limit
  //over current
  //going wrong direction
  //cmd outside soft limits
  //not moving

  uint8_t  justfinished;
  uint16_t target;
  uint16_t position;// current position
  uint16_t last;
  uint16_t current;
  
  
  void checkErrors(uint8_t update){
       //now check for errors:
   //TODO: should we clear error every check?
   //TODO: should we break after finding one error?
   error=0;
   if((status & _BASE_STATUS_UL) && (status & _BASE_STATUS_DIR)) //going up and at limit
      error |= _BASE_ERROR_UL;
   if((status & _BASE_STATUS_LL) && (status & _BASE_STATUS_DIR)==0) //going down and at limit
      error |= _BASE_ERROR_LL;
   //TODO: find out what current is too much!
   //TODO: find out what soft limits are!
     
   //if wrong direction
   if(update && ( (status & _BASE_STATUS_RDIR > 0) == (status & _BASE_STATUS_DIR > 0) ) )
      error |= _BASE_ERROR_DIR;
      
   //if not moving
   if(update && abs(position-target) < _BASE_MOVE_THRESH) ) //TODO: check if uints work like this
      error |= _BASE_ERROR_NMOVE;
   
   if(error)
     error |= _BASE_ERROR_ERR;
    
    
  }
  
  void setMotorDone(){
      l_BASE_SPD;  //disable base motor
      l_BASE_CTRL_B; //set two outputs low
      l_BASE_CTRL_A;
      //TODO: should base speed be brought high now to brake?
      status &=0xfc; //update status to reflect output
  }
  
  void setMotorError(){
      l_BASE_SPD;  //disable base motor
      h_BASE_CTRL_B; //set two outputs low
      h_BASE_CTRL_A;
      //TODO: should base speed be brought high now to brake?
      status |=0x03; //update status to reflect output
  }

//TODO: check direction
  //sends actual commands to motor
  //gets direction from status bit
  void setMotorMove(){
    //don't run this call unless the cmd bit is set
    if((status & _BASE_STATUS_CMD)==0)
      return;  
      l_BASE_SPD;  //disable base motor
      
      if(status & _BASE_STATUS_DIR){
	h_BASE_CTRL_B; 
	l_BASE_CTRL_A;
	status &=0xfc; //update status to reflect output
	status |=0x02; //update status to reflect output
      }
      else{
	l_BASE_CTRL_B; 
	h_BASE_CTRL_A;
	status &=0xfc; //update status to reflect output
	status |=0x01; //update status to reflect output
      }
      
      h_BASE_SPD;  //enable base motor
  }

  
  //read all the necessary things from the sensors
  //does not set _BASE_STATUS_CMD, _BASE_STATUS_DIR
  //never turns base on, even if target is far from position.  Does turn the motor off if in error or done
  //if update,give error if not going in the right direction
  void checkStatus(uint8_t update=1){
   last=position;
   current=ADC_BASE_CURR;
   position=ADC_BASE_POT;
   if(READ_ULIMIT) status |= _BASE_STATUS_UL; else status &= (!_BASE_STATUS_UL);
   if(READ_LLIMIT) status |= _BASE_STATUS_LL; else status &= (!_BASE_STATUS_LL);
   //TODO: find out what current is too much!
   if(position > last) status |= _BASE_STATUS_RDIR; else status &= (!_BASE_STATUS_RDIR); //TODO: make sure increasing voltage means going up
     
     
   //check if at position:
   if(abs(position-target) < BASETOLERANCE) )
      status |= _BASE_STATUS_DONE;

   //now check for errors:
   checkErrors(update)  
     
   if(error)
      setMotorError();
   else if ((status & _BASE_STATUS_DONE) && (status & _BASE_STATUS_CMD)){
      setMotorDone();
   }
  
  }
  
  void setDirection(){
    //TODO: make sure increasing voltage means going up
    status &= (!_BASE_STATUS_DIR);
    if(target> position)
      status |= _BASE_STATUS_DIR;
  }

  //start arm movement to position
  //TODO: check what happens if get a command while moving
  void start(uint8_t _target){
    //clear old flags, in case...
    status &= (!_BASE_STATUS_CMD);
    justfinished=0;
    
    target=_target*4; //10 bit adc
    //TODO: make sure increasing voltage means going up

    setDirection(); //set the direction we intend to go
    
    checkStatus(0); //see if everything is clear to move
    
    if((status & _BASE_STATUS_DONE)==0) && error==0){
      status |= _BASE_STATUS_CMD;
      setMotorMove();
    }
    
  }
  
  //stop the arm, no matter where it was
  //TODO: check what happens if get a command while moving
  void stop(uint8_t _target){
    //clear old flags, in case...
    status &= (!_BASE_STATUS_CMD);
    justfinished=0;
    setMotorDone();    
  }
  
  
  //setup interrupt to check status
  void setup(){
    //TODO: fill this out!
    
    
  }
  
  
  
  
};



//Base state variables:
//Commanded to run 0/1
// Upper Limit
// Lower Limit
// Direction    1->up 0->down
// target position
// current position
// last position
