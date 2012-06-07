
//buffers allow us to log lots of data fast, and then spit it out onto the usb stream when the message comes around.

#ifndef MD_BUFFER_H
#define MD_BUFFER_H

typedef struct {
  uint8_t data[256];
  uint8_t tbuffer[52];  //we actually only keep track of 50, and the last two tell us how we are falling behind
  uint8_t wpos,rpos;
  uint8_t wbuffcount,rbuffcount;
} CircBuffer;


void cb_init(CircBuffer *cb){
    cb->wbuffcount=0;
    cb->rbuffcount=0;
    cb->wpos=0;
    cb->rpos=0;
}

void cb_write(CircBuffer *cb, uint8_t d){
  	uint8_t nextpos=(cb->wpos+1); //will wrap around if >255
	if(nextpos<cb->wpos) wbuffcount++;
	cb->data[nextpos]=d;
	cb->wpos=nextpos;
}

  //find distance from rpos to wpos:
uint8_t cb_getBufferSize(CircBuffer *cb){
  //if buffer has not been lapped
  uint8_t bsize;
  if((cb->wbuffcount-cb->rbuffcount == 0) || ((cb->wbuffcount-cb->rbuffcount == 1) && (cb->wpos < cb->rpos))){
    //then simple find rpos-wpos distance:
    bsize=cb->wpos-cb->rpos;
    return bsize;
  }
  //otherwise, there is lots of data!
  return 255;
}

//copies a chunk of data to buffer, if there is enough data.
//this also updates the read position
uint8_t cb_copyToBuffer(CircBuffer *cb){
  //find distance from rpos to wpos:
  uint8_t bsize  = getBufferSize(cb);
  //TODO: deal with the situation where the wpos-rpos < 50, but the buffer has been lapped
     // this would mean we are sending disconinous data...
  if(bsize>50){ //we'll always transmit a constant size
    for(int i=0;i<50;i++)
      cb->tbuffer[i]=cb->data[cb->rpos+i];    
    cb->rpos=cb->rpos+50
    if(cb->rpos<50) cb->rbuffcount++;
  }
  
  //fill out the end part of the 
    tbuffer[50] = bsize;
    tbuffer[51] = wbuffcount-rbuffcount;
  
  return bsize;
}


uint8_t curr1r[250];
uint8_t tbuffer[52];
// uint8_t curr2r[50];
uint8_t wpos,rpos;
uint8_t wbuffcount,rbuffcount;

//copies current data to buffer
uint8_t copyToBuffer(){
  //find distance from rpos to wpos:
  uint16_t bsize;
  if(wpos > rpos)
    bsize=wpos-rpos;
  else
    bsize=(250+wpos)-rpos;
  //TODO: this would be really simple if the buffer size was the max value of uint8_t
  if(bsize>50){ //we'll always transmit a constant size
    for(int i=0;i<50;i++)
      tbuffer[i]=curr1r[(rpos+i)%250];    
    rpos=(rpos+50)%250;
    if(rpos<50) rbuffcount++;
  }
  return bsize;
}



#endif