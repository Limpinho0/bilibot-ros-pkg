#ifndef MUSIC_H
#define MUSIC_H
#include "pinmapping.h"




//uint16_t _mglobalcount=0;

typedef struct Note{
    uint8_t pitch;
    uint16_t dur;
};

typedef struct NoteList{
    struct Note *list;
    uint8_t count;
};



void setupMusic(){
  //use PB7
     DDRB |= 0x80;
     
//use fast pwm mode, toggling OC0A     
     
  
//TCCR0A
//  01xxxxxx     //COM0A toggle pin 
//  xx00xxxx     //COM0B  not toggling pins
//  xxxx00xx     //N/C
//  xxxxxx11      //fast PWM top= OCR0A
TCCR0A=0x43;
  
 //TCCR0B:
//  00xxxxxx     //force compare trigger
//  xx00xxxx     // N/C
//  xxxx1xxx      //waveform generation: fast PWM top= OCR0A
//  xxxxx101      //prescaler = 1024

TCCR0B=0x0d;

//lets set osc freq to 1Khz:
OCR0A=4;
OCR0B=1; //15625 counts per second


//TIMSK0   - timer 0 interrupts
//  00000xxx     // N/C
//  xxxxx10x      //output compare B,A  off
//  xxxxxxx0      //timer overflow
TIMSK0 = 0x04;
  
}

void enableSound(){
  
   TCCR0A |= 0x40;
}

void disableSound(){
   TCCR0A &= 0xbf;
}

uint16_t _mcountsleft;
uint8_t _mnotesleft;
struct NoteList _mcurrentlist;



void setNote(struct Note n){
   _mcountsleft = n.dur;
   OCR0A=n.pitch;  
}

void setList(struct NoteList nl){
  _mcurrentlist = nl;
  _mnotesleft=nl.count-1;
  setNote(nl.list[0]);  
}


SIGNAL(TIMER0_COMPB_vect){
//  _mglobalcount++;
  if(_mcountsleft==0){
      if(_mnotesleft==0){
	disableSound();
	return;
      }
      uint8_t num=_mcurrentlist.count-_mnotesleft;
      setNote(_mcurrentlist.list[num]);
      _mnotesleft--;
  }
  if(!( TCCR0A & 0x40))
    enableSound();
  _mcountsleft--;
  
}

struct Note ss1[10];
void shortsong1(){
    ss1[0].pitch=15; ss1[0].dur=481;
    ss1[1].pitch=12; ss1[1].dur=481;
    ss1[2].pitch=11; ss1[2].dur=781;
    ss1[3].pitch=14; ss1[3].dur=481;
    ss1[4].pitch=16; ss1[4].dur=181;
    ss1[5].pitch=18; ss1[5].dur=781;
  struct NoteList nl;
  nl.list=ss1;
  nl.count=6;
  setList(nl);
}

void failSong(){
    ss1[0].pitch=20; ss1[0].dur=481;
    ss1[1].pitch=30; ss1[1].dur=481;
  struct NoteList nl;
  nl.list=ss1;
  nl.count=2;
  setList(nl);
}

#endif