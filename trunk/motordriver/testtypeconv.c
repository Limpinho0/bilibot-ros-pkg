

#include <stdio.h>
#include <sys/types.h>


int16_t convToInt(unsigned char c1, unsigned char c2){
  unsigned int a = c2+(c1<<8);
  int16_t *b = (int16_t*) &a;
  return a;
}

void convToBytes(int a, unsigned char *c1, unsigned char *c2){
  *c1=(a>>8);
  *c2=a&0xff;
}


int main(){
  unsigned char c1, c2;
  int16_t a,b;
  
  a=-5;
  convToBytes(a,&c1,&c2);
  printf("%i ->  0x%02x  0x%02x\n",a, c1, c2);
  
  b = c2+(c1<<8);
//   b=convToInt(c1,c2);
  printf("0x%02x  0x%02x -> %i\n", c1, c2, b);
  
 unsigned char c,d,e;
 
 c=250;
 d=4;
 e=d-c;
  printf(" %u - %u = %u\n", d, c, e);
 

 

return 0;
}