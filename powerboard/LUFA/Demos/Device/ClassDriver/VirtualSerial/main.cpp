//main.cpp 
//pulled out the main function from the rest


#include "simplehighmotor.h"
#include "VirtualSerial.h"
#include "pinmapping.h"
#include "adc.h"


static FILE USBSerialStream;

void parseCommand(uint8_t c){
  if(c=='C'){
      Toggle_CREATE_PWR_EN;
  }
    if(c=='K'){
      Toggle_KIN_EN;
  }
#ifdef SIMPLEMOTOR
  
  if(c == 'Y' || c=='H' || c=='T' || c=='G'|| c=='g' || c=='s' || c=='S')
    setMotorMove(c);
#endif  
  
}
#define stransmitf(args...)  {                   \
		  char tstring[50];					\
		  sprintf(tstring,args);			\
		  transmitstring(tstring,strlen(tstring));	\
                    } 

void sendByte(char c){
	CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (uint8_t)c);
}


void transmitstring(char *tx, int len){
	int i;
	for(i=0;i<len;i++)
		sendByte(tx[i]);
}


#ifdef USING_ADC

void printADC(){
  uint16_t pot = ADC_BASE_POT;
  stransmitf("%3u %3u %3u %3u\r\n", pot, ADC_GYRO, ADC_BASE_CURR, ADC_HAND_CURR);
  
}

#endif


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	uint16_t counter=0;
	/* Create a regular character stream for the interface so that it can be used with the stdio.h functions */
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	sei();

//		  LEDs_ToggleLEDs(LEDS_LED2);
		CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
		_delay_ms(1);
		  LEDs_ToggleLEDs(LEDS_LED2);
	for (;;)
	{
//		CheckJoystickMovement();

		/* Must throw away unused bytes from the host, or it will lock up while waiting for the device */
//		CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		/* Echo all received data on the  CDC interface */
		int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		if (!(ReceivedByte < 0)){
		  CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (uint8_t)ReceivedByte);
		  parseCommand((uint8_t)ReceivedByte);
		}
		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
		_delay_ms(1);
		counter++;
		if(counter>10){
		  LEDs_ToggleLEDs(LEDS_LED2);
		  #ifdef USING_ADC
		  printADC();
		  #endif
// 		  stransmitf("hello world");
		  
		  counter=0;
		}
	}
}

/** ISR to periodically toggle the LEDs on the board to indicate that the bootloader is active. */
ISR(TIMER1_OVF_vect, ISR_BLOCK)
{
// 	LEDs_ToggleLEDs(LEDS_LED1);
}
