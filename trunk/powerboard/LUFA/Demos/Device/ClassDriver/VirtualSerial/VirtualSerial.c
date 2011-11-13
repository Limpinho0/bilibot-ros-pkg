/*
             LUFA Library
     Copyright (C) Dean Camera, 2011.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2011  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the VirtualSerial demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */
// extern "C" {
#include "VirtualSerial.h"
#include "wireformat.h"
#include "pinmapping.h"
#include "adc.h"
#include "simplehighmotor.h"
#include "music.h"


/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = 0,

				.DataINEndpointNumber           = CDC_TX_EPNUM,
				.DataINEndpointSize             = CDC_TXRX_EPSIZE,
				.DataINEndpointDoubleBank       = false,

				.DataOUTEndpointNumber          = CDC_RX_EPNUM,
				.DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
				.DataOUTEndpointDoubleBank      = false,

				.NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
				.NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
				.NotificationEndpointDoubleBank = false,
			},
	};

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs
 */
static FILE USBSerialStream;


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












void runTest1(){
  struct Note n1;  n1.pitch=12; n1.dur=500;
  setNote(n1);
  uint8_t result = testMotors();
  if(result)
    failSong();
  stransmitf("motor test curr = %u\r\n",result);
  
}


void parseCommand(uint8_t c){
  if(c=='C'){
      Toggle_CREATE_PWR_EN;
  }
    if(c=='K'){
      Toggle_KIN_EN;
  }
  if(c=='M'){
      shortsong1();
  }
  if(c=='t'){
      runTest1();
  }
  if(c=='f'){
      failSong();
  }
  
#ifdef SIMPLEMOTOR
  
  if(c == 'Y' || c=='H' || c=='T' || c=='y'|| c=='h'|| c=='G'|| c=='g' || c=='s' || c=='S' || c=='q' || c=='a')
    setMotorMove(c);
#endif  
  
}






#ifdef USING_ADC

void printADC(){
  uint16_t pot = ADC_BASE_POT;
  // if we just want a bit from the limits, the READ_LLIMIT should probably include the shift?
  stransmitf("%3u %3u %3u %3u %7i %3u %3u\r\n", pot, ADC_GYRO, ADC_BASE_CURR, ADC_HAND_CURR,lastspeed, 
							((0xff&READ_ULIMIT)>>7)^1, ((0xff&READ_LLIMIT)>>6)^1);
  
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
		   sendtoMid(); 
		if(counter>100){
		  LEDs_ToggleLEDs(LEDS_LED2);
// 		  OCR0A++;
		  #ifdef USING_ADC
		  printADC();
		  #endif
// 		  stransmitf("hello world");
// 		  
		  simpleMotorCheck();
		  counter=0;
		}
	}
}

/** ISR to periodically toggle the LEDs on the board to indicate that the bootloader is active. */
ISR(TIMER1_OVF_vect, ISR_BLOCK)
{
// 	LEDs_ToggleLEDs(LEDS_LED1);
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
SETUP_CREATE_PWR_EN;
SETUP_KIN_EN;
h_KIN_EN;
SETUP_ULIMIT;
Toggle_ULIMIT;
SETUP_LLIMIT;
Toggle_LLIMIT;


#ifdef SIMPLEMOTOR
  setupMotors();
//   setupHandOsc();
#endif
#ifdef USING_ADC
setupADC();
  //start next adc read:
   ADCSRA |= 0x40;
#endif
   
setupMusic();   
   
	/* Disable watchdog if enabled by bootloader/fuses */
//	MCUSR &= ~(1 << WDRF);
//	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	Joystick_Init();
	LEDs_Init();
	USB_Init();
}

/** Checks for changes in the position of the board joystick, sending strings to the host upon each change. */
// void CheckJoystickMovement(void)
// {
// 	uint8_t     JoyStatus_LCL = Joystick_GetStatus();
// 	char*       ReportString  = NULL;
// 	static bool ActionSent    = false;
// 
// 	if (JoyStatus_LCL & JOY_UP)
// 	  ReportString = "Joystick Up\r\n";
// 	else if (JoyStatus_LCL & JOY_DOWN)
// 	  ReportString = "Joystick Down\r\n";
// 	else if (JoyStatus_LCL & JOY_LEFT)
// 	  ReportString = "Joystick Left\r\n";
// 	else if (JoyStatus_LCL & JOY_RIGHT)
// 	  ReportString = "Joystick Right\r\n";
// 	else if (JoyStatus_LCL & JOY_PRESS)
// 	  ReportString = "Joystick Pressed\r\n";
// 	else
// 	  ActionSent = false;
// 
// 	if ((ReportString != NULL) && (ActionSent == false))
// 	{
// 		ActionSent = true;
// 
// 		/* Write the string to the virtual COM port via the created character stream */
// 		fputs(ReportString, &USBSerialStream);
// 
// 		/* Alternatively, without the stream: */
// 		// CDC_Device_SendString(&VirtualSerial_CDC_Interface, ReportString);
// 	}
// }

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

