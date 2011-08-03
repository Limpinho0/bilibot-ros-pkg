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

#include "VirtualSerial.h"
#include "pinmapping.h"

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


void setupMotors(){
   DDRC |= 0xfc;
   DDRF = 0;
  
}

uint16_t handspeed;
uint8_t handon;

SIGNAL(TIMER3_COMPA_vect){
  if(handon==0)
    return;
  l_HAND_SPD;
  
  
}

SIGNAL(TIMER3_COMPB_vect){
  if(handon==0)
    return;
  
  
  
}
SIGNAL(TIMER3_OVF_vect){
  if(handon==0)
   l_HAND_SPD;
  else{
    
    //set at the top:
    h_HAND_SPD;
  }  
  
}

void increaseHandSpeed(){
 if(handspeed<16000)
   handspeed+=100;
 OCR3A=handspeed;
}

void decreaseHandSpeed(){
 if(handspeed>100)
   handspeed-=100;
 OCR3A=handspeed;
}


  void setMotorMove(uint8_t c){
      
 /*     l_BASE_SPD;  //disable base motor
      l_HAND_SPD;  //disable hand motor
 */     
      if(c=='Y'){
	h_BASE_CTRL_B; 
	l_BASE_CTRL_A;
      }
      if(c=='H'){
	l_BASE_CTRL_B; 
	h_BASE_CTRL_A;
      }
      
      if(c=='T'){
	if(handon)
	  increaseHandSpeed();
	else{
	    handon=1;
	    handspeed=1000;
	    OCR3A=handspeed;
	}
	h_HAND_CTRL_B; 
	l_HAND_CTRL_A;
	return;
      }
      if(c=='G'){
	if(handon){
	  if(handspeed<16000)
	    handspeed+=100;
	    OCR3A=handspeed;
	}
	else{
	    handon=1;
	    handspeed=8000;
	    OCR3A=handspeed;
	}
	l_HAND_CTRL_B; 
	h_HAND_CTRL_A;
	return;
      }
      
      if(c=='g'){
	if(handon)
	  decreaseHandSpeed();
	else{
	    handon=1;
	    handspeed=8000;
	    OCR3A=handspeed;
	}
	l_HAND_CTRL_B; 
	h_HAND_CTRL_A;
	return;
      }
//       if(c == 'Y' || c=='H' || c=='T' || c=='G'){
// 	h_BASE_SPD;  //enable base motor
// 	h_HAND_SPD;  //enable hand motor
//       }
      
	handon=0;
	l_BASE_CTRL_A;
	l_BASE_CTRL_B;
	l_HAND_CTRL_A;
	l_HAND_CTRL_B;
      
  }


void setupHandOsc(){
//TCCR3A
//////////  10xxxxxx     //COM3A  OC3A clear on compare
//  00xxxxxx     //COM3A  going to manually toggle pin
//  xx0000xx     //COM1B/C  not toggling pins
//  xxxxxx10      //fast PWM top= ICR3
TCCR3A=0x82;
  
 //TCCR3B:
//  0xxxxxxx     //input capture noise canelation off
//  x1xxxxxx     //input capture edge select = rising
//  xx0xxxxx     // N/C
//  xxx11xxx      //waveform generation: fast pwm
//  xxxxx001      //prescaler = 1
//0x59
TCCR3B=0x59;

//lets set osc freq to 1Khz:
ICR3=16000;
//set both PWM to 50%:
OCR3A=8000;
OCR3B=8000;

//TIMSK3   - timer 3 interrupts
//  00xxxxxx     //N/C
//  xx0xxxxx     //input capture interrupt  
//  xxx0xxxx     // N/C
//  xxxx011x      //output compare B,A on, C off
//  xxxxxxx1      //timer overflow
TIMSK3 = 0x07;
  
}

void parseCommand(uint8_t c){
  if(c=='C'){
      Toggle_CREATE_PWR_EN;
  }
    if(c=='K'){
      Toggle_KIN_EN;
  }
  if(c == 'Y' || c=='H' || c=='T' || c=='G'|| c=='g' || c=='s' || c=='S')
    setMotorMove(c);
}


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
		if(counter>1000){
		  LEDs_ToggleLEDs(LEDS_LED2);
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
  setupMotors();
  setupHandOsc();
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

