
#include "powerboard.h"
#include "wireformat.h"
#include "pinmapping.h"
#include "adc.h"
#include "lowlevelmotor.h"
#include "simplehighmotor.h"
#include "music.h"
#include <stdlib.h>

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
uint8_t seq;
uint8_t* txBuffer;
uint16_t badPkts;
packet_t rxPkt;
status_t rxStat;

void sendByte(char c){
	CDC_Device_SendByte(&VirtualSerial_CDC_Interface, (uint8_t)c);
}

void handleUSB(void)
{
    CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
    USB_USBTask();
}

void transmitArmState(){
    uint8_t payload[6];

    // fill in payload
    payload[0] = ADC_BASE_POT;
    payload[1] = HL_GetTargetPos();
    payload[2] = HL_GetBaseSpeed();
    payload[3] = HL_GetLimitState();
    payload[4] = ADC_BASE_CURR;
    payload[5] = ADC_HAND_CURR;

    CDC_Device_Flush(&VirtualSerial_CDC_Interface);
    packet_t* pkt = PKT_Create(PKTYPE_STATUS_ARM_STATE, seq++, payload, 6);
    uint8_t len = PKT_ToBuffer(pkt, txBuffer); 
    for(int i=0; i < len; i++) {
        sendByte(txBuffer[i]);
        handleUSB();
    }
    free(pkt);
}

void transmitGyroState(){
    uint8_t payload[1];

    // fill in payload
    payload[0] = ADC_GYRO;

    CDC_Device_Flush(&VirtualSerial_CDC_Interface);
    packet_t* pkt = PKT_Create(PKTYPE_STATUS_GYRO_RAW, seq++, payload, 1);
    uint8_t len = PKT_ToBuffer(pkt, txBuffer); 
    for(int i=0; i < len; i++) {
        sendByte(txBuffer[i]);
        handleUSB();
    }
    free(pkt);
}


void setupGeneralState(){
  SETUP_DEMO_BTN; h_DEMO_BTN;
  SETUP_ESTOP_BTN;  h_ESTOP_BTN;
  SETUP_CREATE_ON;  h_CREATE_ON;
  SETUP_CREATE_CHRG_IND; h_CREATE_CHRG_IND;
  SETUP_PWR_IND; h_PWR_IND;
  SETUP_OVR_CHRG; h_OVR_CHRG;
}


uint8_t getGeneralState(){
      uint8_t bstate=0x00;
    //some battery data
    if((READ_PWR_IND) == 0)
      bstate|=0x01;
    if((READ_OVR_CHRG) == 0)
      bstate |=0x02;
    //kinect enabled?
    if(READ_KIN_EN)
      bstate |= 0x04;
    
    //create stuff:
    if(READ_CREATE_PWR_EN)  //create charging enabled?
      bstate |= 0x08;
    if(READ_CREATE_ON)  //create on
      bstate |= 0x10;
    if(READ_CREATE_CHRG_IND)  //create charging?
      bstate |= 0x20;
    
    //io buttons:
    if((READ_DEMO_BTN) == 0)  //Demo button pressed?
      bstate |= 0x40;
    if((READ_ESTOP_BTN) == 0)  //estop button pressed?
      bstate |= 0x80;
    
    return bstate;
  
}

void transmitBattState(){
    uint8_t payload[4];

    // fill in payload
    payload[0] = ADC_BATT;
    payload[1] = ADC_AD0;
    payload[2] = ADC_AD1;
    payload[3] = getGeneralState();
    
      

    CDC_Device_Flush(&VirtualSerial_CDC_Interface);
    packet_t* pkt = PKT_Create(PKTYPE_STATUS_BATT_RAW, seq++, payload, 4);
    uint8_t len = PKT_ToBuffer(pkt, txBuffer); 
    for(int i=0; i < len; i++) {
        sendByte(txBuffer[i]);
        handleUSB();
    }
    free(pkt);
}


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

    uint16_t counter=0;
    seq = 0;
    txBuffer = (uint8_t*)malloc(sizeof(packet_t));
    badPkts = 0;

	/* Create a regular character stream for the interface so that it can be used with the stdio.h functions */
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);

    sei();

    //CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
    CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
    USB_USBTask();
    LEDs_ToggleLEDs(LEDS_LED2);

    uint8_t handOpen = 1;

    for (;;)
    {
        // returns negative on failure, byte value on success
        int16_t rxByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

		if (!(rxByte < 0)){

            // oddly ReceiveByte returns a 16-bit int
            uint8_t byte = (uint8_t)rxByte;

            // incrementally builds packets byte-by-byte 
            if (PKT_Decoded(byte, &rxPkt, &rxStat) != DECODE_STATUS_INCOMPLETE) {
                switch (rxStat.state)
                {
                case DECODE_STATUS_COMPLETE:
                    switch (rxPkt.type) 
                    {
                    case PKTYPE_CMD_SET_ARM_POS:
                        HL_SetBasePosition(rxPkt.payload[0]);
                        break;
                    case PKTYPE_CMD_SET_HAND_POS:
                        break;
                    case PKTYPE_CMD_SET_PWR_STATE:
                        break;
                    case PKTYPE_CMD_ZERO_GYRO:
                        break;
                    case PKTYPE_CMD_TOGGLE_KINECT:
                        Toggle_KIN_EN;
                        break;
                    case PKTYPE_CMD_TOGGLE_CREATE:
                        Toggle_CREATE_ON;
                        if(READ_CREATE_ON) {  // is the create powered?
                            l_CREATE_CHRG_IND; // disable charging
                            l_CREATE_PWR_EN;
                        } else {
                            h_CREATE_PWR_EN;
                            h_CREATE_CHRG_IND;
                        }
                        break;
                    case PKTYPE_CMD_TOGGLE_HAND_STATE:
                        if (!handOpen) {
                            HL_OpenHand();
                            handOpen = 1;
                        } else {
                            HL_CloseHand();
                            handOpen = 0;
                        }
                        break;
                    }
                    break;
                case DECODE_STATUS_INVALID:
                    badPkts++; 
                    break;
                }
                rxStat.recvd = 0;
            }
		}

        // FIXME: this const counter check should be replace by timer delta function
		if (counter > 10000){
            LEDs_ToggleLEDs(LEDS_LED2);
            transmitArmState();
            transmitGyroState();
            transmitBattState();
            counter = 0;
		}

        handleUSB();
        HL_UpdateState();
		counter++;
	}
}

ISR(TIMER1_OVF_vect, ISR_BLOCK) {}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
    SETUP_CREATE_PWR_EN;
    h_CREATE_PWR_EN;
    SETUP_KIN_EN;
    h_KIN_EN;
    SETUP_ULIMIT;
    SETUP_LLIMIT;
    EN_ULIMIT_ISR;
    EN_LLIMIT_ISR;

    setupGeneralState();

    setupMotors();
    setupADC();

    //start next adc read:
    ADCSRA |= 0x40;
   
    setupMusic();   
   
	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	Joystick_Init();
	LEDs_Init();
	USB_Init();
}

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

