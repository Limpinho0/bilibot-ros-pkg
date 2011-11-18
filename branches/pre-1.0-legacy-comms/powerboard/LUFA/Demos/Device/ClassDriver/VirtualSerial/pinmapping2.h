#ifndef PINMAPPING_H
#define PINMAPPING_H

//Atmel pin 1, labeled PE6, is connected to the base lower limit.  The suffix for this pin is: LLIMIT.  It has the following defines: 	
#define SETUP_LLIMIT  DDRE &= 0xBF	
#define  h_LLIMIT PORTE |= 0x40	
#define  l_LLIMIT PORTE &= 0xBF	
#define READ_LLIMIT  PINE & 0x40
//Atmel pin 2, labeled PE7, is connected to the base upper limit.  The suffix for this pin is: ULIMIT.  It has the following defines: 	
#define SETUP_ULIMIT  DDRE &= 0x7F	
#define  h_ULIMIT PORTE |= 0x80	
#define  l_ULIMIT PORTE &= 0x7F	
#define READ_ULIMIT  PINE & 0x80
				
				
				
				
				
				
//Atmel pin 9, labeled PE3, is connected to the FTDI override.  The suffix for this pin is: FTDI_OVERRIDE.  It has the following defines: 	
#define SETUP_FTDI_OVERRIDE  DDRE |= 0x08	
#define  h_FTDI_OVERRIDE PORTE |= 0x08	
#define  l_FTDI_OVERRIDE PORTE &= 0xF7	
#define READ_FTDI_OVERRIDE  PORTE & 0x08
				
				
				
				
				
//Atmel pin 15, labeled PB5, is connected to the throb LED 1.  The suffix for this pin is: THROB1.  It has the following defines: 	
#define SETUP_THROB1  DDRB |= 0x20	
#define  h_THROB1 PORTB |= 0x20	
#define  l_THROB1 PORTB &= 0xDF	
#define READ_THROB1  PORTB & 0x20
//Atmel pin 16, labeled PB6, is connected to the Throb LED 2.  The suffix for this pin is: THROB2.  It has the following defines: 	
#define SETUP_THROB2  DDRB |= 0x40	
#define  h_THROB2 PORTB |= 0x40	
#define  l_THROB2 PORTB &= 0xBF	
#define READ_THROB2  PORTB & 0x40
//Atmel pin 17, labeled PB7, is connected to the General IO 1.  The suffix for this pin is: GIO_1.  It has the following defines: 	
#define SETUP_GIO_1  DDRB |= 0x80	
#define  h_GIO_1 PORTB |= 0x80	
#define  l_GIO_1 PORTB &= 0x7F	
#define READ_GIO_1  PORTB & 0x80
//Atmel pin 18, labeled PE4, is connected to the on button.  The suffix for this pin is: ON_BTN.  It has the following defines: 	
#define SETUP_ON_BTN  DDRE |= 0x10	
#define  h_ON_BTN PORTE |= 0x10	
#define  l_ON_BTN PORTE &= 0xEF	
#define READ_ON_BTN  PORTE & 0x10
//Atmel pin 19, labeled PE5, is connected to the General IO 2.  The suffix for this pin is: GIO_2.  It has the following defines: 	
#define SETUP_GIO_2  DDRE |= 0x20	
#define  h_GIO_2 PORTE |= 0x20	
#define  l_GIO_2 PORTE &= 0xDF	
#define READ_GIO_2  PORTE & 0x20
				
				
				
				
				
				
				
				
				
//Atmel pin 29, labeled PD4, is connected to the create on switch.  The suffix for this pin is: CREATE_ON.  It has the following defines: 	
#define SETUP_CREATE_ON  DDRD |= 0x10	
#define  h_CREATE_ON PORTD |= 0x10	
#define  l_CREATE_ON PORTD &= 0xEF	
#define READ_CREATE_ON  PORTD & 0x10
//Atmel pin 30, labeled PD5, is connected to the create charging ind.  The suffix for this pin is: CREATE_CHRG_IND.  It has the following defines: 	
#define SETUP_CREATE_CHRG_IND  DDRD &= 0xDF	
#define  h_CREATE_CHRG_IND PORTD |= 0x20	
#define  l_CREATE_CHRG_IND PORTD &= 0xDF	
#define READ_CREATE_CHRG_IND  PIND & 0x20
				
				
//Atmel pin 33, labeled PE0, is connected to the Kinect enable.  The suffix for this pin is: KIN_EN.  It has the following defines: 	
#define SETUP_KIN_EN  DDRE |= 0x01	
#define  h_KIN_EN PORTE |= 0x01	
#define  l_KIN_EN PORTE &= 0xFE	
#define READ_KIN_EN  PORTE & 0x01
				
//Atmel pin 35, labeled PC0, is connected to the hand ctrl a.  The suffix for this pin is: HAND_CTRL_A.  It has the following defines: 	
#define SETUP_HAND_CTRL_A  DDRC |= 0x01	
#define  h_HAND_CTRL_A PORTC |= 0x01	
#define  l_HAND_CTRL_A PORTC &= 0xFE	
#define READ_HAND_CTRL_A  PORTC & 0x01
//Atmel pin 36, labeled PC1, is connected to the hand ctrl b.  The suffix for this pin is: HAND_CTRL_B.  It has the following defines: 	
#define SETUP_HAND_CTRL_B  DDRC |= 0x02	
#define  h_HAND_CTRL_B PORTC |= 0x02	
#define  l_HAND_CTRL_B PORTC &= 0xFD	
#define READ_HAND_CTRL_B  PORTC & 0x02
//Atmel pin 37, labeled PC2, is connected to the Base ctrl a.  The suffix for this pin is: BASE_CTRL_A.  It has the following defines: 	
#define SETUP_BASE_CTRL_A  DDRC |= 0x04	
#define  h_BASE_CTRL_A PORTC |= 0x04	
#define  l_BASE_CTRL_A PORTC &= 0xFB	
#define READ_BASE_CTRL_A  PORTC & 0x04
//Atmel pin 38, labeled PC3, is connected to the Base ctrl B.  The suffix for this pin is: BASE_CTRL_B.  It has the following defines: 	
#define SETUP_BASE_CTRL_B  DDRC |= 0x08	
#define  h_BASE_CTRL_B PORTC |= 0x08	
#define  l_BASE_CTRL_B PORTC &= 0xF7	
#define READ_BASE_CTRL_B  PORTC & 0x08
				
				
//Atmel pin 41, labeled PC6, is connected to the hand speed.  The suffix for this pin is: HAND_SPD.  It has the following defines: 	
#define SETUP_HAND_SPD  DDRC |= 0x40	
#define  h_HAND_SPD PORTC |= 0x40	
#define  l_HAND_SPD PORTC &= 0xBF	
#define READ_HAND_SPD  PORTC & 0x40
//Atmel pin 42, labeled PC7, is connected to the base speed.  The suffix for this pin is: BASE_SPD.  It has the following defines: 	
#define SETUP_BASE_SPD  DDRC |= 0x80	
#define  h_BASE_SPD PORTC |= 0x80	
#define  l_BASE_SPD PORTC &= 0x7F	
#define READ_BASE_SPD  PORTC & 0x80
				
				
				
				
				
//Atmel pin 48, labeled PA3, is connected to the over charge.  The suffix for this pin is: OVR_CHRG.  It has the following defines: 	
#define SETUP_OVR_CHRG  DDRA &= 0xF7	
#define  h_OVR_CHRG PORTA |= 0x08	
#define  l_OVR_CHRG PORTA &= 0xF7	
#define READ_OVR_CHRG  PINA & 0x08
//Atmel pin 49, labeled PA2, is connected to the Create charge enable.  The suffix for this pin is: CREATE_PWR_EN.  It has the following defines: 	
#define SETUP_CREATE_PWR_EN  DDRA |= 0x04	
#define  h_CREATE_PWR_EN PORTA |= 0x04	
#define  l_CREATE_PWR_EN PORTA &= 0xFB	
#define READ_CREATE_PWR_EN  PORTA & 0x04
//Atmel pin 50, labeled PA1, is connected to the Power indicator.  The suffix for this pin is: PWR_IND.  It has the following defines: 	
#define SETUP_PWR_IND  DDRA |= 0x02	
#define  h_PWR_IND PORTA |= 0x02	
#define  l_PWR_IND PORTA &= 0xFD	
#define READ_PWR_IND  PORTA & 0x02
				
				
				
//Atmel pin 54, labeled PF7, is connected to the Base pot.  The suffix for this pin is: BASE_POT.  It has the following defines: 	
#define SETUP_BASE_POT  DDRF &= 0x7F	
#define  h_BASE_POT PORTF |= 0x80	
#define  l_BASE_POT PORTF &= 0x7F	
#define READ_BASE_POT  PINF & 0x80
//Atmel pin 55, labeled PF6, is connected to the gyro temp.  The suffix for this pin is: GYRO_TEMP.  It has the following defines: 	
#define SETUP_GYRO_TEMP  DDRF &= 0xBF	
#define  h_GYRO_TEMP PORTF |= 0x40	
#define  l_GYRO_TEMP PORTF &= 0xBF	
#define READ_GYRO_TEMP  PINF & 0x40
//Atmel pin 56, labeled PF5, is connected to the hand current.  The suffix for this pin is: HAND_CURR.  It has the following defines: 	
#define SETUP_HAND_CURR  DDRF &= 0xDF	
#define  h_HAND_CURR PORTF |= 0x20	
#define  l_HAND_CURR PORTF &= 0xDF	
#define READ_HAND_CURR  PINF & 0x20
//Atmel pin 57, labeled PF4, is connected to the base current.  The suffix for this pin is: BASE_CURR.  It has the following defines: 	
#define SETUP_BASE_CURR  DDRF &= 0xEF	
#define  h_BASE_CURR PORTF |= 0x10	
#define  l_BASE_CURR PORTF &= 0xEF	
#define READ_BASE_CURR  PINF & 0x10
//Atmel pin 58, labeled PF3, is connected to the gyro rate.  The suffix for this pin is: GYRO_RATE.  It has the following defines: 	
#define SETUP_GYRO_RATE  DDRF &= 0xF7	
#define  h_GYRO_RATE PORTF |= 0x08	
#define  l_GYRO_RATE PORTF &= 0xF7	
#define READ_GYRO_RATE  PINF & 0x08
//Atmel pin 59, labeled PF2, is connected to the Battery sense.  The suffix for this pin is: BATT_SENSE.  It has the following defines: 	
#define SETUP_BATT_SENSE  DDRF &= 0xFB	
#define  h_BATT_SENSE PORTF |= 0x04	
#define  l_BATT_SENSE PORTF &= 0xFB	
#define READ_BATT_SENSE  PINF & 0x04
//Atmel pin 60, labeled PF1, is connected to the Charge Sense B.  The suffix for this pin is: CHRG_S_B.  It has the following defines: 	
#define SETUP_CHRG_S_B  DDRF &= 0xFD	
#define  h_CHRG_S_B PORTF |= 0x02	
#define  l_CHRG_S_B PORTF &= 0xFD	
#define READ_CHRG_S_B  PINF & 0x02
//Atmel pin 61, labeled PF0, is connected to the Charge Sense A.  The suffix for this pin is: CHRG_S_A.  It has the following defines: 	
#define SETUP_CHRG_S_A  DDRF &= 0xFE	
#define  h_CHRG_S_A PORTF |= 0x01	
#define  l_CHRG_S_A PORTF &= 0xFE	
#define READ_CHRG_S_A  PINF & 0x01


#endif
