//***************************************************************************//
// File      : general.h
//
// Includes  : xc.h
//
//Jean-Francois Bilodeau    MPLab X v5.00   09/10/2018 
//****************************************************************************//

#ifndef __GENERAL_H_
#define	__GENERAL_H_

#define FOSC 132660000L 
#define FCY (FOSC/2)    // FCY of 66.33MHz

#include <xc.h>
#include <libpic30.h>
#include <dsp.h>
#include "math.h"
#include "string.h"

#define USER_IN1 PORTCbits.RC1
#define USER_IN2 PORTCbits.RC2

#define KSW1_PIN PORTBbits.RB7
#define KSW2_PIN PORTBbits.RB8

#define DEBUG_LED LATBbits.LATB9
#define LED_OFF 1
#define LED_ON 0

#define INTELLITROL_SPI2_CS PORTBbits.RB13

#define DRIVE_ADDRESS 0x22// Drive adress should always be a pair number for i2c
#define EEPROM_BASE_ADDRESS 0xA0
#define DRIVE_DATA_LENGTH 5

void PIC_init (void);

#endif	/* GENERAL_H */

