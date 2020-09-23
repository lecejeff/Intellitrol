//***************************************************************************//
// File      : DRV8873S.h
//
// Functions :  void DRV8873_init (void);
//              void DRV8873_write (unsigned char reg, unsigned char value);
//              unsigned char DRV8873_read (unsigned char reg);
//              unsigned char DRV8873_get_fault_status (unsigned char fault);
//              void DRV8873_assert_fault (void);
//              void DRV8873_enable(void);
//              void DRV8873_disable(void);
//              void DRV8873_sleep (void);
//              void DRV8873_wake (void);
//              unsigned char DRV8873_get_diag_status (unsigned char diag);
//              unsigned char DRV8873_fault_state (void);
//
// Includes  :  general.h
//              spi.h
//           
// Purpose   :  Driver for the DRV8873S SPI Motor controller
//              Driver interfaced on SPI port and I/Os, monitored by dsPIC
//              #FAULT : Fault pin, asserted low during fault condition
//              DSBL   : Logic high to disable the DRV8873S
//              #SLEEP : Logic low to put DRV8873 in sleep mode
//              IN1-2  : PWM'd pins to control the motor speed
//              ISEN1-2: Current sense output supervised by dsPIC ADC              
//
//Jean-Francois Bilodeau    MPLab X v5.10    10/02/2020  
//****************************************************************************//
#ifndef __DRV8873S_H__
#define __DRV8873S_H__

#include "general.h"
#include "spi.h"

// Define DRV8873 SPI port
#define DRV8873_SPI SPI_1

// Define DRV8873 pins used in the code (Read / Write pins)
#define DRV8873_FAULT_PIN_STATE PORTAbits.RA0
#define DRV8873_DISABLE_PIN LATBbits.LATB14
#define DRV8873_SLEEP_PIN LATBbits.LATB2

// Define default DRV8873 values for IC1..2..3..4 registers
#define DRV8873_DEFAULT_IC1 0x45 // Toff = 40us, output follow input, 34V/us, PWM mode
#define DRV8873_DEFAULT_IC2 0xEC // ITRIP, OTW, TEMP & CP fault causes latched fault and report on nFault
#define DRV8873_DEFAULT_IC3 0x40 // 
#define DRV8873_DEFAULT_IC4 0x1C

// Define registers index for DRV8873
#define FAULT_REGISTER 0
#define DIAG_REGISTER 1
#define IC1_CONTROL_REGISTER 2
#define IC2_CONTROL_REGISTER 3
#define IC3_CONTROL_REGISTER 4
#define IC4_CONTROL_REGISTER 5

// Define fault indexes for each possible fault
#define DRV8873_FAULT_OLD 0
#define DRV8873_FAULT_TSD 1
#define DRV8873_FAULT_OCP 2
#define DRV8873_FAULT_CPUV 3
#define DRV8873_FAULT_UVLO 4
#define DRV8873_FAULT_OTW 5

// Define diagnostic indexes for each possible diag
#define DRV8873_DIAG_OCP_L2 0
#define DRV8873_DIAG_OCP_H2 1
#define DRV8873_DIAG_OCP_L1 2
#define DRV8873_DIAG_OCP_H1 3
#define DRV8873_DIAG_ITRIP2 4
#define DRV8873_DIAG_ITRIP1 5
#define DRV8873_DIAG_OL2 6
#define DRV8873_DIAG_OL1 7

// Define DRV8873 EN/DIS states
#define DRIVE_DISABLED 1
#define DRIVE_ENABLED 0

// Define DRV8873 SLP/ACT states
#define DRIVE_SLEEPING 0
#define DRIVE_AWAKE 1

// Define DRV8873 fault condition, compliments the nFault pin, which is open drain
// That means a fault is active on a low level
#define DRV8873_FAULT_ACTIVE 0
#define DRV8873_FAULT_INACTIVE 1

// Create a structure with bit field for each possible fault
typedef struct
{
    unsigned char OLD : 1;      // Open-load detect
    unsigned char TSD : 1;      // Temperature shutdown detect
    unsigned char OCP : 1;      // Overcurrent detect
    unsigned char CPUV : 1;     // Charge-pump undervoltage detect
    unsigned char UVLO : 1;     // Under-voltage lockout detect
    unsigned char OTW : 1;      // Overtemperature warning detect
    unsigned char FAULT : 1;    // Complimentary to nFault pin
}STRUCT_DRV8873_FAULT;

// Create union to overwrite bit field with the current fault value, which is a byte
typedef union
{
    STRUCT_DRV8873_FAULT DRV8873_fault;
    unsigned char fault_register;
}UNION_DRV8873_FAULT;

// Create a structure with bit field for each possible diagnostic
typedef struct
{
    unsigned char OCP_L2 : 1;   // Overcurrent on low-side fet of half-bridge 2
    unsigned char OCP_H2 : 1;   // Overcurrent on high-side fet of half-bridge 2
    unsigned char OCP_L1 : 1;   // Overcurrent on low-side fet of half-bridge 1
    unsigned char OCP_H1 : 1;   // Overcurrent on high-side fet of half-bridge 1
    unsigned char ITRIP2 : 1;   // Current regulation status of half-bridge 2
    unsigned char ITRIP1 : 1;   // Current regulation status of half-bridge 1
    unsigned char OL2 : 1;      // Open-load detection of half-bridge 2
    unsigned char OL1 : 1;      // Open-load detection of half-bridge 1
}STRUCT_DRV8873_DIAG;

// Create union to overwrite bit field with the current diag value, which is a byte
typedef union
{
    STRUCT_DRV8873_DIAG DRV8873_diag;
    unsigned char diag_register;
}UNION_DRV8873_DIAG;

typedef struct
{
    UNION_DRV8873_FAULT DRV8873_union_fault;
    UNION_DRV8873_DIAG DRV8873_union_diag;
    unsigned char ic1_control;
    unsigned char ic2_control;
    unsigned char ic3_control;
    unsigned char ic4_control;
    unsigned char fault_flag;
    unsigned char * data;
    unsigned char active_state;
    unsigned char sleep_state;
}STRUCT_DRV8873;

void DRV8873_init (void);
void DRV8873_write (unsigned char reg, unsigned char value);
unsigned char DRV8873_read (unsigned char reg);
unsigned char DRV8873_get_fault_status (unsigned char fault);
void DRV8873_assert_fault (void);
void DRV8873_enable(void);
void DRV8873_disable(void);
void DRV8873_sleep (void);
void DRV8873_wake (void);
unsigned char DRV8873_get_diag_status (unsigned char diag);
unsigned char DRV8873_fault_state (void);
#endif