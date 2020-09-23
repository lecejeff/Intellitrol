//***************************************************************************//
// File      : DRV8873S.c
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
// Includes  :  DRV8873S.h
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
#include "DRV8873S.h"

STRUCT_DRV8873 DRV8873_struct;

//*************************void DRV8873_init (void)***************************//
//Description : Function initializes DRV8873 motor controller
//
//Function prototype : void DRV8873_init (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : DRV8873_init();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void DRV8873_init (void)
{
    // DRV8873 initialization
    
    // DRV8873 #DISABLE pin initialization
    TRISBbits.TRISB14 = 0;      // Drive disable is an output
    DRV8873_enable();           // Drive enable by default
    
    // DRV8873 #SLEEP pin initialization
    TRISBbits.TRISB2 = 0;       // Drive sleep is an output
    DRV8873_wake();   
       
    // Default DRV8873 configuration
    DRV8873_write(IC1_CONTROL_REGISTER, DRV8873_struct.ic1_control); // Toff = 40us, SR of 53V/us, Mode = PWM
    DRV8873_write(IC2_CONTROL_REGISTER, DRV8873_struct.ic2_control); // All faults reported, 4ms OC retry, OC fault causes a fault latchup
    DRV8873_write(IC3_CONTROL_REGISTER, DRV8873_struct.ic3_control); // IC1 rewriteable, Both half-bridge enabled  
    DRV8873_write(IC4_CONTROL_REGISTER, DRV8873_struct.ic4_control); // O-Load diag enabled, delay 300us, 7A trip and I reg enabled

    // Clear existing faults caused by register initialization
    DRV8873_assert_fault();
    
    // DRV8873 #FAULT pin is monitored by an external interrupt, configure the interrupt registers
    // DRV8873 #FAULT pin initialization
    TRISAbits.TRISA0 = 1;       // DRV8873S fault pin is an input
    RPINR0bits.INT1R = 0x10;    // EXT INT1 on RA0 (RPI16), fault interrupt
    IPC5bits.INT1IP = 4;        // Set priority to 4
    IFS1bits.INT1IF = 0;
    INTCON2bits.INT1EP = 1;     // Interrupt active on negative edge
    IEC1bits.INT1IE = 1;        // Enable external interrupt 1
    
    DRV8873_enable();           // Drive enable by default
}

//*******void DRV8873_write (unsigned char reg, unsigned char value)**********//
//Description : Function writes an 8-bit value to a register inside the DRV8873
//
//Function prototype : void DRV8873_write (unsigned char reg, unsigned char value)
//
//Enter params       : unsigned char reg : register to write
//                   : unsigned char value : byte to write
//
//Exit params        : None
//
//Function call      : DRV8873_write(IC1_CONTROL_REGISTER, 0x7A);
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void DRV8873_write (unsigned char reg, unsigned char value)
{
    unsigned char DRV_data[2] = {0, 0};
    DRV_data[0] = (reg<<1);
    DRV_data[1] = value;
    SPI_master_write(DRV8873_SPI, DRV_data, 2, DRV8873_CHIP);
}

//************unsigned char DRV8873_read (unsigned char reg)******************//
//Description : Function reads an 8-bit value from a register inside the DRV8873
//
//Function prototype : unsigned char DRV8873_read (unsigned char reg)
//
//Enter params       : unsigned char reg : register to read
//
//Exit params        : unsigned char : Byte value from the register
//
//Function call      : unsigned char = DRV8873_read(IC1_CONTROL_REGISTER);
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
unsigned char DRV8873_read (unsigned char reg)
{
    unsigned char DRV_data[2] = {0, 0};
    DRV_data[0] = ((reg<<1)|0x40);
    DRV_data[1] = 0xFF;
    SPI_master_write(DRV8873_SPI, DRV_data, 2, DRV8873_CHIP);
    while(!SPI_rx_done(DRV8873_SPI));
    DRV8873_struct.data = SPI_get_rx_buffer(DRV8873_SPI);
    return(DRV8873_struct.data[1]);
}

//******************void DRV8873_assert_fault (void)**************************//
//Description : Function asserts the fault condition by ORing the fault bit in
//              the IC3 control register and clearing the structure fault flag
//
//Function prototype : unsigned char DRV8873_read (unsigned char reg)
//
//Enter params       : unsigned char reg : register to read
//
//Exit params        : unsigned char : Byte value from the register
//
//Function call      : unsigned char = DRV8873_read(IC1_CONTROL_REGISTER);
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void DRV8873_assert_fault (void)
{
    DRV8873_write(IC3_CONTROL_REGISTER, (DRV8873_struct.ic3_control | 0x80));
    __delay_us(100);
    DRV8873_struct.fault_flag = DRV8873_FAULT_INACTIVE; // Clear fault bit
    IEC1bits.INT1IE = 1;                                // Re-enable fault interrupt
}

//********unsigned char DRV8873_get_fault_status (unsigned char fault)********//
//Description : Function returns the status of a single fault parameter
//
//Function prototype : unsigned char DRV8873_get_fault_status (unsigned char fault)
//
//Enter params       : unsigned char fault : fault condition to read (see in DRV8873S.h)
//
//Exit params        : unsigned char : Status of the fault (1 = fault active)
//
//Function call      : unsigned char = DRV8873_get_fault_status(DRV8873_FAULT_OCP);
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
unsigned char DRV8873_get_fault_status (unsigned char fault)
{
    return ((DRV8873_struct.DRV8873_union_fault.fault_register>>fault)&0x01);
}

//******************unsigned char DRV8873_fault_state (void)******************//
//Description : Function returns the status of the structure fault flag
//
//Function prototype : unsigned char DRV8873_fault_state (void)
//
//Enter params       : None
//
//Exit params        : unsigned char : Status of the fault flag
//
//Function call      : unsigned char = DRV8873_fault_state();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
unsigned char DRV8873_fault_state (void)
{
    return DRV8873_struct.fault_flag;   // Return the state of the fault flag
}

//********unsigned char DRV8873_get_diag_status (unsigned char diag)**********//
//Description : Function returns the status of a single diag parameter
//
//Function prototype : unsigned char DRV8873_get_diag_status (unsigned char diag)
//
//Enter params       : unsigned char diag : diagnostic condition to read (see in DRV8873S.h)
//
//Exit params        : unsigned char : Status of the diag (1 = diag active)
//
//Function call      : unsigned char = DRV8873_get_diag_status(DRV8873_DIAG_ITRIP2);
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
unsigned char DRV8873_get_diag_status (unsigned char diag)
{
    return ((DRV8873_struct.DRV8873_union_diag.diag_register>>diag)&0x01);
}

//***********************void DRV8873_disable (void)**************************//
//Description : Function disables DRV8873 motor controller using the #DSBL pin
//
//Function prototype : void DRV8873_disable (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : DRV8873_disable();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void DRV8873_disable (void)
{
    DRV8873_struct.active_state = DRIVE_DISABLED;
    DRV8873_DISABLE_PIN = DRIVE_DISABLED; 
}

//***********************void DRV8873_enable (void)***************************//
//Description : Function enables DRV8873 motor controller using the #DSBL pin
//
//Function prototype : void DRV8873_enable (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : DRV8873_enable();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void DRV8873_enable (void)
{
    DRV8873_struct.active_state = DRIVE_ENABLED;
    DRV8873_DISABLE_PIN = DRIVE_ENABLED;
    __delay_us(100);
}

//***********************void DRV8873_sleep (void)****************************//
//Description : Function put the DRV8873 in sleep mode
//
//Function prototype : void DRV8873_sleep (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : DRV8873_sleep();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void DRV8873_sleep (void)
{
    DRV8873_struct.sleep_state = DRIVE_SLEEPING;
    DRV8873_SLEEP_PIN = DRIVE_SLEEPING;
}

//************************void DRV8873_wake (void)****************************//
//Description : Function wakes the DRV8873 from sleep mode
//
//Function prototype : void DRV8873_wake (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : DRV8873_wake();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void DRV8873_wake (void)
{
    DRV8873_struct.sleep_state = DRIVE_AWAKE;
    DRV8873_SLEEP_PIN = DRIVE_AWAKE; 
    __delay_ms(1.5);    // Device guaranteed operationnal after 1.5ms delay
}

// External interrupt 1 isr
// External interrupt active when DRV8873 has a pending fault to clear
void __attribute__((__interrupt__, no_auto_psv)) _INT1Interrupt(void)
{
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 0;
    DRV8873_struct.fault_flag = DRV8873_FAULT_ACTIVE;
}