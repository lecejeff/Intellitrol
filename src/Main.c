//****************************************************************************//
// File      : Main.c
//
// Purpose   : Intellitrol main
//
// Author    : Jean-Francois Bilodeau
//
// Platform  : MPLab X ide v5.10, XC16 v1.35
//
// Device    : dsPIC33EP512GM304
//****************************************************************************//
#include "general.h"
#include "intellitrol.h"
#include "UART.h"
#include "spi.h"
#include "i2c.h"
#include "pwm.h"
#include "timer.h"
#include "ADC.h"
#include "DRV8873S.h"
#include "QEI.h"

// FICD
#pragma config ICS = PGD2               // ICD Communication Channel Select bits (Communicate on PGEC2 and PGED2)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config BOREN = OFF              //  (BOR is disabled)
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Source Selection (Fast RC Oscillator with divide-by-N with PLL module (FRCPLL))
#pragma config PWMLOCK = OFF            // PWM Lock Enable bit (PWM registers may be written without key sequence)
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

unsigned char test = 0;

extern STRUCT_INTELLITROL INTELLITROL_struct;
extern STRUCT_PID PID_struct;
int main (void) 
{   
    //
    PIC_init();                                 // Basic PIC initialization : clock, PLL, analog off by default                         // Initialize intellitrol   
    // SPI initialization
    SPI_init(DRV8873_SPI, SPI_MODE0, 3, 0);     // SPI1 initialization at Fsck of 10MHz 
    SPI_init(INTELLITROL_SPI, SPI_MODE0, 0, 0); // SPI2 initialization at Fsck of 1MHz 
 
    // UART initialization
    UART_init(INTELLITROL_UART, 57600);        // Initialize UART2, Intellitrol communication interface
    
    // ADC initialization
    ADC_init();                                 // ADC init
    
    // PWM initialization
    PWM_init();                                 // Initialize PWM channels    
    
    // Intellitrol initialization with default parameters
    INTELLITROL_init();  
    
    // I2C initialization
    I2C_init(INTELLITROL_I2C, I2C_mode_slave, INTELLITROL_struct.drive_address);// I2C slave init, Intellitrol communication interface    
    I2C_init(MEMORY_I2C, I2C_mode_master, 0);                                   // I2C master init, I2C memory  
    
    // Read the EEPROM, see if it was previously written. If so, change the
    // intellitrol parameters.
    test = EEPROM_was_written();
    INTELLITROL_restore(test);
    
    // QEI initialization
    QEI_init(QEI_1, INTELLITROL_struct.pid_fs);                            // Initialize QEI1 direction and speed sensing  
    
    // Timer initialization
    TIMER_init(TIMER_1, STATE_REFRESH_FREQUENCY);           // Drive state refresh, FIXED frequency
    TIMER_init(FAULT_FS_TIMER, INTELLITROL_struct.fault_fs);       // Fault actualization, variable       
    TIMER_init(PID_FS_TIMER, INTELLITROL_struct.pid_fs);         // PID actualize, variable   
    TIMER_init(TIMER_8, 5);                                 // 5Hz timer (heartbeat) and mode scanning 
 
    // ADC start
    ADC_start();     
    
    // At this point, all the modules are initialized and ready for usage
    // Display each the RGB led color sequentially
    INTELLITROL_struct.rgb_led_color = RED;
    INTELLITROL_rgb_monitor();
    __delay_ms(500);           // Initialisation delay
    INTELLITROL_struct.rgb_led_color = GREEN;
    INTELLITROL_rgb_monitor();
    __delay_ms(500);           // Initialisation delay
    INTELLITROL_struct.rgb_led_color = BLUE;
    INTELLITROL_rgb_monitor();
    __delay_ms(500);           // Initialisation delay 
    
    // Start the monitoring timers and the intellitrol application program 
    TIMER_start(TIMER_1);           // Intellitrol state refresh
    TIMER_start(FAULT_FS_TIMER);    // Fault monitor
    TIMER_start(PID_FS_TIMER);      // PID state refresh
    TIMER_start(TIMER_8);           // Intellitrol mode change
    
    while (1)
    {                       
        // Drive state refresh service
        if (TIMER_get_state(TIMER_1, TIMER_INT_STATE))
        {    
            INTELLITROL_store_data(INTELLITROL_struct.new_mode);
            INTELLITROL_control(INTELLITROL_struct.new_mode);  
            INTELLITROL_kill_switch();        
            if(INTELLITROL_struct.motor_speed_mode == SET_SPEED_IN_PERC)
            {
                if (INTELLITROL_struct.new_mode == INPUT_MODE_ANALOG)
                {
                    INTELLITROL_drive_analog();
                }

                else
                {
                    INTELLITROL_drive_perc();
                }
            }      

            if (INTELLITROL_struct.motor_speed_mode == SET_SPEED_IN_RPM)
            {
                // Output = setpoint + error
                // Adjust the motor_speed_rpm to the physical drive interface
                // which works on a % scale, from 0 - 100
                INTELLITROL_struct.motor_speed_perc = INTELLITROL_struct.pid_out;
                INTELLITROL_drive_perc();
            }

            if (INTELLITROL_struct.motor_speed_mode == SET_SPEED_IN_PULSE)
            {
                INTELLITROL_struct.motor_speed_perc = INTELLITROL_struct.pid_out;
                INTELLITROL_drive_perc();
            }
            // Killswitch 1 stops a forward motor operation when pressed
            if ((INTELLITROL_struct.killswitch_1_enable == KILLSWITCH_ENABLED) && 
                (INTELLITROL_struct.killswitch_1_state == KILLSWITCH_IS_PRESSED) &&
                (INTELLITROL_struct.motor_direction == DIRECTION_FORWARD))
            {
                INTELLITROL_struct.motor_speed_perc = 0;
                INTELLITROL_struct.motor_speed_rpm = 0;
                INTELLITROL_struct.motor_movement_pulse = 0;
            }
                
            // Killswitch 2 stops a backward motor operation when pressed
            if ((INTELLITROL_struct.killswitch_2_enable == KILLSWITCH_ENABLED) &&
                (INTELLITROL_struct.killswitch_2_state == KILLSWITCH_IS_PRESSED) &&
                (INTELLITROL_struct.motor_direction == DIRECTION_BACKWARD))
            {
                INTELLITROL_struct.motor_speed_perc = 0;
                INTELLITROL_struct.motor_speed_rpm = 0;
                INTELLITROL_struct.motor_movement_pulse = 0;
            }                
        }
        
        // Drive fault monitor service, monitoring of DRV8873 faults, battery voltage and motor current
        if (TIMER_get_state(FAULT_FS_TIMER, TIMER_INT_STATE))
        {
            INTELLITROL_fault_monitor();
        }  
        
        // PID control service
        if (TIMER_get_state(PID_FS_TIMER, TIMER_INT_STATE))
        {   
            QEI_calculate_velocity(QEI_1);
            if (INTELLITROL_struct.motor_speed_mode == SET_SPEED_IN_PULSE)
            {
                INTELLITROL_movement();
            }
            INTELLITROL_pid();            
        }     
        
        // Drive mode change service
        if (TIMER_get_state(TIMER_8, TIMER_INT_STATE))
        {
            INTELLITROL_mode_change();           
        }
    }                         
    return 0;
}

//*****************************void PIC_init (void)***************************//
//Description : Function initializes dsPIC33EP512MC806
//
//Function prototype : void PIC_init (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : PIC_init();
//
//Jean-Francois Bilodeau    MPLab X v5.00    09/10/2018  
//****************************************************************************//
void PIC_init (void)
{
    INTCON1bits.NSTDIS = 1;             // Nested interrupt disabled 
    RCONbits.SWDTEN=0;                  // Watchdog timer disabled 
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    // Input FRC is 7.3728MHz. Output required is 140MHz for 70MIPS  
    // configure operation at 70 MIPS
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    PLLFBDbits.PLLDIV = 70; // M = 72
    CLKDIVbits.PLLPRE=0;    // N1=2
    CLKDIVbits.PLLPOST=0;   // N2=2
    
    // Initiate Clock Switch to FRC oscillator with PLL (NOSC=0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC!= 0b001);
    // Wait for PLL to lock
    while (OSCCONbits.LOCK!= 1);
    
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;   

    IOCON1 = 0x0000;
    IOCON2 = 0x0000;
    IOCON3 = 0x0000;
    
    // Enable change notice interrupt
    IFS1bits.CNIF = 0;
    IPC4bits.CNIP = 7; // High priority interrupt
    IEC1bits.CNIE = 1; 
}

//****************************_CNInterrupt************************************//
//Description : Change notice interrupt handle for QEI module
//
//Function prototype : 
//
//Enter params       : None
//
//Exit params        : UC : direction of vehicule
//
//Function call      : 
//
//Jean-Francois Bilodeau    MPLab X v3.30    26/02/2017  
//****************************************************************************//
void __attribute__((__interrupt__,no_auto_psv)) _CNInterrupt(void)
{  
    static unsigned char state_QEI1_B = 0;
    static unsigned char state_QEI1_A = 0;
    static unsigned char state_CS2 = 0;
    
    IFS1bits.CNIF = 0; 
    
    if ((QEI1B_PIN == 1) && (state_QEI1_B == 0)) // QEI 1 interrupt on QEI1_B leading edge
    {
        state_QEI1_B = 1;
        QEI_interrupt_handle(QEI_1);    // Handle the interrupt 
    }
    else
        state_QEI1_B = QEI1B_PIN;
    
    if ((QEI1A_PIN == 1) && (state_QEI1_A == 0))
    {
        state_QEI1_A = 1;
        QEI_interrupt_handle(QEI_1);
    }
    else
        state_QEI1_A = QEI1A_PIN;
    
    if ((INTELLITROL_SPI2_CS == 0) && (state_CS2 == 1))
    {
        state_CS2 = 0;  
        SPI_slave_initiate();   // Write 1st byte to SPI2BUF (slave function)
    }
    else
        state_CS2 = INTELLITROL_SPI2_CS;
}
    


