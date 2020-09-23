//***************************************************************************//
// File      : 
//
// Functions :  
//
// Includes  :  
//           
// Purpose   :  
//
//Jean-Francois Bilodeau    MPLab X v5.10    10/02/2020  
//****************************************************************************//
#include "intellitrol.h"
#include "math.h"

STRUCT_INTELLITROL INTELLITROL_struct;
STRUCT_PID PID_struct;
extern STRUCT_QEI QEI_struct[QEI_QTY];
extern STRUCT_DRV8873 DRV8873_struct;

//*********************void INTELLITROL_init (void)***************************//
//Description : Function initialize the Intellitrol controller with default 
//              variables
//              Function usually called once before calibration. Since a lot of
//              environment variables can be changed during runtime, these new
//              parameters are saved to the I2C EEPROM
//
//Function prototype : void INTELLITROL_init (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : INTELLITROL_init();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void INTELLITROL_init (void)
{
    // Initialize intellitrol-related IOs not used in others .C or .H files
    // Initialize the debug LED I/O
    TRISBbits.TRISB9 = 0;   // DEBUG led
    DEBUG_LED = LED_OFF;    // Close it during initialization        
    
    // Initialize the mode switch input I/Os
    TRISCbits.TRISC1 = 1;   // IN 1 input
    TRISCbits.TRISC2 = 1;   // IN 2 input
    
    // Initialize the killswitch input I/Os
    TRISBbits.TRISB7 = 1;   // #KSW1 input
    TRISBbits.TRISB8 = 1;   // #KSW2 input       

    INTELLITROL_struct.killswitch_1_enable = KILLSWITCH_DISABLED;       
    INTELLITROL_struct.killswitch_2_enable = KILLSWITCH_DISABLED;  

    INTELLITROL_struct.drv8873_ic1 = DRV8873_DEFAULT_IC1;
    DRV8873_struct.ic1_control = INTELLITROL_struct.drv8873_ic1;
    INTELLITROL_struct.drv8873_ic2 = DRV8873_DEFAULT_IC2;
    DRV8873_struct.ic2_control = INTELLITROL_struct.drv8873_ic2;
    INTELLITROL_struct.drv8873_ic3 = DRV8873_DEFAULT_IC3;
    DRV8873_struct.ic3_control = INTELLITROL_struct.drv8873_ic3;
    INTELLITROL_struct.drv8873_ic4 = DRV8873_DEFAULT_IC4;
    DRV8873_struct.ic4_control = INTELLITROL_struct.drv8873_ic4; 
    
    PID_struct.p_gain = 1068;
    PID_struct.i_gain = 3085;
    
    INTELLITROL_struct.drive_address = DRIVE_ADDRESS;
    
    INTELLITROL_struct.motor_cpr = COUNT_PER_REVOLUTION;
    INTELLITROL_struct.motor_gear_derate = GEAR_DERATE;
    INTELLITROL_struct.motor_max_rpm = MOTOR_MAX_RPM;     
    INTELLITROL_struct.motor_speed_mode = SET_SPEED_IN_RPM;
    INTELLITROL_struct.motor_movement_peak_speed = (unsigned int)((INTELLITROL_struct.motor_max_rpm * 80) / 100);
    
    INTELLITROL_struct.drive_disable_override = INTELLITROL_OVERRIDE_OFF;
    INTELLITROL_struct.motor_overcurrent_threshold = MOTOR_CURRENT_MAX_THRESHOLD;
    INTELLITROL_struct.battery_high_threshold = BATTERY_HIGH_THRESHOLD;  
    INTELLITROL_struct.battery_low_threshold = BATTERY_LOW_THRESHOLD;
    INTELLITROL_struct.analog_idle_threshold = ANALOG_DEFAULT_IDLE_THRESHOLD;   

    //--------------------------------------------------------------------------
    // All variable that must be set regardless of the EEPROM state
    INTELLITROL_struct.analog_value = ANALOG_DEFAULT_OFF;
    
    // Intellitrol default direction and speed
    INTELLITROL_struct.motor_direction = DIRECTION_FORWARD;
    INTELLITROL_struct.motor_speed_perc = 0;
    INTELLITROL_struct.motor_speed_rpm = 0; 
    INTELLITROL_struct.motor_movement_state = MOTOR_MOVEMENT_OVER;
    INTELLITROL_struct.motor_movement_pulse = 0; // A ENLEVER    
    
    // Intellitrol LED overrides defaults
    INTELLITROL_struct.rgb_led_override = INTELLITROL_OVERRIDE_OFF;
    INTELLITROL_struct.debug_led_override = INTELLITROL_OVERRIDE_OFF;    
    
    // Intellitrol tx pointer deference
    INTELLITROL_struct.tx_ptr = &INTELLITROL_struct.tx_data[0];   
    
    // Intellitrol fault monitor defaults
    INTELLITROL_struct.intellitrol_fault = INTELLITROL_NO_FAULT;
    INTELLITROL_struct.fault_fs = INTELLITROL_FAULT_REFRESH_FS; 
    INTELLITROL_struct.fault_clear = 1;
    INTELLITROL_struct.drive_disable_flag = 0;    
    
    // PID variables and refresh frequency
    PID_struct.pid_high_limit = 100;
    INTELLITROL_struct.pid_fs = INTELLITROL_PID_REFRESH_FS;
    PID_struct.error_rpm = 0;   
    INTELLITROL_struct.pid_out = 0;
    INTELLITROL_pid_set_gains();    // Calculate double gains and sample time T     
    
    // Intellitrol mode defaults
    INTELLITROL_struct.old_mode = 10;
    INTELLITROL_struct.new_mode = 20;  
    
    // DRV8873 initialization   
    DRV8873_init();             // Initialize motor driver with default values 
}

void INTELLITROL_restore (unsigned char mode)
{
    unsigned char *ptr;
    if (mode == EEPROM_WAS_WRITTEN)
    {
        // Retrieve killswitch enable varibles
        ptr = EEPROM_read_byte(KSW1_ENABLE_OFFSET);
        INTELLITROL_struct.killswitch_1_enable = *ptr;

        ptr = EEPROM_read_byte(KSW2_ENABLE_OFFSET);
        INTELLITROL_struct.killswitch_2_enable = *ptr;

        ptr = EEPROM_read_byte(DRV8873_IC1_OFFSET);
        INTELLITROL_struct.drv8873_ic1 = *ptr;
        
        ptr = EEPROM_read_byte(DRV8873_IC2_OFFSET);
        INTELLITROL_struct.drv8873_ic2 = *ptr;
        
        ptr = EEPROM_read_byte(DRV8873_IC3_OFFSET);
        INTELLITROL_struct.drv8873_ic3 = *ptr;
        
        ptr = EEPROM_read_byte(DRV8873_IC4_OFFSET);
        INTELLITROL_struct.drv8873_ic4 = *ptr;
        
        ptr = EEPROM_read_integer(QEI_CPR_OFFSET);
        INTELLITROL_struct.motor_cpr = *ptr++ << 8;
        INTELLITROL_struct.motor_cpr = INTELLITROL_struct.motor_cpr | *ptr;

        ptr = EEPROM_read_integer(QEI_GDR_OFFSET);
        INTELLITROL_struct.motor_gear_derate = *ptr++ << 8;
        INTELLITROL_struct.motor_gear_derate = INTELLITROL_struct.motor_gear_derate | *ptr;  

        ptr = EEPROM_read_integer(MOTOR_MAX_RPM_OFFSET);
        INTELLITROL_struct.motor_max_rpm = *ptr++ << 8;
        INTELLITROL_struct.motor_max_rpm = INTELLITROL_struct.motor_max_rpm | *ptr; 
        
        ptr = EEPROM_read_byte(MOTOR_SPEED_MODE_OFFSET);
        INTELLITROL_struct.motor_speed_mode = *ptr;
        
        ptr = EEPROM_read_integer(MOVEMENT_PEAK_SPEED_OFFSET);
        INTELLITROL_struct.motor_movement_peak_speed = *ptr++ << 8;
        INTELLITROL_struct.motor_movement_peak_speed = INTELLITROL_struct.motor_movement_peak_speed | *ptr;     
        
        ptr = EEPROM_read_integer(PID_P_GAIN_OFFSET);
        PID_struct.p_gain = *ptr++ << 8;
        PID_struct.p_gain = PID_struct.p_gain | *ptr; 

        ptr = EEPROM_read_integer(PID_I_GAIN_OFFSET);
        PID_struct.i_gain = *ptr++ << 8;
        PID_struct.i_gain = PID_struct.i_gain | *ptr; 
        
        ptr = EEPROM_read_byte(LATCH_OVERRIDE_OFFSET);
        INTELLITROL_struct.drive_disable_override = *ptr; 

        ptr = EEPROM_read_integer(CURRENT_LIMIT_OFFSET);
        INTELLITROL_struct.motor_overcurrent_threshold = *ptr++ << 8;
        INTELLITROL_struct.motor_overcurrent_threshold = INTELLITROL_struct.motor_overcurrent_threshold | *ptr;  

        ptr = EEPROM_read_integer(BAT_HIGH_OFFSET);
        INTELLITROL_struct.battery_high_threshold = *ptr++ << 8;
        INTELLITROL_struct.battery_high_threshold = INTELLITROL_struct.battery_high_threshold | *ptr; 
       
        ptr = EEPROM_read_integer(BAT_LOW_OFFSET);
        INTELLITROL_struct.battery_low_threshold = *ptr++ << 8;
        INTELLITROL_struct.battery_low_threshold = INTELLITROL_struct.battery_low_threshold | *ptr;

        ptr = EEPROM_read_integer(ANALOG_IDLE_OFFSET);
        INTELLITROL_struct.analog_idle_threshold = *ptr++ << 8;
        INTELLITROL_struct.analog_idle_threshold = INTELLITROL_struct.analog_idle_threshold | *ptr;  
        
        //ptr = EEPROM_read_byte(DRIVE_ADDRESS_OFFSET);
        //INTELLITROL_struct.drive_address = *ptr; 
    }
    
    else
    {
        // Write the header to the memory
        EEPROM_write_header();
        EEPROM_write_byte(KSW1_ENABLE_OFFSET, INTELLITROL_struct.killswitch_1_enable);
        EEPROM_write_byte(KSW2_ENABLE_OFFSET, INTELLITROL_struct.killswitch_2_enable);
        EEPROM_write_byte(DRV8873_IC1_OFFSET, INTELLITROL_struct.drv8873_ic1);
        EEPROM_write_byte(DRV8873_IC2_OFFSET, INTELLITROL_struct.drv8873_ic2);
        EEPROM_write_byte(DRV8873_IC3_OFFSET, INTELLITROL_struct.drv8873_ic3);
        EEPROM_write_byte(DRV8873_IC4_OFFSET, INTELLITROL_struct.drv8873_ic4);
        //EEPROM_write_byte(DRIVE_ADDRESS_OFFSET, INTELLITROL_struct.drive_address);
        EEPROM_write_integer(QEI_CPR_OFFSET, INTELLITROL_struct.motor_cpr);
        EEPROM_write_integer(QEI_GDR_OFFSET, INTELLITROL_struct.motor_gear_derate); 
        EEPROM_write_integer(MOTOR_MAX_RPM_OFFSET, INTELLITROL_struct.motor_max_rpm);
        EEPROM_write_byte(MOTOR_SPEED_MODE_OFFSET, INTELLITROL_struct.motor_speed_mode);
        EEPROM_write_integer(MOVEMENT_PEAK_SPEED_OFFSET, INTELLITROL_struct.motor_movement_peak_speed);
        EEPROM_write_integer(PID_P_GAIN_OFFSET, PID_struct.p_gain);
        EEPROM_write_integer(PID_I_GAIN_OFFSET, PID_struct.i_gain);
        EEPROM_write_byte(LATCH_OVERRIDE_OFFSET, INTELLITROL_struct.drive_disable_override);
        EEPROM_write_integer(CURRENT_LIMIT_OFFSET, INTELLITROL_struct.motor_overcurrent_threshold);
        EEPROM_write_integer(BAT_HIGH_OFFSET, INTELLITROL_struct.battery_high_threshold);
        EEPROM_write_integer(BAT_LOW_OFFSET, INTELLITROL_struct.battery_low_threshold); 
        EEPROM_write_integer(ANALOG_IDLE_OFFSET, INTELLITROL_struct.analog_idle_threshold); 
    }
}

void INTELLITROL_kill_switch (void)
{
    static unsigned char counter_ksw1 = 1;
    static unsigned char counter_ksw2 = 1;
    
    if (KSW1_PIN == 0)
    {
        if (++counter_ksw1 > 20)
        {
            counter_ksw1 = 20;
            INTELLITROL_struct.killswitch_1_state = KILLSWITCH_IS_PRESSED;
        }
    }
    else
    {       
        if (--counter_ksw1 < 1)
        {
            INTELLITROL_struct.killswitch_1_state = KILLSWITCH_NOT_PRESSED;
            counter_ksw1 = 1;
        }
    }
    
    if (KSW2_PIN == 0)
    {
        if (++counter_ksw2 > 20)
        {
            counter_ksw2 = 20;
            INTELLITROL_struct.killswitch_2_state = KILLSWITCH_IS_PRESSED;
        }
    }
    else
    {        
        if (--counter_ksw2 < 1)
        {
            INTELLITROL_struct.killswitch_2_state = KILLSWITCH_NOT_PRESSED;
            counter_ksw2 = 1;
        }
    }    
}

void INTELLITROL_pid_set_gains (void)
{
    INTELLITROL_struct.pid_T = (double)(1.0 / INTELLITROL_struct.pid_fs);
    PID_struct.p_calc_gain = (double)(PID_struct.p_gain / 1000.0);
    PID_struct.i_calc_gain = (double)((PID_struct.i_gain / 1000.0) * INTELLITROL_struct.pid_T); 
}

//*********************void INTELLITROL_pid (void)****************************//
//Description : Function executes a PID routine to control the speed of 
//              the brushed DC motor, in RPM (revolution per minute)
//              Function computes the P and I compensation value based on the
//              actual error and sampling speed.
//
//Function prototype : void INTELLITROL_pid (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : INTELLITROL_pid();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void INTELLITROL_pid (void)
{
    // Mode of operation allows RPM mode control of the motor
    if (INTELLITROL_struct.new_mode != INPUT_MODE_ANALOG)
    {
        // If a motor speed is requested by application
        // Verify that the speed > 0 and that the requested speed does not exceed
        // the maximum RPM characteristic of the motor in use
        if (INTELLITROL_struct.motor_speed_rpm > 0)
        {           
            // Step 1 : Measure the error between the actual speed of the motor
            //          and the wanted speed, in RPM unit  
            //          Error = setpoint - measured
            
            // Get velocity
            PID_struct.actual_rpm = QEI_get_velocity(INTELLITROL_QEI);            
            if (PID_struct.actual_rpm > INTELLITROL_struct.motor_max_rpm){PID_struct.actual_rpm = INTELLITROL_struct.motor_max_rpm;}

            PID_struct.error_rpm = abs(INTELLITROL_struct.motor_speed_rpm - PID_struct.actual_rpm);
            
            if (PID_struct.error_rpm > (double)INTELLITROL_struct.motor_speed_rpm){PID_struct.error_rpm = (double)INTELLITROL_struct.motor_speed_rpm;} 

            // Step 4 : Evaluate the integral factor of the PID
            if (PID_struct.actual_rpm < INTELLITROL_struct.motor_speed_rpm)
            {
                PID_struct.p_value = (PID_struct.p_calc_gain * PID_struct.error_rpm);
                PID_struct.i_term += (PID_struct.i_calc_gain * PID_struct.error_rpm);
                // Limit the cumulative error to system limit, 10000 ATM is arbitrary
                if (PID_struct.i_term > PID_struct.pid_high_limit){PID_struct.i_term = PID_struct.pid_high_limit;} 
            }
            else if (PID_struct.actual_rpm > INTELLITROL_struct.motor_speed_rpm)
            {
                PID_struct.p_value = (PID_struct.p_calc_gain * -PID_struct.error_rpm);
                PID_struct.i_term -= (PID_struct.i_calc_gain * PID_struct.error_rpm);
            }

            // Limit the I term to the output limit of the system, which is
            // in this case driven by the 0-100% driving scheme of the DRV8873S
            //if (PID_struct.i_value > (double)PID_struct.pid_high_limit){PID_struct.i_value = (double)PID_struct.pid_high_limit;}
            //if (PID_struct.i_value < 0){PID_struct.i_value = ;}    

            // Step 5 : Evaluate the derivative factor of the PID
            // D value = derivative gain * diff of error
            //PID_struct.d_input = (double)(PID_struct.actual_rpm - PID_struct.last_actual_rpm);

            // Step 6 : Compute the PID output
            PID_struct.pid_out = (double)((PID_struct.p_value) 
                                            + (PID_struct.i_term));
                                            //+ (-PID_struct.d_calc_gain * PID_struct.d_input));   


            // Limit the PID output to the output limit of the system, which is
            // in this case driven by the 0-100% driving scheme of the DRV8873S                
            if (PID_struct.pid_out > (double)PID_struct.pid_high_limit){PID_struct.pid_out = (double)PID_struct.pid_high_limit;}
            INTELLITROL_struct.pid_out = (signed char)(PID_struct.pid_out);
            
            // Remember some variables
            PID_struct.last_actual_rpm = PID_struct.actual_rpm; 
            PID_struct.last_error = PID_struct.error_rpm;
        }
        
        if (INTELLITROL_struct.motor_speed_rpm == 0)
        {
            INTELLITROL_struct.pid_out = 0;
            PID_struct.pid_out = 0;
            PID_struct.i_term = 0;
            PID_struct.error_rpm = 0;
            PID_struct.actual_rpm = 0;
            PID_struct.last_actual_rpm = 0;
            PID_struct.last_error = 0;
        }
    }
}

void INTELLITROL_movement (void)
{
    static unsigned long movement_step = 0;
    static unsigned long real_movement = 0;
    static unsigned char setup = 0;
    static float accumulator = 0;
    if ((INTELLITROL_struct.motor_movement_pulse > 1000) && (INTELLITROL_struct.motor_movement_state == MOTOR_MOVEMENT_STARTED))
    {     
        if (setup == 0)
        {
            setup = 1;
            real_movement = (unsigned long)(INTELLITROL_struct.motor_movement_pulse * (float)(105.0 / 100));
        }
        
        movement_step = (real_movement / 1000);
        INTELLITROL_struct.motor_actual_pulse = QEI_get_distance(QEI_1);
        INTELLITROL_struct.motor_actual_tour = QEI_get_tour(QEI_1);
        //INTELLITROL_struct.motor_movement_cm = QEI_get_cm(QEI_1);

        // Start the linear movement
        // At 0-10% of movement, ramp up the speed
        if (INTELLITROL_struct.motor_actual_pulse < (250 * movement_step))
        {
            //INTELLITROL_struct.motor_speed_rpm += (unsigned int)(INTELLITROL_struct.motor_movement_peak_speed * INTELLITROL_struct.pid_T);            
            accumulator += (INTELLITROL_struct.motor_movement_peak_speed * INTELLITROL_struct.pid_T);
            INTELLITROL_struct.motor_speed_rpm = (unsigned long)accumulator;
            if (INTELLITROL_struct.motor_speed_rpm > INTELLITROL_struct.motor_movement_peak_speed)
            {
                INTELLITROL_struct.motor_speed_rpm = INTELLITROL_struct.motor_movement_peak_speed;
            }
        }

        if ((INTELLITROL_struct.motor_actual_pulse >= (250 * movement_step)) && 
                (INTELLITROL_struct.motor_actual_pulse <= (750 * movement_step)))
        {
            INTELLITROL_struct.motor_speed_rpm = INTELLITROL_struct.motor_movement_peak_speed;
            accumulator = 0;
        }       
        
        if ((INTELLITROL_struct.motor_actual_pulse > (750 * movement_step)) && (INTELLITROL_struct.motor_actual_pulse < real_movement))
        {
            // If peak speed > 70% of maximum RPM speed of motor, reduce speed to 35% peak speed
            if (INTELLITROL_struct.motor_movement_peak_speed >= ((INTELLITROL_struct.motor_max_rpm * 70)/100))
            {
                if (INTELLITROL_struct.motor_speed_rpm > ((INTELLITROL_struct.motor_movement_peak_speed*30)/100))
                {
                    accumulator += (INTELLITROL_struct.motor_movement_peak_speed * INTELLITROL_struct.pid_T);
                    INTELLITROL_struct.motor_speed_rpm -= (unsigned long)(accumulator);                      
                }
                else
                {
                    INTELLITROL_struct.motor_speed_rpm = ((INTELLITROL_struct.motor_movement_peak_speed*30)/100);
                }
            }
            
            // If peak speed < 70% of maximum RPM speed of motor, reduce speed to 30% peak speed
            else
            {
                if (INTELLITROL_struct.motor_speed_rpm > ((INTELLITROL_struct.motor_movement_peak_speed*50)/100))
                {
                    accumulator += (INTELLITROL_struct.motor_movement_peak_speed * INTELLITROL_struct.pid_T);
                    INTELLITROL_struct.motor_speed_rpm -= (unsigned long)(accumulator);                      
                }
                else
                {
                    INTELLITROL_struct.motor_speed_rpm = ((INTELLITROL_struct.motor_movement_peak_speed*50)/100);
                }                
            }
        }
        
        // If movement is between ±5% of final RPM count, stop the movement
        if (INTELLITROL_struct.motor_actual_pulse > real_movement)
        {  
            INTELLITROL_struct.motor_speed_rpm = 0;
            accumulator = 0;
            setup = 0;
            INTELLITROL_struct.motor_movement_state = MOTOR_MOVEMENT_OVER;  
        }
    }
    else
    {
        accumulator = 0;
        setup = 0;
        movement_step = 0;
    }
}

//**************void INTELLITROL_store_data (unsigned char mode)**************//
//Description : Function checks if the selected serial port has data available,
//              reads the data and fills the receive pointer
//
//Function prototype : void INTELLITROL_store_data (unsigned char mode)
//
//Enter params       : unsigned char mode : Operating mode of Intellitrol
//
//Exit params        : None
//
//Function call      : INTELLITROL_store_data(INTELLITROL_struct.new_mode);
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void INTELLITROL_store_data (unsigned char mode)
{
    switch(mode)
    {                
        case INPUT_MODE_I2C:
            if (I2C_rx_done(INTELLITROL_I2C))   // New data is available through I2C
            {
                INTELLITROL_struct.rx_ptr = I2C_get_rx_buffer(INTELLITROL_I2C);    
                if (!INTELLITROL_fill_receive_buffer())
                {
                    INTELLITROL_struct.intellitrol_fault = INTELLITROL_COM_FAULT;
                }
                I2C_clear_rx_buffer(INTELLITROL_I2C);
            }
            break;

        case INPUT_MODE_UART:
            if (UART_rx_done(INTELLITROL_UART)) // New data is available through UART
            {
                INTELLITROL_struct.rx_ptr = UART_get_rx_buffer(INTELLITROL_UART);
                if (!INTELLITROL_fill_receive_buffer())
                {
                    INTELLITROL_struct.intellitrol_fault = INTELLITROL_COM_FAULT;
                }
                UART_clear_rx_buffer(INTELLITROL_UART);
            }
            break;

        case INPUT_MODE_ANALOG:
            if (ADC_sample_status() == ADC_SAMPLE_READY) // New ADC sample is ready
            {
                INTELLITROL_struct.analog_value = ADC_get_channel(ADC_ANALOG_INPUT_CHANNEL);               
            }                  
            break;

        case INPUT_MODE_SPI:
            if (SPI_rx_done(INTELLITROL_SPI)) // New data is available through SPI
            {
                INTELLITROL_struct.rx_ptr = SPI_get_rx_buffer(INTELLITROL_SPI);
                if (!INTELLITROL_fill_receive_buffer())
                {
                    INTELLITROL_struct.intellitrol_fault = INTELLITROL_COM_FAULT;
                }
                SPI_clear_rx_buffer(INTELLITROL_SPI);
            }
            break;                   
    } 
}

//********************void INTELLITROL_drive_perc (void)**********************//
//Description : Function drives the motor according to the speed_perc variable
//              Function clips the output between 0-100
//
//Function prototype : INTELLITROL_drive_perc
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : INTELLITROL_drive_perc();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void INTELLITROL_drive_perc (void)
{
    if ((INTELLITROL_struct.motor_speed_perc > 0) && (INTELLITROL_struct.motor_speed_perc <= 100))
    {
        if (INTELLITROL_struct.motor_direction == DIRECTION_FORWARD)
        {
            PWM_change_duty(PWM_4H, INTELLITROL_struct.motor_speed_perc);
            PWM_change_duty(PWM_4L, 0);        
        }

        else
        {
            PWM_change_duty(PWM_4H, 0);
            PWM_change_duty(PWM_4L, INTELLITROL_struct.motor_speed_perc);        
        }
    }
    
    else
    {
        PWM_change_duty(PWM_4H, 0);
        PWM_change_duty(PWM_4L, 0);        
    }
}

//***********unsigned char INTELLITROL_fill_receive_buffer (void)*************//
//Description : Function saves the content of the rx_ptr to the rx buffer, and
//              calculates the checksum. If the checksum (sum of bytes 0-3) is
//              equal to the last byte (byte 4), the function returns 1
//              If the checksum is different (communication error), the function
//              returns 0
//
//Function prototype : unsigned char INTELLITROL_fill_receive_buffer (void)
//
//Enter params       : None
//
//Exit params        : unsigned char : 0 = good command, 1 = com error
//
//Function call      : unsigned char = INTELLITROL_fill_receive_buffer();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
unsigned char INTELLITROL_fill_receive_buffer (void)
{
    unsigned char i = 0;
    unsigned int checksum = 0;
    for (i=0; i< DRIVE_DATA_LENGTH; i++)
    {
        INTELLITROL_struct.rx_data[i] = *INTELLITROL_struct.rx_ptr;
        INTELLITROL_struct.rx_ptr++;
    }
    
    // Calculate the checksum
    for (i=0; i < (DRIVE_DATA_LENGTH - 1); i++)
    {
        checksum = checksum + INTELLITROL_struct.rx_data[i];
    }

    // If calculated checksum = content of the last byte received
    if ((checksum&0x00FF) == INTELLITROL_struct.rx_data[DRIVE_DATA_LENGTH-1])
    {       
        return 1;
    } 
    
    else 
    {
        INTELLITROL_struct.intellitrol_fault = INTELLITROL_COM_FAULT;
        return 0;
    }
}

//void INTELLITROL_fill_transmit_buffer (unsigned char c1, unsigned char c2_d1, unsigned char d2)//
//Description : Function fills the transmit buffer before sending it through
//              the actually selected serial port. Since this function is used
//              only to respond to a request from an external host, it will
//              always respond with DRIVE_ADDRESS + 1, the read address
//              This fonction calculates the checksum and places the result in
//              the last buffer location
//
//Function prototype : void INTELLITROL_fill_transmit_buffer (unsigned char c1, unsigned char c2_d1, unsigned char d2)
//
//Enter params       : unsigned char c1 : 1st command byte
//                     unsigned char c2_d1 : 2nd command / 1st data byte
//                     unsigned char d2 : 2nd data byte
//
//Exit params        : None
//
//Function call      : INTELLITROL_fill_transmit_buffer(COM_CTRL1_DRIVE_IC4, 0, INTELLITROL_struct.drv8873_ic4);
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void INTELLITROL_fill_transmit_buffer (unsigned char c1, unsigned char c2_d1, unsigned char d2)
{
    INTELLITROL_struct.tx_data[0] = INTELLITROL_struct.drive_address;   // +1 since this function is only called when the drive responds to a read request
    INTELLITROL_struct.tx_data[1] = c1;
    INTELLITROL_struct.tx_data[2] = c2_d1;
    INTELLITROL_struct.tx_data[3] = d2;
    INTELLITROL_struct.tx_data[4] = (unsigned char)((INTELLITROL_struct.tx_data[0] + INTELLITROL_struct.tx_data[1] + INTELLITROL_struct.tx_data[2] + INTELLITROL_struct.tx_data[3])&0x00FF);
}

//********************void INTELLITROL_mode_change (void)*********************//
//Description : Function checks whether a change in the user switch configuration
//              is detected. If a new operating mode is detected, the function
//              will set the RGB led according to the new operating mode and
//              set the default speed mode either to SPEED_PERC or SPEED_RPM
//              This function is to be called in a Timer-driven interrupt at a
//              fixed rate
//
//Function prototype : void INTELLITROL_mode_change (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : INTELLITROL_mode_change();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void INTELLITROL_mode_change (void)
{
    // Check if a new operating mode was selected
    INTELLITROL_struct.old_mode = INTELLITROL_struct.new_mode;
    INTELLITROL_struct.new_mode = INTELLITROL_get_mode();
    
    // If the new mode is different vs the old mode
    if (INTELLITROL_struct.new_mode != INTELLITROL_struct.old_mode)
    {
        INTELLITROL_clear_rx_buffer();  // Clear the rx buffer
        PWM_change_duty(PWM_4H, 0);     // Stop the motor driver
        PWM_change_duty(PWM_4L, 0);     // Stop the motor driver 
        
        // Set the new operating mode parameters
        switch(INTELLITROL_struct.new_mode)
        {
            case INPUT_MODE_I2C:
                INTELLITROL_struct.rgb_led_color = GREEN;
                //INTELLITROL_struct.motor_speed_mode = SET_SPEED_IN_RPM; 
                break;
                
            case INPUT_MODE_UART:
                INTELLITROL_struct.rgb_led_color = RED;
                //INTELLITROL_struct.motor_speed_mode = SET_SPEED_IN_RPM; 
                break;
                
            case INPUT_MODE_ANALOG:
                INTELLITROL_struct.rgb_led_color = PURPLE; 
                INTELLITROL_struct.motor_speed_mode = SET_SPEED_IN_PERC;             
                break;
                
            case INPUT_MODE_SPI:
                INTELLITROL_struct.rgb_led_color = BLUE; 
                //INTELLITROL_struct.motor_speed_mode = SET_SPEED_IN_RPM; 
                break;
        }
        
        // Stop any motor movement after a mode change
        INTELLITROL_struct.motor_speed_rpm = 0;
        INTELLITROL_struct.motor_speed_perc = 0;
        INTELLITROL_struct.motor_movement_pulse = 0;
        INTELLITROL_struct.motor_speed_perc = 0;
    }
    
    // Toggle the debug LED
    INTELLITROL_debug_toggle();

    // Change the RGB LED color
    INTELLITROL_rgb_monitor();
}

//******************void INTELLITROL_fault_monitor (void)*********************//
//Description : Function checks the state of the different possible faults in the
//              intellitrol system and monitors them. If a fault threshold is
//              reached or if an interrupt from the DRV8873 controller is detected,
//              the fault monitor saves the fault to the intellitrol struct and
//              can turn off the motor driver accordingly or trigger an alarm to
//              the host system.
//              This function is to be called in a Timer-driven interrupt at a 
//              fixed frequency.
//
//Function prototype : void INTELLITROL_fault_monitor (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : INTELLITROL_fault_monitor();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void INTELLITROL_fault_monitor (void)
{
    // Update ADC values from battery monitor and motor current monitor
    INTELLITROL_struct.battery_value = ADC_get_channel(ADC_BATTERY_CHANNEL);
    INTELLITROL_struct.motor_1_current = ADC_get_channel(ADC_MOTOR_1_CHANNEL);
    INTELLITROL_struct.motor_2_current = ADC_get_channel(ADC_MOTOR_2_CHANNEL);
    
    // Check if there is a fault condition on the DRV8873 waiting to be asserted
    if ((DRV8873_fault_state() == DRV8873_FAULT_ACTIVE) && (!INTELLITROL_struct.drive_disable_flag))      // If the nFault pin is 0, there is an active fault
    {
        INTELLITROL_struct.rgb_led_color = DRV8873_FAULT_COLOR;     // Change RGB LED color variable
        INTELLITROL_struct.drive_disable_flag = 1;                  // Disable drive once
        // Read and update the diagnostic and fault registers 
        DRV8873_struct.DRV8873_union_diag.diag_register = DRV8873_read(DIAG_REGISTER);
        DRV8873_struct.DRV8873_union_fault.fault_register = DRV8873_read(FAULT_REGISTER);     
        // Charge-pump undervoltage fault
        if (DRV8873_struct.DRV8873_union_fault.DRV8873_fault.CPUV == 1)
        {
            INTELLITROL_struct.intellitrol_fault = DRV8873_FAULT_CPUV + INTELLITROL_DRV8873_FAULT_BASE;
        }

        // Overcurrent protection fault
        if (DRV8873_struct.DRV8873_union_fault.DRV8873_fault.OCP == 1)
        {
            INTELLITROL_struct.intellitrol_fault = DRV8873_FAULT_OCP + INTELLITROL_DRV8873_FAULT_BASE;
        }

        // Openload fault
        if (DRV8873_struct.DRV8873_union_fault.DRV8873_fault.OLD == 1)
        {
            INTELLITROL_struct.intellitrol_fault = DRV8873_FAULT_OLD + INTELLITROL_DRV8873_FAULT_BASE;
        }

        // Overtemperature warning fault
        if (DRV8873_struct.DRV8873_union_fault.DRV8873_fault.OTW == 1)
        {
            INTELLITROL_struct.intellitrol_fault = DRV8873_FAULT_OTW + INTELLITROL_DRV8873_FAULT_BASE;
        }

        // Temperature shutdown fault
        if (DRV8873_struct.DRV8873_union_fault.DRV8873_fault.TSD == 1)
        {
            INTELLITROL_struct.intellitrol_fault = DRV8873_FAULT_TSD + INTELLITROL_DRV8873_FAULT_BASE;
        }

        // Undervoltage lock-out fault
        if (DRV8873_struct.DRV8873_union_fault.DRV8873_fault.UVLO == 1)
        {
            INTELLITROL_struct.intellitrol_fault = DRV8873_FAULT_UVLO + INTELLITROL_DRV8873_FAULT_BASE;
        }            
    }
    
    // Battery monitor fault
    else if ((INTELLITROL_struct.battery_value > INTELLITROL_struct.battery_high_threshold) &&
            (!INTELLITROL_struct.drive_disable_flag))
    {
        INTELLITROL_struct.rgb_led_color = INTELLITROL_FAULT_COLOR; // Change RGB LED color variable
        INTELLITROL_struct.intellitrol_fault = INTELLITROL_BATTERY_OVERVOLTAGE_FAULT;
        INTELLITROL_struct.drive_disable_flag = 1;
    }
    else if ((INTELLITROL_struct.battery_value < INTELLITROL_struct.battery_low_threshold) &&
            (!INTELLITROL_struct.drive_disable_flag))
    {
        INTELLITROL_struct.rgb_led_color = INTELLITROL_FAULT_COLOR; // Change RGB LED color variable
        INTELLITROL_struct.intellitrol_fault = INTELLITROL_BATTERY_UNDERVOLTAGE_FAULT;
        INTELLITROL_struct.drive_disable_flag = 1;
    } 

    // Motor current monitor fault
    else if (((INTELLITROL_struct.motor_1_current > INTELLITROL_struct.motor_overcurrent_threshold) || 
            (INTELLITROL_struct.motor_2_current > INTELLITROL_struct.motor_overcurrent_threshold)) &&
            (!INTELLITROL_struct.drive_disable_flag))
    {
        INTELLITROL_struct.rgb_led_color = INTELLITROL_FAULT_COLOR; // Change RGB LED color variable
        INTELLITROL_struct.intellitrol_fault = INTELLITROL_OVERCURRENT_FAULT;  
        INTELLITROL_struct.drive_disable_flag = 1;
    }

    // No fault
    else 
    {   
        if (!INTELLITROL_struct.drive_disable_flag)
        {
            INTELLITROL_struct.intellitrol_fault = INTELLITROL_NO_FAULT; 
        }
    }

    // If the user decided to override the disable feature, the fault does not stop
    // the DRV8873 motor driver.
    // If the used did no override the disable feature, any fault causes the DRV8873
    // motor driver to stop and sets the DEBUG LED to permanent ON
    
    // If the disable feature is not overridden AND the fault_clear bit is not set
    if ((INTELLITROL_struct.drive_disable_override == INTELLITROL_OVERRIDE_OFF) && (!INTELLITROL_struct.fault_clear))
    {
        // If there is an active fault
        if (INTELLITROL_struct.intellitrol_fault != INTELLITROL_NO_FAULT)
        {
            if (INTELLITROL_struct.drive_disable_flag)
            {
                DRV8873_disable();  // Disable the motor driver
                INTELLITROL_struct.debug_led_override = 1;
            }
        }
    }

    // User wants to clear faults (I.E sent a fault_clear command)
    if (INTELLITROL_struct.fault_clear)
    {
        INTELLITROL_assert_fault();
        switch(INTELLITROL_struct.new_mode)
        {
            case INPUT_MODE_I2C:
                INTELLITROL_struct.rgb_led_color = GREEN;
                //INTELLITROL_struct.motor_speed_mode = SET_SPEED_IN_RPM; 
                break;
                
            case INPUT_MODE_UART:
                INTELLITROL_struct.rgb_led_color = RED;
                //INTELLITROL_struct.motor_speed_mode = SET_SPEED_IN_RPM; 
                break;
                
            case INPUT_MODE_ANALOG:
                INTELLITROL_struct.rgb_led_color = PURPLE; 
                //INTELLITROL_struct.motor_speed_mode = SET_SPEED_IN_PERC;             
                break;
                
            case INPUT_MODE_SPI:
                INTELLITROL_struct.rgb_led_color = BLUE; 
                //INTELLITROL_struct.motor_speed_mode = SET_SPEED_IN_RPM; 
                break;
        }
        INTELLITROL_rgb_monitor();
        INTELLITROL_struct.drive_disable_flag = 0;       
        // If override bit was not set, reset motor status to off
        if (INTELLITROL_struct.drive_disable_override != INTELLITROL_OVERRIDE_ON)
        {
            INTELLITROL_struct.motor_movement_pulse = 0;
            INTELLITROL_struct.motor_speed_rpm = 0;            
            INTELLITROL_struct.pid_out = 0;
            INTELLITROL_struct.motor_speed_perc = 0;
        }
        
    }
}

//******************void INTELLITROL_assert_fault (void)**********************//
//Description : Function asserts the fault monitor by clearing the debug LED
//              override, clearing the fault_clear flag and physically asserting
//              the DRV8873
//              After the fault assert, the functions reads back the diag and
//              fault register of the DRV8873
//
//Function prototype : void INTELLITROL_fault_monitor (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : INTELLITROL_fault_monitor();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void INTELLITROL_assert_fault (void)
{
    INTELLITROL_struct.fault_clear = 0;                         // Clear the fault flag       
    INTELLITROL_struct.debug_led_override = 0;                  // Clear DEBUG LED override       
    DRV8873_assert_fault();                                     // Automatically assert fault on DRV8873
    DRV8873_enable();
    INTELLITROL_struct.intellitrol_fault = INTELLITROL_NO_FAULT;// Clear the fault variable      
}

//****************void INTELLITROL_clear_rx_buffer (void)*********************//
//Description : Function clears the content of the rx buffer to all 0s
//
//Function prototype : void INTELLITROL_clear_rx_buffer (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : INTELLITROL_clear_rx_buffer();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void INTELLITROL_clear_rx_buffer (void)
{
    unsigned char i = 0;
    for (i=0; i < DRIVE_DATA_LENGTH; i++)
    {
        INTELLITROL_struct.rx_data[i] = 0;
    }
}

//unsigned char INTELLITROL_analog_get_percentage (unsigned char dir, unsigned int analog_value)//
//Description : Function returns the percentage value (0-100) associated with the
//              analog value read by the ADC and the wanted direction of the motor
//              The maths work this way :
//
//              ^  AVDD, FW, speed max (100% on in forward)
//              |  FW, speed ++
//              |  FW, speed ++
//              |  FW, speed ++ 
//              -  ANALOG_DEFAULT_OFF (0%, off)
//              |  BW, speed ++            
//              |  BW, speed ++
//              |  BW, speed ++
//             --- AVSS, BW, speed max (100% on in backward)
//              
//Function prototype : unsigned char INTELLITROL_analog_get_percentage (unsigned char dir, unsigned int analog_value)
//
//Enter params       : unsigned char dir : Wanted direction of the motor
//                   : unsigned int analog_value : Analog input channel value
//
//Exit params        : unsigned char : percentage (0 - 100)
//
//Function call      : unsigned char = INTELLITROL_analog_get_percentage(DIRECTION_FORWARD, INTELLITROL_struct.analog_value);
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
unsigned char INTELLITROL_analog_get_percentage (unsigned char dir, unsigned int analog_value)
{
    unsigned long temp = 0;
    if (dir == DIRECTION_FORWARD)
    {
        temp = (((analog_value - ANALOG_DEFAULT_OFF) * 100.0) / ANALOG_DEFAULT_OFF);
        return (unsigned char)(temp);
    }
    
    else if (dir == DIRECTION_BACKWARD)
    {
        temp = (((ANALOG_DEFAULT_OFF - analog_value) * 100.0) / ANALOG_DEFAULT_OFF);
        return (unsigned char)(temp);
    }
    else return 0;
}

//*****************unsigned char INTELLITROL_get_mode (void)******************//
//Description : Function reads the mode input switches and return the actual
//              mode
//              
//Function prototype : unsigned char INTELLITROL_get_mode (void)
//
//Enter params       : None
//
//Exit params        : unsigned char : actual operating mode, from 0-3
//
//Function call      : unsigned char = INTELLITROL_get_mode();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
unsigned char INTELLITROL_get_mode (void)
{
    //Slave SPI mode, 3
    if ((USER_IN1 == 1) && (USER_IN2 == 1))
    {
        return INPUT_MODE_SPI;
    }
    //Analog input mode, 2
    else if ((USER_IN1 == 0) && (USER_IN2 == 1))
    {
        return INPUT_MODE_ANALOG;
    }
    //UART mode, 1
    else if ((USER_IN1 == 1) && (USER_IN2 == 0))
    {
        return INPUT_MODE_UART;
    }
    //Slave I2C mode, 0
    else if ((USER_IN1 == 0) && (USER_IN2 == 0))
    {
        return INPUT_MODE_I2C;
    }
        
    else 
    {
        return 255;
    }
}

//*********unsigned char INTELLITROL_rgb_convert (unsigned char value)********//
//Description : Function scales a 0-FF hex variable to a 0-100 scale
//              
//Function prototype : unsigned char INTELLITROL_rgb_convert (unsigned char value)
//
//Enter params       : unsigned char value : Value to scale down
//
//Exit params        : unsigned char : scaled down variable value
//
//Function call      : unsigned char = INTELLITROL_get_mode(0xF5);
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
unsigned char INTELLITROL_rgb_convert (unsigned char value)
{
    unsigned char temp = 0;
    if (value <= 3){return 0;}
    else if (value >= 255){return 100;}
    else
    {
        temp = (unsigned char)((float)(value / 2.55));
        return temp;
    }
}

//**************unsigned char INTELLITROL_rgb_monitor (void)******************//
//Description : Function modifies the RGB LED color if the RGB override is not
//              in effect
//              
//Function prototype : unsigned char INTELLITROL_rgb_monitor (void)
//
//Enter params       : None
//
//Exit params        : unsigned char : override status
//
//Function call      : unsigned char = INTELLITROL_rgb_monitor();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
unsigned char INTELLITROL_rgb_monitor (void)
{
    if (!INTELLITROL_struct.rgb_led_override)
    {
        INTELLITROL_struct.rgb_led_red = INTELLITROL_rgb_convert((INTELLITROL_struct.rgb_led_color>>16) & 0xFF);
        INTELLITROL_struct.rgb_led_green = INTELLITROL_rgb_convert((INTELLITROL_struct.rgb_led_color>>8) & 0xFF);
        INTELLITROL_struct.rgb_led_blue = INTELLITROL_rgb_convert(INTELLITROL_struct.rgb_led_color&0xFF);
        PWM_change_duty(PWM_5H, INTELLITROL_struct.rgb_led_blue);       // BLUE
        PWM_change_duty(PWM_6L, INTELLITROL_struct.rgb_led_green);      // GREEN
        PWM_change_duty(PWM_6H, INTELLITROL_struct.rgb_led_red);        // RED
        return 0;
    }
    
    else
    {
        // Override is in effect
        PWM_change_duty(PWM_5H, 100);   // Turn the RGB BLUE LED off
        PWM_change_duty(PWM_6L, 100);   // Turn the RGB GREEN LED off
        PWM_change_duty(PWM_6H, 100);   // Turn the RGB RED LED off 
        return 1;
    }
}

//*************unsigned char INTELLITROL_debug_toggle (void)******************//
//Description : Function controls the debug led according to the override bit
//              With override off, the debug LED flashes at a certain rate
//              With override on, the debug LEG stays to the ON state
//              
//Function prototype : unsigned char INTELLITROL_debug_toggle (void)
//
//Enter params       : None
//
//Exit params        : unsigned char : override status
//
//Function call      : unsigned char = INTELLITROL_debug_toggle();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
unsigned char INTELLITROL_debug_toggle (void)
{
    if (!INTELLITROL_struct.debug_led_override)
    {
        DEBUG_LED = !DEBUG_LED;
        INTELLITROL_struct.debug_led = DEBUG_LED;
        return 0;       // DEBUG override not in effect, function return 0
    }
    else 
    {
        DEBUG_LED = LED_ON;     // FAULT, do not toggle the DEBUG LED
        return 1;               // DEBUG override is in effect, function return 1
    }
}

//********************void INTELLITROL_drive_analog (void)********************//
//Description : Function controls the PWM signals sent to the DRV8873 to control
//              the speed of the motor
//
//              ^  AVDD, FW, speed max (100% on in forward)
//              |  FW, speed ++
//              |  FW, speed ++
//              |  FW, speed ++ 
//              -  ANALOG_IDLE_THRESHOLD (variable)
//              -  ANALOG_DEFAULT_OFF (0%, off)
//              -  ANALOG_IDLE_THRESHOLD (variable)
//              |  BW, speed ++            
//              |  BW, speed ++
//              |  BW, speed ++
//             --- AVSS, BW, speed max (100% on in backward)
//              
//Function prototype : void INTELLITROL_drive_analog (void)
//
//Enter params       : None
//
//Exit params        : None
//
//Function call      : INTELLITROL_drive_analog();
//
//Jean-Francois Bilodeau    MPLab X v5.10    11/02/2020 
//****************************************************************************//
void INTELLITROL_drive_analog (void)
{
    // If the analog input value is inside the IDLE threshold
    if ((INTELLITROL_struct.analog_value <= (ANALOG_DEFAULT_OFF - INTELLITROL_struct.analog_idle_threshold))
       && (INTELLITROL_struct.analog_value >= (ANALOG_DEFAULT_OFF + INTELLITROL_struct.analog_idle_threshold)))
    {
        PWM_change_duty(PWM_4H, 0); // Stop the drive
        PWM_change_duty(PWM_4L, 0); // Stop the drive
    } 
    // If the analog value is in the forward direction
    if (INTELLITROL_struct.analog_value > (INTELLITROL_struct.analog_idle_threshold + ANALOG_DEFAULT_OFF))
    {
        INTELLITROL_struct.motor_direction = DIRECTION_FORWARD;
        PWM_change_duty(PWM_4H, INTELLITROL_analog_get_percentage(INTELLITROL_struct.motor_direction, INTELLITROL_struct.analog_value));
        PWM_change_duty(PWM_4L, 0);
    }
    // If the analog value is in the backward direction
    // value is between 0 and ((3.3/2) - INTELLITROL_struct.analog_idle_threshold)
    if (INTELLITROL_struct.analog_value < (ANALOG_DEFAULT_OFF - INTELLITROL_struct.analog_idle_threshold))
    {
        INTELLITROL_struct.motor_direction = DIRECTION_BACKWARD;
        PWM_change_duty(PWM_4H, 0);
        PWM_change_duty(PWM_4L, INTELLITROL_analog_get_percentage(INTELLITROL_struct.motor_direction, INTELLITROL_struct.analog_value));        
    }     
}



void INTELLITROL_control (unsigned char mode)
{
    unsigned char uc_temp = 0;
    unsigned int ui_temp = 0;
    if ((mode == INPUT_MODE_I2C)||(mode == INPUT_MODE_UART)||(mode == INPUT_MODE_SPI))
    {
        // If the address is valid
        if (INTELLITROL_struct.rx_data[DRIVE_ADDRESS_INDEX] == INTELLITROL_struct.drive_address)
        {
            // Look at the 1st byte : command byte
            switch (INTELLITROL_struct.rx_data[DRIVE_CONTROL1_INDEX])
            {
                // Set new direction, 0 = Forward, 1 = Backward
                case COM_CTRL1_DIRECTION:
                    INTELLITROL_struct.motor_direction = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX] & 0x01;
                    break;                
                
                // Set new speed, in a scale of 0-100 percent
                // Limit the variable received to 100
                case COM_CTRL1_SPEED_PERC:
                    INTELLITROL_struct.motor_speed_perc = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX];
                    if (INTELLITROL_struct.motor_speed_perc > 100){INTELLITROL_struct.motor_speed_perc = 100;}
                    break;  
                 
                // Set new speed, in a scale to 0-65535 rpm
                case COM_CTRL1_SPEED_RPM:
                    INTELLITROL_struct.motor_speed_rpm = ((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX] << 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]);                 
                    if (INTELLITROL_struct.motor_speed_rpm > INTELLITROL_struct.motor_max_rpm){INTELLITROL_struct.motor_speed_rpm = INTELLITROL_struct.motor_max_rpm;}
                    break;

                // Change the speed mode, RPM mode = 0, PERC mode = 1, PULSE mode = 2
                case COM_CTRL1_SPEED_MODE:
                    uc_temp = INTELLITROL_struct.motor_speed_mode;
                    INTELLITROL_struct.motor_speed_mode = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX];
                    if (INTELLITROL_struct.motor_speed_mode != uc_temp)
                    {
                        // Stop any motor movement after a mode change
                        INTELLITROL_struct.motor_speed_rpm = 0;
                        INTELLITROL_struct.motor_speed_perc = 0;
                        INTELLITROL_struct.motor_movement_pulse = 0;
                        INTELLITROL_struct.motor_speed_perc = 0;
                    }
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_byte(MOTOR_SPEED_MODE_OFFSET, INTELLITROL_struct.motor_speed_mode);
                    }                    
                    break;
                    
                // Set a new PID P gain, in a scale to 0-65535
                case COM_CTRL1_PID_P_GAIN:
                    PID_struct.p_gain = ((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]);
                    INTELLITROL_pid_set_gains();
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_integer(PID_P_GAIN_OFFSET, PID_struct.p_gain);
                    }                    
                    break;

                // Set a new PID I gain, in a scale to 0-65535
                case COM_CTRL1_PID_I_GAIN:
                    PID_struct.i_gain = ((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]);
                    INTELLITROL_pid_set_gains();
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_integer(PID_I_GAIN_OFFSET, PID_struct.i_gain);
                    }                       
                    break;

                case COM_CTRL1_DRIVE_DISABLE_OVRRD:
                    INTELLITROL_struct.drive_disable_override = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX] & 0x01;
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_byte(LATCH_OVERRIDE_OFFSET, INTELLITROL_struct.drive_disable_override);
                    }  
                    break;
                    
                // Change DRV8873 IC1 register parameter
                case COM_CTRL1_DRIVE_IC1:
                    INTELLITROL_struct.drv8873_ic1 = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX];
                    DRV8873_write(IC1_CONTROL_REGISTER, INTELLITROL_struct.drv8873_ic1);
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_byte(DRV8873_IC1_OFFSET, INTELLITROL_struct.drv8873_ic1);
                    } 
                    break; 
                 
                // Change DRV8873 IC2 register parameter
                case COM_CTRL1_DRIVE_IC2:
                    INTELLITROL_struct.drv8873_ic2 = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX];
                    DRV8873_write(IC2_CONTROL_REGISTER, INTELLITROL_struct.drv8873_ic2);
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_byte(DRV8873_IC2_OFFSET, INTELLITROL_struct.drv8873_ic2);
                    }                    
                    break; 

                // Change DRV8873 IC register parameter
                case COM_CTRL1_DRIVE_IC3:
                    INTELLITROL_struct.drv8873_ic3 = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX];
                    DRV8873_write(IC3_CONTROL_REGISTER, INTELLITROL_struct.drv8873_ic3);
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_byte(DRV8873_IC3_OFFSET, INTELLITROL_struct.drv8873_ic3);
                    }                          
                    break; 
                
                // Change DRV8873 IC4 register parameter
                case COM_CTRL1_DRIVE_IC4:
                    INTELLITROL_struct.drv8873_ic4 = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX];
                    DRV8873_write(IC4_CONTROL_REGISTER, INTELLITROL_struct.drv8873_ic4);
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_byte(DRV8873_IC4_OFFSET, INTELLITROL_struct.drv8873_ic4);
                    }                          
                    break;                

                // Set a new motor current threshold, which should not exceed ((2^12) - 1)
                case COM_CTRL1_CURR_LIMIT:
                    INTELLITROL_struct.motor_overcurrent_threshold = (((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]) & 0x0FFF);
                    if (INTELLITROL_struct.motor_overcurrent_threshold > 0xFFF){INTELLITROL_struct.motor_overcurrent_threshold = 0xFFF;}
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_integer(CURRENT_LIMIT_OFFSET, INTELLITROL_struct.motor_overcurrent_threshold);
                    }                     
                    break; 

                // Set a new BATT low threshold, which should not be below 0
                case COM_CTRL1_BATT_LOW_THRES:
                    INTELLITROL_struct.battery_low_threshold = (((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]) & 0x0FFF);
                    if (INTELLITROL_struct.battery_low_threshold > 0xFFF){INTELLITROL_struct.battery_low_threshold = 0xFFF;}
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_integer(BAT_LOW_OFFSET, INTELLITROL_struct.battery_low_threshold);
                    }                     
                    break; 
                    
                // Set a new BATT high threshold, which should not exceed ((2^12) - 1)
                case COM_CTRL1_BATT_HIGH_THRES:
                    INTELLITROL_struct.battery_high_threshold = (((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]) & 0x0FFF);
                    if (INTELLITROL_struct.battery_high_threshold > 0xFFF){INTELLITROL_struct.battery_high_threshold = 0xFFF;}
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_integer(BAT_HIGH_OFFSET, INTELLITROL_struct.battery_high_threshold);
                    }                      
                    break;  
                    
                // Set a new BATT high threshold, which should not exceed ((2^12) - 1)
                case COM_CTRL1_ANALOG_IDLE_THRES:
                    INTELLITROL_struct.analog_idle_threshold = (((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]) & 0x0FFF);
                    if (INTELLITROL_struct.analog_idle_threshold > 0xFFF){INTELLITROL_struct.analog_idle_threshold = 0xFFF;}
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_integer(ANALOG_IDLE_OFFSET, INTELLITROL_struct.analog_idle_threshold);
                    }                        
                    break; 
                    
                // Set a new motor CPR characteristic
                case COM_CTRL1_MOTOR_QEI_CPR:
                    INTELLITROL_struct.motor_cpr = ((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]);
                    QEI_set_cpr(QEI_1, INTELLITROL_struct.motor_cpr); 
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_integer(QEI_CPR_OFFSET, INTELLITROL_struct.motor_cpr);
                    }                    
                    break;
                    
                // Set a new motor Gear Derate characteristic
                case COM_CTRL1_MOTOR_QEI_GDR:
                    INTELLITROL_struct.motor_gear_derate = ((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]);
                    QEI_set_gear_derate(QEI_1, INTELLITROL_struct.motor_gear_derate);
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_integer(QEI_GDR_OFFSET, INTELLITROL_struct.motor_gear_derate);
                    }                        
                    break;

                // Set a new motor MAX rpm value
                case COM_CTRL1_MOTOR_MAX_RPM:
                    INTELLITROL_struct.motor_max_rpm = ((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]);
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_integer(MOTOR_MAX_RPM_OFFSET, INTELLITROL_struct.motor_max_rpm);
                    }                     
                    break;
                    
                case COM_CTRL1_MOVEMENT_PULSE:
                    INTELLITROL_struct.motor_movement_pulse = ((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]);
                    break;

                case COM_CTRL1_MOVEMENT_STATE:
                    INTELLITROL_struct.motor_movement_state = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX] & 0x01;
                    if (INTELLITROL_struct.motor_movement_state == MOTOR_MOVEMENT_STARTED)
                    {
                        QEI_reset_distance(QEI_1);
                        QEI_reset_tour(QEI_1);
                        //QEI_reset_cm(QEI_1);
                    }
                    else
                    {
                        INTELLITROL_struct.motor_speed_rpm = 0;
                    }
                    break; 
                    
                case COM_CTRL1_MOVEMENT_PEAK_SPEED:
                    INTELLITROL_struct.motor_movement_peak_speed = ((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]);
                    if (INTELLITROL_struct.motor_movement_peak_speed > INTELLITROL_struct.motor_max_rpm){INTELLITROL_struct.motor_movement_peak_speed = INTELLITROL_struct.motor_max_rpm;}
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_integer(MOVEMENT_PEAK_SPEED_OFFSET, INTELLITROL_struct.motor_movement_peak_speed);
                    }                     
                    break;
                    
                case COM_CTRL1_MOVEMENT_TOUR:
                    INTELLITROL_struct.motor_movement_tour = ((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]);
                    INTELLITROL_struct.motor_movement_pulse = (INTELLITROL_struct.motor_movement_tour * QEI_struct[QEI_1].pulse_per_tour);
                    break;
                    
                case COM_CTRL1_KILLSWITCH_1_ENABLE:
                    INTELLITROL_struct.killswitch_1_enable = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX] & 0x01;
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_integer(KSW1_ENABLE_OFFSET, INTELLITROL_struct.killswitch_1_enable);
                    }                    
                    break;
                    
                case COM_CTRL1_KILLSWITCH_2_ENABLE:
                    INTELLITROL_struct.killswitch_2_enable = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX] & 0x01;
                    if (INTELLITROL_struct.eeprom_enable == 1)
                    {
                        EEPROM_write_integer(KSW2_ENABLE_OFFSET, INTELLITROL_struct.killswitch_2_enable);
                    }                     
                    break;
                    
                case COM_CTRL1_EEPROM_STATE:
                    INTELLITROL_struct.eeprom_enable = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX] & 0x01;                        
                    break;                    

                //case COM_CTRL1_SET_MOVEMENT_CM:
                //    INTELLITROL_struct.motor_movement_cm = ((INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX]<< 8) | INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX]);
                //    break; 
                    
                // Write-only variable
                // Set the fault clear flag, will assert faults from Intellitrol and DRV8873
                case COM_CTRL1_CLEAR_FAULTS:
                    INTELLITROL_struct.fault_clear = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX] & 0x01;
                    break;     
                   
                // Special functions
                // Change the Intellitrol address
                case CHANGE_DRIVE_ADDRESS:
                    if (INTELLITROL_struct.rx_data[DRIVE_CONTROL2_DATA1_INDEX] == CHANGE_DRIVE_ADDRESS_BYTE)
                    {
                        // Successful address change
                        INTELLITROL_struct.drive_address = INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX];
                        I2C_init(INTELLITROL_I2C, I2C_mode_slave, INTELLITROL_struct.drive_address); // Re-initialize I2C module with new address
                    }
                    break;                    
                
                // Request a read operation
                case DRIVE_READ_COMMAND:
                    switch (INTELLITROL_struct.rx_data[DRIVE_DATA2_INDEX])
                    {
                        // Write / read variables
                        case COM_CTRL1_NULL:
                            INTELLITROL_fill_transmit_buffer(0, 0, 0);
                            break;
                            
                        case COM_CTRL1_DIRECTION:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_DIRECTION, 0, INTELLITROL_struct.motor_direction);
                            break;                            
                            
                        case COM_CTRL1_SPEED_PERC:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_SPEED_PERC, 0, INTELLITROL_struct.motor_speed_perc);
                            break; 

                        case COM_CTRL1_SPEED_RPM:           
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_SPEED_RPM, ((INTELLITROL_struct.motor_speed_rpm >> 8)&0x00FF), INTELLITROL_struct.motor_speed_rpm);
                            break;
                            
                        case COM_CTRL1_SPEED_MODE:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_SPEED_MODE, 0, INTELLITROL_struct.motor_speed_mode);
                            break;

                        case COM_CTRL1_PID_P_GAIN:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_PID_P_GAIN, ((PID_struct.p_gain >> 8)&0x00FF), PID_struct.p_gain);                    
                            break;

                        case COM_CTRL1_PID_I_GAIN:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_PID_I_GAIN, (((unsigned int)PID_struct.i_gain >> 8)&0x00FF), (unsigned int)PID_struct.i_gain); 
                            break;                         

                        case COM_CTRL1_DRIVE_DISABLE_OVRRD:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_DRIVE_DISABLE_OVRRD, 0, INTELLITROL_struct.drive_disable_override);
                            break;
                            
                        case COM_CTRL1_DRIVE_IC1:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_DRIVE_IC1, 0, INTELLITROL_struct.drv8873_ic1);
                            break; 

                        case COM_CTRL1_DRIVE_IC2:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_DRIVE_IC2, 0, INTELLITROL_struct.drv8873_ic2);
                            break; 

                        case COM_CTRL1_DRIVE_IC3:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_DRIVE_IC3, 0, INTELLITROL_struct.drv8873_ic3);
                            break; 

                        case COM_CTRL1_DRIVE_IC4:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_DRIVE_IC4, 0, INTELLITROL_struct.drv8873_ic4);
                            break;                            

                        case COM_CTRL1_CURR_LIMIT:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_CURR_LIMIT, ((INTELLITROL_struct.motor_overcurrent_threshold >> 8)&0x00FF), INTELLITROL_struct.motor_overcurrent_threshold);
                            break;

                        case COM_CTRL1_BATT_LOW_THRES:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_BATT_LOW_THRES, ((INTELLITROL_struct.battery_low_threshold >> 8)&0x00FF), INTELLITROL_struct.battery_low_threshold);
                            break;
                            
                        case COM_CTRL1_BATT_HIGH_THRES:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_BATT_HIGH_THRES, ((INTELLITROL_struct.battery_high_threshold >> 8)&0x00FF), INTELLITROL_struct.battery_high_threshold);
                            break;
                            
                        case COM_CTRL1_ANALOG_IDLE_THRES:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_ANALOG_IDLE_THRES, ((INTELLITROL_struct.analog_idle_threshold >> 8)&0x00FF), INTELLITROL_struct.analog_idle_threshold);
                            break;
                            
                        case COM_CTRL1_MOTOR_QEI_CPR:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_MOTOR_QEI_CPR, ((INTELLITROL_struct.motor_cpr >> 8)&0x00FF), INTELLITROL_struct.motor_cpr);
                            break;

                        case COM_CTRL1_MOTOR_QEI_GDR:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_MOTOR_QEI_GDR, ((INTELLITROL_struct.motor_gear_derate >> 8)&0x00FF), INTELLITROL_struct.motor_gear_derate);
                            break; 
                            
                        case COM_CTRL1_MOTOR_MAX_RPM:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_MOTOR_MAX_RPM, ((INTELLITROL_struct.motor_max_rpm >> 8)&0x00FF), INTELLITROL_struct.motor_max_rpm);
                            break;
                            
                        case COM_CTRL1_MOVEMENT_PULSE:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_MOVEMENT_PULSE, ((INTELLITROL_struct.motor_movement_pulse >> 8)&0x00FF), INTELLITROL_struct.motor_movement_pulse);
                            break;
                            
                        case COM_CTRL1_MOVEMENT_TOUR:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_MOVEMENT_TOUR, ((INTELLITROL_struct.motor_movement_tour >> 8)&0x00FF), INTELLITROL_struct.motor_movement_tour);
                            break;
                            
                        case COM_CTRL1_MOVEMENT_STATE:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_MOVEMENT_STATE, 0, INTELLITROL_struct.motor_movement_state);
                            break;                        
                            
                        case COM_CTRL1_MOVEMENT_PEAK_SPEED: 
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_MOVEMENT_PEAK_SPEED, ((INTELLITROL_struct.motor_movement_peak_speed >> 8)&0x00FF), INTELLITROL_struct.motor_movement_peak_speed);
                            break;
                            
                        case COM_CTRL1_EEPROM_STATE:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL1_EEPROM_STATE, 0, INTELLITROL_struct.eeprom_enable);                            
                            break;
                            
                        // Read-only variables
                        // Fault from intellitrol
                        case COM_CTRL2_INTELLITROL_FAULT:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_INTELLITROL_FAULT, 0, INTELLITROL_struct.intellitrol_fault);
                            break;
                         
                        case COM_CTRL2_DRV8873_FAULT:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_DRV8873_FAULT, 0, INTELLITROL_struct.drv8873_fault);
                            break;
                            
                        case COM_CTRL2_DRV8873_DIAG:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_DRV8873_DIAG, 0, INTELLITROL_struct.drv8873_diag);
                            break;
                            
                        case COM_CTRL2_MOTOR_1_CURRENT:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_MOTOR_1_CURRENT, ((INTELLITROL_struct.motor_1_current >> 8)&0x00FF), INTELLITROL_struct.motor_1_current);
                            break;

                        case COM_CTRL2_MOTOR_2_CURRENT:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_MOTOR_2_CURRENT, ((INTELLITROL_struct.motor_2_current >> 8)&0x00FF), INTELLITROL_struct.motor_2_current);
                            break;                            
                            
                        case COM_CTRL2_BATTERY_VALUE:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_BATTERY_VALUE, ((INTELLITROL_struct.battery_value >> 8)&0x00FF), INTELLITROL_struct.battery_value);
                            break;

                        case COM_CTRL2_ANALOG_VALUE:
                            INTELLITROL_struct.analog_value = ADC_get_channel(ADC_ANALOG_INPUT_CHANNEL);
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_ANALOG_VALUE, ((INTELLITROL_struct.analog_value >> 8)&0x00FF), INTELLITROL_struct.analog_value);
                            break;                            
                            
                        case COM_CTRL2_PID_RPM_ERROR:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_PID_RPM_ERROR, (((unsigned int)PID_struct.error_rpm >> 8)&0x00FF), (unsigned int)PID_struct.error_rpm);
                            break;

                        case COM_CTRL2_PID_SUM_ERROR:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_PID_SUM_ERROR, (((unsigned int)PID_struct.i_term >> 8)&0x00FF), (unsigned int)PID_struct.i_term);
                            break; 
                            
                        case COM_CTRL2_PID_P_VALUE:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_PID_P_VALUE, (((unsigned int)PID_struct.p_value >> 8)&0x00FF), (unsigned int)PID_struct.p_value);
                            break;

                        case COM_CTRL2_PID_I_VALUE:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_PID_I_VALUE, (((unsigned int)PID_struct.i_value >> 8)&0x00FF), (unsigned int)PID_struct.i_value);
                            break;
                            
                        case COM_CTRL2_PID_OUT:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_PID_OUT, 0, (unsigned char)PID_struct.pid_out);
                            break;                        

                        case COM_CTRL2_ACTUAL_SPEED_RPM:                      
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_ACTUAL_SPEED_RPM, ((PID_struct.actual_rpm >> 8)&0x00FF), PID_struct.actual_rpm);
                            break; 
                            
                        case COM_CTRL2_ACTUAL_MOVEMENT_PULSE:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_ACTUAL_MOVEMENT_PULSE, ((INTELLITROL_struct.motor_actual_pulse >> 8)&0x00FF), INTELLITROL_struct.motor_actual_pulse);
                            break;
                            
                        case COM_CTRL2_ACTUAL_MOVEMENT_TOUR:       
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_ACTUAL_MOVEMENT_TOUR, ((INTELLITROL_struct.motor_actual_tour >> 8)&0x00FF), INTELLITROL_struct.motor_actual_tour);
                            break;
                            
                        case COM_CTRL2_ACTUAL_MOVEMENT_CM:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_ACTUAL_MOVEMENT_CM, 0, 0);
                            break;
                        
                        case COM_CTRL2_KILLSWITCH_1_STATE:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_KILLSWITCH_1_STATE, 0, INTELLITROL_struct.killswitch_1_state);
                            break;
                            
                        case COM_CTRL2_KILLSWITCH_2_STATE:
                            INTELLITROL_fill_transmit_buffer(COM_CTRL2_KILLSWITCH_2_STATE, 0, INTELLITROL_struct.killswitch_2_state);
                            break;                  
                            
                        case DRIVE_GET_OPERATING_MODE:
                            INTELLITROL_fill_transmit_buffer(DRIVE_GET_OPERATING_MODE, 0, INTELLITROL_struct.new_mode);
                            break;
                            
                    } 

                    switch (mode)
                    {
                        case INPUT_MODE_I2C:
                            I2C_fill_transmit_buffer(INTELLITROL_I2C, INTELLITROL_struct.tx_ptr, DRIVE_DATA_LENGTH);
                            break;

                        case INPUT_MODE_UART:
                            UART_fill_transmit_buffer(INTELLITROL_UART, INTELLITROL_struct.tx_ptr, DRIVE_DATA_LENGTH);
                            UART_send_tx_buffer(INTELLITROL_UART);
                            break;

                        case INPUT_MODE_SPI:
                            SPI_fill_transmit_buffer(INTELLITROL_SPI, INTELLITROL_struct.tx_ptr, DRIVE_DATA_LENGTH);
                            break;
                    }                     
                break;                 
            }
            
            INTELLITROL_struct.rx_data[DRIVE_ADDRESS_INDEX] = 0;    // Wait for new command
        }                    
    }  
}