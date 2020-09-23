#ifndef __INTELLITROL_H__
#define __INTELLITROL_H__

#include "general.h"
#include "pwm.h"
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "ADC.h"
#include "DRV8873S.h"
#include "QEI.h"
#include "timer.h"
#include "24LCXX.h"

// Colors - fully saturated colors defined here
#define RED     0xFF0000		// Red      -- UART mode
#define GREEN	0x00FF00		// Green    -- I2C mode
#define BLUE	0x0000FF		// Blue     -- SPI mode
#define PURPLE  0xFF00FF        // Purple   -- Analog mode
#define PINK    0xFF04C9		// Pink     -- Intellitrol fault
#define CORAL   0x42DB96		// Coral    -- DRV8873 fault

#define YELLOW  0xFFFF00
#define ORANGE  0xFF8000
#define GREY    0x6E6E6E
#define FOREST  0x25844C
#define BURNT   0xDFA44C
#define BROWN   0x5F4B2F
#define MAROON  0x6D1313
#define SKIN    0xF8BFBF

// intellitrol general defines
#define DRIVE_WRITE 0
#define DRIVE_READ 1
#define DRIVE_ADDRESS_INDEX 0
#define DRIVE_CONTROL1_INDEX 1
#define DRIVE_CONTROL2_DATA1_INDEX 2
#define DRIVE_DATA2_INDEX 3
#define DRIVE_CHECKSUM_INDEX 4
#define STATE_REFRESH_FREQUENCY 50000

// intellitrol control defines
#define DIRECTION_FORWARD 0
#define DIRECTION_BACKWARD 1
#define SET_SPEED_IN_RPM 0
#define SET_SPEED_IN_PERC 1
#define SET_SPEED_IN_PULSE 2

#define MOTOR_MOVEMENT_STARTED 0
#define MOTOR_MOVEMENT_OVER 1

#define KILLSWITCH_IS_PRESSED 0
#define KILLSWITCH_NOT_PRESSED 1
#define KILLSWITCH_ENABLED 1
#define KILLSWITCH_DISABLED 0
#define EEPROM_ENABLED 1
#define EEPROM_DISABLED 0

// intellitrol modes defines
#define INPUT_MODE_I2C 0
#define INPUT_MODE_UART 1
#define INPUT_MODE_ANALOG 2
#define INPUT_MODE_SPI 3

#define CHANGE_DRIVE_ADDRESS_BYTE 0x5A

// intellitrol analog defines
#define ANALOG_DEFAULT_OFF 0x7FF
#define ANALOG_DEFAULT_MAX 0xFFF
#define ANALOG_DEFAULT_MIN 0
#define ANALOG_REFRESH_FREQUENCY 50000
#define ANALOG_DEFAULT_IDLE_THRESHOLD 20

// intellitrol battery defines
#define BATTERY_LOW_THRESHOLD 0x656         // Battery voltage is under 8V
#define BATTERY_HIGH_THRESHOLD 0xE05        // Battery voltage is over 18V

// intellitrol motor current monitor define
#define MOTOR_CURRENT_MAX_THRESHOLD 0x6BE   // Motor current exceeds 3.0A

// intellitrol faults defines
#define INTELLITROL_NO_FAULT 0
#define INTELLITROL_FAULT_RGB_OVERRIDE 1
#define INTELLITROL_FAULT_DEBUG_OVERRIDE 2
#define INTELLITROL_BATTERY_OVERVOLTAGE_FAULT 3
#define INTELLITROL_BATTERY_UNDERVOLTAGE_FAULT 4
#define INTELLITROL_OVERCURRENT_FAULT 5
#define INTELLITROL_COM_FAULT 6

#define INTELLITROL_DRV8873_FAULT_BASE 8

#define INTELLITROL_FAULT_COLOR ORANGE
#define DRV8873_FAULT_COLOR MAROON

// Intellitrol sampling defines
#define INTELLITROL_FAULT_FS 0
#define INTELLITROL_PID_FS 1

#define INTELLITROL_PID_REFRESH_FS 30
#define INTELLITROL_FAULT_REFRESH_FS 1024

// Timers assignation
#define FAULT_FS_TIMER          TIMER_2
#define PID_FS_TIMER            TIMER_6

#define INTELLITROL_OVERRIDE_OFF 0
#define INTELLITROL_OVERRIDE_ON 1

//**************************Intellitrol serial ports****************************
// intellitrol uart defines
#define INTELLITROL_UART UART_2

// intellitrol SPI defines
#define INTELLITROL_SPI SPI_2

// intellitrol QEI degine
#define INTELLITROL_QEI QEI_1

// intellitrol I2C defines
#define INTELLITROL_I2C I2C_port_1
#define MEMORY_I2C I2C_port_2

//**************************Intellitrol commands********************************
// intellitrol control 1 bytes options (read / write commands)
#define COM_CTRL1_NULL 0

// Every command that drives the motor, modifies it's operating mode or set it's
// parameters should be listed here 
#define COM_CTRL1_DIRECTION                 1   // Set / Get motor direction
#define COM_CTRL1_MOTOR_QEI_CPR             2   // Set / Get motor encoder count per revolution
#define COM_CTRL1_MOTOR_QEI_GDR             3   // Set / Get motor encoder gear derate ration
#define COM_CTRL1_MOTOR_MAX_RPM             4   // Set / Get motor maximum RPM
#define COM_CTRL1_SPEED_MODE                5   // Set / Get motor control mode
#define COM_CTRL1_SPEED_PERC                6   // Set / Get motor speed in % scale
#define COM_CTRL1_SPEED_RPM                 7   // Set / Get motor speed in RPM scale
#define COM_CTRL1_MOVEMENT_STATE            8   // Set / Get 
#define COM_CTRL1_MOVEMENT_PEAK_SPEED       9   // Set / Get motor movement peak speed
#define COM_CTRL1_MOVEMENT_PULSE            10  // Set / Get motor movement pulse #
#define COM_CTRL1_MOVEMENT_TOUR             11  // Set / Get motor movement tour #
#define COM_CTRL1_MOVEMENT_CM               12  // Set / Get motor movement CM #
#define COM_CTRL1_PID_P_GAIN                13  // Set / Get PID P gain, value internally divided by 1000.0
#define COM_CTRL1_PID_I_GAIN                14  // Set / Get PID I gain, value internally divided by 1000.0
#define COM_CTRL1_KILLSWITCH_1_ENABLE       15  // Set / Get killswitch enable status of killswitch 1
#define COM_CTRL1_KILLSWITCH_2_ENABLE       16  // Set / Get killswitch enable status of killswitch 2

// Every software threshold that sets a fault should be listed here
#define COM_CTRL1_DRIVE_DISABLE_OVRRD       17  // Set / Get drive latch fault override
#define COM_CTRL1_CURR_LIMIT                18  // Set / Get Motor current limit threshold
#define COM_CTRL1_BATT_LOW_THRES            19  // Set / Get battery low threshold
#define COM_CTRL1_BATT_HIGH_THRES           20  // Set / Get battery high threshold
#define COM_CTRL1_ANALOG_IDLE_THRES         21  // Set / Get intellitrol analog input threshold

// DRV8873 change default register value
#define COM_CTRL1_DRIVE_IC1                 22  // Set / Get DRV8873 IC1 register
#define COM_CTRL1_DRIVE_IC2                 23  // Set / Get DRV8873 IC2 register
#define COM_CTRL1_DRIVE_IC3                 24  // Set / Get DRV8873 IC3 register
#define COM_CTRL1_DRIVE_IC4                 25  // Set / Get DRV8873 IC4 register
#define COM_CTRL1_EEPROM_STATE              26  // Enable / Disable EEPROM save feature

// Write only commands
#define COM_CTRL1_CLEAR_FAULTS              127 // Clear Intellitrol fault flag

// Read only commands
#define COM_CTRL2_INTELLITROL_FAULT         128
#define COM_CTRL2_DRV8873_FAULT             129
#define COM_CTRL2_DRV8873_DIAG              130
#define COM_CTRL2_MOTOR_1_CURRENT           131
#define COM_CTRL2_MOTOR_2_CURRENT           132
#define COM_CTRL2_BATTERY_VALUE             133
#define COM_CTRL2_ANALOG_VALUE              134
#define COM_CTRL2_PID_RPM_ERROR             135
#define COM_CTRL2_PID_SUM_ERROR             136
#define COM_CTRL2_PID_P_VALUE               137
#define COM_CTRL2_PID_I_VALUE               138
#define COM_CTRL2_PID_OUT                   139
#define COM_CTRL2_ACTUAL_SPEED_RPM          140
#define COM_CTRL2_ACTUAL_MOVEMENT_PULSE     141
#define COM_CTRL2_ACTUAL_MOVEMENT_TOUR      142
#define COM_CTRL2_ACTUAL_MOVEMENT_CM        143
#define COM_CTRL2_KILLSWITCH_1_STATE        144
#define COM_CTRL2_KILLSWITCH_2_STATE        145

// Special functions
#define DRIVE_GET_OPERATING_MODE 253
#define CHANGE_DRIVE_ADDRESS 254
#define DRIVE_READ_COMMAND 255
//******************************************************************************

typedef struct
{
    // General Intellitrol variables
    unsigned char drive_address;
    unsigned char new_mode;
    unsigned char old_mode;
    unsigned char killswitch_1_state;
    unsigned char killswitch_2_state;
    unsigned char killswitch_1_enable;
    unsigned char killswitch_2_enable;
    unsigned char eeprom_enable;
    unsigned char eeprom_written;
    unsigned long rgb_led_color;
  
    // Direction and speed variables
    unsigned char motor_direction;
    unsigned char motor_speed_perc;
    unsigned int motor_speed_rpm;
    unsigned char motor_scaled_rpm;
    unsigned int motor_max_rpm;
    unsigned char motor_speed_mode;
    
    // PID variables
    unsigned int pid_p_gain;
    unsigned int pid_i_gain;
    unsigned int pid_fs;
    double pid_T;
    signed char pid_out;
    
    // Movement control
    unsigned long motor_actual_tour;
    unsigned long motor_movement_pulse;
    unsigned long motor_actual_pulse;
    unsigned char motor_movement_state;
    unsigned int motor_movement_peak_speed;
    unsigned int motor_movement_tour;
    unsigned int motor_movement_cm;
    
    // LED indicator variables
    unsigned char debug_led;
    unsigned char debug_led_override;
    unsigned char rgb_led_red;
    unsigned char rgb_led_green;
    unsigned char rgb_led_blue;
    unsigned char rgb_led_override;
    
    // Fault monitor variables
    unsigned int fault_fs;
    unsigned char intellitrol_fault;
    unsigned char drv8873_fault;
    unsigned char drv8873_diag;
    unsigned char drive_disable_override;
    unsigned char fault_clear;
    unsigned char drive_disable_flag;
    
    // Intellitrol data variables
    unsigned char rx_data[DRIVE_DATA_LENGTH];
    unsigned char * rx_ptr;
    unsigned char tx_data[DRIVE_DATA_LENGTH];
    unsigned char * tx_ptr;
    
    // Local DRV8873 variables
    unsigned char drv8873_ic1;
    unsigned char drv8873_ic2;
    unsigned char drv8873_ic3;
    unsigned char drv8873_ic4;
    
    // Intellitrol motor current variables
    unsigned int motor_overcurrent_threshold;
    unsigned int motor_1_current;
    unsigned int motor_2_current; 
    unsigned int motor_cpr;
    unsigned int motor_gear_derate;
    
    // Intellitrol battery variables
    unsigned int battery_value;
    unsigned int battery_low_threshold;
    unsigned int battery_high_threshold;
    
    // Intellitrol analog input variables
    unsigned int analog_value;
    unsigned char adc_first_data;
    unsigned int analog_idle_threshold; // Threshold around idle value
}STRUCT_INTELLITROL;

typedef struct
{   
    double pid_out;        // Scaled with drive_perc function (0-100%)
    
    double i_calc_gain;         // calculated i gain, i_gain / 1000
    double p_calc_gain;         // calculated p gain, p_gain / 1000
    unsigned int i_gain;        // integral gain set by user
    unsigned int p_gain;        // proportional gain set by user
    double i_value;          
    double p_value;     
    double i_term;
    unsigned int actual_rpm;// Scaled with acceptable RPM range (unsigned int)
    unsigned int last_actual_rpm;
    double last_error;
    double error_rpm;  //
    unsigned char pid_high_limit;    
}STRUCT_PID;

unsigned char INTELLITROL_rgb_convert (unsigned char value);
unsigned char INTELLITROL_rgb_monitor (void);
unsigned char INTELLITROL_debug_toggle (void);
unsigned char INTELLITROL_get_mode (void);
void INTELLITROL_store_data (unsigned char mode);
unsigned char INTELLITROL_fill_receive_buffer (void);
void INTELLITROL_control (unsigned char mode);
void INTELLITROL_init (void);
void INTELLITROL_restore (unsigned char mode);
unsigned char INTELLITROL_analog_get_percentage (unsigned char dir, unsigned int analog_value);
unsigned char INTELLITROL_get_fault_monitor_status (void);
void INTELLITROL_drive_perc (void);
void INTELLITROL_drive_analog (void);
void INTELLITROL_clear_rx_buffer (void);
void INTELLITROL_mode_change (void);
void INTELLITROL_fill_transmit_buffer (unsigned char c1, unsigned char c2_d1, unsigned char d2);
void INTELLITROL_fault_monitor (void);
void INTELLITROL_assert_fault (void);
void INTELLITROL_pid (void);
void INTELLITROL_movement (void);
void INTELLITROL_pid_set_gains (void);
void INTELLITROL_kill_switch (void);

#endif