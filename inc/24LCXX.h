#ifndef __24LCXX_H__
#define	__24LCXX_H__

#include "general.h"

#define EEPROM_WAS_WRITTEN 1
#define EEPROM_NOT_WRITTEN 0

#define EEPROM_I2C_ADDRESS 0xA0
#define EEPROM_HEADER_ADDRESS 0
#define EEPROM_HEADER_LENGTH 16
#define EEPROM_DATA_OFFSET 16

#define QEI_CPR_OFFSET              16
#define QEI_GDR_OFFSET              18
#define MOTOR_MAX_RPM_OFFSET        20
#define MOVEMENT_PEAK_SPEED_OFFSET  22
#define PID_P_GAIN_OFFSET           24
#define PID_I_GAIN_OFFSET           26
#define KSW1_ENABLE_OFFSET          29
#define KSW2_ENABLE_OFFSET          31
#define LATCH_OVERRIDE_OFFSET       33
#define CURRENT_LIMIT_OFFSET        34
#define BAT_LOW_OFFSET              36
#define BAT_HIGH_OFFSET             38
#define ANALOG_IDLE_OFFSET          40
#define DRV8873_IC1_OFFSET          43
#define DRV8873_IC2_OFFSET          45
#define DRV8873_IC3_OFFSET          47
#define DRV8873_IC4_OFFSET          49
#define MOTOR_SPEED_MODE_OFFSET     51
#define DRIVE_ADDRESS_OFFSET        53

void EEPROM_write(unsigned int adr, unsigned char *ptr, unsigned char length);
unsigned char * EEPROM_read(unsigned int adr, unsigned char length);
void EEPROM_write_byte(unsigned int adr, unsigned char byte);
void EEPROM_write_integer(unsigned int adr, unsigned int value);
unsigned char * EEPROM_read_byte(unsigned int adr);
unsigned char * EEPROM_read_integer(unsigned int adr);
unsigned char EEPROM_was_written (void);
void EEPROM_write_header (void);
#endif