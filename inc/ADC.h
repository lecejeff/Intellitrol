//***************************************************************************//
// File      : ADC.h
//
// Functions :  void ADC_init (void);
//              void ADC_start (void);
//              void ADC_stop (void);
//              unsigned char ADC_sample_status (void);
//              unsigned int ADC_get_channel (unsigned char channel);
//
// Includes  :  general.h
//           
// Purpose   :  Driver for the dsPIC33EP ADC core
//              ADC input on 4 seperate channels :
//              1) Motor 1 current sense
//              2) Motor 2 current sense
//              3) Intellitrol analog input control
//              4) Battery monitor
//
//Jean-Francois Bilodeau    MPLab X v5.10    10/02/2020  
//****************************************************************************//
#ifndef __ADC_H_
#define	__ADC_H_

#include "general.h"

#define ADC_MOTOR_1_CHANNEL 0
#define ADC_MOTOR_2_CHANNEL 1
#define ADC_ANALOG_INPUT_CHANNEL 2
#define ADC_BATTERY_CHANNEL 3
#define ADC_AVERAGE_LENGTH 1024

#define ADC_SAMPLE_READY 1
#define ADC_SAMPLE_NOT_READY 0

typedef struct
{
    unsigned long motor1_average;
    unsigned long motor2_average;
    unsigned long analog_in_average;
    unsigned long battery_monitor_average;
    unsigned int average_length;
    unsigned int average_counter;
}ADC_average_struct;

typedef struct
{
    unsigned int battery_voltage;
    unsigned int motor_1_current;
    unsigned int motor_2_current;
    unsigned int analog_input;
    unsigned char data_ready;
}STRUCT_ADC;

void ADC_init (void);
void ADC_start (void);
void ADC_stop (void);
unsigned char ADC_sample_status (void);
unsigned int ADC_get_channel (unsigned char channel);

#endif	

