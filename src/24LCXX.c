/*
 * File:   24LCXX.c
 * Author: jeanf
 *
 * Created on March 20, 2020, 5:29 PM
 */
#include "i2c.h"
#include "24LCXX.h"

extern STRUCT_I2C i2c_struct[I2C_MODULE_QTY];

unsigned char EEPROM_header[16] = {"INTELLITROLBLDC!"};
unsigned char response[2];

void EEPROM_write_byte(unsigned int adr, unsigned char byte)
{
    unsigned char ptr[3];
    ptr[0] = EEPROM_I2C_ADDRESS;
    ptr[1] = adr & 0x00FF;
    ptr[2] = byte;
    I2C_master_write(I2C_port_2, ptr, 3);
    __delay_us(10000);
}

void EEPROM_write_integer(unsigned int adr, unsigned int value)
{
    unsigned char ptr[4];
    ptr[0] = EEPROM_I2C_ADDRESS;
    ptr[1] = (adr & 0x00FF);
    ptr[2] = ((value&0xFF00)>>8);
    ptr[3] = (value & 0x00FF);
    I2C_master_write(I2C_port_2, ptr, 4);  
    __delay_us(10000);
}

unsigned char * EEPROM_read_byte(unsigned int adr)
{
    unsigned char ptr[2];
    ptr[0] = EEPROM_I2C_ADDRESS;
    ptr[1] = (adr & 0x00FF);
    I2C_master_read(I2C_port_2, ptr, 2, 1);  
    __delay_us(10000);
    return I2C_get_rx_buffer(I2C_port_2);
}

unsigned char * EEPROM_read_integer(unsigned int adr)
{
    unsigned char ptr[2];
    
    unsigned char *resp;
    ptr[0] = EEPROM_I2C_ADDRESS;
    ptr[1] = (adr & 0x00FF);
    I2C_master_read(I2C_port_2, ptr, 2, 1); 
    __delay_us(10000);
    resp = I2C_get_rx_buffer(I2C_port_2); 
    response[0] = *resp;
    ptr[0] = EEPROM_I2C_ADDRESS;
    ptr[1] = (adr & 0x00FF) + 1;
    I2C_master_read(I2C_port_2, ptr, 2, 1); 
    __delay_us(10000);
    resp = I2C_get_rx_buffer(I2C_port_2); 
    response[1] = *resp;
    return &response[0];
}

void EEPROM_write(unsigned int adr, unsigned char *ptr, unsigned char length)
{
    if (length <= EEPROM_RX_LENGTH)
    {
        I2C_master_write(I2C_port_2, ptr, length);
        __delay_us(10000);
    }
}

unsigned char * EEPROM_read(unsigned int adr, unsigned char length)
{
    unsigned char ptr[2];
    if (length <= EEPROM_RX_LENGTH)
    {
        ptr[0] = EEPROM_I2C_ADDRESS;
        ptr[1] = (adr & 0x00FF);
        I2C_master_read(I2C_port_2, ptr, 2, length);
        __delay_us(10000);
        return (I2C_get_rx_buffer(I2C_port_2)); 
    }
    else
        return 0;
}

void EEPROM_write_header (void)
{
    unsigned char ptr[2 + EEPROM_HEADER_LENGTH];
    unsigned char i = 0;
    ptr[0] = EEPROM_I2C_ADDRESS;
    ptr[1] = EEPROM_HEADER_ADDRESS;
    for (i=2; i < EEPROM_HEADER_LENGTH + 2; i++)
    {
        ptr[i] = EEPROM_header[i-2];       
    }
    I2C_master_write(I2C_port_2, ptr, 2 + EEPROM_HEADER_LENGTH);
    __delay_us(10000);
}

unsigned char EEPROM_was_written (void)
{
    unsigned char ptr[2] = {0};
    unsigned char test[EEPROM_HEADER_LENGTH];
    unsigned char *resp;
    unsigned char i, err;
    ptr[0] = EEPROM_I2C_ADDRESS;    
    for (i=0; i<EEPROM_HEADER_LENGTH; i++)
    {
        ptr[1] = EEPROM_HEADER_ADDRESS + i;
        I2C_master_read(I2C_port_2, ptr, 2, 1);         
        resp = I2C_get_rx_buffer(I2C_port_2); 
        __delay_ms(10);
        test[i] = *resp;
        resp++;
        if (test[i] != EEPROM_header[i])
        {
            err = 1;
        }
    }
    if (err == 0){return EEPROM_WAS_WRITTEN;}
    else {return EEPROM_NOT_WRITTEN;}    
}

