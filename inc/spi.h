//***************************************************************************//
// File      : spi.h
//
// Functions :  void SPI_init (unsigned char mode, unsigned char ppre, unsigned char spre, unsigned char channel); 
//              void SPI_master_write (unsigned char *data, unsigned char length, unsigned char chip, unsigned char channel);
//              void SPI_fill_transmit_buffer (unsigned char * data, unsigned char length, unsigned char channel);
//              unsigned char SPI_rx_done (unsigned char channel);
//              unsigned char * SPI_get_rx_buffer (unsigned char channel);
//              void SPI_master_deassert_cs (unsigned char chip);
//              void SPI_master_assert_cs (unsigned char chip);
//              void SPI_slave_initiate (void);
//              void SPI_clear_rx_buffer (unsigned char channel);
//
// Includes  :  general.h
//           
// Purpose   :  Driver for the dsPIC33EP SPI peripheral
//              2 seperate SPI channel on Intellitrol
//              SPI_1 - DRV8873 control port, SPI master between dsPIC and peripheral
//              SPI_2 - Intellitrol control port, SPI slave between dsPIC and external master
//
//Jean-Francois Bilodeau    MPLab X v5.10    10/02/2020  
//****************************************************************************//
#ifndef __spi_h_
#define __spi_h_

#include "general.h"

//******************************************************************************
// Definition of SPI modules
#define SPI_1 0
#define SPI_2 1
#define SPI_QTY 2

//******************************************************************************
// Various SPI chip select defines
#define DRV8873_CHIP 1
#define EXTSPI2_CHIP 2
#define NONE 255

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

//******************************************************************************
// SPI module pin configuration
#define DRV8873_CS LATBbits.LATB0      // DRV8873 /CS is RB0
#define EXTSPI2_CS LATBbits.LATB13      
#define SPI_MSG_LENGTH DRIVE_DATA_LENGTH
typedef struct
{
    unsigned char spi_chip;
    unsigned char spi_tx_data[SPI_MSG_LENGTH];
    unsigned char spi_rx_data[SPI_MSG_LENGTH];
    unsigned char spi_tx_length;
    unsigned char spi_state;
    unsigned char spi_done;
    unsigned char spi_rx_cnt;
    unsigned char spi_tx_cnt;
}SPI_struct;

void SPI_init (unsigned char channel, unsigned char mode, unsigned char ppre, unsigned char spre); 
void SPI_master_write (unsigned char channel, unsigned char *data, unsigned char length, unsigned char chip);
void SPI_fill_transmit_buffer (unsigned char channel, unsigned char * data, unsigned char length);
unsigned char SPI_rx_done (unsigned char channel);
unsigned char * SPI_get_rx_buffer (unsigned char channel);
void SPI_master_deassert_cs (unsigned char chip);
void SPI_master_assert_cs (unsigned char chip);
void SPI_slave_initiate (void);
void SPI_clear_rx_buffer (unsigned char channel);
#endif

