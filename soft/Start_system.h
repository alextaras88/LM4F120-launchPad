#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "main.h"

/******************************************************************************/
#define CS_LOW_SSI2     GPIOB->DATA &= ~(1<<5); 
#define CS_HIGH_SSI2    GPIOB->DATA |= (1<<5); 
/******************************************************************************/

////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/* INIT PERIPHERY                                                             */
/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////

void Clock_Init(void);
/******************************************************************************/
void GPIO_Init(void);
/******************************************************************************/
void Timer0_Init(void);
void Timer1_Init(void);
void Timer2_Init(void);
/******************************************************************************/
void SPI_Init(void);
void SPI_Write(uint8_t data);
uint8_t SPI_Read(uint8_t data);
uint16_t SPI_Read16(uint16_t data);
/******************************************************************************/
void I2C_Init(void);
void i2c_addr(uint8_t addr, uint8_t rw);
void i2c_write_one_byte(uint8_t data);
void i2c_write_data(uint8_t data, uint8_t command);
uint8_t i2c_read_data(uint8_t command);
/******************************************************************************/
void UART_Init(void);
char uart_readChar(void);
void uart_printChar(char c);
void uart_printString(char * string);
char* uart_readString(char delimiter);
/******************************************************************************/