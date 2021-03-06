#pragma once

#include "main.h"

/******************************************************************************/
#define CS_LOW_MAX6675     GPIOB->DATA &= ~(1<<5); 
#define CS_HIGH_MAX6675    GPIOB->DATA |= (1<<5); 
/******************************************************************************/
void MAX6675_SPI_Init(void);
uint16_t max6675_GetData(void);
float max6675_GetTemp(void);
/******************************************************************************/