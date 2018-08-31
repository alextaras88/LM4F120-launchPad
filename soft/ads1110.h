#pragma once

#include "main.h"

/******************************************************************************/
/* ADS1110 CONFIGURATION REGISTER BIT NR	         		      */
/******************************************************************************/

#define		ST_RDY		7
#define		BIT_0_1		6
#define		BIT_0		5
#define		CS		4
#define		DR_1		3
#define		DR_0		2
#define		PGA_1		1
#define		PGA_0		0

/******************************************************************************/
/* ADS1110 CONFIG REGISTER BITS		                                      */
/******************************************************************************/
// Bit 0, 1

#define		GAIN_1		(0x00)
#define		GAIN_2		(0x01)
#define		GAIN_4		(0x02)
#define		GAIN_8		(0x03)

// Bit 2, 3

#define		SPS_240		(0x00)
#define		SPS_60		(0x04)
#define		SPS_30		(0x08)
#define		SPS_15		(0x0C)

// Bit 4

#define		ADS1110_MODE_CONTINUOUS			(0x00)
#define		ADS1110_MODE_SINGLE			(0x10)

// Bit 5, 6

#define		BIT_5		(0x00)
#define		BIT_6		(0x00)

// Bit 7

#define		RDY		(0x80)

/******************************************************************************/
/*									      */								
/******************************************************************************/
typedef enum
{
  PGA_GAIN_1  = GAIN_1,
  PGA_GAIN_2  = GAIN_2,
  PGA_GAIN_4  = GAIN_4,
  PGA_GAIN_8  = GAIN_8
    
} ads1110__gain;

typedef enum
{
  DATARATE_240SPS       = SPS_240,
  DATARATE_60SPS	= SPS_60,
  DATARATE_30SPS	= SPS_30,
  DATARATE_15SPS	= SPS_15
    
} ads1110_datarate;
/******************************************************************************/
/*									      */									
/******************************************************************************/
void Write_ADS1110_Config(uint8_t addr, ads1110_datarate dr, ads1110__gain gain);
uint16_t Get_ADS1110_Data(uint8_t addr);
