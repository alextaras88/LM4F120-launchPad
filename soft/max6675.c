#include "max6675.h"

/******************************************************************************/
/* MAX6675                                                                    */
/* Cold-Junction-Compensated K-Thermocoupleto-DigitalConverter(0°C to +1024°C)*/
/* SPI, 16bit                                                                 */
/******************************************************************************/
/******************************************************************************/
/* MAX6675_SPI (SSI2)                                                                 */
/******************************************************************************/

/******************************************************************************/
/* PB4 - SSI2Clk-CLK                                                          */
/* PB5 - CS Software                                                          */
/* PB6-  SSI2Rx-MISO                                                          */
/* PB7-  SSI2Tx-MOSI                                                          */
/******************************************************************************/
void MAX6675_SPI_Init(void){
  
  SYSCTL->RCGCSSI |= (1<<2);                // Enable SSI2 Clock
  SYSCTL->RCGCGPIO |= (1<<1);               // Enable PORTB Clock
  
  GPIOB->AFSEL |= (1<<4)|(1<<5)|(1<<6)|(1<<7);          // GPIO SSI2 Configure
  GPIOB->AFSEL &= ~(1<<5);                              // PB5- GPIO I/O
  GPIOB->PCTL |= (2<<16)|(2<<20)|(2<<24)|(2<<28);       // PNC 4, 5, 6, 7 ,GPIO Pins and Alternate Functions
  GPIOB->DEN |= (1<<4)|(1<<5)|(1<<6)|(1<<7);            // Digital I/O
  GPIOB->DIR |= (1<<5);                     // CS Pin
                              // CS Pin High State
  
  SSI2->CR1 &= ~(1<<1);                     // SSI Operation Is Disabled.
  SSI2->CR1 = 0x00;                         // Reset CR1 Register
  SSI2->CPSR = 80;                          // Divider Clock, 80 MHz/80 = 1 MHz
  SSI2->CR0 = (0xF<<0);                     // 8 Bit data
  SSI2->CR0 &= ~(1<<6);                     // CPOL = 0
  SSI2->CR0 |= (1<<7);                      // CPHA = 1
  SSI2->CR1 |= (1<<1);                      // SSI Operation Is Enabled.
  
}
/******************************************************************************/
uint16_t max6675_GetData(void){
  
  
  
 
}
/******************************************************************************/
float max6675_GetTemp(void){
  
  uint16_t data;
  CS_LOW_MAX6675;
  data = SPI_Read16(0x000);	
  CS_HIGH_MAX6675 ;                                                                      
  data = (0x0FFF &(data>>3));	
  return  (float)(data/4.096);
  
}
/******************************************************************************/