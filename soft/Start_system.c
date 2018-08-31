#include "Start_system.h"

////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/* SYSTEM CONTROL                                                             */
/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
void Clock_Init(void){
  
  SYSCTL->RCC2 |= 0x80000000;              // USERCC2       
  SYSCTL->RCC2 |= 0x00000800;              // BYPASS2, PLL bypass
  SYSCTL->RCC = (SYSCTL->RCC &~0x000007C0) // Clear XTAL 6-10 bits
    +0x00000540;                           // 10101, 16 MHz Crystal
  SYSCTL->RCC2 &= ~0x00000070;             // Configure main osc sourse
  SYSCTL->RCC2 &= ~0x00002000;             // active PLL by clear PWRDN
  SYSCTL->RCC2 |= 0x40000000;              // Set system divider: 400 MHz PLL
  SYSCTL->RCC2 = (SYSCTL->RCC2 &~0x1FC00000)// Clear system clock divider
    +(4<<22);                              // Configure for 80 MHz clock
  while((SYSCTL->RIS&0x00000040)==0){};    // Wait for PLLRIS bit
  SYSCTL->RCC2 &= ~0x00000800;             // Enable PLL by clear BYPASS2
  
}
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/* GPIO                                                                       */
/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
void GPIO_Init(void){
  
  SYSCTL->RCGCGPIO |= (1<<5);              // Clock PORTF
  GPIOF->DIR |= (1<<1)|(1<<2)|(1<<3);      // PF1, PF2, PF3 - Output
  GPIOF->DEN |= (1<<1)|(1<<2)|(1<<3);      // PF1, PF2, PF3 - Enable Digytal
  
}
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/* TIMERS                                                                     */
/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
void Timer0_Init(void){
  
  SYSCTL->RCGCTIMER |= (1<<0);             // Timer0 Clock Enable
  TIMER0->CTL &= ~ 0x00000001;             // Disable Timer During Configuration     
  TIMER0->CFG = 0x00000000;                // 32 bit Mode
  TIMER0->TAMR = 0x00000002;               // Periodic Mode
  TIMER0->TAMR |= 0x00000010;              // UP Count Mode
  TIMER0->TAILR = 0x02625A00;              // Value 8000000, 100ms
  TIMER0->CTL |= 0x00000001;               // Enable Timer
  
}
/******************************************************************************/
void Timer1_Init(void){
  
  SYSCTL->RCGCTIMER |= (1<<1);             // Timer1 Clock Enable
  TIMER1->CTL &= ~(1<<0);                  // Disable Timer During Configuration     
  TIMER1->CFG = 0x00000000;                // 32 bit Mode
  TIMER1->TAMR |= (0x2<<0);                // Periodic Mode
  TIMER1->TAMR &= ~(1<<4);                 // UP Count Mode
  TIMER1->TAILR = 0x02625A00;              // Value 40000000, 500ms
  TIMER1->IMR |= (1<<0);                   // Enabled Interrupt
  NVIC_EnableIRQ(TIMER1A_IRQn);            // from CMSIS Library
  TIMER1->CTL |= (1<<0);                   // Enable Timer
  
}
/******************************************************************************/
void Timer2_Init(void){
  
  SYSCTL->RCGCTIMER |= (1<<2);             // Timer2 Clock Enable
  TIMER2->CTL &= ~(1<<0);                  // Disable Timer During Configuration     
  TIMER2->CFG = 0x00000000;                // 32 bit Mode
  TIMER2->TAMR |= (0x2<<0);                // Periodic Mode
  TIMER2->TAMR &= ~(1<<4);                 // UP Count Mode
  TIMER2->TAILR = 0x000C3500;              // Value 800000, 10ms
  TIMER2->IMR |= (1<<0);                   // Enabled Interrupt
  NVIC_EnableIRQ(TIMER2A_IRQn);            // from CMSIS Library
  TIMER2->CTL |= (1<<0);                   // Enable Timer
  
}
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/* COMUNICATIONS                                                              */
/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////

/******************************************************************************/
/* SPI (SSI2)                                                                 */
/******************************************************************************/

/******************************************************************************/
/* PB4 - SSI2Clk-CLK                                                          */
/* PB5 - CS Software                                                          */
/* PB6-  SSI2Rx-MISO                                                          */
/* PB7-  SSI2Tx-MOSI                                                          */
/******************************************************************************/
void SPI_Init(void){
  
  SYSCTL->RCGCSSI |= (1<<2);                // Enable SSI2 Clock
  SYSCTL->RCGCGPIO |= (1<<1);               // Enable PORTB Clock
  
  GPIOB->AFSEL |= (1<<4)|(1<<5)|(1<<6)|(1<<7);          // GPIO SSI2 Configure
  GPIOB->AFSEL &= ~(1<<5);                              // PB5- GPIO I/O
  GPIOB->PCTL |= (2<<16)|(2<<20)|(2<<24)|(2<<28);       // PNC 4, 5, 6, 7 ,GPIO Pins and Alternate Functions
  GPIOB->DEN |= (1<<4)|(1<<5)|(1<<6)|(1<<7);            // Digital I/O
  GPIOB->DIR |= (1<<5);                     // CS Pin
  CS_HIGH_SSI2;                             // CS Pin High State
  
  SSI2->CR1 &= ~(1<<1);                     // SSI Operation Is Disabled.
  SSI2->CR1 = 0x00;                         // Reset CR1 Register
  SSI2->CPSR = 80;                          // Divider Clock, 80 MHz/80 = 1 MHz
  SSI2->CR0 = (0x7<<0);                     // 8 Bit data
  SSI2->CR0 &= ~(1<<6);                     // CPOL = 0
  SSI2->CR0 &= ~(1<<7);                     // CPHA = 0
  SSI2->CR1 |= (1<<1);                      // SSI Operation Is Enabled.
  
}
/******************************************************************************/
void SPI_Write(uint8_t data){
  
  while((SSI2->SR & (1<<0))==0);
  SSI2->DR = data;
  while((SSI2->SR & (1<<4)));
  
}
/******************************************************************************/
uint8_t SPI_Read(uint8_t data){
  
  while((SSI2->SR & (1<<0))==0);
  SSI2->DR = data;
  
  while((SSI2->SR & (1<<4)));
  
  return (uint8_t) SSI2->DR;
  
}
/******************************************************************************/
uint16_t SPI_Read16(uint16_t data){
  
  while((SSI2->SR & (1<<0))==0);
  SSI2->DR = data;
  
  while((SSI2->SR & (1<<4)));
  
  return (uint16_t) SSI2->DR;
  
}
/******************************************************************************/
/* I2C 1                                                                      */
/******************************************************************************/

/******************************************************************************/
/* PA6 - SCL                                                                  */
/* PA7 - SDA                                                                  */
/******************************************************************************/
void I2C_Init(void){
  
  SYSCTL->RCGCI2C |= (1<<1);            // Enable I2C Clock
  SYSCTL->RCGCGPIO |= (1<<0);           // Enable PORT A Clock
  
  GPIOA->AFSEL |= (1<<6)|(1<<7);        // GPIO Pins and Alternate Functions  
  GPIOA->DEN |= (1<<6)|(1<<7);          // GPIO Digital I/O
  GPIOA->ODR |=(1<<7);                  // SDA Pin is Open-Drain
  GPIOA->PCTL &= ~0xFF000000;           // PA6, PA7 - I2C1
  GPIOA->PCTL = (3<<28)|(3<<24);        // PUT3 PA6, PA7
  
  I2C1->MCR |= (1<<4);                  // Master Mode is Enabled.
  /****************************************************************************/
  /*  TPR = (System Clock/(2*(SCL_LP + SCL_HP)*SCL_CLK))-1;                   */
  /*  TPR = (80MHz/(2*(6+4)*100000))-1;                                       */
  /*  TPR = 39                                                                */
  /****************************************************************************/
  I2C1->MTPR = 0x00000027;              // TPR Bits = 39
  
}
/******************************************************************************/
void i2c_addr(uint8_t addr, uint8_t rw){
  
  switch(rw){
  case 0: I2C1->MSA = (addr<<1)+0;
  break;
  case 1: I2C1->MSA = (addr<<1)+1;
  break;   
  }
}
/******************************************************************************/
void i2c_write_one_byte(uint8_t data){
  
  I2C1->MDR = data;
  
  I2C1->MCS = (1<<2)|(1<<1)|(1<<0);
  
  while( (I2C1->MCS & (1<<0)) == 0);
  
  if( (I2C1->MCS & (1<<1) ) != 0){
    
    I2C1->MCS = (1<<2);
    
  } 
}
/******************************************************************************/
void i2c_write_data(uint8_t data, uint8_t command){
  
  I2C1->MDR = data;
  
  I2C1->MCS = command;
  
  while( (I2C1->MCS & (1<<0)) != 0);
  
  if( (I2C1->MCS & (1<<1) ) != 0)
  {
    if( (I2C1->MCS & (1<<4)) == 1){}
    else
    {
      I2C1->MCS = (1<<2);
      while( (I2C1->MCS & (1<<0)) != 0);
    }
  }        
}
/******************************************************************************/
uint8_t i2c_read_data(uint8_t command){
  
  uint8_t data;
  
  I2C1->MCS = command;
  
  while( (I2C1->MCS & (1<<0)) != 0);
  
  data =  I2C1->MDR;
  
  while( (I2C1->MCS & (1<<0)) != 0);
  
  if( (I2C1->MCS & (1<<1) ) != 0)
  {
    if( (I2C1->MCS & (1<<4)) == 1){}
    else
    {
      I2C1->MCS = (1<<2);
      while( (I2C1->MCS & (1<<0)) != 0); 
    }
  } 
  return data;
}
/******************************************************************************/
/* UART                                                                       */
/******************************************************************************/
void UART_Init(void){
  
  SYSCTL->RCGCUART |= (1<<0); 
  SYSCTL->RCGCGPIO |= (1<<0);
  
  GPIOA->AFSEL = (1<<1)|(1<<0); 
  GPIOA->PCTL = (1<<0)|(1<<4);  
  GPIOA->DEN = (1<<0)|(1<<1); 
  
  UART0->CTL &= ~(1<<0);
  /****************************************************************************/
  /* BRD = 80,000,000 / (16 * 115,200) = 43.40748888888888888888888888888888  */
  /* UARTFBRD[DIVFRAC] = integer(0.47048 * 64 + 0.5) = 26                     */
  /****************************************************************************/
  UART0->IBRD = 43;
  UART0->FBRD = 26;
  UART0->LCRH = (0x3<<5)|(1<<4);     // 8-bit, no parity, 1-stop bit
  
  UART0->CC = 0x0; 
  
  UART0->CTL = (1<<0)|(1<<8)|(1<<9); 
  
}
/******************************************************************************/
char uart_readChar(void){
  
  char c;
  while((UART0->FR & (1<<4)) != 0); 
  c = UART0->DR;                  
  return c;                    
}

/******************************************************************************/
void uart_printChar(char c){
  
  while((UART0->FR & (1<<5)) != 0);
  UART0->DR = c;           
}
/******************************************************************************/
void uart_printString(char * string){
  
  while(*string)
  {
    uart_printChar(*(string++));
  }
}
/******************************************************************************/
char* uart_readString(char delimiter)
{
  int stringSize = 0;
  char* string = (char*)calloc(10,sizeof(char));
  char c = uart_readChar(); 
  uart_printChar(c);
  
  while(c!=delimiter)
  { 
    
    *(string+stringSize) = c; // put the new character at the end of the array
    stringSize++;
    
    if((stringSize%10) == 0) // string length has reached multiple of 10
    {
      string = (char*)realloc(string,(stringSize+10)*sizeof(char)); // adjust string size by increasing size by another 10
    }
    
    c = uart_readChar();
    uart_printChar(c); // display the character the user typed
  }
  
  if(stringSize == 0)
  {
    
    return '\0'; // null car
  }
  return string;
}
/******************************************************************************/





