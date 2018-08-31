#include "main.h"

uint16_t adc = 0;
float mv = 0;
int16_t adc_1115 = 0;
uint16_t dac = 0;
char str[8];

void TIMER1A_Handler(void){
  
  GPIOF->DATA ^= (1<<1);
  TIMER1->ICR |= (1<<0);
  
}
void TIMER2A_Handler(void){
  
  //adc_1115 = ADS1115_READ_ADC_Diff_A0_1(0x48, DATARATE_16SPS, FSR_0_256);    
  GPIOF->DATA ^= (1<<2);
  Write_mcp4725(MCP_4725_ADDR, dac, MCP_4725_WRITE_DAC);
  if( dac < 4096) dac++;
  else dac = 0;
  TIMER2->ICR |= (1<<0);
}

int main(void){
  
  Clock_Init();
  GPIO_Init();
  I2C_Init();
  Timer0_Init();
  //Timer1_Init();
 // Timer2_Init();
 // SPI_Init();
  //MAX6675_SPI_Init();
  
  Write_ADS1110_Config(0x48, DATARATE_15SPS, PGA_GAIN_8);
 
  while(1){
    
    if((TIMER0->RIS & 0x00000001) == 1){
      
    // temp = max6675_GetTemp();
      
     // adc_1115 = ADS1115_READ_ADC_Diff_A0_1(0x48, DATARATE_32SPS, FSR_0_256);
      adc = Get_ADS1110_Data(0x48);
      mv = (adc * ((float)2.048/65535))*1000;
      sprintf(str, "%f", mv);
      
      GPIOF->DATA ^= (1<<3);
      TIMER0->ICR |= (1<<0);
    }
    
  }
  
}


