#include "ads1110.h"

/******************************************************************************/
/* ADS1110 16 bit ADC, 1 single/differential channel   i2c                    */
/* PGA - 1,2,4,8   SPS - 15,30,60,240                                         */
/******************************************************************************/

void Write_ADS1110_Config(uint8_t addr, ads1110_datarate dr, ads1110__gain gain){
  
  uint8_t config = gain | dr | ADS1110_MODE_CONTINUOUS | BIT_5 | BIT_6 | RDY;
  
  i2c_addr(addr,0);
  i2c_write_one_byte(config);			
  
}
/******************************************************************************/
uint16_t Get_ADS1110_Data(uint8_t addr){
  
  uint8_t byte1, byte2, byte3, conf;
  uint16_t data_adc;
  
  i2c_addr(addr,1);			
  byte1 = i2c_read_data((1<<0)|(1<<1)|(1<<3));
  byte2 = i2c_read_data((1<<0)|(1<<3));
  byte3 = i2c_read_data((1<<0)|(1<<2));	
  
  conf = byte3;
  if( (conf & (1<<7)) == 0){
    
    data_adc = (byte1<<8)|byte2;
    
    return data_adc;
    
  }
  
  else return 0;
}
/******************************************************************************/
