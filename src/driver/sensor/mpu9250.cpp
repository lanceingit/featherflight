#include <stdbool.h>
#include <stdint.h>

#include "board.h"

#include "FreeRTOS.h"
#include "task.h"

#include "mpu9250.h"


MPU9250::MPU9250():
	_accel_filter_x(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_x(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ)

{

}


bool MPU9250::init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC,ENABLE );//PORTA???? 
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1 , ENABLE );//SPI1???? 
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //?PU9250????
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //????
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);	//?????IO
  GPIO_SetBits(GPIOC,GPIO_Pin_2);//??  ??SPI??
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //PA567?????? 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA,GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);//??  ??SPI??
	
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); //PA5??? SPI1
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); //PA6??? SPI1
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); //PA7??? SPI1
	
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//??SPI1
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//????SPI1

  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI??????????
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPI??
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//????8????
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//????? SPI_CPOL_High
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//??????1????
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//????? SPI_CPOL_High
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//??????1????
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS???????
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;		//??????????:????????32(168M/32=5.25M)
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//?????MSB???
  SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC???????
  SPI_Init(SPI1, &SPI_InitStructure);  //??SPI_InitStruct???????????SPIx???
	
  SPI_Cmd(SPI1,ENABLE);//??SPI2		
  MPU_9250_DISENABLE;
  
  SPI1_SetSpeed(SPI_BaudRatePrescaler_4);
  
  
  if(MPU9250_Read_Reg(WHO_AM_I)==0x71)	//?????9250???
  {	
    MPU9250_Write_Reg(PWR_MGMT_1,0X80); //????,??MPU9250  0X80
    vTaskDelay(M2T(10));	
    MPU9250_Write_Reg(PWR_MGMT_1,0X01); //?????
    vTaskDelay(M2T(1));	
    MPU9250_Write_Reg(PWR_MGMT_2,0X00); //??????????
    vTaskDelay(M2T(1));	
    MPU9250_Write_Reg(CONFIG,0X03);     //????? 0x03 41hz (5.9ms delay) fs=1khz		
    vTaskDelay(M2T(1));	
    MPU9250_Write_Reg(SMPLRT_DIV,0x00); //???1000/(1+0)=1000HZ    
    vTaskDelay(M2T(1));	
    MPU9250_Write_Reg(GYRO_CONFIG,0X18);  //??????? 0X18 ??2000?    0X08  ??500? 
    vTaskDelay(M2T(1));	
    MPU9250_Write_Reg(ACCEL_CONFIG,0x18); //???????? 0X18 ??16g     0x08 ??4g
    vTaskDelay(M2T(1));	
    MPU9250_Write_Reg(ACCEL_CONFIG2,0x0B);//??????1khz ???41hz (11.80ms delay)			
    vTaskDelay(M2T(1));	
    MPU9250_Write_Reg(MAG_INT_ENABLE, 0x01);		
    vTaskDelay(M2T(1));	
    MPU9250_Write_Reg(MAG_INT_PIN_CFG,0x10);       
    vTaskDelay(M2T(1));	
/**********************Init MAG **********************************/					 
    MPU9250_Write_Reg(USER_CTRL,0X20);     //??MPU9250 AUX                                0X30
    vTaskDelay(M2T(1));	
    MPU9250_Write_Reg(MAG_I2C_MST_CTRL, 0x5D);
    vTaskDelay(M2T(1));	
    //MPU9250_Mag_Init();	
    
    return true;
  }
  else
  {
    return false;
  }  
  
}

void MPU9250::SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{
  SPI1->CR1&=0XFFC7;//?3-5??,???????
  SPI1->CR1|=SPI_BaudRatePrescaler;	//??SPI1?? 
  SPI_Cmd(SPI1,ENABLE); //??SPI1
} 

uint8_t MPU9250::MPU9250_Read_Reg(uint8_t reg)
{
  uint8_t reg_val;
  uint8_t status_temp;
  bool mpu9250_readreg_ok = true; 
  MPU_9250_ENABLE;
  if (SPI1_ReadWriteByte(reg|0x80,&status_temp) == 0)    //?????+????
  {
    mpu9250_readreg_ok = false;
  }
  if (mpu9250_readreg_ok)
  {
    SPI1_ReadWriteByte(0xff,&reg_val);//??????
  }
  //SPI1_ReadWriteByte(reg|0x80); 
  //reg_val=SPI1_ReadWriteByte(0xff);//??????
  
  MPU_9250_DISENABLE;
  return(reg_val);
}

uint8_t MPU9250::MPU9250_Write_Reg(uint8_t reg,uint8_t value)
{
  uint8_t status;
  uint8_t status_temp;
  bool mpu9250_writereg_ok = true;
	
  MPU_9250_ENABLE;//??SPI??
  if(SPI1_ReadWriteByte(reg,&status) == 0) //?????+????
  {
    mpu9250_writereg_ok = false;
  }
  
  if (mpu9250_writereg_ok)
  {
    SPI1_ReadWriteByte(value,&status_temp);//??????
  }  
  
  MPU_9250_DISENABLE; //??MPU9250
  return(status);//?????
}

uint8_t MPU9250::MPU9250_Read_Regs(uint8_t reg, uint8_t *buf,uint8_t num)
{
  uint8_t i;
  MPU_9250_ENABLE;

  uint8_t status_temp;
  
  if (SPI1_ReadWriteByte(reg|0x80,&status_temp) == 0) //?????+???? 
  {
    return 0;
  }
  
  for (i = 0; i < num; i++)
  {
    if (SPI1_ReadWriteByte(0xff,&buf[i]) == 0)//?????? 
    {
      return 0;
    }      
  }
  
  MPU_9250_DISENABLE;
  
  return 1; 
}

uint8_t MPU9250::SPI1_ReadWriteByte(uint8_t TxData,uint8_t *byte)
{		
  uint8_t retry=0;				 	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //?????SPI???????:????????
  {
    retry++;
    if(retry>200)return 0;
  }			  
  SPI_I2S_SendData(SPI1, TxData); //????SPIx??????
  retry=0;

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //?????SPI???????:?????????
  //while((SPI2->SR &SPI_I2S_FLAG_RXNE) == RESET);	
  {
    retry++;
    if(retry>200)
    {
      return 0;
    }    
  }
  
  *byte = SPI_I2S_ReceiveData(SPI1);
  return 1;
  //return SPI_I2S_ReceiveData(SPI1); //????SPIx???????					    
}

void MPU9250::read()
{
  uint8_t  MPU9250_buf[22] = {0};	                                //spi???????
    
  if (MPU9250_Read_Regs(ACCEL_XOUT_H, MPU9250_buf, 22) != 0)    
  {
    float x_in_new = (float)((int16_t)((MPU9250_buf[0] << 8) | MPU9250_buf[1])*(9.80665f /2048));   /* Acc.X */
    float y_in_new = (float)((int16_t)((MPU9250_buf[2] << 8) | MPU9250_buf[3])*(9.80665f /2048));   /* Acc.Y */
    float z_in_new = (float)((int16_t)((MPU9250_buf[4] << 8) | MPU9250_buf[5])*(9.80665f /2048));   /* Acc.Z */

    _acc[0] = _accel_filter_x.apply(x_in_new);
    _acc[1] = _accel_filter_y.apply(y_in_new);
    _acc[2] = _accel_filter_z.apply(z_in_new);
      
    float x_gyro_in_new = (float)((int16_t)((MPU9250_buf[8] << 8) | MPU9250_buf[9])*(0.0174532 / 16.4));       /* Gyr.X */
    float y_gyro_in_new = (float)((int16_t)((MPU9250_buf[10] << 8) | MPU9250_buf[11])*(0.0174532 / 16.4));     /* Gyr.Y */
    float z_gyro_in_new = (float)((int16_t)((MPU9250_buf[12] << 8) | MPU9250_buf[13])*(0.0174532 / 16.4));     /* Gyr.Z */
             
	_gyro[0] = _gyro_filter_x.apply(x_gyro_in_new);
	_gyro[1] = _gyro_filter_y.apply(y_gyro_in_new);
	_gyro[2] = _gyro_filter_z.apply(z_gyro_in_new);       
      
  }
  
}
