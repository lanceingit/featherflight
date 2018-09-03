#ifndef __USBD_CDC_VCP_H
#define __USBD_CDC_VCP_H

#ifdef __cplusplus
 extern "C" {
#endif 
     
#include "chip_config.h"
#include "usbd_cdc_core.h"
#include "usbd_conf.h"
#include <stdbool.h>

#define USB_USART_REC_LEN	 	512				//USB���ڽ��ջ���������ֽ���

extern u8  USB_USART_RX_BUF[USB_USART_REC_LEN]; //���ջ���,���USB_USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 


typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}LINE_CODING;
 


uint16_t VCP_Init     (void);
uint16_t VCP_DeInit   (void);
uint16_t VCP_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
uint16_t VCP_DataTx   (uint8_t data);
uint16_t VCP_DataRx   (uint8_t* Buf, uint32_t Len);
void usb_printf(const char* fmt,...); 
bool usb_read(uint8_t* data, uint16_t len);
void usb_write(uint8_t* data, uint16_t len);
bool usb_get(uint8_t* data);
bool usb_IsEmpty(void);


#ifdef __cplusplus
}
#endif

#endif 
















