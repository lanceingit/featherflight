/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011-2014  Bill Nesbitt
*/

#include <string.h>
#include "chip_config.h"
#include "board_config.h"
#include "serial.h"


static serialPort_t serialPort1;
static serialPort_t serialPort3;
static serialPort_t serialPort4;
static serialPort_t serialPort6;
static serialPort_t serialPort7;

static void serialUart1Init(void);
static void serialUart3Init(void);
static void serialUart4Init(void);
static void serialUart6Init(void);
static void serialUart7Init(void);

static void serialIRQHandler(serialPort_t *s);




static void serialUart1Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
    // Enable USART1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(SERIAL_UART1_PORT_RCC, ENABLE);
	
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = SERIAL_UART1_RX_PIN | SERIAL_UART1_TX_PIN;
    GPIO_Init(SERIAL_UART1_PORT, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(SERIAL_UART1_PORT, SERIAL_UART1_RX_SOURCE, GPIO_AF_USART1);
    GPIO_PinAFConfig(SERIAL_UART1_PORT, SERIAL_UART1_TX_SOURCE, GPIO_AF_USART1);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


static void serialUart3Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
    // Enable USART1 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(SERIAL_UART3_PORT_RCC, ENABLE);
	
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = SERIAL_UART3_RX_PIN | SERIAL_UART3_TX_PIN;
    GPIO_Init(SERIAL_UART3_PORT, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(SERIAL_UART3_PORT, SERIAL_UART3_RX_SOURCE, GPIO_AF_USART3);
    GPIO_PinAFConfig(SERIAL_UART3_PORT, SERIAL_UART3_TX_SOURCE, GPIO_AF_USART3);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


static void serialUart4Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
    // Enable USART1 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_AHB1PeriphClockCmd(SERIAL_UART4_PORT_RCC, ENABLE);
	
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = SERIAL_UART4_RX_PIN | SERIAL_UART4_TX_PIN;
    GPIO_Init(SERIAL_UART4_PORT, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(SERIAL_UART4_PORT, SERIAL_UART4_RX_SOURCE, GPIO_AF_UART4);
    GPIO_PinAFConfig(SERIAL_UART4_PORT, SERIAL_UART4_TX_SOURCE, GPIO_AF_UART4);

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


static void serialUart6Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
    // Enable USART1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(SERIAL_UART6_PORT_RCC, ENABLE);
	
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = SERIAL_UART6_RX_PIN | SERIAL_UART6_TX_PIN;
    GPIO_Init(SERIAL_UART6_PORT, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(SERIAL_UART6_PORT, SERIAL_UART6_RX_SOURCE, GPIO_AF_USART6);
    GPIO_PinAFConfig(SERIAL_UART6_PORT, SERIAL_UART6_TX_SOURCE, GPIO_AF_USART6);

    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


static void serialUart7Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
    // Enable USART1 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);
    RCC_AHB1PeriphClockCmd(SERIAL_UART7_PORT_RCC, ENABLE);
	
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = SERIAL_UART7_RX_PIN | SERIAL_UART7_TX_PIN;
    GPIO_Init(SERIAL_UART7_PORT, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(SERIAL_UART7_PORT, SERIAL_UART7_RX_SOURCE, GPIO_AF_UART7);
    GPIO_PinAFConfig(SERIAL_UART7_PORT, SERIAL_UART7_TX_SOURCE, GPIO_AF_UART7);

    NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


serialPort_t * serialOpen(USART_TypeDef *USARTx, uint32_t baud, uint8_t* rxBuf, uint16_t rxBufSize, uint8_t* txBuf, uint16_t txBufSize) 
{
    serialPort_t *s = NULL;
    
    if (USARTx == USART1) {
		serialUart1Init();
        s = &serialPort1;
    } else if (USARTx == USART3) {
		serialUart3Init();
        s = &serialPort3;  
    } else if (USARTx == UART4) {
		serialUart4Init();
        s = &serialPort4;    
    } else if (USARTx == USART6) {
		serialUart6Init();
        s = &serialPort6;    
    } else if (USARTx == UART7) {
		serialUart7Init();
        s = &serialPort7;    
    }    
    
    s->USARTx = USARTx;
    s->baudRate = baud;
    s->rxBufSize = rxBufSize;
    s->rxBuf = rxBuf;
    s->txBufSize = txBufSize;
    s->txBuf = txBuf;
    
    Fifo_Create(&s->rxFifo, rxBuf, rxBufSize);
    Fifo_Create(&s->txFifo, txBuf, txBufSize);

    USART_InitTypeDef USART_InitStructure;

    // reduce oversampling to allow for higher baud rates
    USART_OverSampling8Cmd(s->USARTx, ENABLE);
	
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = s->baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(s->USARTx, &USART_InitStructure);

    USART_Cmd(s->USARTx, ENABLE);    
		
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
    
    return s;
}

void serialWrite(serialPort_t *s, unsigned char ch) 
{
    Fifo_WriteForce(&s->txFifo, ch);

    USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);
}

bool serialAvailable(serialPort_t *s) 
{
	return !Fifo_IsEmpty(&s->rxFifo);
}

uint8_t serialRead(serialPort_t *s) {
    uint8_t ch;
    
    Fifo_Read(&s->rxFifo, &ch);
    
    return ch;
}


//
// Interrupt handlers
//
static void serialIRQHandler(serialPort_t *s) 
{
    uint16_t SR = s->USARTx->SR;
	
    if (SR & USART_FLAG_RXNE) {
        Fifo_WriteForce(&s->rxFifo, s->USARTx->DR);
    }

    if (SR & USART_FLAG_TXE) {
        if (!Fifo_IsEmpty(&s->txFifo)) {
            uint8_t ch;
            Fifo_Read(&s->txFifo, &ch);
            s->USARTx->DR = ch;      
	    }
        // EOT
        else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }
}


void USART1_IRQHandler(void) 
{
    serialIRQHandler(&serialPort1);
}

void USART3_IRQHandler(void) 
{
    serialIRQHandler(&serialPort3);
}

void UART4_IRQHandler(void) 
{
    serialIRQHandler(&serialPort4);
}

void USART6_IRQHandler(void) 
{
    serialIRQHandler(&serialPort6);
}

void UART7_IRQHandler(void) 
{
    serialIRQHandler(&serialPort7);
}
