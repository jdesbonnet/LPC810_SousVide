/*
 * myuart.h
 *
 *  Created on: 30 Jun 2013
 *      Author: joe
 */

#ifndef MYUART_H_
#define MYUART_H_

// Need this for bit constants
#include "lpc8xx_uart.h"

#define UART_BUF_SIZE (16)

/* UART status register bit definition. */
#define UART_STAT_RXRDY         (0x01<<0)
#define UART_STAT_RXIDLE        (0x01<<1)
#define UART_STAT_TXRDY         (0x01<<2)
#define UART_STAT_TXIDLE        (0x01<<3)
#define UART_STAT_CTS           (0x01<<4)
#define UART_STAT_CTS_DELTA     (0x01<<5)
#define UART_STAT_TXINT_DIS     (0x01<<6)

#define UART_BUF_FLAG_EOL (0x01<<0)

void MyUARTInit(LPC_USART_TypeDef *UARTx, uint32_t baudrate);
void MyUARTSendByte(LPC_USART_TypeDef *UARTx, uint8_t v);
void MyUARTSendString(LPC_USART_TypeDef *UARTx, uint8_t *buf, uint32_t len);
void MyUARTSendStringZ(LPC_USART_TypeDef *UARTx, uint8_t *buf);
void MyUARTSendDrain(LPC_USART_TypeDef *UARTx);

// Not sure if external users of buffer need the 'volatile' qualifier
uint8_t* MyUARTGetBuf(void);
uint32_t MyUARTGetBufIndex(void);

uint32_t MyUARTGetBufFlags(void);

void MyUARTBufReset(void);
uint32_t MyUARTBufCopy(uint8_t *buf);

void MyUARTPrintDecimal(LPC_USART_TypeDef *UARTx, int32_t i);

#endif /* MYUART_H_ */
