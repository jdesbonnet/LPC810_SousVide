/*
 * myuart.c
 *
 *  Created on: 30 Jun 2013
 *      Author: joe
 */

#include <string.h>

#include "LPC8xx.h"			/* LPC8xx Peripheral Registers */

#include "myuart.h"

//volatile uint8_t uart_rxbuf[UART_BUF_SIZE];
//volatile uint32_t uart_rxi=0;
//volatile uint32_t uart_buf_flags;

/*****************************************************************************
** Function name:		UARTInit
**
** Descriptions:		Initialize UART port, setup pin select,
**						clock, parity, stop bits, FIFO, etc.
**
** parameters:			UART baudrate
** Returned value:		None
**
*****************************************************************************/
void MyUARTInit(uint32_t baudrate)
{
	uint32_t UARTSysClk;

	// Removed from parameters
	LPC_USART_TypeDef *UARTx = LPC_USART0;

	//UARTClock_Init( UARTx );
	LPC_SYSCON->UARTCLKDIV = 1;     /* divided by 1 */
	NVIC_DisableIRQ(UART0_IRQn);
	/* Enable UART clock */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<14);
	/* Peripheral reset control to UART, a "1" bring it out of reset. */
	LPC_SYSCON->PRESETCTRL &= ~(0x1<<3);
	LPC_SYSCON->PRESETCTRL |= (0x1<<3);


	UARTSysClk = SystemCoreClock/LPC_SYSCON->UARTCLKDIV;
	LPC_USART0->CFG = DATA_LENG_8|PARITY_NONE|STOP_BIT_1; /* 8 bits, no Parity, 1 Stop bit */
	LPC_USART0->BRG = UARTSysClk/16/baudrate-1;	/* baud rate */
		/*
			Integer divider:
			BRG = UARTSysClk/(Baudrate * 16) - 1
			Frational divider:
			FRG = ((UARTSysClk / (Baudrate * 16 * (BRG + 1))) - 1) where FRG = (LPC_SYSCON->UARTFRDADD + 1) / (LPC_SYSCON->UARTFRDSUB + 1)
		*/
		/*	(1) The easist way is set SUB value to 256, -1 encoded, thus SUB register is 0xFF.
				(2) In ADD register value, depending on the value of UartSysClk, baudrate, BRG register value, and SUB register value, be careful
				about the order of multiplyer and divider and make sure any multiplyer doesn't exceed 32-bit boundary and any divider doesn't get
				down below one(integer 0).
				(3) ADD should be always less than SUB. */

	LPC_SYSCON->UARTFRGDIV = 0xFF;
	LPC_SYSCON->UARTFRGMULT = (((UARTSysClk / 16) * (LPC_SYSCON->UARTFRGDIV + 1)) / (baudrate * (UARTx->BRG + 1))) - (LPC_SYSCON->UARTFRGDIV + 1);

	LPC_USART0->STAT = CTS_DELTA | DELTA_RXBRK;		/* Clear all status bits. */
  /* Enable the UART Interrupt. */



	NVIC_EnableIRQ(UART0_IRQn);
	//LPC_USART0->TXDATA='*';
	LPC_USART0->INTENSET = RXRDY | TXRDY | DELTA_RXBRK;	/* Enable UART interrupt */
	LPC_USART0->CFG |= UART_EN;

	return;
}


void MyUARTSendByte (uint8_t v) {
	  // wait until data can be written to TXDATA
	  while ( ! (LPC_USART0->STAT & (1<<2)) );
	  LPC_USART0->TXDATA = v;
}

void MyUARTSendDrain () {
	// Wait for TXIDLE flag to be asserted
	while ( ! (LPC_USART0->STAT & (1<<3)) );
}

void MyUARTSendString (uint8_t *buf, uint32_t len) {
	int i;
	for (i = 0; i < len; i++) {
		MyUARTSendByte(buf[i]);
	}
}

/**
 * Send zero terminated string.
 */
void MyUARTSendStringZ (char *buf) {
	while (*buf != 0) {
		MyUARTSendByte((uint8_t)*buf);
		buf++;
	}
}

void UART0_IRQHandler(void)
{

	uint32_t uart_status = LPC_USART0->STAT;

	LPC_USART0->TXDATA = '$';

	// UM10601 §15.6.3, Table 162, p181. USART Status Register.
	// Bit 0 RXRDY: 1 = data is available to be read from RXDATA
	// Bit 2 TXRDY: 1 = data may be written to TXDATA
	if (uart_status & UART_STAT_RXRDY ) {

		LPC_USART0->TXDATA='*';
		//uart_rxbuf[uart_rxi] = LPC_USART0->RXDATA;

		// echo
		//MyUARTSendByte(LPC_USART0,uart_rxbuf[uart_rxi]);

		// If CR flag EOL
#if 0
		if (uart_rxbuf[uart_rxi]=='\r') {
			uart_buf_flags |= UART_BUF_FLAG_EOL;
			uart_rxbuf[uart_rxi]=0;
		} else {
			uart_rxi++;
			if (uart_rxi == UART_BUF_SIZE) {
				MyUARTBufReset();
			}
		}
#endif

	} else if (uart_status & UART_STAT_TXRDY ){

		LPC_USART0->INTENCLR = 0x04;
	}

  return;
}


void MyUARTPrintDecimal (int32_t i) {
	uint8_t buf[16];
	uint32_t j=0;

	if (i==0) {
		MyUARTSendByte('0');
		return;
	}

	if (i<0) {
		MyUARTSendByte('-');
		i *= -1;
	}
	while (i>0) {
		buf[j++] = '0' + i%10;
		i /= 10;
	}
	while (j>0) {
		MyUARTSendByte(buf[--j]);
	}
}


void MyUARTPrintHex (uint32_t v) {
	int i,h;
	for (i = 28; i >=0 ; i-=4) {
		h = (v>>i) & 0x0f;
		if (h<10) {
			MyUARTSendByte('0'+h);
		} else{
			MyUARTSendByte('A'+h-10);
		}
	}
}

#if 0

int parse_dec(uint8_t *buf, uint8_t **end) {
	int v=0;
	while (isDigit(*buf)) {
		v *= 10;
		v += (*buf - '0');
		buf++;
	}
	*end = buf;
	return v;
}



void print_dec(uint8_t *buf, uint32_t v) {
	if (v==0) {
		*buf='0';
		*(buf+1)=0;
		return;
	}
	uint8_t *s = buf;
	while (v>0) {
		*s++ = '0' + (v%10);
		v /= 10;
	}
	*s=0;

	// reverse
	int len = s - buf;
	int i;
	uint8_t t;
	for (i = 0; i < len/2; i++) {
		s--;
		t = *s;
		*s = *buf;
		*buf = t;
		buf++;
	}

}


/**
 * Return 1 if v is a decimal digit. Else return 0.
 */
int isDigit (uint8_t v) {
	return (v>='0'&&v<='9') ? 1:0;
}

/**
 * Return zero terminated string length
 */
int getLength (uint8_t *s) {
	int len=0;
	while (*s++) len++;
	return len;
}
void execute_cmd (uint8_t *cmd) {

	switch (*cmd) {

	case 'V': {
		MyUARTSendStringZ ((uint8_t*)"PiPM 0.1.3\r\n");
	}
	break;

	} // end switch

}

#endif
