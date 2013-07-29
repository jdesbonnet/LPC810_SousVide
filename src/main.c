/****************************************************************************
 *   $Id:: lpc8xx_wkt.c 3635 2012-10-31 00:31:46Z usb00423                     $
 *   Project: NXP LPC8xx Wakeup timer example
 *
 *   Description:
 *     This file contains wakeup timer test modules.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 
 * Permission to use, copy, modify, and distribute this software and its 
 * documentation is hereby granted, under NXP Semiconductors' 
 * relevant copyright in the software, without fee, provided that it 
 * is used in conjunction with NXP Semiconductors microcontrollers. This 
 * copyright, permission, and disclaimer notice must appear in all copies of 
 * this code.
****************************************************************************/
#include "LPC8xx.h"			/* LPC8xx Peripheral Registers */
#include "lpc8xx_gpio.h"

#include "myuart.h"

// Trace buffer
#include <cr_mtb_buffer.h>
__CR_MTB_BUFFER(32);

int isDigit(uint8_t v);
int parse_dec(uint8_t *buf, uint8_t **end);
void print_dec(uint8_t *buf, uint32_t v);
void execute_cmd(uint8_t *buf);
void delay(uint32_t d);
void blink(void);

#define SYSTICK_DELAY		(SystemCoreClock/100)


/* This define should be enabled if you want to      */
/* maintain an SWD/debug connection to the LPC810,   */
/* but it will prevent you from having access to the */
/* LED on the LPC810 Mini Board, which is on the     */
/* SWDIO pin (PIO0_2).                               */
//#define USE_SWD


// Interrupt channel to use with pin interrupt
#define CHANNEL (1)


#define HEATING_ELEMENT_EN
#define HEATING_ELEMENT_PORT (0)
#define HEATING_ELEMENT_PIN (1)

#define ULED1_EN
#define ULED1_PORT (0)
#define ULED1_PIN (2)

#define SW1_EN
#define SW1_PORT (0)
#define SW1_PIN (1)

volatile uint32_t timeTick = 0;
volatile uint32_t interruptFlags = 0;



void configurePins()
{
  /* Enable SWM clock */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 7);

  /* Pin Assign 8 bit Configuration */
  /* U0_TXD */
  /* U0_RXD */
  LPC_SWM->PINASSIGN0 = 0xffff0004UL;

  /* Pin Assign 1 bit Configuration */
  #if !defined(USE_SWD)
    /* Pin setup generated via Switch Matrix Tool
       ------------------------------------------------
       PIO0_5 = RESET
       PIO0_4 = U0_TXD
       PIO0_3 = GPIO            - Disables SWDCLK
       PIO0_2 = GPIO (User LED) - Disables SWDIO
       PIO0_1 = GPIO
       PIO0_0 = U0_RXD
       ------------------------------------------------
       NOTE: SWD is disabled to free GPIO pins!
       ------------------------------------------------ */
    LPC_SWM->PINENABLE0 = 0xffffffbfUL;
  #else
    /* Pin setup generated via Switch Matrix Tool
       ------------------------------------------------
       PIO0_5 = RESET
       PIO0_4 = U0_TXD
       PIO0_3 = SWDCLK
       PIO0_2 = SWDIO
       PIO0_1 = GPIO
       PIO0_0 = U0_RXD
       ------------------------------------------------
       NOTE: LED on PIO0_2 unavailable due to SWDIO!
       ------------------------------------------------ */
    LPC_SWM->PINENABLE0 = 0xffffffb3UL;
  #endif

#ifdef ULED1_EN
    // Set LED driver pin as output and set to logic high
    GPIOSetDir(ULED1_PORT,ULED1_PIN,1);
    GPIOSetBitValue(ULED1_PORT,ULED1_PIN,1);

    // Set SW1 pin as input
    GPIOSetDir(SW1_PORT,SW1_PIN,0);
#endif




}

/*****************************************************************************
**   Main Function  main()
******************************************************************************/
int main (void)
{
	uint8_t buf[16];



	SystemCoreClockUpdate();


	SysTick_Config( SYSTICK_DELAY );



	//
	// Enable and configure GPIO
	//

	GPIOInit();

#if 0
	/* Enable AHB clock to the GPIO domain. */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);

	/* Peripheral reset control to GPIO and GPIO INT, a "1" bring it out of reset. */
	LPC_SYSCON->PRESETCTRL &= ~(0x1<<10);
	LPC_SYSCON->PRESETCTRL |= (0x1<<10);
#endif



	configurePins();

	//UARTInit(LPC_USART0, 115200);
	MyUARTInit(LPC_USART0, 115200);


	LPC_USART0->INTENSET = 0x01;	/* Enable UART interrupt */

	MyUARTSendStringZ (LPC_USART0, (uint8_t*)"Welcome!\r\n");



	GPIOSetPinInterrupt( CHANNEL , /* channel */
		    				SW1_PORT,
		    				SW1_PIN,
		    				0, /* sense 0=edge*/
		    				0 /* polarity, 0=active low */
		    				);
	GPIOPinIntEnable( CHANNEL /* channel */,
		    		0 /* falling edge */ );


	while (1) {
		__WFI();

		if (interruptFlags & 0x01) {
			//GPIOPinIntClear(CHANNEL);
			blink();

			print_dec(buf, timeTick);
			MyUARTSendStringZ(LPC_USART0, buf);
			MyUARTSendStringZ(LPC_USART0, (uint8_t *) "\r\n");

			blink();

			interruptFlags = 0;
		}

	}


		  while (1) {
			  GPIOSetBitValue(ULED1_PORT,ULED1_PIN,
					  GPIOGetPinValue(SW1_PORT,SW1_PIN)
			  );


			  if (MyUARTGetBufFlags() & UART_BUF_FLAG_EOL) {
				  MyUARTBufCopy(buf);
				  MyUARTBufReset();
				  execute_cmd(buf);
			  }



		  }

		  // Step 6. Use the ARM WFI (Wait For Interrupt) instruction.
		  //__WFI();





}

void execute_cmd (uint8_t *cmd) {

	switch (*cmd) {

	case 'V': {
		MyUARTSendStringZ (LPC_USART0, (uint8_t*)"PiPM 0.1.3\r\n");
	}
	break;

	} // end switch

}

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
		*buf++;
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


void delay (uint32_t d) {
	while (--d != 0) {
			__NOP();
	}
}

void blink () {
	  GPIOSetBitValue(ULED1_PORT,ULED1_PIN, 0);
	  delay (100000);
	  GPIOSetBitValue(ULED1_PORT,ULED1_PIN, 1);
	  delay (100000);
}

/* SysTick interrupt happens every 10 ms */
void SysTick_Handler(void)
{
  timeTick++;
  //LPC_USART0->TXDATA = "*";
}

void PININT1_IRQHandler(void)
{
	MyUARTSendStringZ (LPC_USART0, (uint8_t*)"^");

	interruptFlags |= 0x01;

	// Clear the interrupt
	LPC_PIN_INT->IST = (1<<CHANNEL);
	return;
}


