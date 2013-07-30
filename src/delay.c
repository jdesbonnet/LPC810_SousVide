#include "LPC8xx.h"			/* LPC8xx Peripheral Registers */
#include "delay.h"

extern volatile uint32_t timeTick;


void delay (uint32_t d) {
	while (--d != 0) {
			__NOP();
	}
}

/**
 * Delay for t_ms milliseconds. Uses the SysTick timer which has a resolution of
 * 10ms, so < 10ms will result in no delay.
 */
void delayMilliseconds(uint32_t t_ms) {
	uint32_t t_cs = t_ms/10;
	if (t_cs==0) {
		return;
	}
	uint32_t end = timeTick + t_cs;
	while (timeTick != end) ;
}
/**
 * Delay for t_ms milliseconds. Uses the SysTick timer which has a resolution of
 * 10ms, so < 10ms will result in no delay.
 */
void delayMicroseconds(uint32_t t_ms) {
	uint32_t t_cs = t_ms/10;
	if (t_cs==0) {
		return;
	}
	uint32_t end = timeTick + t_cs;
	while (timeTick != end) ;
}
