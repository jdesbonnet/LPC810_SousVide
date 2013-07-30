#include "LPC8xx.h"			/* LPC8xx Peripheral Registers */
#include "delay.h"

extern volatile uint32_t timeTick;

uint32_t delayLoopCalibration=1000; /* guesstimate in case delay_init() not called */

void delay_init () {
	// Calibrate our simple spin-loop delay using the SysTick timer
	uint32_t startCalib = timeTick;
	delay(1<<20);

	// Number of delay loop iterations per 10ms
	delayLoopCalibration =  1<<20 / (timeTick - startCalib) ;

	// Number of ns per iteration
	//delayLoopCalibration = ((timeTick - startCalib) * 10000000) / 1<<20 ;
}

/**
 * Short delay spin-loop.
 */
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
 * Delay for t_us microseconds.
 */
void delayMicroseconds(uint32_t t_us) {
	// This calculation is going to be noticable for very short delays
	//uint32_t niter = (delayLoopCalibration*10000)/t_us;
	//uint32_t niter = (t_us * 1000) / delayLoopCalibration;
	//delay(niter);
	uint32_t niter = t_us*14/10;
	delay(niter);
}
