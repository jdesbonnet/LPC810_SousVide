/**
 * A simple one wire big-bang library.
 *
 */

#include "LPC8xx.h"
#include "lpc8xx_gpio.h"

#include "onewire.h"
#include "crc8.h"

#include "delay.h"


uint32_t ow_port;
uint32_t ow_pin;

void ow_init(int port, int pin) {
	ow_port=port;
	ow_pin=pin;
}

void ow_low() {
	// set direction output
	GPIOSetDir(ow_port, ow_pin, 1);
	// set low
	GPIOSetBitValue(ow_port, ow_pin, 0);
}
void ow_high() {
	// set direction input (high Z) and let pull-up R bring high
	GPIOSetDir(ow_port, ow_pin, 0);
}
int ow_read() {
	GPIOSetDir(ow_port, ow_pin, 0);
	return GPIOGetPinValue(ow_port, ow_pin);
}
int ow_bit_read () {
	GPIOSetBitValue(0,2, 0);
	delayMicroseconds(20);
	ow_low();
	delayMicroseconds(20);
	ow_high();
	delayMicroseconds(2);

	GPIOSetBitValue(0,2, 1);
	int b = ow_read();
	GPIOSetBitValue(0,2, 0);

	delayMicroseconds(65);
	GPIOSetBitValue(0,2, 1);
	return b;
}

int ow_reset() {
	ow_low();
	delayMicroseconds(480);
	ow_high();
	delayMicroseconds(70);

	int detect = ow_bit_read();
	ow_high();
	delayMicroseconds(410);

	return detect;
}
void ow_bit_write (int b) {
	ow_low();
	if (b) {
		delayMicroseconds(6);
		ow_high();
		delayMicroseconds(56);
	} else {
		delayMicroseconds(60);
		ow_high();
	}
}
void ow_byte_write (int data) {
	int i;
	// Send LSB first.
	for (i = 0; i < 8; i++) {
		ow_bit_write(data & 0x01);
		data >>= 1;
	}
}

