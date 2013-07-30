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
	// set direction output
	GPIOSetDir(ow_port, ow_pin, 1);
	// set high
	GPIOSetBitValue(ow_port, ow_pin, 1);
}
int ow_read_bit() {
	GPIOSetDir(ow_port, ow_pin, 0);
	return GPIOGetPinValue(ow_port, ow_pin);
}

int ow_reset() {
	ow_low();
	delayMicroseconds(480);
	ow_high();
	delayMicroseconds(70);

	int detect = ow_read_bit();
	ow_high();
	delayMicroseconds(410);

	return detect;
}
void ow_write_bit (int b) {
	ow_low();
	if (b) {
		delayMicroseconds(6);
		ow_high();
		delayMicroseconds(56);
	} else {
		ow_low();
		delayMicroseconds(60);
		ow_high();
	}
}
void ow_write_byte (int data) {
	int i;
	// Send LSB first.
	for (i = 0; i < 8; i++) {
		ow_write_bit(data & 0x01);
		data >>= 1;
	}
}

