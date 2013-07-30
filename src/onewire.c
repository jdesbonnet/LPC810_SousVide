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

/**
 * Issue a read slot and return the result.
 *
 * @return 0 or 1
 */
int ow_bit_read () {
	GPIOSetBitValue(0,2, 0);
	delayMicroseconds(20);
	ow_low();
	delay(1);
	ow_high();

	delay(1);

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

	int detect = ow_read();
	ow_high();

	delayMicroseconds(410);

	return ~detect;
}

void ow_bit_write (int b) {

	// Write slot duration min 60µs
	ow_low();
	if (b) {
		// having trouble getting this in the 1-15µs range. Need better delay mechanism.
		delay(1); // max 15µs, min 1µs (?)
		ow_high();
		delayMicroseconds(60);
	} else {
		delayMicroseconds(66);
		ow_high();
	}

	// Recovery time
	delayMicroseconds(5);
}
void ow_byte_write (int data) {
	int i;

	// Send LSB first.

	for (i = 0; i < 8; i++) {
		ow_bit_write(data & 0x01);
		data >>= 1;
	}


	// Send MSB first.
	/*
	for (i = 0; i < 8; i++) {
		ow_bit_write(data & 0x80);
		data <<= 1;
	}
	*/

}

