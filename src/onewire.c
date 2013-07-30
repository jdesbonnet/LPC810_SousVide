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
 * Issue a read slot and return the result. The result must be read within
 * 15µs of the
 *
 * @return 0 or 1
 */
int ow_bit_read () {
	GPIOSetBitValue(0,2, 0);//debug

	// The read slow starts with the bus is diven low.
	// We have 15µs from the falling edge read the bus.
	ow_low();
	delay(1); // Must be held low for at least 1µs

	// Bring bus high again. And read within the 15µs time interval
	// (already a few µs used by by now...)
	ow_high();
	delay(1);

	GPIOSetBitValue(0,2, 1); //debug
	int b = ow_read();
	GPIOSetBitValue(0,2, 0); //debug

	// Read slots must be a minimum of 60µs in duration with a minimum of 1µs
	// recovery time between slots. Rather than monitor bus to check for end
	// of slot, just delay for a period well exceeding the 60µs slot time.
	delayMicroseconds(65);

	GPIOSetBitValue(0,2, 1); //debug

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

}

int ow_byte_read () {
	int i, data = 0;
	for (i = 0; i < 8; i++) {
		data >>= 1;
		data |= ow_bit_read() ? 0x80 : 0x00;
	}
	return data;
}

uint64_t ow_uint64_read () {
	uint64_t data = 0;

	int i;
	for (i = 0; i < 8; i++) {
		data <<= 8;
		data |= ow_byte_read();
	}

	return data;
}

