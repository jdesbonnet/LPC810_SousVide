#include "LPC8xx.h"
#include "lpc8xx_gpio.h"

#include "onewire.h"
#include "crc8.h"

// http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf

/**
 * Assume there is just one DS18B20 device on the bus.
 */
int32_t ds18b20_temperature_read () {
	if ( ! ow_reset() ) {
		return -999;
	}

	// Skip ROM command
	ow_byte_write (0x55);

	delayMicroseconds(1000);

	// Issue Convert command
	ow_byte_write (0x44);

	delayMicroseconds(1000);

	// Poll for conversion complete
	while ( ! ow_bit_read() ) ;

	// Issue command to read scratch pad
	ow_byte_write (0xBE);

	// Read data (up to 9 bytes, but only interested in first two)
	int i;
	int16_t data=0;
	for (i = 0; i < 16; i++) {
		data >>= 1;
		if (ow_bit_read()) {
			data |= 0x8000;
		}
	}

	// 16 bit temperature data is interpreted as a signed 16 bit integer.
	// of 12 bit resolution (by default -- the DS18B20 can be configured
	// for lower resolutions). To get °C multiply by (1/16)°C
	// Return temperature * 10;
	return  (data * 10) / 16;

}
