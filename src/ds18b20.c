#include "LPC8xx.h"
#include "lpc8xx_gpio.h"

#include "onewire.h"
#include "crc8.h"

// http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf

int32_t ds18b20_cmd_convert () {
	if ( ! ow_reset() ) {
		return -999;
	}

	// Issue Convert command
	ow_write_byte (0x44);
}
