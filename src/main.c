/****************************************************************************
 *   Project: LPC810 Sous Vide cooking controller
 *
 *   Description:
 *     A sous vide water bath temperature controller
 *
 ****************************************************************************
 Originally based on NXP example code, but completely edited/changed.
****************************************************************************/

#include "LPC8xx.h"			/* LPC8xx Peripheral Registers */
#include "lpc8xx_gpio.h"

#include "myuart.h"
#include "delay.h"
#include "onewire.h"
#include "ds18b20.h"

// Trace buffer
#include <cr_mtb_buffer.h>
__CR_MTB_BUFFER(32);


void delay(uint32_t d);
void delayMilliseconds (uint32_t d);
void blink(uint32_t n, uint32_t on_t, uint32_t off_t);
int32_t readTemperature (void);
void readOutTemperature (void);
void heatingElementOn(void);
void heatingElementOff(void);
void setHeaterDutyCycle(int dutyCycle0to1024);
void experimentalWarmUp(uint32_t setPointTemperature);
void setOneShotHeating (int time_ms);
void sep(void);
void crlf(void);
void debug(char *key, int32_t value);
void printDataRecord (void);
void print_record(int32_t array[], int len);


#define SYSTICK_DELAY		(SystemCoreClock/100)


/* This define should be enabled if you want to      */
/* maintain an SWD/debug connection to the LPC810,   */
/* but it will prevent you from having access to the */
/* LED on the LPC810 Mini Board, which is on the     */
/* SWDIO pin (PIO0_2).                               */
//#define USE_SWD

#define USE_UART

// Interrupt channel to use with pin interrupt
#define CHANNEL (1)


#define HEATING_ELEMENT_EN
#define HEATING_ELEMENT_PORT (0)
#define HEATING_ELEMENT_PIN (0)

#define ULED1_EN
#define ULED1_PORT (0)
#define ULED1_PIN (2)

#define SW1_EN
#define SW1_PORT (0)
#define SW1_PIN (1)

#define OW_PORT (0)
#define OW_PIN (3)

#define BASE_TEMPERATURE (54000)

/** Heater PWM cycle period in SysTicks (10ms) */
#define HEATER_PWM_PERIOD (3000)

#define SEP ' '

volatile uint32_t timeTick = 0;
volatile uint32_t interruptFlags = 0;
volatile uint32_t swDownTime=0;

volatile uint32_t heaterDutyCycle = 0;

/** Heating element status bits.
 * Bit 0: Set to 1 if element is on, 0 if off
 * Bit 1: Set to 1 if one-shot mode is selected.
 **/
volatile uint32_t heaterStatus = 0;
/**
 * If in one-shot mode, this will be the SysTick end time.
 */
volatile uint32_t oneShotEnd=0;

// These PID parameters are marked volatile because I hope
// to implement simple key commands (in UART IRQ) to
// manipulate those values.
int32_t Kp=10, Ki=3, Kd=40;
int32_t integral=20000;
int32_t derivative=0;

// These are global so that I can output temperature log
// records while doing test heating.
int32_t currentTemperature;
int32_t setPointTemperature;
int32_t output=0;
int32_t error=0;

void configurePins()
{
  /* Enable switch-matrix (SWM) clock */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 7);


#if 0
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
#endif


    /*
    Aiming for:
    PIO0_5 = RESET
    PIO0_4 = U0_TXD
    PIO0_3 = GPIO            - Disables SWDCLK
    PIO0_2 = GPIO (User LED) - Disables SWDIO
    PIO0_1 = GPIO
    PIO0_0 = GPIO
	*/

    /* Pin Assign 8 bit Configuration */
    /* U0_TXD */
    LPC_SWM->PINASSIGN0 = 0xffffff04UL;

    /* Pin Assign 1 bit Configuration */
    /* RESET */
    LPC_SWM->PINENABLE0 = 0xffffffbfUL;


#ifdef ULED1_EN
    // Set LED driver pin as output and set to logic high
    GPIOSetDir(ULED1_PORT,ULED1_PIN,1);
    GPIOSetBitValue(ULED1_PORT,ULED1_PIN,1);

    // Set SW1 pin as input
    GPIOSetDir(SW1_PORT,SW1_PIN,0);
#endif


	GPIOSetDir(HEATING_ELEMENT_PORT,HEATING_ELEMENT_PIN, 1);



}

/*****************************************************************************
**   Main Function  main()
******************************************************************************/
int main (void)
{

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


	//
	// Configure SW1 pin to trigger interrupt when pressed.
	//
	GPIOSetPinInterrupt( CHANNEL , /* channel */
		    				SW1_PORT,
		    				SW1_PIN,
		    				0, /* sense 0=edge*/
		    				0 /* polarity, 0=active low */
		    				);
	GPIOPinIntEnable( CHANNEL /* channel */,
		    		0 /* falling edge */ );

	// Configure one wire pin pull-up resistor
	// UM10601 §6.5.6, Table 55, p64
	// set MODE=0x02 (pull-up resistor enabled)
	// Pull-up too weak. Need external 4k7 R.
	//LPC_IOCON->PIO0_3 = 0x02 << 3;

	ow_init (OW_PORT,OW_PIN);


	//
	// Initialize UART and display hello message
	//
#ifdef USE_UART
	MyUARTInit(115200);
	MyUARTSendStringZ ("LPC810_SousVide_0.2.0\r\nDS18B20: ");

	uint64_t rom_addr = ds18b20_rom_read();
	MyUARTPrintHex((uint32_t)(rom_addr >> 32) );
	MyUARTPrintHex((uint32_t)(rom_addr & 0x00000000ffffffff ));
	crlf();

	debug ("T_start",readTemperature());
#endif

	// Initialize delay library (to calibrate short delay loop)
	delay_init();

	/*
	 * Get temperature set-point. This is set by pressing SW1 once for each degree C
	 * above 54°C. Ie one press (the minimum) is 55°C. A pause of more than 5 seconds
	 * signals that that user has finished setting the temperature. To confirm selection
	 * the LED will blink for each button press before entering control mode.
	 */
	int nButtonPress = 0;
	while ( (nButtonPress==0) || ((timeTick - swDownTime) < 500) ) {

		// Wait for interrupt. Two probable sources:
		// SysTick every 10ms and pin interrupt from SW1
		__WFI();

		// Was interrupt due to SW1?
		if (interruptFlags & 0x01) {
			// This blink as well as confirming button press
			// should also act as a debounce.
			blink(1,250,0);
			nButtonPress++;
			interruptFlags = 0;
			//debug ("nButtonPress",nButtonPress);
		}

	}

	// Echo back the number of button press to the user
	blink (nButtonPress,500,500);

	setPointTemperature =  BASE_TEMPERATURE
			+ 1000*nButtonPress;

	debug ("T_set",setPointTemperature);

	// Delay for 5s before entering control mode to avoid confusion
	// with flashing LEDs.
	delayMilliseconds (5000);

	/*
	 * Enter control mode. If under temperature LED will blink slowly. If at set-point
	 * +/- 1°C LED will be solid on. If over temperature LED will blink fast. This
	 * mode can only be exited by reset/power cycle. At any time the user can press
	 * the UI button and the temperature will be readout by blinking the LED.
	 */


	currentTemperature = readTemperature();

	// If delta Temperature > 10C exec warmup procedure
	if (setPointTemperature - currentTemperature > 10000) {
		experimentalWarmUp(setPointTemperature);
	}

	int32_t prevError= setPointTemperature - currentTemperature;
	int32_t dt;
	uint32_t prevTime = timeTick;
	uint32_t now;

	//
	// PID control loop. Using fixed point math.
	//
	while (1) {

		// Read temperature in 0.001°C units. Eg 45200 = 45.2°C.
		currentTemperature = readTemperature();


		// PID calculations
		error = setPointTemperature - currentTemperature;
		now = timeTick;
		dt = now - prevTime;

		// Fixed point scaling factor of 1e-2 (and 1e2 for dt)
		integral += (error*dt)/10000;

		// Fixed point scaling
		derivative = ((error - prevError)*10000)/dt;

		output = Kp*error
				+ Ki*integral
				+ Kd*derivative;


		prevError = error;
		prevTime = now;

		// So say for 10C error, output = 10000*Kp.

		heaterDutyCycle = output/1000;
		setHeaterDutyCycle(heaterDutyCycle);

		printDataRecord();

		//blink (1,500,500);

		delayMilliseconds(10000);

		// Did we receive a button press?
#if 0
		if (interruptFlags & 0x01) {
			// Read out temperature
			delayMilliseconds(5000);
			readOutTemperature();
			delayMilliseconds(5000);
			interruptFlags = 0;
		}
#endif

	}

}

void printDataRecord () {
	int32_t array[8];
	currentTemperature = readTemperature();
	array[0] = timeTick;
	array[1] = currentTemperature;
	array[2] = setPointTemperature;
	array[3] = Kp*error;
	array[4] = Ki*integral;
	array[5] = Kd*derivative;
	array[6] = output;
	array[7] = heaterDutyCycle;

	MyUARTSendStringZ("DATA:");
		int i;

		for (i = 0; i < 8; i++) {
			sep();
			MyUARTPrintDecimal(array[i]);
		}
		crlf();

	//print_record(array,8);
}

/**
 * Read out current temperature in °C by blinking the LED. LED blinked in two
 * sequences: first sequence for the temperature (°C) tens digit followed by
 * a delay of 2s. Then the second sequence for the least significant digit (the units).
 * Zero is represented by a one short blink of 100ms where as all other digits use 500ms
 * blinks. Temperatures of < 10°C or > 99°C not supported.
 * Example: 42°C: 4 x normal blinks, delay of 2s, 2 normal blinks.
 * Example: 60°C: 6 x normal blinks, delay of 2s, 1 short blink.
 */
void readOutTemperature (void) {
	int32_t currentTemperatureDeg = readTemperature()/1000;

	// Tens
	blink(currentTemperatureDeg/10,500,500);

	delayMilliseconds(2000);

	// Units
	uint32_t d = currentTemperatureDeg%10;
	if (d==0) {
		blink(1, 100, 500);
	} else {
		blink (d,500,500);
	}
}


/**
 * Blink LED 'n' times. Specify on and off time in milliseconds with on_t, off_t.
 */
void blink (uint32_t n, uint32_t on_t, uint32_t off_t) {
	while (n--) {
		GPIOSetBitValue(ULED1_PORT,ULED1_PIN, 0);
		delayMilliseconds (on_t);
		GPIOSetBitValue(ULED1_PORT,ULED1_PIN, 1);
		delayMilliseconds (off_t);
	}
}

/**
 * Read temperature from DS18B20
 * @Return in °C*1000 eg 27400 = 27.4°C
 */
int32_t readTemperature () {
	return ds18b20_temperature_read();
}

void heatingElementOn() {
	GPIOSetBitValue(HEATING_ELEMENT_PORT,HEATING_ELEMENT_PIN, 1);
	heaterStatus |= (1<<0);
}
void heatingElementOff() {
	GPIOSetBitValue(HEATING_ELEMENT_PORT,HEATING_ELEMENT_PIN, 0);
	//heaterStatus &= ~(1<<0);
	heaterStatus = 0; // This cancels any oneshot in progress also.
}

void printDS18B20Address () {
	uint64_t rom_addr = ds18b20_rom_read();
	MyUARTPrintHex(rom_addr >> 32 );
	MyUARTPrintHex((uint32_t)(rom_addr & 0x00000000ffffffff ));
	crlf(); //MyUARTSendStringZ ("\r\n");
}

void experimentalWarmUp (uint32_t setPointTemperature) {
	uint32_t e;
	uint32_t t0,t1,t2,t3;
	t0 = readTemperature();

	// Burn 1 in systicks
#define BURN1_ST 9000

	debug ("T0",t0);

	// Test burn
	/*
	setHeaterDutyCycle(1024);
	delayMilliseconds(120000);
	setHeaterDutyCycle(0);
	*/
	setOneShotHeating(BURN1_ST);
	while ( heaterStatus&(1<<1) ) {
		__WFI();
		if (timeTick%1000==0) {
			printDataRecord();
		}
	}

	// Some settle time
	//delayMilliseconds(10000);

	e = timeTick + 12000;
	while (timeTick<e) {
		__WFI();
		if (timeTick%1000==0) {
			printDataRecord();
		}
	}


#ifdef USE_EXP_CURVEFIT
	t1 = readTemperature();
	debug ("T1",t1);

	delayMilliseconds(10000);
	t2 = readTemperature();

	debug ("T2",t2);
	printDataRecord();

	delayMilliseconds(10000);
	t3 = readTemperature();
	debug ("T3",t3);
	printDataRecord();

	uint32_t expfrac = ((t3-t2)*256)/(t2-t1);
	// TODO: expfrac must be <256
	if (expfrac >= 256) {
		debug ("ERR",1);
	}

	uint32_t j = 0,delta;
	do {
		delta = (t3-t2) * expfrac;
		t2 = t3;
		t3 += delta/256;
		j++;
	} while (delta > 16);
	//fprintf (stderr,"estimated final temp=%d j=%d delta=%d\n", t3,j,delta);

	// t3 has final settle temperature
	debug ("Te1",t3);
#else
	t3 = readTemperature();
#endif

	// Second burn: how many more seconds do we need to bring to
	// 3C of target?
	uint32_t burn2_time = (( (setPointTemperature-3000) - t3)
			* (BURN1_ST*10) ) / (t3-t0);

	debug ("b2_t",burn2_time);

	//setHeaterDutyCycle(1024);
	//delayMilliseconds(burn2_time);
	//setHeaterDutyCycle(0);
	setOneShotHeating(burn2_time/10);
	while (heaterStatus&(1<<1)) {
		__WFI();
		if (timeTick%1000==0) {
			printDataRecord();
		}
	}

	// Allow to settle
	//delayMilliseconds(300000);
	e = timeTick + 6000;
	while (timeTick<e) {
		__WFI();
		if (timeTick%1000==0) {
			printDataRecord();
		}
	}

#ifdef BURN3_EN
	int32_t t5 = readTemperature();

	// Trim burn
	uint32_t burn3_time = ((setPointTemperature - t5)
			* (120000 + burn2_time))/ (t5 - t0);

	debug ("b3_t",burn3_time);

	setHeaterDutyCycle(1024);
	delayMilliseconds(burn3_time);
	setHeaterDutyCycle(0);

	delayMilliseconds(60000);
#endif

}

/**
 * Set the heating element duty cycle. Allowed values 0 - 1023.
 */
void setHeaterDutyCycle (int dutyCycle) {
	if (dutyCycle < 0) {
		heaterDutyCycle=0;
		return;
	}
	heaterDutyCycle = (dutyCycle * HEATER_PWM_PERIOD) / 1024;
}

void setOneShotHeating (int time_ticks) {
	heaterStatus |= (1<<1); // set one shot mode flag
	oneShotEnd = timeTick + time_ticks;
	heatingElementOn();
}

/**
 * Print separator character. Moving this to a separate
 * function saved ~ 4 bytes per call (presumably because
 * there are no params).
 */
void sep(void) {
	MyUARTSendByte (SEP);
}

void crlf(void) {
	MyUARTSendStringZ ("\r\n");
}
/**
 * Send debug message to UART.
 */
void debug(char *key, int32_t value) {
	MyUARTSendStringZ("DEBUG: ");
	MyUARTSendStringZ(key);
	MyUARTSendByte('=');
	MyUARTPrintDecimal(value);
	crlf();
}

void print_record(int32_t array[], int len) {
	MyUARTSendStringZ("DATA:");
	int i;

	for (i = 0; i < len; i++) {
		sep();
		MyUARTPrintDecimal(array[i]);
	}
	crlf();
}
/**
 * SysTick interrupt happens every 10 ms. Update timeTick
 * global and implement heating element PWM.
 **/
void SysTick_Handler(void) {
	timeTick++;

	if (heaterStatus&(1<<1)) {
		if (timeTick == oneShotEnd) {
			// If one shot mode and reached end time, element off.
			heatingElementOff();
			//heaterStatus = 0; // clear one shot and element one flag
		}

	} else {

		// Implement PWM mode
		if ( (timeTick%HEATER_PWM_PERIOD) >= heaterDutyCycle) {
			heatingElementOff();
			//heaterStatus = 0;
		} else {
			heatingElementOn();
			//heaterStatus = 1;
		}

	}

}

/**
 * Pin interrupt handler. Used to handle UI button.
 */
void PININT1_IRQHandler(void) {

	// Eliminate switch bounce
	if ( (timeTick-swDownTime) > 20) {
		//MyUARTSendStringZ (LPC_USART0, (uint8_t*)"^");
		interruptFlags |= 0x01;
		swDownTime = timeTick;
	}

	// Clear the interrupt
	LPC_PIN_INT->IST = (1<<CHANNEL);
	return;
}

#if 0
void UART0_IRQHandler(void)
{
	uint32_t rx;
	uint32_t uart_status = LPC_USART0->STAT;

	LPC_USART0->TXDATA = '^';

	// UM10601 §15.6.3, Table 162, p181. USART Status Register.
	// Bit 0 RXRDY: 1 = data is available to be read from RXDATA
	// Bit 2 TXRDY: 1 = data may be written to TXDATA
	if (uart_status & UART_STAT_RXRDY ) {

		// handle incoming byte
		rx = LPC_USART0->RXDATA;

		switch (rx) {
		case 'P':
			Kp++;
			break;
		case 'I':
			Ki++;
			break;
		case 'D':
			Kd++;
			break;
		case 'p':
			Kp--;
			break;
		case 'i':
			Ki--;
			break;
		case 'd':
			Kd--;
			break;
		case 'C':
			integral = 0;
			break;
		case 'H':
			setHeaterDutyCycle(1024);
			break;
		case 'c':
			integral += 100;
			break;
		//case '*':
			//LPC_USART0->TXDATA = '^';
		}
	} else if (uart_status & UART_STAT_TXRDY ){

		LPC_USART0->INTENCLR = 0x04;
	}

  return;
}
#endif
