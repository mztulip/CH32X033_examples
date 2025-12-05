#include "ch32fun.h"
#include <stdio.h>

// use defines to make more meaningful names for our GPIO pins
#define PIN_1 PA9

int main()
{
	SystemInit();

	funGpioInitAll(); // Enable GPIOs
	funPinMode( PIN_1,     GPIO_CFGLR_OUT_10Mhz_PP ); // Set PIN_1 to output
	

	while(1)
	{
		funDigitalWrite( PIN_1,     FUN_HIGH ); // Turn on PIN_1
		Delay_Ms( 250 );
		funDigitalWrite( PIN_1,     FUN_LOW );  // Turn off PIN_1
		Delay_Ms( 250 );
	}
}
