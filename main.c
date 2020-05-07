/*
 * main.c
 *
 *  Created on: 09.04.2020
 *      Author: silvere
 */
#define STM32L4

#include <stdio.h>
#include "hts221.h"
#include "systick.h"


int main(void)
{
	float hum;
	float temperature;

	clock_setup();
	systick_ms_setup();
	uart_setup();
	i2c2_setup();
	hts221_enable();

	who_i_am(HTS221_ADDR, WHO_I_AM);

	while (1) {
		msleep(2000);

		hum = humidity_read();
		printf("Humidity is: %.2f%%\n",hum);
		printf("\n");

		temperature = temperature_read();
		printf("Temperature is: %.2f°c\n",temperature);
		printf("\n");
	}
	return 0;
}
