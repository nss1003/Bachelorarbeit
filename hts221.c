/*
 * hts221.c
 *
 *  Created on: 30 Apr 2020
 *      Author: silvere
 */
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "hts221.h"


void clock_setup(void)
{
	/* FIXME - this should eventually become a clock struct helper setup */
		rcc_osc_on(RCC_HSI16);

		flash_prefetch_enable();
		flash_set_ws(4);
		flash_dcache_enable();
		flash_icache_enable();
		/* 16MHz / 4 = > 4 * 40 = 160MHz VCO => 80MHz main pll  */
		rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_HSI16, 4, 40,
				0, 0, RCC_PLLCFGR_PLLR_DIV2);
		rcc_osc_on(RCC_PLL);

		rcc_periph_clock_enable(RCC_GPIOB);
		rcc_periph_clock_enable(RCC_GPIOC);
		rcc_periph_clock_enable(RCC_USART3);
		rcc_periph_clock_enable(RCC_I2C2);


		rcc_set_sysclk_source(RCC_CFGR_SW_PLL); /* careful with the param here! */
		rcc_wait_for_sysclk_status(RCC_PLL);

		/* FIXME - eventually handled internally */
		rcc_ahb_frequency = 80e6;
		rcc_apb1_frequency = 80e6;
		rcc_apb2_frequency = 80e6;
}

void uart_setup(void)
{
	/* Setup GPIO pins for USART3 transmit. */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4|GPIO5);

	/* Setup UART4 TX and RX pin as alternate function. */
	gpio_set_af(GPIOC, GPIO_AF7, GPIO5);
	gpio_set_af(GPIOC, GPIO_AF7, GPIO4);

	//USART3 setup for printf commands
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	/* Finally enable USART3. */
	usart_enable(USART3);
}

void i2c2_setup(void)
{
	/* Setup SDA and SLC for I2C communication*/
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, SCL);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, SDA);

	/* Setup SDA and SCL pin as alternate function. */
	gpio_set_af(GPIOB, GPIO_AF4, SCL);
	gpio_set_af(GPIOB, GPIO_AF4, SDA);

	i2c_peripheral_disable(I2C2);
	i2c_enable_analog_filter(I2C2);

	i2c_set_speed(I2C2,i2c_speed_sm_100k, 8);
	i2c_enable_stretching(I2C2);

	i2c_set_7bit_addr_mode(I2C2);
	i2c_peripheral_enable(I2C2);

}

/**
 * Use USART3 as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART3, '\r');
			}
			usart_send_blocking(USART3, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}


void who_i_am(uint8_t dev, uint8_t reg)
{

	uint8_t reg_cmd[2];
	reg_cmd[0] = reg;
	i2c_transfer7(I2C2, dev, reg_cmd, 1, (reg_cmd+1), 1);

	printf("Wo i am = 0x%02x\n",reg_cmd[1]);
}

void hts221_enable(void)
{
	/*
	 * PD = 1 --> Turn on the device
	 * BDU = 1 --> Output register not update until MSB and LSB reading
	 * ORD1 = 1 ODR0 = 0 --> Set output data rate to 7Hz
	 */
	uint8_t cmd[2];
	cmd[0] = CTRL_REG1;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+1), 1);
	cmd[1] |= (HTS221_PD_ON | HTS221_BDU_ON | HTS221_ODR1_ON);
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 2, NULL, 0);
}


float temperature_read(void){

	uint16_t T0_degc;
	uint16_t T1_degc;
	int16_t T0_out;
	int16_t T1_out;
	int16_t T_out;
	uint16_t T0_degc_x8;
	uint16_t T1_degc_x8;

	uint8_t reg;
	uint8_t cmd[4];
	uint8_t msb;

	int32_t tmp;
	static float temperature = 0.0;

	// Read T0_degc_x8 and T1_degc_x8
	cmd[0] = TO_DEGC_X8;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+1), 1);
	cmd[0] = T1_DEGC_X8;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+2), 1);

	// Read the most significant bit of T1_DEGC and T0_DEGC
	cmd[0] = T1_T0_MSB;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+3), 1);

	// Calculate the T0_degc and T1_degc values
	T0_degc_x8 = (((uint16_t)(cmd[3] & 0x02)) << 8) | ((uint16_t)cmd[1]);
	T1_degc_x8 = (((uint16_t)(cmd[3] & 0x0C)) << 6) | ((uint16_t)cmd[2]);
	T0_degc = T0_degc_x8>>3;
	T1_degc = T1_degc_x8>>3;

	// Read T0_OUT less significant bit
	cmd[0] = T0_OUT_L;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+1), 1);

	// Read T0_OUT most significant bit
	cmd[0] = T0_OUT_M;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+2), 1);
	T0_out = (((int16_t)cmd[2])<<8) | (int16_t)cmd[1];


	// Read T1_OUT less significant bit
	cmd[0] = T1_OUT_L;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+1), 1);

	// Read T1_OUT most significant bit
	cmd[0] = T1_OUT_M;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+2), 1);
	T1_out = (((int16_t)cmd[2])<<8) | (int16_t)cmd[1];


	// Read T_OUT less significant bit
	cmd[0] = T_OUT_L;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+1), 1);

	// Read T_OUT most significant bit
	cmd[0] = T_OUT_M;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+2), 1);
	T_out = (((int16_t)cmd[2])<<8) | (int16_t)cmd[1];

	// Calculate the temperature value
	tmp = ((int32_t)(T_out - T0_out)) * ((int32_t)(T1_degc - T0_degc));
	temperature = ((float)tmp / (float)(T1_out - T0_out)) + (float)(T0_degc);

	return temperature;
}


float humidity_read(void){
	uint16_t H0_T0_out;
	uint16_t H1_T0_out;
	uint16_t H_out;
	uint16_t H0_rh;
	uint16_t H1_rh;

	uint8_t cmd[3];
	int32_t tmp;

	static float humidity = 0.0;

	// Read H0_rh and H1_rh coefficients
	cmd[0] = HO_RH_X2;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+1), 1);

	cmd[0] = H1_RH_X2;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+2), 1);

	// Divide by two the content of registers H0_RH_X2 and H1_RH_X2
	H0_rh = cmd[1]>>1;
	H1_rh = cmd[2]>>1;

	// Read H0_T0_OUT less significant bit
	cmd[0] = HO_TO_OUT_L;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+1), 1);

	// Read H0_T0_OUT most significant bit
	cmd[0] = HO_TO_OUT_M;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+1), 1);
	H0_T0_out  = (((uint16_t)cmd[2])<<8) | (uint16_t)cmd[1];

	// Read H1_T0_OUT less significant bit
	cmd[0] = H1_TO_OUT_L;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+1), 1);

	// Read H1_T0_OUT most significant bit
	cmd[0] = H1_TO_OUT_M;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+2), 1);
	H1_T0_out  = ((uint16_t)cmd[2]<<8) | (uint16_t)cmd[1];

	// Read H_OUT less significant bit
	cmd[0] = H_OUT_L;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+1), 1);

	// Read H1_T0_OUT most significant bit
	cmd[0] = H_OUT_M;
	i2c_transfer7(I2C2, HTS221_ADDR, cmd, 1, (cmd+2), 1);
	H_out  = (((uint16_t)cmd[2])<<8) | (uint16_t)cmd[1];

	tmp = ((int32_t)(H_out - H0_T0_out)) * ((int32_t)H1_rh - H0_rh);
	humidity = ((((float)tmp  / (float)(H1_T0_out - H0_T0_out)) + (float)H0_rh));

	if(humidity > 1000.0)
		humidity = 1000.0;

	return humidity;
}
