/*
 * hts221.h
 *
 *  Created on: 30 Apr 2020
 *      Author: silvere
 */

#ifndef HTS221_H_
#define HTS221_H_

#define SCL GPIO10
#define SDA GPIO11
#define HTS221_ADDR 0x5F
#define WHO_I_AM 0x0F


#define H_OUT_L 0x28 //Output for humidity less significant bit
#define H_OUT_M 0x29 //Output for humidity most significant bit
#define HO_RH_X2 0x30
#define H1_RH_X2 0x31
#define HO_TO_OUT_L 0x36
#define HO_TO_OUT_M 0x37
#define H1_TO_OUT_L 0x3A
#define H1_TO_OUT_M 0x3B


#define T_OUT_L 0x2A //Output for temperature less significant bit
#define T_OUT_M 0x2B //Output for temperature most significant bit
#define	TO_DEGC_X8 0x32
#define T1_DEGC_X8 0x33
#define T1_T0_MSB 0x35
#define T0_OUT_L 0x3C
#define T0_OUT_M 0x3D
#define T1_OUT_L 0x3E
#define T1_OUT_M 0x3F

#define PD 7
#define HTS221_PD_ON (1 << PD)
#define BDU 2
#define HTS221_BDU_ON (1 << BDU)
#define ODR0 1
#define ODR1 1
#define HTS221_ODR0_ON (1 << ODR0)
#define HTS221_ODR1_ON (1 << ODR1)
#define CTRL_REG1 0x20

void clock_setup(void);
void uart_setup(void);
void i2c2_setup(void);

void who_i_am(uint8_t dev, uint8_t reg);
void hts221_enable(void);
float temperature_read(void);
float humidity_read(void);
int _write(int file, char *ptr, int len);


#endif /* HTS221_H_ */
