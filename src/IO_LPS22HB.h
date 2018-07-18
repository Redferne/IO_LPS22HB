/***************************************************************************
  This is a library for the LPS22HB Absolute Digital Barometer

  Designed to work with all kinds of LPS22HB Breakout Boards

  These sensors use I2C, 2 pins are required to interface, as this :
	VDD to 3.3V DC
	SCL to A5
	SDA to A4
	GND to common groud 

  Written by Adrien Chapelet for IoThings
 ***************************************************************************/

#ifndef _IO_LPS22HB_h

#define _IO_LPS22HB_h
#include <Arduino.h>


#define LPS22HB_WHO_AM_I	  0x0F //Who am I
#define LPS22HB_RES_CONF	  0x1A //Resolution
#define LPS22HB_CTRL_REG1	  0x10
#define LPS22HB_CTRL_REG2	  0x11
#define LPS22HB_STATUS_REG	0x27
#define LPS22HB_PRES_OUT_XL	0x28 //LSB
#define LPS22HB_PRES_OUT_L	0x29
#define LPS22HB_PRES_OUT_H	0x2A //MSB
#define LPS22HB_TEMP_OUT_L	0x2B //LSB
#define LPS22HB_TEMP_OUT_H	0x2C //MSB

#define LPS22HB_ODR_75_HZ   0x50
#define LPS22HB_ODR_50_HZ   0x40
#define LPS22HB_ODR_25_HZ   0x30
#define LPS22HB_ODR_10_HZ   0x20
#define LPS22HB_ODR_1_HZ    0x10
#define LPS22HB_ODR_ONESHOT 0x0

#define LPS22HB_TDA         0x02
#define LPS22HB_PDA         0x01
#define LPS22HB_LP_MODE     0x01
#define LPS22HB_BDU_EN      0x02
#define LPS22HB_ADD_INC     0x10
#define LPS22HB_I2C_DIS     0x08
#define LPS22HB_SW_RESET    0x04


#define LPS22HB_WHO_AM_I_VALUE	0xB1 // Expected return value of WHO_AM_I register

#define LPS22HB_SPI_SPEED           10000000
#define LPS22HB_SPI_BITORDER        MSBFIRST
#define LPS22HB_SPI_MODE            SPI_MODE0

class IO_LPS22HB {
public:
	IO_LPS22HB();

	bool begin(uint8_t address, uint8_t mode=LPS22HB_ODR_ONESHOT, int8_t cs=-1, SPIClass * spi=NULL);

	uint8_t whoAmI();
	float readTemperature();

	float readPressure();
	uint32_t readPressureUI();
	uint32_t readPressureRAW();

private:
	uint8_t _address;
	uint8_t read8(uint8_t reg);
	void write8(uint8_t reg, uint8_t data);
	bool status(uint8_t data);
	SPIClass * _spi;
	int8_t _cs;
	uint8_t _mode;
};

#endif
