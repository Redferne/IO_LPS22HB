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

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "IO_LPS22HB.h"

IO_LPS22HB::IO_LPS22HB()
{
}
bool IO_LPS22HB::begin(uint8_t address, uint8_t mode, int8_t cs, SPIClass * spi) {
	_address = address;
	_cs = cs;
	_spi = spi;
	_mode = mode;

	if (_cs == -1) {
		Wire.begin(); // Bad pratice...
		if (whoAmI() != LPS22HB_WHO_AM_I_VALUE)
			return false;
		write8(LPS22HB_RES_CONF, 0x0); // normal mode
		write8(LPS22HB_CTRL_REG1, _mode);
	} else {
		pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
		if (read8(LPS22HB_WHO_AM_I) != LPS22HB_WHO_AM_I_VALUE)
			return false;

		uint8_t data;
		data = read8(LPS22HB_RES_CONF);
		data &= ~LPS22HB_LP_MODE;
		write8(LPS22HB_RES_CONF, data);
		data = read8(LPS22HB_CTRL_REG1);
		data &= ~LPS22HB_BDU_EN;
		data |= mode;
		write8(LPS22HB_CTRL_REG1, data);
		data = read8(LPS22HB_CTRL_REG2);
		data = data | LPS22HB_I2C_DIS;
		write8(LPS22HB_CTRL_REG2, data);
	}
	return true;
}

uint8_t IO_LPS22HB::whoAmI() {
	if (_cs == -1) {
		Wire.beginTransmission(_address);
		Wire.write(LPS22HB_WHO_AM_I);
		Wire.endTransmission();
		Wire.requestFrom(_address, 1);
		return Wire.read();
	} else {
		return read8(LPS22HB_WHO_AM_I);
	}
}

float IO_LPS22HB::readPressure() {

	if (_mode == LPS22HB_ODR_ONESHOT) {
		uint8_t data = read8(LPS22HB_CTRL_REG2);
		data |= 1;
		write8(LPS22HB_CTRL_REG2, data); // Trigger oneshot measurement
	}

	if (status(LPS22HB_PDA))
		return NAN;

	uint8_t pressOutXL = read8(LPS22HB_PRES_OUT_XL);
	uint8_t pressOutL = read8(LPS22HB_PRES_OUT_L);
	uint8_t pressOutH = read8(LPS22HB_PRES_OUT_H);

	long val = ( ((long)pressOutH << 24) | ((long)pressOutL << 16) | ((long)pressOutXL << 8)) >> 8;
	return val / 4096.0f; // hPa
}

uint32_t IO_LPS22HB::readPressureRAW() {

	if (_mode == LPS22HB_ODR_ONESHOT) {
		uint8_t data = read8(LPS22HB_CTRL_REG2);
		data |= 1;
		write8(LPS22HB_CTRL_REG2, data); // Trigger oneshot measurement
	}

	if (status(LPS22HB_PDA))
		return 0;

	uint8_t pressOutXL = read8(LPS22HB_PRES_OUT_XL);
	uint8_t pressOutL = read8(LPS22HB_PRES_OUT_L);
	uint8_t pressOutH = read8(LPS22HB_PRES_OUT_H);

	int32_t val = ((pressOutH << 24) | (pressOutL << 16) | (pressOutXL << 8));
	val >> 8;
	val = val + 0x400000;
	//if (val == 1.00) readPressure();
	return (uint32_t)val;
}

uint32_t IO_LPS22HB::readPressureUI() {

	if (_mode == LPS22HB_ODR_ONESHOT) {
		uint8_t data = read8(LPS22HB_CTRL_REG2);
		data |= 1;
		write8(LPS22HB_CTRL_REG2, data); // Trigger oneshot measurement
	}

	if (status(LPS22HB_PDA))
		return 0;

	uint8_t pressOutXL = read8(LPS22HB_PRES_OUT_XL);
	uint8_t pressOutL = read8(LPS22HB_PRES_OUT_L);
	uint8_t pressOutH = read8(LPS22HB_PRES_OUT_H);

	uint32_t val = ((pressOutH << 24) | (pressOutL << 16) | (pressOutXL << 8)) >> 8;
	return val/4096;
}

float IO_LPS22HB::readTemperature() {

	if (_mode == LPS22HB_ODR_ONESHOT) {
		uint8_t data = read8(LPS22HB_CTRL_REG2);
		data |= 1;
		write8(LPS22HB_CTRL_REG2, data); // Trigger oneshot measurement
	}

	if (status(LPS22HB_TDA))
		return NAN;

	uint8_t tempOutL = read8(LPS22HB_TEMP_OUT_L);
	uint8_t tempOutH = read8(LPS22HB_TEMP_OUT_H);

	int16_t val = (tempOutH << 8) | (tempOutL & 0xff);
	return ((float)val)/100.0f;
}

bool IO_LPS22HB::status(uint8_t status) {

	uint8_t count = 1000 / 5;
	uint8_t data;
	while (count--) {
		if (read8(LPS22HB_STATUS_REG) & status)
			return false;
		delay(5);
	}
	return true;
}

uint8_t IO_LPS22HB::read8(uint8_t reg) {
	uint8_t value;
	if (_cs == -1) {
		Wire.beginTransmission(_address);
		Wire.write(reg);
		Wire.endTransmission();
		Wire.requestFrom(_address, 1);
		return Wire.read();
	} else {
		_spi->beginTransaction(SPISettings(LPS22HB_SPI_SPEED, LPS22HB_SPI_BITORDER, LPS22HB_SPI_MODE));
    digitalWrite(_cs, LOW);
		_spi->transfer(reg | 0x80); // Read!
		value = _spi->transfer(0);
    digitalWrite(_cs, HIGH);
		_spi->endTransaction();
		return value;
	}
}

void IO_LPS22HB::write8(uint8_t reg, uint8_t data) {
	if (_cs == -1) {
		Wire.beginTransmission(_address);
		Wire.write(reg);
		Wire.write(data);
		Wire.endTransmission();
	} else {
		_spi->beginTransaction(SPISettings(LPS22HB_SPI_SPEED, LPS22HB_SPI_BITORDER, LPS22HB_SPI_MODE));
    digitalWrite(_cs, LOW);
		_spi->transfer(reg);
		_spi->transfer(data);
    digitalWrite(_cs, HIGH);
		_spi->endTransaction();
	}
	return;
}
