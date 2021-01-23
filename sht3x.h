/*
 * sht3.h
 *
 *  Created on: 30/9/2019
 *      Author: Jesús
 */

#ifndef SHT3X_H_
#define SHT3X_H_

#include "driverlib.h"
#include "hal_LCD.h"

//**********************************************************************************************************************************************************
// Defines for I2C communication
#define SHT3X_I2C_ADDRESS 0x44
#define SHT3X_MEASUREMENT_MSB 0x24
#define SHT3X_MEASUREMENT_LSB 0x16

//**********************************************************************************************************************************************************
// Function prototypes
void initSht3x(void);
void getMeasure(void);
static void initWrite(void);
static void initRead(void);
uint16_t getTemp(void);
uint16_t getHum(void);
void showTempHum(uint16_t, uint16_t);

#endif /* SHT3X_H_ */
