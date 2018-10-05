/*
 * i2c.h
 *
 *  Created on: 04/10/2018
 *      Author: pm
 */

#ifndef I2C_H_
#define I2C_H_


void init_i2c(void);

int i2c_write(uint8_t addr, uint8_t reg, uint8_t *val, uint8_t size);

int i2c_read(uint8_t addr, uint8_t reg, uint8_t *val, uint8_t size);


#endif /* I2C_H_ */
