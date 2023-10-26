
#ifndef USBTMC_APP_H
#define USBTMC_APP_H

void     usbtmc_app_task_iter(void);

/*
 * Copyright (c) 2016, Alex Taradov <alex@taradov.com>
 * All rights reserved.
*/

#ifndef _I2C_MASTER_H_
#define _I2C_MASTER_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/*- Prototypes --------------------------------------------------------------*/
int i2c_init(int freq);
bool i2c_start(int addr);
bool i2c_stop(void);
bool i2c_read_byte(uint8_t *byte, bool last);
bool i2c_write_byte(uint8_t byte);
bool i2c_busy(int addr);
void i2c_pins(int mask, int value);
void ftoa(float num, char *str);

#endif // _I2C_MASTER_H_


#endif
