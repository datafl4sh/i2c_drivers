/*
 * Portable I2C drivers for various devices.
 * I2C low level.
 * Copyright (C) 2018  Matteo Cicuttin
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>
#include <stdlib.h>

struct i2c_bus {
    int fd;
};

int i2c_bus_open(struct i2c_bus *, const char *);
int i2c_bus_read(struct i2c_bus *, uint8_t, uint8_t *, size_t);
int i2c_bus_write(struct i2c_bus *, uint8_t, uint8_t *, size_t);
int i2c_bus_close(struct i2c_bus *);

#endif /* _I2C_H_ */
