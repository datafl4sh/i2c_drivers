/*
 * Portable I2C drivers for various devices.
 * DHT12 temperature/humidity sensor.
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

#include "dht12.h"

#define DHT12_I2C_ADDR  0x5C

void dht12_read(struct i2c_bus *bus, struct dht12_data *data)
{
    int err;
    uint8_t addr = DHT12_I2C_ADDR;
    uint8_t reg = 0x00;
    err = i2c_bus_write(bus, addr, &reg, 1);
        
    uint8_t raw_data[5];
    err = i2c_bus_read(bus, addr, raw_data, 5);
    
    data->hum = raw_data[0] + 0.1*raw_data[1];
    data->temp = raw_data[2] + 0.1*raw_data[3];
}
