/*
 * Portable I2C drivers for various devices.
 * Bosch BME280 temperature/pressure/humidity sensor.
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

#ifndef _BME280_H_
#define _BME280_H_

#define BME280_I2C_ADDR         0x76

#define BME280_REG_C1_BASE      0x88
#define BME280_REG_C2_BASE      0xE1
#define BME280_REG_ID           0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_SENS_BASE    0xF7

#define BME280_CALDATA_LEN      42
#define BME280_CALDATA1_LEN     26
#define BME280_CALDATA2_LEN     16
#define BME280_SENSDATA_LEN     12

#define BME280_SMP_DISABLE          0x00
#define BME280_OVERSMP_TEMP_1X      (0x01 << 5)
#define BME280_OVERSMP_TEMP_2X      (0x02 << 5)
#define BME280_OVERSMP_TEMP_4X      (0x03 << 5)
#define BME280_OVERSMP_TEMP_8X      (0x04 << 5)
#define BME280_OVERSMP_TEMP_16X     (0x05 << 5)

#define BME280_OVERSMP_PRESS_1X     (0x01 << 2)
#define BME280_OVERSMP_PRESS_2X     (0x02 << 2)
#define BME280_OVERSMP_PRESS_4X     (0x03 << 2)
#define BME280_OVERSMP_PRESS_8X     (0x04 << 2)
#define BME280_OVERSMP_PRESS_16X    (0x05 << 2)

#define BME280_MODE_SLEEP           0x00
#define BME280_MODE_FORCE           0x01
#define BME280_MODE_NORMAL          0x03

#define BME280_OVERSMP_HUM_1X       0x01
#define BME280_OVERSMP_HUM_2X       0x02
#define BME280_OVERSMP_HUM_4X       0x03
#define BME280_OVERSMP_HUM_8X       0x04
#define BME280_OVERSMP_HUM_16X      0x05

#include "i2c.h"

struct bme280_ctx
{
    struct i2c_bus  *bus;
    
    uint16_t    T1;
    int16_t     T2;
    int16_t     T3;
    uint16_t    P1;
    int16_t     P2;
    int16_t     P3;
    int16_t     P4;
    int16_t     P5;
    int16_t     P6;
    int16_t     P7;
    int16_t     P8;
    int16_t     P9;
    uint8_t     H1;
    int16_t     H2;
    uint8_t     H3;
    int16_t     H4;
    int16_t     H5;
    int8_t      H6;

    int32_t     raw_press;
    int32_t     raw_temp;
    int32_t     raw_hum;

    double      t_fine;
};

struct bme280_data
{
    double temp;
    double press;
    double hum;
};

int bme280_init(struct i2c_bus *, struct bme280_ctx *);
void bme280_get_data(struct bme280_ctx *, struct bme280_data *);
void bme280_configure(struct bme280_ctx *, uint8_t, uint8_t, uint8_t, uint8_t);

#endif /* _BME280_H_ */
