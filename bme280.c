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

#include <stdio.h>
#include "bme280.h"

int
bme280_init(struct i2c_bus *bus, struct bme280_ctx *ctx)
{
    uint8_t     caldata[BME280_CALDATA_LEN];
    uint8_t     reg;
    uint8_t     addr;
    int         err;
    
    if (!bus || !ctx)
        return -1;
    
    ctx->bus = bus;
    
    addr = BME280_I2C_ADDR;
    
    reg = BME280_REG_ID;
    err = i2c_bus_write(ctx->bus, addr, &reg, 1);
    err = i2c_bus_read(ctx->bus, addr, &reg, 1);
    
    if ( reg != 0x60 )
    {
        printf("bme280_init: device returned wrong ID\n");
        return -1;
    }
    
    
    reg = BME280_REG_C1_BASE;
    err = i2c_bus_write(ctx->bus, addr, &reg, 1);
    err = i2c_bus_read(ctx->bus, addr, caldata, BME280_CALDATA1_LEN);
    
    reg = BME280_REG_C2_BASE;
    err = i2c_bus_write(ctx->bus, addr, &reg, 1);
    err = i2c_bus_read(ctx->bus, addr, caldata+BME280_CALDATA1_LEN,
                       BME280_CALDATA2_LEN);

#ifdef BME280_DEBUG
    for (size_t i = 0; i < BME280_CALDATA1_LEN; i++)
        printf("%2.2x ", caldata[i]);
    printf("\n");
    
    for (size_t i = 0; i < BME280_CALDATA2_LEN; i++)
        printf("%2.2x ", caldata[BME280_CALDATA1_LEN+i]);
    printf("\n");
#endif

    ctx->T1 = caldata[1] << 8 | caldata[0];
    ctx->T2 = caldata[3] << 8 | caldata[2];
    ctx->T3 = caldata[5] << 8 | caldata[4];
    
    ctx->P1 = caldata[7] << 8 | caldata[6];
    ctx->P2 = caldata[9] << 8 | caldata[8];
    ctx->P3 = caldata[11] << 8 | caldata[10];
    ctx->P4 = caldata[13] << 8 | caldata[12];
    ctx->P5 = caldata[15] << 8 | caldata[14];
    ctx->P6 = caldata[17] << 8 | caldata[16];
    ctx->P7 = caldata[19] << 8 | caldata[18];
    ctx->P8 = caldata[21] << 8 | caldata[20];
    ctx->P9 = caldata[23] << 8 | caldata[22];
    
    ctx->H1 = caldata[25];
    ctx->H2 = (int16_t)(caldata[27] << 8 | caldata[26]);
    ctx->H3 = caldata[28];
    ctx->H4 = (caldata[30] & 0xF) | (caldata[29] << 4);
    ctx->H5 = ((caldata[30] >> 4) & 0xF) | (caldata[31] << 4);
    ctx->H6 = (int8_t)caldata[32];

    return 0;
}

static int
bme280_acquire_sensor_data(struct bme280_ctx *ctx)
{
    uint8_t sd[BME280_SENSDATA_LEN];
    uint8_t addr;
    uint8_t reg;
    int err;
    
    addr = BME280_I2C_ADDR;
    reg = BME280_REG_SENS_BASE;
    err = i2c_bus_write(ctx->bus, addr, &reg, 1);
    err = i2c_bus_read(ctx->bus, addr, sd, BME280_SENSDATA_LEN);
    
    ctx->raw_press = (sd[0] << 12) | (sd[1] << 4) | (sd[2] >> 4);
    ctx->raw_temp = (sd[3] << 12) | (sd[4] << 4) | (sd[5] >> 4);
    ctx->raw_hum = (sd[6] << 8) | sd[7];
    
    return 0;
}

static double
bme280_compensate_temperature(struct bme280_ctx *ctx)
{
    const double temp_min = -40.0;
    const double temp_max = 85.0;
    
    double q1, q2;
    q1 = (ctx->raw_temp/16384.0 - (ctx->T1/1024.0));
    q1 = q1 * ctx->T2;
    q2 = (ctx->raw_temp/131072.0 - (ctx->T1/8192.0));
    q2 = q2 * q2 * ctx->T3;
    ctx->t_fine = q1+q2;
    
    double temp = (q1+q2)/5120.0;
    
    if (temp < temp_min)
        return temp_min;
    if (temp > temp_max)
        return temp_max;
    return temp;
}

static double
bme280_compensate_pressure(struct bme280_ctx *ctx)
{
    const double press_min = 30000.0;
    const double press_max = 110000.0;
    
    double q1, q2, q3;
    q1 = (ctx->t_fine/2.0) - 64000.0;
    q2 = q1 * q1 * ctx->P6 / 32768.0;
    q2 = q2 + q1 * ctx->P5 * 2.0;
    q2 = (q2/4.0) + (ctx->P4 * 65536.0);
    q3 = ctx->P3 * q1 * q1 / 524288.0;
    q1 = (q3 + ctx->P2 * q1) / 524288.0;
    q1 = (1.0 + q1/32768.0) * ctx->P1;

    if (q1 != 0.0)
    {
        q3 = 1048576.0 - ctx->raw_press;
        q3 = (q3 - (q2/4096.0)) * 6250.0 / q1;
        q1 = (ctx->P9 * q3 * q3) / 2147483648.0;
        q2 = q3 * ctx->P8 / 32768.0;
        q3 = q3 + (q1 + q2 + ctx->P7)/16.0;
    
        if (q3 < press_min)
            return press_min;
        if (q3 > press_max)
            return press_max;
        return q3;
    }
    
    return press_min;
}

static double
bme280_compensate_humidity(struct bme280_ctx *ctx)
{
	const double humidity_min = 0.0;
	const double humidity_max = 100.0;
	double humidity;
    double q1, q2, q3, q4, q5, q6;

	q1 = ctx->t_fine - 76800.0;
	q2 = (((double)ctx->H4) * 64.0 + (((double)ctx->H5) / 16384.0) * q1);
	q3 = ctx->raw_hum - q2;
	q4 = ((double)ctx->H2) / 65536.0;
	q5 = (1.0 + (((double)ctx->H3) / 67108864.0) * q1);
	q6 = 1.0 + (((double)ctx->H6) / 67108864.0) * q1 * q5;
	q6 = q3 * q4 * (q5 * q6);
	humidity = q6 * (1.0 - ((double)ctx->H1) * q6 / 524288.0);

	if (humidity > humidity_max)
		humidity = humidity_max;
	else if (humidity < humidity_min)
		humidity = humidity_min;

    return humidity;
}

void
bme280_get_data(struct bme280_ctx *ctx, struct bme280_data *data)
{
    /* check for errors */
    bme280_acquire_sensor_data(ctx);
    data->temp = bme280_compensate_temperature(ctx);
    data->press = bme280_compensate_pressure(ctx)/100;
    data->hum = bme280_compensate_humidity(ctx);
}

void
bme280_configure(struct bme280_ctx *ctx, uint8_t temp_ovs,
                 uint8_t press_ovs, uint8_t hum_ovs, uint8_t mode)
{
    /* check for errors */
    int err;
    uint8_t addr = BME280_I2C_ADDR;
    uint8_t data[2];
    data[0] = BME280_REG_CTRL_MEAS;
    data[1] = temp_ovs | press_ovs | mode;
    err = i2c_bus_write(ctx->bus, addr, data, 2);

    data[0] = BME280_REG_CTRL_HUM;
    err = i2c_bus_read(ctx->bus, addr, data, 1);
    data[0] = (data[0] & 0x7) | hum_ovs;
    err = i2c_bus_write(ctx->bus, addr, data, 1);
}
