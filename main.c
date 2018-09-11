/*
 * Portable I2C drivers for various devices.
 * 
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
#include <unistd.h>
#include "bme280.h"
#include "dht12.h"

int main(void)
{
    struct i2c_bus bus;
    struct bme280_ctx ctx;
    
    i2c_bus_open(&bus, "/dev/iic1");
    
    bme280_init(&bus, &ctx);
    
    FILE *fp;
    fp = fopen("sensordata.dat", "w");
    
    while(1)
    {
        bme280_configure(&ctx, BME280_OVERSMP_TEMP_16X, BME280_OVERSMP_PRESS_16X, BME280_OVERSMP_HUM_16X, BME280_MODE_FORCE);
        sleep(1);
        struct bme280_data sensor_data;
        bme280_get_data(&ctx, &sensor_data);
        sleep(1);
        
        struct dht12_data dht12d;
        dht12_read(&bus, &dht12d);
        
        printf("%f %f %f ", sensor_data.temp, sensor_data.press, sensor_data.hum);
        printf("%f %f\n", dht12d.temp, dht12d.hum);
        
        fprintf(fp, "%f %f %f ", sensor_data.temp, sensor_data.press, sensor_data.hum);
        fprintf(fp, "%f %f\n", dht12d.temp, dht12d.hum);
        
        fflush(fp);
        
        sleep(3);
    }
    
    fclose(fp);
    
    i2c_bus_close(&bus);
}
