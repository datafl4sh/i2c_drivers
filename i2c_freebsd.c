/*
 * Portable I2C drivers for various devices.
 * I2C low level, FreeBSD interface.
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

#include <sys/cdefs.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <unistd.h>
#include <strings.h>
#include <dev/iicbus/iic.h>

#include "i2c.h"

int
i2c_bus_open(struct i2c_bus *ctx, const char *bus)
{
    int err;
    
    bzero(ctx, sizeof(struct i2c_bus));
    
    if ( (ctx->fd = open(bus, O_RDWR)) < 0 )
    {
        perror("i2c_open");
        return -1;
    }

    return 0;
}

int
i2c_bus_read(struct i2c_bus *ctx, uint8_t addr, uint8_t *buf, size_t len)
{
    int err;
    struct iic_msg msg;
    struct iic_rdwr_data rdwr;
    
    msg.slave = (addr << 1) | 1;
    msg.len = len;
    msg.buf = buf;
    msg.flags = IIC_M_RD;
    
    rdwr.msgs = &msg;
    rdwr.nmsgs = 1;
    
    err = ioctl(ctx->fd, I2CRDWR, &rdwr);
    if ( err < 0 )
    {
        perror("i2c_read");
        return err;
    }

    return err;
}

int
i2c_bus_write(struct i2c_bus *ctx, uint8_t addr, uint8_t *buf, size_t len)
{
    int err;
    struct iic_msg msg;
    struct iic_rdwr_data rdwr;
    
    msg.slave = addr << 1;
    msg.len = len;
    msg.buf = buf;
    msg.flags = 0;
    
    rdwr.msgs = &msg;
    rdwr.nmsgs = 1;
    
    err = ioctl(ctx->fd, I2CRDWR, &rdwr);
    if ( err < 0 )
    {
        perror("i2c_write");
        return err;
    }

    return err;
}

int
i2c_bus_close(struct i2c_bus *ctx)
{
    return close(ctx->fd);
}
