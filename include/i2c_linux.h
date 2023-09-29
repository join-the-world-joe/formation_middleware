/**
* @file     i2c_linux.h
* @brief    i2c_linux.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     01/12/2018
*/

#ifndef __I2C_LINUX_H__
#define __I2C_LINUX_H__

#define I2C_BUS_1_SIGNATURE "i2c-1"

int i2c_linux_init(register_i2c_bus_parameters *);
int i2c_linux_ioctl(i2c_ioctl_parameters *);
int i2c_linux_deinit(unregister_i2c_bus_parameters *);

#endif // __I2C_LINUX_H__

