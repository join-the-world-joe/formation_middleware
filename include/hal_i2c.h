/**
* @file     hal_i2c.h
* @brief    hal_i2c.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     01/12/2018
*/

#ifndef __HAL_I2C_H__
#define __HAL_I2C_H__

int hal_i2c_init(register_i2c_bus_parameters *);
int hal_i2c_ioctl(i2c_ioctl_parameters *);
int hal_i2c_deinit(unregister_i2c_bus_parameters *);


#endif // __HAL_I2C_H__

