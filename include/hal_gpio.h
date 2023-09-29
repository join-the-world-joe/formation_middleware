/**
* @file     hal_gpio.h
* @brief    hal_gpio.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     09/11/2018
*/

#ifndef __HAL_GPIO_H__
#define __HAL_GPIO_H__

int hal_gpio_init(register_gpio_port_parameters*);
int hal_gpio_ioctl(gpio_ioctl_parameters*);
int hal_gpio_deinit(unregister_gpio_port_parameters*);

#endif // __HAL_GPIO_H__
