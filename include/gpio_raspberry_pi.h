
/**
* @file     gpio_raspberry_pi.h
* @brief    gpio_raspberry_pi.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     22/09/2018
*/

#ifndef __GPIO_RASPBERRY_PI_H__
#define __GPIO_RASPBERRY_PI_H__

#include "gpio_controller.h"

#define GPIO_SIGNATURE_AD_DRDY  "AD-DRDY"
//#define GPIO_SIGNATURE_AD_START "AD-START"
#define GPIO_SIGNATURE_DA_LDAC  "DA-LDAC"
#define GPIO_SIGNATURE_RESET    "RESET"
#define GPIO_SIGNATURE_BELL     "BELL"
#define GPIO_SIGNATURE_AC_LOSS       "AC-LOSS"
#define GPIO_SIGNATURE_LED_READY     "LED-READY"
#define GPIO_SIGNATURE_LED_RUN       "LED-RUN"
#define GPIO_SIGNATURE_LED_ERROR     "LED-ERROR"



#define GPIO_PIN_NB_AD_DRDY           7
//#define GPIO_PIN_NB_AD_START       27
#define GPIO_PIN_NB_DA_LDAC          22
#define GPIO_PIN_NB_RESET            18
#define GPIO_PIN_NB_BELL             16
#define GPIO_PIN_NB_AC_LOSS          12
#define GPIO_PIN_NB_LED_READY        25
#define GPIO_PIN_NB_LED_RUN          24
#define GPIO_PIN_NB_LED_ERROR        23



#define EXPORT_TO_DIRECTION_DELAY 50000

int raspberry_pi_gpio_init(register_gpio_port_parameters *);
int gpio_set_level(gpio_ioctl_parameters *);
int gpio_get_level(gpio_ioctl_parameters *);
int raspberry_pi_gpio_deinit(unregister_gpio_port_parameters *);

#endif // __GPIO_RASPBERRY_PI_H__

