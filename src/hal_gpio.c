
/**
* @file     hal_gpio.c
* @brief    hal_gpio.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     09/11/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "log_controller.h"
#include "gpio_controller.h"

#include "gpio_raspberry_pi.h"

#include "debug.h"

/*
@param[in]      signature
                The signature of gpio port
@param[in]      direction
                The direction of gpio port
@param[out]     dev_fd
                The file descriptor of gpio port
*/
int hal_gpio_init(register_gpio_port_parameters *gpio_port_param)
{
    int ret = 0, length, gpio_number;
    char *_signature = NULL;
    uint8_t _direction;
    register_gpio_port_parameters *_gpio_port_param = gpio_port_param;

    if(!_gpio_port_param) {
        printf("hal_gpio_init incorrect parameters\n");
        return -1;
    }

    _direction = _gpio_port_param->in.direction;
    if(_direction != GPIO_DIRECTION_IN && _direction != GPIO_DIRECTION_OUT) {
        printf("hal_gpio_init incorrect direction\n");
        return -1;
    }

    _signature = _gpio_port_param->in.signature;
    length = strlen(_signature);
    if(length == 0 || length > MAX_LENGTH_OF_GPIO_PORT_SIGNATURE) {
        printf("hal_gpio_init incorrect parameters\n");
        return -1;
    }

#ifdef DATA_FLOW_DEBUG
    if(!strcmp(_signature, GPIO_SIGNATURE_AD_DRDY)) {
        gpio_number = GPIO_PIN_NB_AD_DRDY;
#if 0
    } else if(!strcmp(_signature, GPIO_SIGNATURE_AD_START)) {
        gpio_number = GPIO_PIN_NB_AD_START;
#endif
    } else if(!strcmp(_signature, GPIO_SIGNATURE_DA_LDAC)) {
        gpio_number = GPIO_PIN_NB_DA_LDAC;
    } else if(!strcmp(_signature, GPIO_SIGNATURE_RESET)) {
        gpio_number = GPIO_PIN_NB_RESET;
    } else if(!strcmp(_signature, GPIO_SIGNATURE_LED_READY)) {
        gpio_number = GPIO_PIN_NB_LED_READY;
    } else if(!strcmp(_signature, GPIO_SIGNATURE_LED_RUN)) {
        gpio_number = GPIO_PIN_NB_LED_RUN;
    } else if(!strcmp(_signature, GPIO_SIGNATURE_LED_ERROR)) {
        gpio_number = GPIO_PIN_NB_LED_ERROR;
    } else if(!strcmp(_signature, GPIO_SIGNATURE_BELL)) {
        gpio_number = GPIO_PIN_NB_BELL;
    } else {
        printf("unknown signature %s\n", _signature);
        return -1;
    }

    _gpio_port_param->out.dev_fd = gpio_number;
    printf("DATA_FOW_DEBUG(GPIO): hal_gpio_init, signature:%s, gpio_number:%d, dev_fd:%d\n", _signature, gpio_number, _gpio_port_param->out.dev_fd);
    return 0;
#endif

    ret = raspberry_pi_gpio_init(_gpio_port_param);
    if(ret < 0) {
        printf("hal_gpio_init->raspberry_pi_gpio_init fail\n");
        return -1;
    }

    return ret;
}

int hal_gpio_ioctl(gpio_ioctl_parameters *gpio_ioctl_param)
{
    int ret = 0, _cmd;
    //uint8_t _level;
    int *_level_in, _level_out;
    int32_t _dev_fd;
    uint8_t _direction;
    gpio_ioctl_parameters *_gpio_ioctl_param = gpio_ioctl_param;

    if(!_gpio_ioctl_param) {
        printf("hal_gpio_ioctl fail, incorrect parameters\n");
        return -1;
    }

    _cmd = _gpio_ioctl_param->cmd;

    switch(_cmd) {

        case GPIO_IOCTL_CMD_SET_LEVEL:
             _dev_fd = _gpio_ioctl_param->set_level_param.in.dev_fd;
             if(_dev_fd < 0) {
                 printf("hal_gpio_ioctl fail, incorrect dev_fd %d\n", _dev_fd);
                 return -1;
             }

             _direction = _gpio_ioctl_param->set_level_param.in.direction;
             if(_direction!=GPIO_DIRECTION_OUT) {
                 printf("hal_gpio_ioctl fail, incorrect direction %d\n", _direction);
                 return -1;
             }

             _level_out = _gpio_ioctl_param->set_level_param.in.level;
             if(_level_out!=GPIO_LEVEL_HIGH && _level_out !=GPIO_LEVEL_LOW) {
                 printf("hal_gpio_ioctl fail, incorrect level value %d\n", _level_out);
                 return -1;
             }

#if 0
             _level = _gpio_ioctl_param->set_level_param.in.level;
             if(_level!=GPIO_LEVEL_HIGH && _level !=GPIO_LEVEL_LOW) {
                 printf("hal_gpio_ioctl fail, incorrect level value %d\n", _level);
                 return -1;
             }
#endif

#ifdef DATA_FLOW_DEBUG
             printf("DATA_FOW_DEBUG(GPIO): hal_gpio_ioctl.set_level, dev_fd:%d level:%s\n", _dev_fd, _level_out==GPIO_LEVEL_HIGH?"High":"Low");
             return 0; 
#endif             

             ret = gpio_set_level(_gpio_ioctl_param);
             if(ret < 0) {
                 printf("hal_gpio_ioctl->gpio_set_level fail\n");
                 return -1;
             }

             return 0;
        break;

        case GPIO_IOCTL_CMD_GET_LEVEL:
             _dev_fd = _gpio_ioctl_param->get_level_param.temp.dev_fd;
             if(_dev_fd < 0) {
                 printf("hal_gpio_ioctl fail, incorrect dev_fd %d\n", _dev_fd);
                 return -1;
             }

             _direction = _gpio_ioctl_param->get_level_param.temp.direction;
             if(_direction!=GPIO_DIRECTION_IN) {
                 printf("hal_gpio_ioctl fail, incorrect direction %d\n", _direction);
                 return -1;
             }

             _level_in = _gpio_ioctl_param->get_level_param.out.level;
             if(!_level_in) {
                 printf("hal_gpio_ioctl fail, incorrect level_in NULL \n");
                 return -1;
             }

#ifdef DATA_FLOW_DEBUG
             printf("DATA_FOW_DEBUG(GPIO): hal_gpio_ioctl.get_level, dev_fd:%d\n", _dev_fd);
             if(*_level_in == GPIO_LEVEL_LOW) {
                 *_level_in = GPIO_LEVEL_HIGH;
             } else {
                 *_level_in = GPIO_LEVEL_LOW;
             }
             return 0; 
#endif             

             ret = gpio_get_level(_gpio_ioctl_param);
             if(ret < 0) {
                 printf("hal_gpio_ioctl->gpio_set_level fail\n");
                 return -1;
             }

             return 0;
        break;

        default:
             printf("hal_gpio_ioctl fail, un-supported cmd %d", _cmd);
             return -1;
        break;

    }

    return ret;
}


/*
@param[in]      signature
                The signature of gpio port
@param[out]     dev_fd
                The file descriptor of gpio port
*/
int hal_gpio_deinit(unregister_gpio_port_parameters *gpio_port_param)
{
    int ret = 0, _dev_fd;
    unregister_gpio_port_parameters *_gpio_port_param = gpio_port_param;
  
    if(!_gpio_port_param) {
        printf("hal_gpio_deinit incorrect parameters\n");
        return -1;
    }
    
    _dev_fd = _gpio_port_param->temp.dev_fd;
    if(_dev_fd < 0) {
        printf("hal_gpio_deinit incorrect device file descriptor\n");
        return -1;
    }
    
#ifdef DATA_FLOW_DEBUG
    printf("DATA_FLOW_DEBUG(GPIO): hal_gpio_deinit, dev_fd = %d\n", _dev_fd);
    return 0;
#endif

    ret = raspberry_pi_gpio_deinit(_gpio_port_param);
    if(ret < 0) {
        printf("hal_gpio_deinit->raspberry_pi_gpio_deinit fail\n");
        return -1;
    }

    return ret;
}

