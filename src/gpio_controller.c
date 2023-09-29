
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "log_controller.h"
#include "gpio_controller.h"
#include "hal_gpio.h"

/*

                GPIO_Management         Logical_GPIOS               Physical_GPIOS             
                gpio_controller         gpio                        gpio_xxxx_Platform
Data Model      controller_context      gpio_context per port       gpio_ioctl_parameters
*/

int gpio_controller_init(gpio_controller_context *gpio_controller_ctx, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param)
{
    int ret = 0, _nb_gpio_ports = NB_GPIO_PORTS, i, j;
    gpio_context *_gpio_ctx = NULL;
    gpio_controller_context *_controller_ctx = gpio_controller_ctx;
    log_controller_context *_log_controller_ctx = log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param = log_controller_ioctl_param; 

    if(!_controller_ctx || !_log_controller_ctx || !_log_controller_ioctl_param) {
        printf("gpio_controller_init fail, incorrect parameters\n");
        return -1;
    }

    _controller_ctx->count = 0;
    _controller_ctx->_log_controller_ctx = _log_controller_ctx;
    _controller_ctx->_log_controller_ioctl_param = _log_controller_ioctl_param;
    _controller_ctx->nb_gpio_ports = _nb_gpio_ports;

    _gpio_ctx = _controller_ctx->gpio_ctx;
    
    // we'd better initialize all the important variables here    
    for(i=0; i<_nb_gpio_ports; i++) {
        _gpio_ctx[i].index = i;
        _gpio_ctx[i].in_use = 0;
        _gpio_ctx[i].dev_fd = -1;
        _gpio_ctx[i].direction = GPIO_DIRECTION_UNKNOWN;
        memset(_gpio_ctx[i].signature, 0, MAX_LENGTH_OF_GPIO_PORT_SIGNATURE);
    }

    _controller_ctx->ops.init = gpio_controller_init;
    _controller_ctx->ops.register_context = gpio_controller_register_gpio_port;
    _controller_ctx->ops.unregister_context = gpio_controller_unregister_gpio_port;
    _controller_ctx->ops.list_resources = gpio_controller_list_resources;
    _controller_ctx->ops.ioctl = gpio_controller_ioctl;
    _controller_ctx->ops.deinit = gpio_controller_deinit;

    _controller_ctx->io_ops.init = hal_gpio_init;
    _controller_ctx->io_ops.ioctl = hal_gpio_ioctl;
    _controller_ctx->io_ops.deinit = hal_gpio_deinit;

    return ret;
}


int gpio_controller_register_gpio_port(gpio_controller_context *gpio_controller_ctx, register_gpio_port_parameters *gpio_port_param)
{
    int ret = 0, i, done;
    uint8_t _direction;
    uint32_t _nb_gpio_ports;
    char *_port_signatrue = NULL;
    gpio_context *_gpio_ctx = NULL;
    gpio_ops *_io_ops = NULL;
    register_gpio_port_parameters *_gpio_port_param = gpio_port_param;
    gpio_controller_context *_controller_ctx = gpio_controller_ctx;

    if(!_controller_ctx || !_gpio_port_param) {
        printf("gpio_controller_bind fail, incorrect parameters\n");
        return -1;
    }

    _nb_gpio_ports = _controller_ctx->nb_gpio_ports;
    _gpio_ctx = _controller_ctx->gpio_ctx;
    _io_ops = &_controller_ctx->io_ops;
    _port_signatrue = _gpio_port_param->in.signature;
    _direction = _gpio_port_param->in.direction;

    if(_controller_ctx->count + 1 > _nb_gpio_ports) {
        printf("gpio context in controller is full, there might be something wrong\n");
        return -1;
    }

    ret = _io_ops->init(_gpio_port_param);
    if(ret < 0) {
        printf("gpio_controller_register->io_ops->init fail\n");
        return -1;
    }

    // first, find a free context 
    for(i=0, done=0; i<_nb_gpio_ports; i++) {

        if(_gpio_ctx[i].in_use == 0) {

            // record input parameters
            _gpio_ctx[i].direction = _direction; 
            strcpy(_gpio_ctx[i].signature, _port_signatrue);

            // record output parameters
            _gpio_ctx[i].dev_fd = _gpio_port_param->out.dev_fd;

            // change the status of this gpio_context
            _gpio_ctx[i].in_use = 1;

            done = 1; // all the related contents are recoreded in gpio_context

            break;
        }
    }

    if(!done) {
        printf("gpio_controller_register fail, can't record port %s\n", _port_signatrue);
        return -1;
    }

    _controller_ctx->count++; /* if done, increase the object counter */

    return ret;
}

int gpio_controller_unregister_gpio_port(gpio_controller_context *gpio_controller_ctx, unregister_gpio_port_parameters *gpio_port_param)
{
    int ret = 0, index, done, i, same, find;
    uint32_t _nb_gpio_ports;
    char *_port_signatrue = NULL;
    gpio_context *_gpio_ctx = NULL;
    gpio_ops *_io_ops = NULL;
    unregister_gpio_port_parameters *_gpio_port_param = gpio_port_param;
    gpio_controller_context *_controller_ctx = gpio_controller_ctx;

    if(!_controller_ctx || !_gpio_port_param) {
        printf("gpio_controller_unregister_gpio_port fail, incorrect parameters\n");
        return -1;
    }

    _nb_gpio_ports = _controller_ctx->nb_gpio_ports;
    _gpio_ctx = _controller_ctx->gpio_ctx;
    _io_ops = &_controller_ctx->io_ops;
    _port_signatrue = _gpio_port_param->in.signature;

    if((_controller_ctx->count - 1) < 0) {
        printf("gpio context in controller is empty, there might be something wrong\n");
        return -1;
    }

    // check if we can find this signature in gpio_context
    for(index=0, find=0; index<_nb_gpio_ports; index++) {

        same = !strcmp(_gpio_ctx[index].signature, _port_signatrue);
        if(same) {

            _gpio_port_param->temp.dev_fd = _gpio_ctx[index].dev_fd;

            find = 1;

            break;
        }
    }

    if(!find) {
        printf("signature %s doesn't match any recorded signature in gpio_context of gpio_controller\n", _port_signatrue);
        return -1;
    }

    ret = _io_ops->deinit(_gpio_port_param);
    if(ret < 0) {
        printf("gpio_controller_unregister_gpio_port->io_ops->deinit fail\n");
        return -1;
    }

    _gpio_ctx[index].in_use = 0;

    return ret;
}

void gpio_controller_deinit(gpio_controller_context *gpio_controller_ctx)
{
    int ret = 0, index;
    uint32_t _nb_gpio_ports;
    gpio_context *_gpio_ctx = NULL;
    gpio_ops *_io_ops = NULL;
    gpio_controller_context *_controller_ctx = gpio_controller_ctx;
    unregister_gpio_port_parameters gpio_port_param;

    if(!_controller_ctx) {
        printf("gpio_controller_deinit fail, incorrect parameters\n");
        return ;
    }

    _io_ops = &_controller_ctx->io_ops;
    _gpio_ctx = _controller_ctx->gpio_ctx; 
    _nb_gpio_ports = _controller_ctx->nb_gpio_ports;

    for(index=0; index<_nb_gpio_ports; index++) {
        if(_gpio_ctx[index].in_use) {
            memset(gpio_port_param.in.signature, 0, MAX_LENGTH_OF_GPIO_PORT_SIGNATURE);
            strcpy(gpio_port_param.in.signature, _gpio_ctx[index].signature);
            ret = _controller_ctx->ops.unregister_context(_controller_ctx, &gpio_port_param);
            if(ret < 0) {
                printf("gpio_controller_deinit->unregister_gpio_port fail, port %s\n", _gpio_ctx[index].signature);
                return ;
            }
        }
    }

    return ;
}

void gpio_controller_list_resources(gpio_controller_context *gpio_controller_ctx)
{
    int index;
    uint32_t _nb_gpio_ports;
    gpio_context *_gpio_ctx = NULL;
    gpio_controller_context *_controller_ctx = gpio_controller_ctx;

    if(!_controller_ctx) {
        printf("gpio_controller_list_resources fail, incorrect parameters\n");
        return ;
    }

    _gpio_ctx = _controller_ctx->gpio_ctx;
    _nb_gpio_ports = _controller_ctx->nb_gpio_ports;

    printf("***********************************GPIO Resources***********************************\n");
    for(index=0; index<_nb_gpio_ports; index++) {
        if(_gpio_ctx[index].in_use) {
            printf("Index: %d\n", index);
            printf("Port Signature: %s\n", _gpio_ctx[index].signature);
            printf("Direction: %s\n", _gpio_ctx[index].direction==GPIO_DIRECTION_OUT?"Output":"Input");
            printf("Device FD: %d\n", _gpio_ctx[index].dev_fd);
        }
    }
    printf("************************************************************************************\n");
}

int gpio_controller_ioctl(gpio_controller_context *gpio_controller_ctx, uint16_t cmd, gpio_controller_ioctl_parameters *gpio_controller_ioctl_param)
{
    int ret = 0, same, done, index;
    char *_signature = NULL;
    uint8_t _level;
    uint16_t _cmd = cmd;    
    uint32_t _nb_gpio_ports;
    gpio_ops *_io_ops = NULL;
    gpio_context *_gpio_ctx = NULL;
    gpio_ioctl_parameters gpio_ioctl_param;
    gpio_controller_context *_controller_ctx = gpio_controller_ctx;
    gpio_controller_ioctl_parameters *_controller_ioctl_param = gpio_controller_ioctl_param;
    
    if(!_controller_ctx || !_controller_ioctl_param || _cmd < GPIO_CONTROLLER_IOCTL_CMD_NB_BEGIN || _cmd > GPIO_CONTROLLER_IOCTL_CMD_NB_END) {
        printf("gpio_controller_ioctl incorrect parameters\n");
        return -1;
    }

    _nb_gpio_ports = _controller_ctx->nb_gpio_ports;
    _io_ops = &_controller_ctx->io_ops;
    
    switch(_cmd) {

        case GPIO_CONTROLLER_IOCTL_CMD_ACQUIRE_GPIO_CONTEX:
             _gpio_ctx = _controller_ctx->gpio_ctx;
             _signature = _controller_ioctl_param->acquire_gpio_ctx_param.in.signature;
              
             for(index=0, done=0; index<_nb_gpio_ports; index++) {
                 same = !strcmp(_signature, _gpio_ctx[index].signature);
                 if(same) {
    
                     if(_gpio_ctx[index].in_use) {
                         _controller_ioctl_param->temp.index = index;
    
                         _controller_ioctl_param->temp.gpio_ctx = &_gpio_ctx[index];   
    
                         done = 1;
                
                         break;
                     }
                 }
             }
    
             if(done == 0) {
                 printf("gpio_controller_ioctl->acquire_gpio_context fail\n");
                 return -1;
             }

             return 0; // acquire gpio context done!!!
        break;

        case GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL:
             //_level = _controller_ioctl_param->set_level_param.in.level;
             index = _controller_ioctl_param->temp.index;
             if(index < 0 || index >= _nb_gpio_ports) {
                 printf("gpio_controller_ioctl set level, incorrect index = %d\n", index);
                 return -1;
             }

             _gpio_ctx = _controller_ioctl_param->temp.gpio_ctx; 
             if(_gpio_ctx != &_controller_ctx->gpio_ctx[index]) {
                 printf("gpio_controller_ioctl, incorrect ioctl parameters\n");
                 return -1;
             }
             
             gpio_ioctl_param.cmd = GPIO_IOCTL_CMD_SET_LEVEL;
             gpio_ioctl_param.set_level_param.in.dev_fd = _gpio_ctx->dev_fd;
             gpio_ioctl_param.set_level_param.in.direction = _gpio_ctx->direction;
             gpio_ioctl_param.set_level_param.in.level = _controller_ioctl_param->set_level_param.in.level;
             ret = _io_ops->ioctl(&gpio_ioctl_param);
             if(ret < 0) {
                 printf("gpio_controller_ioctl set level -> io_ops->ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        case GPIO_CONTROLLER_IOCTL_CMD_GET_LEVEL:
             index = _controller_ioctl_param->temp.index;
             if(index < 0 || index >= _nb_gpio_ports) {
                 printf("gpio_controller_ioctl set level, incorrect index = %d\n", index);
                 return -1;
             }

             _gpio_ctx = _controller_ioctl_param->temp.gpio_ctx; 
             if(_gpio_ctx != &_controller_ctx->gpio_ctx[index]) {
                 printf("gpio_controller_ioctl, incorrect ioctl parameters\n");
                 return -1;
             }

             gpio_ioctl_param.cmd = GPIO_IOCTL_CMD_GET_LEVEL;
             gpio_ioctl_param.get_level_param.temp.dev_fd = _gpio_ctx->dev_fd;
             gpio_ioctl_param.get_level_param.temp.direction = _gpio_ctx->direction;
             gpio_ioctl_param.get_level_param.out.level = &_controller_ioctl_param->get_level_param.out.level;
             ret = _io_ops->ioctl(&gpio_ioctl_param);
             if(ret < 0) {
                 printf("gpio_controller_ioctl set level -> io_ops->ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        default:
             printf("gpio_controller_ioctl, unknown cmd %d\n", _cmd);
             return -1;
        break;
    }

    
    return ret;
}

#if 0
// for the purposes of testing the gpio module
int main(void)
{
    int ret = 0, cmd;
    logging_ctx log_ctx;
    gpio_controller_context gpio_controller_ctx;    
    register_gpio_port_parameters gpio_port_ad_start;
    register_gpio_port_parameters gpio_port_ad_drdy;
    register_gpio_port_parameters gpio_port_da_ldac;
    register_gpio_port_parameters gpio_port_ad_reset;
    register_gpio_port_parameters gpio_port_ad_bell;
    unregister_gpio_port_parameters unregister_gpio_port;
    gpio_controller_ioctl_parameters gpio_controller_ioctl_param;

    ret = gpio_controller_init(&gpio_controller_ctx, &log_ctx);
    if(ret < 0) {
        printf("gpio_controller_init fail\n");
        return -1;
    }
    
#if 0
    strcpy(gpio_port_ad_start.in.signature, "AD-START");
    gpio_port_ad_start.in.direction = GPIO_DIRECTION_OUT;
    ret = gpio_controller_register_gpio_port(&gpio_controller_ctx, &gpio_port_ad_start);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_ad_start.in.signature);
        return -1;
    }
#endif

    strcpy(gpio_port_ad_drdy.in.signature, "AD-DRDY");
    gpio_port_ad_drdy.in.direction = GPIO_DIRECTION_IN;
    ret = gpio_controller_ctx.ops.register_context(&gpio_controller_ctx, &gpio_port_ad_drdy);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_ad_drdy.in.signature);
        return -1;
    }

    strcpy(gpio_port_da_ldac.in.signature, "DA-LDAC");
    gpio_port_da_ldac.in.direction = GPIO_DIRECTION_OUT;
    ret = gpio_controller_ctx.ops.register_context(&gpio_controller_ctx, &gpio_port_da_ldac);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_da_ldac.in.signature);
        return -1;
    }

    strcpy(gpio_port_ad_reset.in.signature, "RESET");
    gpio_port_ad_reset.in.direction = GPIO_DIRECTION_OUT;
    ret = gpio_controller_ctx.ops.register_context(&gpio_controller_ctx, &gpio_port_ad_reset);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_ad_reset.in.signature);
        return -1;
    }

    strcpy(gpio_port_ad_bell.in.signature, "BELL");
    gpio_port_ad_bell.in.direction = GPIO_DIRECTION_OUT;
    ret = gpio_controller_ctx.ops.register_context(&gpio_controller_ctx, &gpio_port_ad_bell);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_ad_bell.in.signature);
        return -1;
    }

#if 0
    strcpy(gpio_port_ad_bell.in.signature, "BELL");
    gpio_port_ad_bell.in.direction = GPIO_DIRECTION_OUT;
    ret = gpio_controller_ctx.ops.register_context(&gpio_controller_ctx, &gpio_port_ad_bell);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_ad_bell.in.signature);
        return -1;
    }
#endif

    gpio_controller_ctx.ops.list_resources(&gpio_controller_ctx);

#if 0
    memset(unregister_gpio_port.in.signature, 0, MAX_LENGTH_OF_GPIO_PORT_SIGNATURE);
    strcpy(unregister_gpio_port.in.signature, "BELL");
    ret = gpio_controller_ctx.ops.unregister_context(&gpio_controller_ctx, &unregister_gpio_port);
    if(ret < 0) {
        printf("gpio_controller_unregister_gpio_port fail, port %s\n", unregister_gpio_port.in.signature);
        return -1;
    }
#endif

    gpio_controller_list_resources(&gpio_controller_ctx);

    strcpy(gpio_controller_ioctl_param.acquire_gpio_ctx_param.in.signature, "BELL");
    ret = gpio_controller_ctx.ops.ioctl(&gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_ACQUIRE_GPIO_CONTEX, &gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to acquire gpio context\n");
        return -1;
    }
    
    //printf("index = %d, dev_fd = %d\n", gpio_controller_ioctl_param.temp.index, gpio_controller_ioctl_param.temp.gpio_ctx->dev_fd);
    
    gpio_controller_ioctl_param.set_level_param.in.level = GPIO_LEVEL_HIGH;
    ret = gpio_controller_ctx.ops.ioctl(&gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL, &gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        return -1;
    }

    sleep(1);

    gpio_controller_ioctl_param.set_level_param.in.level = GPIO_LEVEL_LOW;
    ret = gpio_controller_ctx.ops.ioctl(&gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL, &gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        return -1;
    }

    sleep(1);

    strcpy(gpio_controller_ioctl_param.acquire_gpio_ctx_param.in.signature, "AD-DRDY");
    ret = gpio_controller_ioctl(&gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_ACQUIRE_GPIO_CONTEX, &gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to acquire gpio context\n");
        return -1;
    }

    while(1) {

        cmd = GPIO_CONTROLLER_IOCTL_CMD_GET_LEVEL;
        ret = gpio_controller_ioctl(&gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_GET_LEVEL, &gpio_controller_ioctl_param);
        if(ret < 0) {
            printf("gpio_controller_ioctl fail to acquire gpio context\n");
            continue;
        }

        printf("Level: %d\n", gpio_controller_ioctl_param.get_level_param.out.level);
        usleep(100000);
    }

    gpio_controller_ctx.ops.deinit(&gpio_controller_ctx);
    //gpio_controller_ctx.ops.deinit(&gpio_controller_ctx);
    //gpio_controller_ctx.ops.deinit(&gpio_controller_ctx);

    gpio_controller_ctx.ops.list_resources(&gpio_controller_ctx);

    return ret;
}
#endif
