/**
* @file     eeprom_controller.c
* @brief    eeprom_controller.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     12/11/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "log_controller.h"
#include "i2c_controller.h"
#include "eeprom_controller.h"
#include "hal_eeprom.h"

#include "debug.h"

#ifdef DATA_FLOW_DEBUG

#include "i2c_controller.h"

#endif

void eeprom_controller_list_resources(eeprom_controller_context *eeprom_controller_ctx)
{
    int _nb_eeprom_devices, index, _nb_bytes, j;
    eeprom_context *_eeprom_ctx = NULL;
    eeprom_controller_context *_controller_ctx = eeprom_controller_ctx;

    if(!_controller_ctx) {
        printf("eeprom_controller_list_resources fail, incorrect parameters\n");
        return ;
    }

    _eeprom_ctx = _controller_ctx->eeprom_ctx;
    _nb_eeprom_devices = _controller_ctx->nb_eeprom_devices;

    printf("***********************************IO Expander Resources***********************************\n");
    for(index=0; index<_nb_eeprom_devices; index++) {
        if(_eeprom_ctx[index].in_use) {
     
            printf("Index: %d\n", index);
            printf("Bus Type: %s\n", _eeprom_ctx[index].bus_type == EEPROM_BUS_TYPE_SPI?"SPI":"I2C");
            printf("EEP Rom Signature: %s\n", _eeprom_ctx[index].signature);
            printf("I2C Slave Address: 0x%x\n", _eeprom_ctx[index].dev_addr);

        }
    }
    printf("********************************************************************************************\n");

}

int eeprom_controller_init(eeprom_controller_context *eeprom_controller_ctx, log_controller_context *log_ctx)
{
    int ret = 0, _nb_eeprom_devices = NB_EEPROM_DEVICES, i;
    eeprom_context *_eeprom_ctx = NULL;
    log_controller_context *_log_ctx = log_ctx;
    eeprom_controller_context *_controller_ctx = eeprom_controller_ctx;

    if(!_controller_ctx || !_log_ctx) {
        printf("eeprom_controller_init fail, incorrect parameters\n");
        return -1;
    }

    _controller_ctx->count = 0;
    _controller_ctx->log_ctx = _log_ctx;   
    _controller_ctx->nb_eeprom_devices = _nb_eeprom_devices;

    _eeprom_ctx = _controller_ctx->eeprom_ctx;
    
    // we'd better initialize all the important variables here    
    for(i=0; i<_nb_eeprom_devices; i++) {
        _eeprom_ctx[i].index = i;
        _eeprom_ctx[i].in_use = 0;
        _eeprom_ctx[i].i2c_controller_ctx = NULL;
        _eeprom_ctx[i].i2c_controller_ioctl_param = NULL;
        _eeprom_ctx[i].bus_type = EEPROM_BUS_TYPE_UNKNOWN;
        memset(_eeprom_ctx[i].signature, 0, MAX_LENGTH_OF_EEPROM_SIGNATURE);
    }
    
    _controller_ctx->ops.init = eeprom_controller_init;
    _controller_ctx->ops.register_context = eeprom_controller_register_eeprom;
    //_controller_ctx->ops.unregister_context = io_expander_controller_unregister_io_expander;
    _controller_ctx->ops.list_resources = eeprom_controller_list_resources;
    _controller_ctx->ops.ioctl = eeprom_controller_ioctl;
    //_controller_ctx->ops.deinit = io_expander_controller_deinit;
    
    _controller_ctx->eeprom_ops.init = hal_eeprom_init;
    //_controller_ctx->io_expander_ops.ioctl = hal_io_expander_ioctl;
    _controller_ctx->eeprom_ops.deinit = hal_eeprom_deinit;

    return ret;
}

int eeprom_controller_register_eeprom(eeprom_controller_context *eeprom_controller_ctx, register_eeprom_parameters *eeprom_param)
{
    int ret = 0, i, done, _bus_type, j, _nb_bytes;
    char *_terminal_signature = NULL;
    uint32_t _nb_eeprom_devices;
    char *_eeprom_signatrue = NULL;
    eeprom_context *_eeprom_ctx = NULL;
    eeprom_ops *_eeprom_ops = NULL;
    register_eeprom_parameters *_eeprom_param = eeprom_param;
    eeprom_controller_context *_controller_ctx = eeprom_controller_ctx;

    if(!_controller_ctx || !_eeprom_param) {
        printf("eeprom_controller_register_eeprom fail, incorrect parameters\n");
        return -1;
    }

    _nb_eeprom_devices = _controller_ctx->nb_eeprom_devices;
    _eeprom_ctx = _controller_ctx->eeprom_ctx;
    _eeprom_ops = &_controller_ctx->eeprom_ops;
    _eeprom_signatrue = _eeprom_param->in.signature;
    _bus_type = _eeprom_param->in.bus_type;

    if(_bus_type != EEPROM_BUS_TYPE_IIC) {
        printf("we only support iic kind eeprom\n");
        return -1;
    }

    if(_controller_ctx->count + 1 > _nb_eeprom_devices) {
        printf("eeprom context in controller is full, there might be something wrong\n");
        return -1;
    }

    if(_eeprom_param->in.nb_bytes > MAX_MEMORY_SIZE_IN_BYTES_OF_EEPROM) {
        printf("eeprom_controller_register_eeprom fail, exceeding the max. amount of io ports");
        return -1;
    }

    ret = _eeprom_ops->init(_eeprom_param);
    if(ret < 0) {
        printf("eeprom_controller_register_eeprom->io_ops->init fail\n");
        return -1;
    }

    // first, find a free context 
    for(i=0, done=0; i<_nb_eeprom_devices; i++) {

        if(_eeprom_ctx[i].in_use == 0) {

            _nb_bytes = _eeprom_param->in.nb_bytes;
           
            // record input parameters 
            _eeprom_ctx[i].bus_type = _bus_type;
            _eeprom_ctx[i].nb_bytes = _nb_bytes;
            if(_bus_type == EEPROM_BUS_TYPE_IIC) {
                _eeprom_ctx[i].i2c_controller_ioctl_param = _eeprom_param->in.i2c_controller_ioctl_param;
                _eeprom_ctx[i].i2c_controller_ctx = _eeprom_param->in.i2c_controller_ctx;
            }
            strcpy(_eeprom_ctx[i].signature, _eeprom_signatrue);

            // record output parameters
            _eeprom_ctx[i].dev_addr = _eeprom_param->out.dev_addr;

            // change the status of this eeprom_context
            _eeprom_ctx[i].in_use = 1;

            done = 1; // all the related contents are recoreded in eeprom_context

            break;
        }
    }

    if(!done) {
        printf("eeprom_controller_register_eeprom fail, can't record port %s\n", _eeprom_signatrue);
        return -1;
    }

    _controller_ctx->count++; /* if done, increase the object counter */

    return ret;
}

int eeprom_controller_ioctl(eeprom_controller_context *eeprom_controller_ctx, uint16_t cmd, eeprom_controller_ioctl_parameters *eeprom_controller_ioctl_param)
{
    int ret = 0, same, done, index, _nb_eeprom_devices;
    char *_eeprom_signature = NULL;
    uint16_t _cmd = cmd;    
    eeprom_ops *_eeprom_ops = NULL;
    eeprom_context *_eeprom_ctx = NULL;
    eeprom_ioctl_parameters eeprom_ioctl_param;
    eeprom_controller_context *_controller_ctx = eeprom_controller_ctx;
    eeprom_controller_ioctl_parameters *_controller_ioctl_param = eeprom_controller_ioctl_param;
    
    if(!_controller_ctx || !_controller_ioctl_param || _cmd < EEPROM_CONTROLLER_IOCTL_CMD_NB_BEGIN || _cmd > EEPROM_CONTROLLER_IOCTL_CMD_NB_END) {
        printf("eeprom_controller_ioctl incorrect parameters\n");
        return -1;
    }

    _nb_eeprom_devices = _controller_ctx->nb_eeprom_devices;
    _eeprom_ops = &_controller_ctx->eeprom_ops;
    
    switch(_cmd) {

        case EEPROM_CONTROLLER_IOCTL_CMD_ACQUIRE_EEPROM_CONTEX:
             _eeprom_ctx = _controller_ctx->eeprom_ctx;
             _eeprom_signature = _controller_ioctl_param->acquire_eeprom_ctx_param.in.signature;         
             for(index=0, done=0; index<_nb_eeprom_devices; index++) {
                 same = !strcmp(_eeprom_signature, _eeprom_ctx[index].signature);
                 if(same) {
    
                    _controller_ioctl_param->temp.index = index;
    
                    _controller_ioctl_param->temp.eeprom_ctx = &_eeprom_ctx[index];   
    
                    done = 1;
    
                    break;
                 }
             }
    
             if(done == 0) {
                 printf("eeprom_controller_ioctl->acquire_eeprom_context fail\n");
                 return -1;
             }

             return 0; // acquire eeprom context done!!!
        break;

        case EEPROM_CONTROLLER_IOCTL_CMD_PROGRAM_SINGLE_BYTE:
             printf("EEPROM_CONTROLLER_IOCTL_CMD_PROGRAM_SINGLE_BYTE\n");

             index = _controller_ioctl_param->temp.index;
             if(index < 0 || index >= _nb_eeprom_devices) {
                 printf("eeprom_controller_ioctl, incorrect index = %d\n", index);
                 return -1;
             }

             _eeprom_ctx = _controller_ioctl_param->temp.eeprom_ctx; 
             if(_eeprom_ctx != &_controller_ctx->eeprom_ctx[index]) {
                 printf("eeprom_controller_ioctl, incorrect ioctl parameters\n");
                 return -1;
             }
             
             eeprom_ioctl_param.cmd = EEPROM_IOCTL_CMD_PROGRAM_SINGLE_BYTE;
             
             ret = _eeprom_ops->ioctl(&eeprom_ioctl_param);
             if(ret < 0) {
                 printf("io_expander_controller_ioctl->io_expander_ops->ioctl fail\n");
                 return -1;
             }
             
             return 0;
        break;

        default:
             printf("eeprom_controller_ioctl, unknown cmd %d\n", _cmd);
             return -1;
        break;
    }

    
    return ret;
}



#if 0

int main(void)
{
    uint16_t cmd;
    int ret = 0;
    log_controller_context log_ctx;
    register_i2c_bus_parameters i2c_bus_param;
    i2c_controller_context i2c_controller_ctx;
    i2c_controller_ioctl_parameters i2c_controller_ioctl_param;
    register_eeprom_parameters eeprom_param;
    eeprom_controller_context   eeprom_controller_ctx;
    eeprom_controller_ioctl_parameters eeprom_controller_ioctl_param;
    log_controller_ioctl_parameters log_controller_ioctl_param;

    ret = i2c_controller_init(&i2c_controller_ctx, &log_ctx, &log_controller_ioctl_param);
    if(ret < 0) {
        printf("i2c_controller_init fail\n");
        return ret;
    }

    strcpy(i2c_bus_param.in.signature, "i2c-1");
    ret = i2c_controller_ctx.ops.register_context(&i2c_controller_ctx, &i2c_bus_param);
    if(ret < 0) {
        printf("i2c_controller->register_context fail\n");
        return -1;
    }

    cmd = I2C_CONTROLLER_IOCTL_CMD_ACQUIRE_I2C_CONTEX;
    strcpy(i2c_controller_ioctl_param.acquire_i2c_ctx_param.in.signature, "i2c-1");
    ret = i2c_controller_ctx.ops.ioctl(&i2c_controller_ctx, cmd, &i2c_controller_ioctl_param);
    if(ret < 0) {
        printf("i2c_controller->ioctl fail\n");
        return -1;
    }

    i2c_controller_ctx.ops.list_resources(&i2c_controller_ctx);

    ret = eeprom_controller_init(&eeprom_controller_ctx, &log_ctx);
    if(ret < 0) {
        printf("eeprom_controller_init fail\n");
        return -1;
    }

    strcpy(eeprom_param.in.signature, "U6");
    eeprom_param.in.bus_type = EEPROM_BUS_TYPE_IIC;
    eeprom_param.in.nb_bytes = MEMORY_SIZE_IN_BYTES_OF_24C02;
    eeprom_param.in.i2c_controller_ctx = &i2c_controller_ctx;
    eeprom_param.in.i2c_controller_ioctl_param = &i2c_controller_ioctl_param;
    ret = eeprom_controller_ctx.ops.register_context(&eeprom_controller_ctx, &eeprom_param);
    if(ret < 0) {
        printf("i2c_controller->register_context fail\n");
        return -1;
    }

    eeprom_controller_ctx.ops.list_resources(&eeprom_controller_ctx);

    cmd = EEPROM_CONTROLLER_IOCTL_CMD_ACQUIRE_EEPROM_CONTEX;
    strcpy(eeprom_controller_ioctl_param.acquire_eeprom_ctx_param.in.signature, "U6");
    ret = eeprom_controller_ctx.ops.ioctl(&eeprom_controller_ctx, cmd, &eeprom_controller_ioctl_param);
    if(ret < 0) {
        printf("eeprom_controller->ioctl fail\n");
        return -1;
    }

    //printf("dev_addr: 0x%x\n", eeprom_controller_ioctl_param.temp.eeprom_ctx->dev_addr);

    cmd = EEPROM_CONTROLLER_IOCTL_CMD_PROGRAM_SINGLE_BYTE;
    eeprom_controller_ioctl_param.program_single_byte.in.offset = 0;
    eeprom_controller_ioctl_param.program_single_byte.in.data = 0x33;
    ret = eeprom_controller_ctx.ops.ioctl(&eeprom_controller_ctx, cmd, &eeprom_controller_ioctl_param);
    if(ret < 0) {
        printf("eeprom_controller->ioctl fail\n");
        return -1;
    }

}

#endif

