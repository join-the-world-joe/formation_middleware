#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "log_controller.h"
#ifdef DATA_FLOW_DEBUG
#include "i2c_controller.h"
#endif

#include "i2c_controller.h"

#include "ssd1306_oled_controller.h"
#include "hal_ssd1306_oled.h"



#include "debug.h"

//creat example context
int ssd1306_oled_controller_init(ssd1306_oled_controller_context *ssd1306_oled_controller_ctx, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param)
{
    int ret = 0, _nb_ssd1306_oled_devices = NB_SSD1306_OLED_DEVICES, i;
    ssd1306_oled_context *_ssd1306_oled_ctx = NULL;
    
    ssd1306_oled_controller_context *_controller_ctx = ssd1306_oled_controller_ctx;
    log_controller_context *_log_controller_ctx = log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param = log_controller_ioctl_param;

    if(!ssd1306_oled_controller_ctx || !log_controller_ctx || !log_controller_ioctl_param) {
        printf("ssd1306_oled_controller_init fail, incorrect parameters\n");
        return -1;
    }

    _controller_ctx->count = 0;
    _controller_ctx->log_ctx = _log_controller_ctx;
    _controller_ctx->log_ioctl_param = _log_controller_ioctl_param;
    _controller_ctx->nb_ssd1306_oled_devices = _nb_ssd1306_oled_devices;

    _ssd1306_oled_ctx = _controller_ctx->ssd1306_oled_ctx;

    for(i=0; i<_nb_ssd1306_oled_devices; i++) {
        _ssd1306_oled_ctx[i].index = i;
        _ssd1306_oled_ctx[i].in_use = 0;
        _ssd1306_oled_ctx[i].dev_addr = 0;
        _ssd1306_oled_ctx[i].i2c_controller_ctx = NULL;
        _ssd1306_oled_ctx[i].i2c_controller_ioctl_param = NULL;
        memset(_ssd1306_oled_ctx[i].signature, 0, MAX_LENGTH_OF_SSD1306_OLED_SIGNATURE);
    }

    _controller_ctx->ops.init = ssd1306_oled_controller_init;
    _controller_ctx->ops.register_context = ssd1306_oled_controller_register_ssd1306_oled;
    _controller_ctx->ops.unregister_context = ssd1306_oled_controller_unregister_ssd1306_oled;
    _controller_ctx->ops.list_resources = ssd1306_oled_controller_list_resources;
    _controller_ctx->ops.ioctl = ssd1306_oled_controller_ioctl;
    _controller_ctx->ops.deinit = ssd1306_oled_controller_deinit;
    
    _controller_ctx->ssd1306_oled_ops.init = hal_ssd1306_oled_init;
    _controller_ctx->ssd1306_oled_ops.ioctl = hal_ssd1306_oled_ioctl;
    _controller_ctx->ssd1306_oled_ops.deinit = hal_ssd1306_oled_deinit;
}

void ssd1306_oled_controller_deinit(ssd1306_oled_controller_context *ssd1306_oled_controller_ctx)
{
    int ret = 0, index;
    int _nb_ssd1306_oled_devices;
    ssd1306_oled_context *_ssd1306_oled_ctx = NULL;
    ssd1306_oled_hal_ops *_ssd1306_oled_ops = NULL;
    ssd1306_oled_controller_context *_controller_ctx = ssd1306_oled_controller_ctx;
    unregister_ssd1306_oled_parameters ssd1306_oled_param;

    if(!_controller_ctx) {
        printf("ssd1306_oled_controller_deinit fail, incorrect parameters\n");
        return ;
    }

    _ssd1306_oled_ops = &_controller_ctx->ssd1306_oled_ops;
    _ssd1306_oled_ctx = _controller_ctx->ssd1306_oled_ctx; 
    _nb_ssd1306_oled_devices = _controller_ctx->nb_ssd1306_oled_devices;

    for(index=0; index<_nb_ssd1306_oled_devices; index++) {
        if(_ssd1306_oled_ctx[index].in_use) {
            //printf("ssd1306_oled_controller_deinit: index %d is in use\n", index);
            memset(ssd1306_oled_param.in.signature, 0, MAX_LENGTH_OF_SSD1306_OLED_SIGNATURE);
            strcpy(ssd1306_oled_param.in.signature, _ssd1306_oled_ctx[index].signature);
            ret = _controller_ctx->ops.unregister_context(_controller_ctx, &ssd1306_oled_param);
            if(ret < 0) {
                printf("ssd1306_oled_controller_deinit->unregister_context fail, device: %s\n", _ssd1306_oled_ctx[index].signature);
                return ;
            }
        }
    }

    return ;

}

//register/unregister example
int ssd1306_oled_controller_register_ssd1306_oled(ssd1306_oled_controller_context *ssd1306_oled_controller_ctx, register_ssd1306_oled_parameters *ssd1306_oled_param)
{
    int ret = 0, i, done;
    uint32_t _nb_ssd1306_oled_devices;
    char *_ssd1306_oled_signatrue = NULL;
    ssd1306_oled_context *_ssd1306_oled_ctx = NULL;
    ssd1306_oled_hal_ops *_ssd1306_oled_ops = NULL;
    register_ssd1306_oled_parameters *_ssd1306_oled_param = ssd1306_oled_param;
    ssd1306_oled_controller_context *_controller_ctx = ssd1306_oled_controller_ctx;

    if(!_controller_ctx || !_ssd1306_oled_param) {
        printf("ssd1306_oled_controller_register_ssd1306_oled fail, incorrect parameters\n");
        return -1;
    }

    _nb_ssd1306_oled_devices = _controller_ctx->nb_ssd1306_oled_devices;
    _ssd1306_oled_ctx = _controller_ctx->ssd1306_oled_ctx;
    _ssd1306_oled_ops = &_controller_ctx->ssd1306_oled_ops;
    _ssd1306_oled_signatrue = _ssd1306_oled_param->in.signature;

    if(_controller_ctx->count + 1 > _nb_ssd1306_oled_devices) {
        printf("ssd1306_oled context in controller is full, there might be something wrong\n");
        return -1;
    }

    ret = _ssd1306_oled_ops->init(ssd1306_oled_param);
    if(ret < 0) {
        printf("ssd1306_oled_controller_register_ssd1306_oled->_ssd1306_oled_ops->init fail\n");
        return -1;
    }

    // first, find a free context 
    for(i=0, done=0; i<_nb_ssd1306_oled_devices; i++) {

        if(_ssd1306_oled_ctx[i].in_use == 0) {

            // record input parameters
            _ssd1306_oled_ctx[i].i2c_controller_ctx = ssd1306_oled_param->in.i2c_controller_ctx;
            _ssd1306_oled_ctx[i].i2c_controller_ioctl_param = ssd1306_oled_param->in.i2c_controller_ioctl_param;

            strcpy(_ssd1306_oled_ctx[i].signature, _ssd1306_oled_signatrue);

            // record output parameters
            _ssd1306_oled_ctx[i].dev_addr = ssd1306_oled_param->out.dev_addr;

            // change the status of this ssd1306_oled_context
            _ssd1306_oled_ctx[i].in_use = 1;

            done = 1; // all the related contents are recoreded in ssd1306_oled_context

            break;
        }
    }

    if(!done) {
        printf("ssd1306_oled_controller_register_ssd1306_oled fail, can't record ssd1306_oled device %s\n", _ssd1306_oled_signatrue);
        return -1;
    }

    _controller_ctx->count++; /* if done, increase the object counter */

    return ret;
}

int ssd1306_oled_controller_unregister_ssd1306_oled(ssd1306_oled_controller_context *ssd1306_oled_controller_ctx, unregister_ssd1306_oled_parameters *ssd1306_oled_param)
{
    int ret = 0, index, done, i, same, find;
    uint32_t _nb_ssd1306_oled_devices;
    char *_ssd1306_oled_signatrue = NULL;
    ssd1306_oled_context *_ssd1306_oled_ctx = NULL;
    ssd1306_oled_hal_ops *_ssd1306_oled_ops = NULL;
    unregister_ssd1306_oled_parameters *_ssd1306_oled_param = ssd1306_oled_param;
    ssd1306_oled_controller_context *_controller_ctx = ssd1306_oled_controller_ctx;

    if(!_controller_ctx || !_ssd1306_oled_param) {
        printf("ssd1306_oled_controller_unregister_ssd1306_oled fail, incorrect parameters\n");
        return -1;
    }

    _nb_ssd1306_oled_devices = _controller_ctx->nb_ssd1306_oled_devices;
    _ssd1306_oled_ctx = _controller_ctx->ssd1306_oled_ctx;
    _ssd1306_oled_ops = &_controller_ctx->ssd1306_oled_ops;
    _ssd1306_oled_signatrue = _ssd1306_oled_param->in.signature;

    if((_controller_ctx->count - 1) < 0) {
        printf("ssd1306_oled_controller_unregister_ssd1306_oled ssd1306_oled context in controller is empty, there might be something wrong\n");
        return -1;
    }

    // check if we can find this signature in gpio_context
    for(index=0, find=0; index<_nb_ssd1306_oled_devices; index++) {

        same = !strcmp(_ssd1306_oled_ctx[index].signature, _ssd1306_oled_signatrue);
        if(same) {

            find = 1;

            break;
        }
    }

    if(!find) {
        printf("ssd1306_oled_controller_unregister_ssd1306_oled signature %s doesn't match any recorded signature in ssd1306_oled_context of ssd1306_oled_controller\n", _ssd1306_oled_signatrue);
        return -1;
    }

    ret = _ssd1306_oled_ops->deinit(_ssd1306_oled_param);
    if(ret < 0) {
        printf("ssd1306_oled_controller_unregister_ssd1306_oled->ad_ops->deinit fail\n");
        return -1;
    }

    _ssd1306_oled_ctx[index].in_use = 0;

    return ret;

}

void ssd1306_oled_controller_list_resources(ssd1306_oled_controller_context *ssd1306_oled_controller_ctx)
{
    int _nb_ssd1306_oled_devices, index, j;
    ssd1306_oled_context *_ssd1306_oled_ctx = NULL;
    ssd1306_oled_controller_context *_controller_ctx = ssd1306_oled_controller_ctx;

    if(!_controller_ctx) {
        printf("ssd1306_oled_controller_list_resources fail, incorrect parameters\n");
        return ;
    }

    _ssd1306_oled_ctx = _controller_ctx->ssd1306_oled_ctx;
    _nb_ssd1306_oled_devices = _controller_ctx->nb_ssd1306_oled_devices;
    
    printf("***********************************SSD1306_OLED Resources***************************************\n");
    for(index=0; index<_nb_ssd1306_oled_devices; index++) {
        if(_ssd1306_oled_ctx[index].in_use) {
     
            printf("Index: %d\n", index);
            printf("SSD1306_OLED Signature: %s\n", _ssd1306_oled_ctx[index].signature);
            printf("---------------------------------------------------------------------------------------\n");
            //printf("SSD1306_OLED i2c\n", _ssd1306_oled_ctx[index].i2c_controller_ioctl_param);
            printf("----------------------------------------------------------------------------------------\n");
        }
    }
    printf("****************************************************************************************\n");

}

//control example
int ssd1306_oled_controller_ioctl(ssd1306_oled_controller_context *ssd1306_oled_controller_ctx, uint16_t cmd, ssd1306_oled_controller_ioctl_parameters *ssd1306_oled_controller_ioctl_param)
{
    int ret = 0, same, done, index, _nb_ssd1306_oled_devices;
    char *_ssd1306_oled_signature = NULL;
    uint16_t _cmd = cmd;
    ssd1306_oled_hal_ops *_ssd1306_oled_ops = NULL;
    ssd1306_oled_context *_ssd1306_oled_ctx = NULL;
    ssd1306_oled_ioctl_parameters ssd1306_oled_ioctl_param;
    ssd1306_oled_controller_context *_controller_ctx = ssd1306_oled_controller_ctx;
    ssd1306_oled_controller_ioctl_parameters *_controller_ioctl_param = ssd1306_oled_controller_ioctl_param;
    
    if(!_controller_ctx || !_controller_ioctl_param || _cmd < SSD1306_OLED_CONTROLLER_IOCTL_CMD_NB_BEGIN || _cmd > SSD1306_OLED_CONTROLLER_IOCTL_CMD_NB_END) {
        printf("ssd1306_oled_controller_ioctl incorrect parameters\n");
        return -1;
    }

    _nb_ssd1306_oled_devices = _controller_ctx->nb_ssd1306_oled_devices;
    _ssd1306_oled_ops = &_controller_ctx->ssd1306_oled_ops;

    switch(_cmd) {

        case SSD1306_OLED_CONTROLLER_IOCTL_CMD_ACQUIRE_SSD1306_OLED_CONTEXT:
            
            _ssd1306_oled_ctx = _controller_ctx->ssd1306_oled_ctx;
            _ssd1306_oled_signature = _controller_ioctl_param->acquire_ssd1306_oled_ctx_param.in.signature;
            for(index=0, done=0; index<_nb_ssd1306_oled_devices; index++) {
                same = !strcmp(_ssd1306_oled_signature, _ssd1306_oled_ctx[index].signature);
                if(same) {
            
                   if(_ssd1306_oled_ctx[index].in_use) {
            
                       _controller_ioctl_param->temp.index = index;
            
                       _controller_ioctl_param->temp.ssd1306_oled_ctx = &_ssd1306_oled_ctx[index];
                   
                       done = 1;
            
                       break;
                   }
                }
            }
            
            if(done == 0) {
                printf("ssd1306_oled_controller_ioctl->acquire_ssd1306_oled_context fail\n");
                return -1;
            }
            
            return 0; // acquire ssd1306_oled context done!!!
        break;

        case SSD1306_OLED_CONTROLLER_IOCTL_CMD_WRITE_BYTE:

            index = _controller_ioctl_param->temp.index;
            if(index < 0 || index >= _nb_ssd1306_oled_devices) {
                printf("ssd1306_oled_controller_ioctl, incorrect index = %d\n", index);
                return -1;
            }
            
            _ssd1306_oled_ctx = _controller_ioctl_param->temp.ssd1306_oled_ctx; 
            if(_ssd1306_oled_ctx != &_controller_ctx->ssd1306_oled_ctx[index]) {
                printf("ssd1306_oled_controller_ioctl, incorrect ioctl parameters\n");
                return -1;
            }

            ssd1306_oled_ioctl_param.cmd = SSD1306_OLED_IOCTL_CMD_WRITE_BYTE;
            ssd1306_oled_ioctl_param.write_byte_param.temp.i2c_controller_ctx = _ssd1306_oled_ctx->i2c_controller_ctx;
            ssd1306_oled_ioctl_param.write_byte_param.temp.i2c_controller_ioctl_param = _ssd1306_oled_ctx->i2c_controller_ioctl_param;
            ssd1306_oled_ioctl_param.write_byte_param.in.byte = &_controller_ioctl_param->write_byte_param.in.byte;
            ssd1306_oled_ioctl_param.write_byte_param.in.row_idx = &_controller_ioctl_param->write_byte_param.in.row_idx;
            ssd1306_oled_ioctl_param.write_byte_param.in.column_idx = &_controller_ioctl_param->write_byte_param.in.column_idx;

            ret = _ssd1306_oled_ops->ioctl(&ssd1306_oled_ioctl_param);
            if(ret < 0) {
                printf("ssd1306_oled_controller_ioctl->ssd1306_oled_ops->ioctl fail\n");
                return -1;
            }

            return 0;
        break;

        case SSD1306_OLED_CONTROLLER_IOCTL_CMD_WRITE_CHAR:

            index = _controller_ioctl_param->temp.index;
            if(index < 0 || index >= _nb_ssd1306_oled_devices) {
                printf("ssd1306_oled_controller_ioctl, incorrect index = %d\n", index);
                return -1;
            }
            
            _ssd1306_oled_ctx = _controller_ioctl_param->temp.ssd1306_oled_ctx; 
            if(_ssd1306_oled_ctx != &_controller_ctx->ssd1306_oled_ctx[index]) {
                printf("ssd1306_oled_controller_ioctl, incorrect ioctl parameters\n");
                return -1;
            }

            ssd1306_oled_ioctl_param.cmd = SSD1306_OLED_IOCTL_CMD_WRITE_CHAR;
            ssd1306_oled_ioctl_param.write_char_param.temp.i2c_controller_ctx = _ssd1306_oled_ctx->i2c_controller_ctx;
            ssd1306_oled_ioctl_param.write_char_param.temp.i2c_controller_ioctl_param = _ssd1306_oled_ctx->i2c_controller_ioctl_param;
            ssd1306_oled_ioctl_param.write_char_param.in._char = &_controller_ioctl_param->write_char_param.in._char;
            ssd1306_oled_ioctl_param.write_char_param.in.char_row_idx = &_controller_ioctl_param->write_char_param.in.char_row_idx;
            ssd1306_oled_ioctl_param.write_char_param.in.char_column_idx = &_controller_ioctl_param->write_char_param.in.char_column_idx;

            ret = _ssd1306_oled_ops->ioctl(&ssd1306_oled_ioctl_param);
            if(ret < 0) {
                printf("ssd1306_oled_controller_ioctl->ssd1306_oled_ops->ioctl fail\n");
                return -1;
            }

            return 0;
        break;

        case SSD1306_OLED_CONTROLLER_IOCTL_CMD_START_SCOLL:

            index = _controller_ioctl_param->temp.index;
            if(index < 0 || index >= _nb_ssd1306_oled_devices) {
                printf("ssd1306_oled_controller_ioctl, incorrect index = %d\n", index);
                return -1;
            }
            
            _ssd1306_oled_ctx = _controller_ioctl_param->temp.ssd1306_oled_ctx; 
            if(_ssd1306_oled_ctx != &_controller_ctx->ssd1306_oled_ctx[index]) {
                printf("ssd1306_oled_controller_ioctl, incorrect ioctl parameters\n");
                return -1;
            }

            ssd1306_oled_ioctl_param.cmd = SSD1306_OLED_IOCTL_CMD_START_SCOLL;
            ssd1306_oled_ioctl_param.start_scoll_param.temp.i2c_controller_ctx = _ssd1306_oled_ctx->i2c_controller_ctx;
            ssd1306_oled_ioctl_param.start_scoll_param.temp.i2c_controller_ioctl_param = _ssd1306_oled_ctx->i2c_controller_ioctl_param;

            ret = _ssd1306_oled_ops->ioctl(&ssd1306_oled_ioctl_param);
            if(ret < 0) {
                printf("ssd1306_oled_controller_ioctl->ssd1306_oled_ops->ioctl fail\n");
                return -1;
            }

            return 0;
        break;

        case SSD1306_OLED_CONTROLLER_IOCTL_CMD_STOP_SCOLL:

            index = _controller_ioctl_param->temp.index;
            if(index < 0 || index >= _nb_ssd1306_oled_devices) {
                printf("ssd1306_oled_controller_ioctl, incorrect index = %d\n", index);
                return -1;
            }
            
            _ssd1306_oled_ctx = _controller_ioctl_param->temp.ssd1306_oled_ctx; 
            if(_ssd1306_oled_ctx != &_controller_ctx->ssd1306_oled_ctx[index]) {
                printf("ssd1306_oled_controller_ioctl, incorrect ioctl parameters\n");
                return -1;
            }

            ssd1306_oled_ioctl_param.cmd = SSD1306_OLED_IOCTL_CMD_STOP_SCOLL;
            ssd1306_oled_ioctl_param.stop_scoll_param.temp.i2c_controller_ctx = _ssd1306_oled_ctx->i2c_controller_ctx;
            ssd1306_oled_ioctl_param.stop_scoll_param.temp.i2c_controller_ioctl_param = _ssd1306_oled_ctx->i2c_controller_ioctl_param;

            ret = _ssd1306_oled_ops->ioctl(&ssd1306_oled_ioctl_param);
            if(ret < 0) {
                printf("ssd1306_oled_controller_ioctl->ssd1306_oled_ops->ioctl fail\n");
                return -1;
            }

            return 0;
        break;


        default:
            printf("ssd1306_oled_controller_ioctl, unknown cmd %d\n", _cmd);
            return -1;
        break;

    }

    return ret;
}

#if 0

int main()
{
    uint16_t cmd;
    int ret = 0, i;
    log_controller_context log_ctx;
    i2c_controller_context i2c_controller_ctx;
    ssd1306_oled_controller_context ssd1306_oled_controller_ctx;

    register_i2c_bus_parameters i2c_bus_param;
    register_ssd1306_oled_parameters ssd1306_oled_param;

    log_controller_ioctl_parameters log_controller_ioctl_param;
    i2c_controller_ioctl_parameters i2c_controller_ioctl_param;
    ssd1306_oled_controller_ioctl_parameters ssd1306_oled_controller_ioctl_param;

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

    ret = ssd1306_oled_controller_init(&ssd1306_oled_controller_ctx, &log_ctx, &log_controller_ioctl_param);
    if(ret < 0) {
        printf("ssd1306_oled_controller_init fail\n");
        return ret;
    }

    strcpy(ssd1306_oled_param.in.signature, "SSD1306_OLED");
    ssd1306_oled_param.in.i2c_controller_ctx = &i2c_controller_ctx;
    ssd1306_oled_param.in.i2c_controller_ioctl_param = &i2c_controller_ioctl_param;
    ret = ssd1306_oled_controller_ctx.ops.register_context(&ssd1306_oled_controller_ctx, &ssd1306_oled_param);
    if(ret < 0) {
        printf("ssd1306_oled_controller->register_context fail\n");
        return -1;
    }

    ssd1306_oled_controller_ctx.ops.list_resources(&ssd1306_oled_controller_ctx);

    cmd = SSD1306_OLED_CONTROLLER_IOCTL_CMD_ACQUIRE_SSD1306_OLED_CONTEXT;
    strcpy(ssd1306_oled_controller_ioctl_param.acquire_ssd1306_oled_ctx_param.in.signature, "SSD1306_OLED");
    ret = ssd1306_oled_controller_ctx.ops.ioctl(&ssd1306_oled_controller_ctx, cmd, &ssd1306_oled_controller_ioctl_param);
    if(ret < 0) {
        printf("ssd1306_oled_controller->ioctl fail\n");
        return -1;
    }

    cmd = SSD1306_OLED_CONTROLLER_IOCTL_CMD_WRITE_BYTE;
    ssd1306_oled_controller_ioctl_param.write_byte_param.in.byte = 0x03;
    ssd1306_oled_controller_ioctl_param.write_byte_param.in.row_idx = 0;
    ssd1306_oled_controller_ioctl_param.write_byte_param.in.column_idx = 0;
    ret = ssd1306_oled_controller_ctx.ops.ioctl(&ssd1306_oled_controller_ctx, cmd, &ssd1306_oled_controller_ioctl_param);
    if(ret < 0) {
        printf("ssd1306_oled_controller->ioctl fail\n");
        return -1;
    }

    cmd = SSD1306_OLED_CONTROLLER_IOCTL_CMD_WRITE_CHAR;
    ssd1306_oled_controller_ioctl_param.write_char_param.in._char = '0';
    ssd1306_oled_controller_ioctl_param.write_char_param.in.char_row_idx = 0;
    ssd1306_oled_controller_ioctl_param.write_char_param.in.char_column_idx = 0;
    ret = ssd1306_oled_controller_ctx.ops.ioctl(&ssd1306_oled_controller_ctx, cmd, &ssd1306_oled_controller_ioctl_param);
    if(ret < 0) {
        printf("ssd1306_oled_controller->ioctl fail\n");
        return -1;
    }

    cmd = SSD1306_OLED_CONTROLLER_IOCTL_CMD_WRITE_CHAR;
    ssd1306_oled_controller_ioctl_param.write_char_param.in._char = '1';
    ssd1306_oled_controller_ioctl_param.write_char_param.in.char_row_idx = 1;
    ssd1306_oled_controller_ioctl_param.write_char_param.in.char_column_idx = 1;
    ret = ssd1306_oled_controller_ctx.ops.ioctl(&ssd1306_oled_controller_ctx, cmd, &ssd1306_oled_controller_ioctl_param);
    if(ret < 0) {
        printf("ssd1306_oled_controller->ioctl fail\n");
        return -1;
    }


    return 0;

}

#endif

