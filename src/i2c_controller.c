/**
* @file     i2c_controller.c
* @brief    i2c_controller.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     01/12/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "log_controller.h"
#include "log_controller.h"
#include "i2c_controller.h"
#include "hal_i2c.h"

void i2c_controller_list_resources(i2c_controller_context *i2c_controller_ctx)
{
    int _nb_i2c_bus, index;
    i2c_context *_i2c_ctx = NULL;
    i2c_controller_context *_controller_ctx = i2c_controller_ctx;

    if(!_controller_ctx) {
        printf("i2c_controller_list_resources fail, incorrect parameters\n");
        return ;
    }

    _i2c_ctx = _controller_ctx->i2c_ctx;
    _nb_i2c_bus = _controller_ctx->nb_i2c_bus;

    printf("***********************************IIC BUS Resources***********************************\n");
    for(index=0; index<_nb_i2c_bus; index++) {
        if(_i2c_ctx[index].in_use) {
            printf("Index: %d\n", index);
            printf("Path: %s\n", _i2c_ctx[index].path);
            printf("IIC Signature: %s\n", _i2c_ctx[index].signature);
            printf("Device FD: %d\n", _i2c_ctx[index].dev_fd);
        }
    }
    printf("***************************************************************************************\n");

}

int i2c_controller_init(i2c_controller_context *i2c_controller_ctx, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param)
{
    int ret = 0, _nb_i2c_bus = NB_I2C_BUS, i;
    i2c_context *_i2c_ctx = NULL;
    i2c_controller_context *_controller_ctx = i2c_controller_ctx;
    log_controller_context *_log_controller_ctx = log_controller_ctx;
        log_controller_ioctl_parameters *_log_controller_ioctl_param = log_controller_ioctl_param;

    if(!_controller_ctx || !_log_controller_ctx ||!_log_controller_ioctl_param) {
        printf("i2c_controller_init fail, incorrect parameters\n");
        return -1;
    }

    _controller_ctx->count = 0;
    _controller_ctx->_log_controller_ctx = _log_controller_ctx;   
    _controller_ctx->_log_controller_ioctl_param = _log_controller_ioctl_param;
    _controller_ctx->nb_i2c_bus = _nb_i2c_bus;

    _i2c_ctx = _controller_ctx->i2c_ctx;
    
    // we'd better initialize all the important variables here    
    for(i=0; i<_nb_i2c_bus; i++) {
        _i2c_ctx[i].index = i;
        _i2c_ctx[i].in_use = 0;
        _i2c_ctx[i].dev_fd = -1;
        memset(_i2c_ctx[i].signature, 0, MAX_LENGTH_OF_I2C_BUS_SIGNATURE);
        memset(_i2c_ctx[i].path, 0, MAX_LENGTH_OF_I2C_DEV_PATH);
    }
    
    _controller_ctx->ops.init = i2c_controller_init;
    _controller_ctx->ops.register_context = i2c_controller_register_i2c_bus;
    _controller_ctx->ops.unregister_context = i2c_controller_unregister_i2c_bus;
    _controller_ctx->ops.list_resources = i2c_controller_list_resources;
    _controller_ctx->ops.ioctl = i2c_controller_ioctl;
    _controller_ctx->ops.deinit = i2c_controller_deinit;
    
    _controller_ctx->i2c_ops.init = hal_i2c_init;
    _controller_ctx->i2c_ops.ioctl = hal_i2c_ioctl;
    _controller_ctx->i2c_ops.deinit = hal_i2c_deinit;

    return ret;
}

void i2c_controller_deinit(i2c_controller_context *i2c_controller_ctx)
{
    int ret = 0, index;
    int _nb_i2c_bus;
    i2c_context *_i2c_ctx = NULL;
    i2c_bus_ops *_i2c_ops = NULL;
    i2c_controller_context *_controller_ctx = i2c_controller_ctx;
    unregister_i2c_bus_parameters i2c_bus_param;

    if(!_controller_ctx) {
        printf("i2c_controller_deinit fail, incorrect parameters\n");
        return ;
    }

    _i2c_ops = &_controller_ctx->i2c_ops;
    _i2c_ctx = _controller_ctx->i2c_ctx; 
    _nb_i2c_bus = _controller_ctx->nb_i2c_bus;

    for(index=0; index<_nb_i2c_bus; index++) {
        if(_i2c_ctx[index].in_use) {
            //printf("spi_controller_deinit: index %d is in use\n", index);
            memset(i2c_bus_param.in.signature, 0, MAX_LENGTH_OF_I2C_BUS_SIGNATURE);
            strcpy(i2c_bus_param.in.signature, _i2c_ctx[index].signature);
            ret = _controller_ctx->ops.unregister_context(_controller_ctx, &i2c_bus_param);
            if(ret < 0) {
                printf("i2c_controller_deinit->unregister_spi_bus fail, port %s\n", _i2c_ctx[index].signature);
                return ;
            }
        }
    }

    return ;
}

int i2c_controller_register_i2c_bus(i2c_controller_context *i2c_controller_ctx, register_i2c_bus_parameters *i2c_bus_param)
{
    int ret = 0, i, done;
    int _nb_i2c_bus;
    char *_bus_signatrue = NULL;
    i2c_context *_i2c_ctx = NULL;
    i2c_bus_ops *_i2c_ops = NULL;
    register_i2c_bus_parameters *_i2c_bus_param = i2c_bus_param;
    i2c_controller_context *_controller_ctx = i2c_controller_ctx;

    if(!_controller_ctx || !_i2c_bus_param) {
        printf("i2c_controller_register_i2c_bus fail, incorrect parameters\n");
        return -1;
    }

    _nb_i2c_bus = _controller_ctx->nb_i2c_bus;
    _i2c_ctx = _controller_ctx->i2c_ctx;
    _i2c_ops = &_controller_ctx->i2c_ops;
    _bus_signatrue = _i2c_bus_param->in.signature;
    
    if(_controller_ctx->count + 1 > _nb_i2c_bus) {
        printf("i2c context in controller is full, there might be something wrong\n");
        return -1;
    }

    ret = _i2c_ops->init(_i2c_bus_param);
    if(ret < 0) {
        printf("i2c_controller_register->io_ops->init fail\n");
        return -1;
    }

    // first, find a free context 
    for(i=0, done=0; i<_nb_i2c_bus; i++) {

        if(_i2c_ctx[i].in_use == 0) {

            // record input parameters 
            strcpy(_i2c_ctx[i].signature, _bus_signatrue);

            // record output parameters
            _i2c_ctx[i].dev_fd = _i2c_bus_param->out.dev_fd;

            // record path parameters
            strcpy(_i2c_ctx[i].path, _i2c_bus_param->temp.path);

            // change the status of this gpio_context
            _i2c_ctx[i].in_use = 1;

            done = 1; // all the related contents are recoreded in gpio_context

            break;
        }
    }

    if(!done) {
        printf("i2c_controller_register_i2c_bus fail, can't record port %s\n", _bus_signatrue);
        return -1;
    }

    _controller_ctx->count++; /* if done, increase the object counter */

    return ret;
}

int i2c_controller_unregister_i2c_bus(i2c_controller_context *i2c_controller_ctx, unregister_i2c_bus_parameters *i2c_bus_param)
{
    int ret = 0, index, done, i, same, find;
    int _nb_i2c_bus;
    char *_bus_signatrue = NULL;
    i2c_context *_i2c_ctx = NULL;
    i2c_bus_ops *_i2c_ops = NULL;
    unregister_i2c_bus_parameters *_i2c_bus_param = i2c_bus_param;
    i2c_controller_context *_controller_ctx = i2c_controller_ctx;

    if(!_controller_ctx || !_i2c_bus_param) {
        printf("i2c_controller_unregister_i2c_bus fail, incorrect parameters\n");
        return -1;
    }

    _nb_i2c_bus = _controller_ctx->nb_i2c_bus;
    _i2c_ctx = _controller_ctx->i2c_ctx;
    _i2c_ops = &_controller_ctx->i2c_ops;
    _bus_signatrue = _i2c_bus_param->in.signature;

    if((_controller_ctx->count - 1) < 0) {
        printf("i2c context in controller is empty, there might be something wrong\n");
        return -1;
    }

    // check if we can find this signature in gpio_context
    for(index=0, find=0; index<_nb_i2c_bus; index++) {

        same = !strcmp(_i2c_ctx[index].signature, _bus_signatrue);
        if(same) {

            _i2c_bus_param->temp.dev_fd = _i2c_ctx[index].dev_fd;

            find = 1;

            break;
        }
    }

    if(!find) {
        printf("signature %s doesn't match any recorded signature in i2c_context of i2c_controller\n", _bus_signatrue);
        return -1;
    }
    
    ret = _i2c_ops->deinit(_i2c_bus_param);
    if(ret < 0) {
        printf("i2c_controller_unregister_i2c_bus->i2c_ops->deinit fail\n");
        return -1;
    }

    _i2c_ctx[index].in_use = 0;

    return ret;
}

int i2c_controller_ioctl(i2c_controller_context *i2c_controller_ctx, uint16_t cmd, i2c_controller_ioctl_parameters *i2c_controller_ioctl_param)
{
    int ret = 0, same, done, index;
    char *_signature = NULL;
    uint16_t _cmd = cmd;    
    int _nb_i2c_bus;
    i2c_bus_ops *_i2c_ops = NULL;
    i2c_context *_i2c_ctx = NULL;
    i2c_ioctl_parameters i2c_ioctl_param;
    i2c_controller_context *_controller_ctx = i2c_controller_ctx;
    i2c_controller_ioctl_parameters *_controller_ioctl_param = i2c_controller_ioctl_param;
    
    if(!_controller_ctx || !_controller_ioctl_param || _cmd < I2C_CONTROLLER_IOCTL_CMD_NB_BEGIN || _cmd > I2C_CONTROLLER_IOCTL_CMD_NB_END) {
        printf("i2c_controller_ioctl incorrect parameters\n");
        return -1;
    }

    _nb_i2c_bus = _controller_ctx->nb_i2c_bus;
    _i2c_ops = &_controller_ctx->i2c_ops;
    
    switch(_cmd) {

        case I2C_CONTROLLER_IOCTL_CMD_ACQUIRE_I2C_CONTEX:
             _i2c_ctx = _controller_ctx->i2c_ctx;
             _signature = _controller_ioctl_param->acquire_i2c_ctx_param.in.signature;         
             for(index=0, done=0; index<_nb_i2c_bus; index++) {
                 same = !strcmp(_signature, _i2c_ctx[index].signature);
                 if(same) {
    
                    _controller_ioctl_param->temp.index = index;
    
                    _controller_ioctl_param->temp.i2c_ctx = &_i2c_ctx[index];   
    
                    done = 1;
    
                    break;
                 }
             }
    
             if(done == 0) {
                 printf("i2c_controller_ioctl->acquire_i2c_context fail\n");
                 return -1;
             }

             return 0; // acquire i2c context done!!!
        break;

        case I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES:
             index = _controller_ioctl_param->temp.index;
             if(index < 0 || index >= _nb_i2c_bus) {
                 printf("i2c_controller_ioctl, incorrect index = %d\n", index);
                 return -1;
             }
             
             _i2c_ctx = _controller_ioctl_param->temp.i2c_ctx; 
             if(_i2c_ctx != &_controller_ctx->i2c_ctx[index]) {
                 printf("i2c_controller_ioctl, incorrect ioctl parameters\n");
                 return -1;
             }

             i2c_ioctl_param.cmd = I2C_IOCTL_CMD_WRITE_MULTI_BYTES;
             i2c_ioctl_param.write_multi_bytes_param.in._i2c_dev_addr = &_controller_ioctl_param->write_multi_bytes_param.in.i2c_dev_addr;
             i2c_ioctl_param.write_multi_bytes_param.in._data = _controller_ioctl_param->write_multi_bytes_param.in.data;
             i2c_ioctl_param.write_multi_bytes_param.in._dev_fd = _i2c_ctx->dev_fd;
             i2c_ioctl_param.write_multi_bytes_param.in._length = &_controller_ioctl_param->write_multi_bytes_param.in.length;
             ret = _i2c_ops->ioctl(&i2c_ioctl_param);
             if(ret < 0) {
                 printf("i2c_controller_ioctl->i2c_ops->ioctl fail");
                 return -1;
             }

             return 0; 
        break;

        case I2C_CONTROLLER_IOCTL_CMD_READ_MULTI_BYTES:
             index = _controller_ioctl_param->temp.index;
             if(index < 0 || index >= _nb_i2c_bus) {
                 printf("i2c_controller_ioctl, incorrect index = %d\n", index);
                 return -1;
             }

             _i2c_ctx = _controller_ioctl_param->temp.i2c_ctx; 
             if(_i2c_ctx != &_controller_ctx->i2c_ctx[index]) {
                 printf("i2c_controller_ioctl, incorrect ioctl parameters\n");
                 return -1;
             }

             i2c_ioctl_param.cmd = I2C_IOCTL_CMD_READ_MULTI_BYTES;
             i2c_ioctl_param.read_multi_bytes_param.in._dev_fd = _i2c_ctx->dev_fd;
             i2c_ioctl_param.read_multi_bytes_param.in._i2c_dev_addr = &_controller_ioctl_param->read_multi_bytes_param.in.i2c_dev_addr;
             i2c_ioctl_param.read_multi_bytes_param.in._wr_data = _controller_ioctl_param->read_multi_bytes_param.in.wr_data;
             i2c_ioctl_param.read_multi_bytes_param.in._wr_length = &_controller_ioctl_param->read_multi_bytes_param.in.wr_length;
             i2c_ioctl_param.read_multi_bytes_param.in._rd_length = &_controller_ioctl_param->read_multi_bytes_param.in.rd_length;
             i2c_ioctl_param.read_multi_bytes_param.out._rd_data = _controller_ioctl_param->read_multi_bytes_param.out.rd_data;
             ret = _i2c_ops->ioctl(&i2c_ioctl_param);
             if(ret < 0) {
                 printf("i2c_controller_ioctl->i2c_ops->ioctl fail");
                 return -1;
             }

             return 0;
        break;
#if 0
        case CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE:
             index = _controller_ioctl_param->temp.index;
             if(index < 0 || index >= _nb_spi_bus) {
                 printf("spi_controller_ioctl, incorrect index = %d\n", index);
                 return -1;
             }
             
             _spi_ctx = _controller_ioctl_param->temp.spi_ctx; 
             if(_spi_ctx != &_controller_ctx->spi_ctx[index]) {
                 printf("spi_controller_ioctl, incorrect ioctl parameters\n");
                 return -1;
             }

             _tx_buf = _controller_ioctl_param->transfer_message_param.in.tx_buf;
             _tx_len = &_controller_ioctl_param->transfer_message_param.in.tx_len;
             _rx_buf = _controller_ioctl_param->transfer_message_param.out.rx_buf;
             _rx_len = &_controller_ioctl_param->transfer_message_param.out.rx_len;
             spi_ioctl_param.cmd = SPI_IOCTL_CMD_TRANSFER_MESSAGE;
             spi_ioctl_param.transfer_message_param.in.mode = _spi_ctx->mode;
             spi_ioctl_param.transfer_message_param.in.speed = _spi_ctx->speed;
             spi_ioctl_param.transfer_message_param.in.cs_polarity = _spi_ctx->cs_polarity;
             spi_ioctl_param.transfer_message_param.in.bits_per_word = _spi_ctx->bits_per_word;
             spi_ioctl_param.transfer_message_param.in.dev_fd = _spi_ctx->dev_fd;
             spi_ioctl_param.transfer_message_param.in._tx_buf = _tx_buf;
             spi_ioctl_param.transfer_message_param.in._tx_len = _tx_len;
             spi_ioctl_param.transfer_message_param.in_out._rx_buf = _rx_buf;
             spi_ioctl_param.transfer_message_param.in_out._rx_len = _rx_len;
             ret = _spi_ops->ioctl(&spi_ioctl_param);
             if(ret < 0) {
                 printf("spi_controller_ioctl->spi_ops->ioctl fail\n");
                 return -1;
             }

             return 0;
        break;
#endif
        default:
             printf("i2c_controller_ioctl, unknown cmd %d\n", _cmd);
             return -1;
        break;
    }

    
    return ret;
}



#if 0

int main(void)
{
    uint16_t cmd;
    int ret = 0, i;
    log_controller_context log_ctx;
    register_i2c_bus_parameters i2c_bus_param;
    i2c_controller_context i2c_controller_ctx;
    i2c_controller_ioctl_parameters i2c_controller_ioctl_param;
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

#if 1

    for(i=0; i<5; i++) {

        cmd = I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES;
        i2c_controller_ioctl_param.write_multi_bytes_param.in.i2c_dev_addr = 0x51;
        i2c_controller_ioctl_param.write_multi_bytes_param.in.data[0] = i;
        i2c_controller_ioctl_param.write_multi_bytes_param.in.data[1] = i;
        i2c_controller_ioctl_param.write_multi_bytes_param.in.length = 2;
        ret = i2c_controller_ctx.ops.ioctl(&i2c_controller_ctx, cmd, &i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("i2c_controller->ioctl fail\n");
            return -1;
        }

        sleep(1);
    }

    for(i=0; i<5; i++) {

        cmd = I2C_CONTROLLER_IOCTL_CMD_READ_MULTI_BYTES;
        i2c_controller_ioctl_param.read_multi_bytes_param.in.i2c_dev_addr = 0x51;
        i2c_controller_ioctl_param.read_multi_bytes_param.in.wr_data[0] = i;
        i2c_controller_ioctl_param.read_multi_bytes_param.in.wr_length = 1;
        i2c_controller_ioctl_param.read_multi_bytes_param.in.rd_length = 1;
        ret = i2c_controller_ctx.ops.ioctl(&i2c_controller_ctx, cmd, &i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("i2c_controller->ioctl fail\n");
            return -1;
        }

        printf("rd_data[%d] 0x%x\n", i, i2c_controller_ioctl_param.read_multi_bytes_param.out.rd_data[0]);

        sleep(1);
    }
    
#endif

#if 0
    cmd = I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES;
    i2c_controller_ioctl_param.write_multi_bytes_param.in.i2c_dev_addr = 0x51;
    i2c_controller_ioctl_param.write_multi_bytes_param.in.data[0] = 0x4;
    i2c_controller_ioctl_param.write_multi_bytes_param.in.data[1] = 0x33;
    //i2c_controller_ioctl_param.write_multi_bytes_param.in.data[1] = 0x78;
    i2c_controller_ioctl_param.write_multi_bytes_param.in.length = 2;
    ret = i2c_controller_ctx.ops.ioctl(&i2c_controller_ctx, cmd, &i2c_controller_ioctl_param);
    if(ret < 0) {
        printf("i2c_controller->ioctl fail\n");
        return -1;
    }


    sleep(1);

    cmd = I2C_CONTROLLER_IOCTL_CMD_READ_MULTI_BYTES;
    i2c_controller_ioctl_param.read_multi_bytes_param.in.i2c_dev_addr = 0x51;
    i2c_controller_ioctl_param.read_multi_bytes_param.in.wr_data[0] = 3;
    i2c_controller_ioctl_param.read_multi_bytes_param.in.wr_length = 1;
    i2c_controller_ioctl_param.read_multi_bytes_param.in.rd_length = 1;
    ret = i2c_controller_ctx.ops.ioctl(&i2c_controller_ctx, cmd, &i2c_controller_ioctl_param);
    if(ret < 0) {
        printf("i2c_controller->ioctl fail\n");
        return -1;
    }

    printf("rd_data[0] 0x%x\n", i2c_controller_ioctl_param.read_multi_bytes_param.out.rd_data[0]);

    sleep(1);

    cmd = I2C_CONTROLLER_IOCTL_CMD_READ_MULTI_BYTES;
    i2c_controller_ioctl_param.read_multi_bytes_param.in.i2c_dev_addr = 0x51;
    i2c_controller_ioctl_param.read_multi_bytes_param.in.wr_data[0] = 4;
    i2c_controller_ioctl_param.read_multi_bytes_param.in.wr_length = 1;
    i2c_controller_ioctl_param.read_multi_bytes_param.in.rd_length = 1;
    ret = i2c_controller_ctx.ops.ioctl(&i2c_controller_ctx, cmd, &i2c_controller_ioctl_param);
    if(ret < 0) {
        printf("i2c_controller->ioctl fail\n");
        return -1;
    }

    printf("rd_data[1] 0x%x\n", i2c_controller_ioctl_param.read_multi_bytes_param.out.rd_data[0]);

#endif

    i2c_controller_ctx.ops.deinit(&i2c_controller_ctx);

    return ret;
}

#endif 

