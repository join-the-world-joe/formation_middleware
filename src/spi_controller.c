/**
* @file     spi_controller.c
* @brief    spi_controller.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     09/11/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "log_controller.h"
#include "spi_controller.h"
#include "hal_spi.h"

void spi_controller_list_resources(spi_controller_context *spi_controller_ctx)
{
    int _nb_spi_bus, index;
    spi_context *_spi_ctx = NULL;
    spi_controller_context *_controller_ctx = spi_controller_ctx;

    if(!_controller_ctx) {
        printf("gpio_controller_list_resources fail, incorrect parameters\n");
        return ;
    }

    _spi_ctx = _controller_ctx->spi_ctx;
    _nb_spi_bus = _controller_ctx->nb_spi_bus;

    printf("***********************************SPI BUS Resources***********************************\n");
    for(index=0; index<_nb_spi_bus; index++) {
        if(_spi_ctx[index].in_use) {
            printf("Index: %d\n", index);
            printf("Port Signature: %s\n", _spi_ctx[index].signature);
            printf("SPI Mode: %d\n", _spi_ctx[index].mode);
            printf("SPI Speed: %d\n", _spi_ctx[index].speed);
            printf("SPI CS_Polarity %d\n", _spi_ctx[index].cs_polarity);
            printf("SPI Bits Per Word: %d\n", _spi_ctx[index].bits_per_word);
            printf("Device FD: %d\n", _spi_ctx[index].dev_fd);
        }
    }
    printf("***************************************************************************************\n");

}

int spi_controller_init(spi_controller_context *spi_controller_ctx, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param)
{
    int ret = 0, _nb_spi_bus = NB_SPI_BUS, i;
    spi_context *_spi_ctx = NULL;
    spi_controller_context *_controller_ctx = spi_controller_ctx;
    log_controller_context *_log_controller_ctx = log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param = log_controller_ioctl_param;

    if(!_controller_ctx || !_log_controller_ctx || !_log_controller_ioctl_param) {
        printf("gpio_controller_init fail, incorrect parameters\n");
        return -1;
    }

    _controller_ctx->count = 0;
    _controller_ctx->nb_spi_bus = _nb_spi_bus;
    _controller_ctx->_log_controller_ctx = _log_controller_ctx; 
    _controller_ctx->_log_controller_ioctl_param = _log_controller_ioctl_param;
    _spi_ctx = _controller_ctx->spi_ctx;
    
    // we'd better initialize all the important variables here    
    for(i=0; i<_nb_spi_bus; i++) {
        _spi_ctx[i].index = i;
        _spi_ctx[i].in_use = 0;
        _spi_ctx[i].dev_fd = -1;
        memset(_spi_ctx[i].signature, 0, MAX_LENGTH_OF_SPI_BUS_SIGNATURE);
    }
    
    _controller_ctx->ops.init = spi_controller_init;
    _controller_ctx->ops.register_context = spi_controller_register_spi_bus;
    _controller_ctx->ops.unregister_context = spi_controller_unregister_spi_bus;
    _controller_ctx->ops.list_resources = spi_controller_list_resources;
    _controller_ctx->ops.ioctl = spi_controller_ioctl;
    _controller_ctx->ops.deinit = spi_controller_deinit;
    
    _controller_ctx->spi_ops.init = hal_spi_init;
    _controller_ctx->spi_ops.ioctl = hal_spi_ioctl;
    _controller_ctx->spi_ops.deinit = hal_spi_deinit;

    return ret;
}

void spi_controller_deinit(spi_controller_context *spi_controller_ctx)
{
    int ret = 0, index;
    int _nb_spi_bus;
    spi_context *_spi_ctx = NULL;
    spi_bus_ops *_spi_ops = NULL;
    spi_controller_context *_controller_ctx = spi_controller_ctx;
    unregister_spi_bus_parameters spi_bus_param;

    if(!_controller_ctx) {
        printf("gpio_controller_deinit fail, incorrect parameters\n");
        return ;
    }

    _spi_ops = &_controller_ctx->spi_ops;
    _spi_ctx = _controller_ctx->spi_ctx; 
    _nb_spi_bus = _controller_ctx->nb_spi_bus;

    for(index=0; index<_nb_spi_bus; index++) {
        if(_spi_ctx[index].in_use) {
            //printf("spi_controller_deinit: index %d is in use\n", index);
            memset(spi_bus_param.in.signature, 0, MAX_LENGTH_OF_SPI_BUS_SIGNATURE);
            strcpy(spi_bus_param.in.signature, _spi_ctx[index].signature);
            ret = _controller_ctx->ops.unregister_context(_controller_ctx, &spi_bus_param);
            if(ret < 0) {
                printf("spi_controller_deinit->unregister_spi_bus fail, port %s\n", _spi_ctx[index].signature);
                return ;
            }
        }
    }

    return ;
}

int spi_controller_register_spi_bus(spi_controller_context *spi_controller_ctx, register_spi_bus_parameters *spi_bus_param)
{
    int ret = 0, i, done;
    uint32_t _nb_spi_bus;
    char *_bus_signatrue = NULL;
    spi_context *_spi_ctx = NULL;
    spi_bus_ops *_spi_ops = NULL;
    register_spi_bus_parameters *_spi_bus_param = spi_bus_param;
    spi_controller_context *_controller_ctx = spi_controller_ctx;

    if(!_controller_ctx || !_spi_bus_param) {
        printf("spi_controller_register_spi_bus fail, incorrect parameters\n");
        return -1;
    }

    _nb_spi_bus = _controller_ctx->nb_spi_bus;
    _spi_ctx = _controller_ctx->spi_ctx;
    _spi_ops = &_controller_ctx->spi_ops;
    _bus_signatrue = _spi_bus_param->in.signature;
    
    if(_controller_ctx->count + 1 > _nb_spi_bus) {
        printf("spi context in controller is full, there might be something wrong\n");
        return -1;
    }

    ret = _spi_ops->init(_spi_bus_param);
    if(ret < 0) {
        printf("spi_controller_register->io_ops->init fail\n");
        return -1;
    }

    // first, find a free context 
    for(i=0, done=0; i<_nb_spi_bus; i++) {

        if(_spi_ctx[i].in_use == 0) {

            // record input parameters 
            _spi_ctx[i].mode = _spi_bus_param->in.mode;
            _spi_ctx[i].speed = _spi_bus_param->in.speed;
            _spi_ctx[i].cs_polarity = _spi_bus_param->in.cs_polarity;
            _spi_ctx[i].bits_per_word = _spi_bus_param->in.bits_per_word; 
            strcpy(_spi_ctx[i].signature, _bus_signatrue);

            // record output parameters
            _spi_ctx[i].dev_fd = _spi_bus_param->out.dev_fd;

            
            // change the status of this gpio_context
            _spi_ctx[i].in_use = 1;

            done = 1; // all the related contents are recoreded in gpio_context

            break;
        }
    }

    if(!done) {
        printf("spi_controller_register fail, can't record port %s\n", _bus_signatrue);
        return -1;
    }

    _controller_ctx->count++; /* if done, increase the object counter */

    return ret;
}

int spi_controller_unregister_spi_bus(spi_controller_context *spi_controller_ctx, unregister_spi_bus_parameters *spi_bus_param)
{
    int ret = 0, index, done, i, same, find;
    uint32_t _nb_spi_bus;
    char *_bus_signatrue = NULL;
    spi_context *_spi_ctx = NULL;
    spi_bus_ops *_spi_ops = NULL;
    unregister_spi_bus_parameters *_spi_bus_param = spi_bus_param;
    spi_controller_context *_controller_ctx = spi_controller_ctx;

    if(!_controller_ctx || !_spi_bus_param) {
        printf("spi_controller_unregister_spi_bus fail, incorrect parameters\n");
        return -1;
    }

    _nb_spi_bus = _controller_ctx->nb_spi_bus;
    _spi_ctx = _controller_ctx->spi_ctx;
    _spi_ops = &_controller_ctx->spi_ops;
    _bus_signatrue = _spi_bus_param->in.signature;

    if((_controller_ctx->count - 1) < 0) {
        printf("spi context in controller is empty, there might be something wrong\n");
        return -1;
    }

    // check if we can find this signature in gpio_context
    for(index=0, find=0; index<_nb_spi_bus; index++) {

        same = !strcmp(_spi_ctx[index].signature, _bus_signatrue);
        if(same) {

            _spi_bus_param->temp.dev_fd = _spi_ctx[index].dev_fd;

            find = 1;

            break;
        }
    }

    if(!find) {
        printf("signature %s doesn't match any recorded signature in spi_context of spi_controller\n", _bus_signatrue);
        return -1;
    }
    
    ret = _spi_ops->deinit(_spi_bus_param);
    if(ret < 0) {
        printf("spi_controller_unregister_spi_bus->spi_ops->deinit fail\n");
        return -1;
    }

    _spi_ctx[index].in_use = 0;

    return ret;
}

int spi_controller_ioctl(spi_controller_context *spi_controller_ctx, uint16_t cmd, spi_controller_ioctl_parameters *spi_controller_ioctl_param)
{
    int ret = 0, same, done, index;
    char *_signature = NULL;
    uint8_t *_tx_buf, *_rx_buf;
    uint32_t *_tx_len, *_rx_len;
    uint16_t _cmd = cmd;    
    uint32_t _nb_spi_bus;
    spi_bus_ops *_spi_ops = NULL;
    spi_context *_spi_ctx = NULL;
    spi_ioctl_parameters spi_ioctl_param;
    spi_controller_context *_controller_ctx = spi_controller_ctx;
    spi_controller_ioctl_parameters *_controller_ioctl_param = spi_controller_ioctl_param;
    
    if(!_controller_ctx || !_controller_ioctl_param || _cmd < SPI_CONTROLLER_IOCTL_CMD_NB_BEGIN || _cmd > SPI_CONTROLLER_IOCTL_CMD_NB_END) {
        printf("spi_controller_ioctl incorrect parameters\n");
        return -1;
    }

    _nb_spi_bus = _controller_ctx->nb_spi_bus;
    _spi_ops = &_controller_ctx->spi_ops;
    
    switch(_cmd) {

        case SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX:
             _spi_ctx = _controller_ctx->spi_ctx;
             _signature = _controller_ioctl_param->acquire_spi_ctx_param.in.signature;         
             for(index=0, done=0; index<_nb_spi_bus; index++) {
                 same = !strcmp(_signature, _spi_ctx[index].signature);
                 if(same) {
    
                    _controller_ioctl_param->temp.index = index;
    
                    _controller_ioctl_param->temp.spi_ctx = &_spi_ctx[index];   
    
                    done = 1;
    
                    break;
                 }
             }
    
             if(done == 0) {
                 printf("spi_controller_ioctl->acquire_spi_context fail\n");
                 return -1;
             }

             return 0; // acquire spi context done!!!
        break;

        case SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE:
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

        default:
             printf("spi_controller_ioctl, unknown cmd %d\n", _cmd);
             return -1;
        break;
    }

    
    return ret;
}


#if 0
// for ADC test purposes
int main(void)
{
    uint32_t sample;
    double voltage;
    int ret = 0, cmd;
    uint8_t *_rx_buf, status;
    logging_ctx log_ctx;
    spi_controller_context spi_controller_ctx;
    register_spi_bus_parameters spi_bus_param_bus0_node0;
    spi_controller_ioctl_parameters spi_controller_ioctl_param;

    ret = spi_controller_init(&spi_controller_ctx, &log_ctx);
    if(ret < 0) {
        printf("spi_controller_init fail\n");
        return -1;
    }

    strcpy(spi_bus_param_bus0_node0.in.signature, "/dev/spidev0.0");
    spi_bus_param_bus0_node0.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node0.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node0.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node0.in.bits_per_word = SPI_BPW_08BITS;
    ret = spi_controller_ctx.ops.register_context(&spi_controller_ctx, &spi_bus_param_bus0_node0);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX;
    strcpy(spi_controller_ioctl_param.acquire_spi_ctx_param.in.signature, "/dev/spidev0.0");
    ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->acquire_spi_context fail\n");
        return -1;
    }

    // config0
    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x60;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x01;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 2;
    ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x61;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x03;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 2;
    ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x63;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x00;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 2;
    ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x64;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x00;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 2;
    ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x65;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x00;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 2;
    ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    _rx_buf = spi_controller_ioctl_param.transfer_message_param.out.rx_buf;

    while(1) {
#if 1
        cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x64;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x02;
        spi_controller_ioctl_param.transfer_message_param.in.tx_len = 2;
        ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
        if(ret < 0) {
            printf("spi_controller_ioctl->transfer_message fail\n");
            return -1;
        }

        cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x65;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x00;
        spi_controller_ioctl_param.transfer_message_param.in.tx_len = 2;
        ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
        if(ret < 0) {
            printf("spi_controller_ioctl->transfer_message fail\n");
            return -1;
        }
#endif
        cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x80;
        spi_controller_ioctl_param.transfer_message_param.in.tx_len = 1;
        ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
        if(ret < 0) {
            printf("spi_controller_ioctl->transfer_message fail\n");
            return -1;
        }

        cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x30;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x00;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[2] = 0x00;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[3] = 0x00;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[4] = 0x00;
        spi_controller_ioctl_param.transfer_message_param.in.tx_len = 5;
        ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
        if(ret < 0) {
            printf("spi_controller_ioctl->transfer_message fail\n");
            return -1;
        }

        status = _rx_buf[1];

        if(status & 0x80) {

            sample = _rx_buf[2] << 16 | _rx_buf[3] << 8 | _rx_buf[4];
            
            voltage = (5.0 * sample) / 7864320;
            if((status &0x1f) == 0x09) 
                printf("status 0x%x sample 0x%x voltage: %f\n", status, sample, voltage);

        }
           
        //sleep(1);

#if 1
        cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x64;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x00;
        spi_controller_ioctl_param.transfer_message_param.in.tx_len = 2;
        ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
        if(ret < 0) {
            printf("spi_controller_ioctl->transfer_message fail\n");
            return -1;
        }

        cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x65;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x00;
        spi_controller_ioctl_param.transfer_message_param.in.tx_len = 2;
        ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
        if(ret < 0) {
            printf("spi_controller_ioctl->transfer_message fail\n");
            return -1;
        }
#endif
        
    }

    return ret;
}
#endif 

#if 0
// for DAC test purposes
int main(void)
{
    int ret = 0, cmd;
    logging_ctx log_ctx;
    spi_controller_context spi_controller_ctx;
    register_spi_bus_parameters spi_bus_param_bus0_node6;
    spi_controller_ioctl_parameters spi_controller_ioctl_param;
    uint8_t prefix_bits, control_bits, address_bits, feature_bits;
    uint16_t data_bits; 
    double voltage, vref;
    uint16_t pcm_code;

    ret = spi_controller_init(&spi_controller_ctx, &log_ctx);
    if(ret < 0) {
        printf("spi_controller_init fail\n");
        return -1;
    }

    strcpy(spi_bus_param_bus0_node6.in.signature, "/dev/spidev0.6");
    spi_bus_param_bus0_node6.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node6.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node6.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node6.in.bits_per_word = SPI_BPW_08BITS;
    ret = spi_controller_register_spi_bus(&spi_controller_ctx, &spi_bus_param_bus0_node6);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX;
    strcpy(spi_controller_ioctl_param.acquire_spi_ctx_param.in.signature, "/dev/spidev0.6");
    ret = spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->acquire_spi_context fail\n");
        return -1;
    }
    
    spi_controller_list_resources(&spi_controller_ctx);

    prefix_bits = 0;
    control_bits = 0x09;
    address_bits = 0;
    data_bits = 0xc0;
    feature_bits = 0;    
    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = (prefix_bits << 4) | control_bits;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = (address_bits << 4) | (data_bits >> 12);
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[2] = data_bits >> 4;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[3] = (data_bits << 4) | feature_bits;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 4;
    ret = spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    voltage = 3.3;
    vref = 5.0;
    pcm_code = (voltage/vref)*0xFFFF;
    printf("pcm_code: 0x%x\n", pcm_code);

    printf("1");
    prefix_bits = 0;
    control_bits = 0x03;
    address_bits = 0;
    data_bits = 0xFFFF;
    feature_bits = 0;    
    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = (prefix_bits << 4) | control_bits;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = (address_bits << 4) | (data_bits >> 12);
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[2] = data_bits >> 4;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[3] = (data_bits << 4) | feature_bits;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 4;
    ret = spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    sleep(5);
    printf("2");
    prefix_bits = 0;
    control_bits = 0x03;
    address_bits = 0;
    data_bits = 0xFFFF/2;
    feature_bits = 0;    
    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = (prefix_bits << 4) | control_bits;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = (address_bits << 4) | (data_bits >> 12);
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[2] = data_bits >> 4;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[3] = (data_bits << 4) | feature_bits;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 4;
    ret = spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    sleep(5);
    printf("4");
    prefix_bits = 0;
    control_bits = 0x03;
    address_bits = 0;
    data_bits = 0xFFFF/4;
    feature_bits = 0;    
    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = (prefix_bits << 4) | control_bits;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = (address_bits << 4) | (data_bits >> 12);
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[2] = data_bits >> 4;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[3] = (data_bits << 4) | feature_bits;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 4;
    ret = spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    sleep(5);
    printf("6");
    prefix_bits = 0;
    control_bits = 0x03;
    address_bits = 0;
    data_bits = 0xFFFF/6;
    feature_bits = 0;    
    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = (prefix_bits << 4) | control_bits;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = (address_bits << 4) | (data_bits >> 12);
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[2] = data_bits >> 4;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[3] = (data_bits << 4) | feature_bits;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 4;
    ret = spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    sleep(5);
    return ret;
}
#endif 

#if 0

int main(void)
{
    char *_signature = NULL;
    int ret = 0, cmd, _dev_fd, i;
    uint8_t output = 0;
    logging_ctx log_ctx;
    spi_controller_context spi_controller_ctx;
    register_spi_bus_parameters spi_bus_param_bus0_node0;
    register_spi_bus_parameters spi_bus_param_bus0_node1;
    register_spi_bus_parameters spi_bus_param_bus0_node2;
    register_spi_bus_parameters spi_bus_param_bus0_node3;
    register_spi_bus_parameters spi_bus_param_bus0_node4;
    register_spi_bus_parameters spi_bus_param_bus0_node5;
    register_spi_bus_parameters spi_bus_param_bus0_node6;
    register_spi_bus_parameters spi_bus_param_bus0_node7;
    spi_controller_ioctl_parameters spi_controller_ioctl_param;

    ret = spi_controller_init(&spi_controller_ctx, &log_ctx);
    if(ret < 0) {
        printf("spi_controller_init fail\n");
        return -1;
    }

    strcpy(spi_bus_param_bus0_node0.in.signature, "/dev/spidev0.0");
    spi_bus_param_bus0_node0.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node0.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node0.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node0.in.bits_per_word = SPI_BPW_08BITS;
    ret = spi_controller_register_spi_bus(&spi_controller_ctx, &spi_bus_param_bus0_node0);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        return -1;
    }

    strcpy(spi_bus_param_bus0_node1.in.signature, "/dev/spidev0.1");
    spi_bus_param_bus0_node1.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node1.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node1.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node1.in.bits_per_word = SPI_BPW_08BITS;
    ret = spi_controller_register_spi_bus(&spi_controller_ctx, &spi_bus_param_bus0_node1);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        return -1;
    }

    strcpy(spi_bus_param_bus0_node2.in.signature, "/dev/spidev0.2");
    spi_bus_param_bus0_node2.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node2.in.speed = SPI_SPEED_10MHZ;
    spi_bus_param_bus0_node2.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node2.in.bits_per_word = SPI_BPW_08BITS;
    ret = spi_controller_register_spi_bus(&spi_controller_ctx, &spi_bus_param_bus0_node2);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        return -1;
    }

    strcpy(spi_bus_param_bus0_node3.in.signature, "/dev/spidev0.3");
    spi_bus_param_bus0_node3.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node3.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node3.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node3.in.bits_per_word = SPI_BPW_08BITS;
    ret = spi_controller_register_spi_bus(&spi_controller_ctx, &spi_bus_param_bus0_node3);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        return -1;
    }

    strcpy(spi_bus_param_bus0_node4.in.signature, "/dev/spidev0.4");
    spi_bus_param_bus0_node4.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node4.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node4.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node4.in.bits_per_word = SPI_BPW_08BITS;
    ret = spi_controller_register_spi_bus(&spi_controller_ctx, &spi_bus_param_bus0_node4);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        return -1;
    }

    strcpy(spi_bus_param_bus0_node5.in.signature, "/dev/spidev0.5");
    spi_bus_param_bus0_node5.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node5.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node5.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node5.in.bits_per_word = SPI_BPW_08BITS;
    ret = spi_controller_register_spi_bus(&spi_controller_ctx, &spi_bus_param_bus0_node5);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        return -1;
    }

    strcpy(spi_bus_param_bus0_node6.in.signature, "/dev/spidev0.6");
    spi_bus_param_bus0_node6.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node6.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node6.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node6.in.bits_per_word = SPI_BPW_08BITS;
    ret = spi_controller_register_spi_bus(&spi_controller_ctx, &spi_bus_param_bus0_node6);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        return -1;
    }

    strcpy(spi_bus_param_bus0_node7.in.signature, "/dev/spidev0.7");
    spi_bus_param_bus0_node7.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node7.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node7.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node7.in.bits_per_word = SPI_BPW_08BITS;
    ret = spi_controller_register_spi_bus(&spi_controller_ctx, &spi_bus_param_bus0_node7);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        return -1;
    }

    spi_controller_list_resources(&spi_controller_ctx);

    cmd = SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX;
    strcpy(spi_controller_ioctl_param.acquire_spi_ctx_param.in.signature, "/dev/spidev0.3");
    ret = spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->acquire_spi_context fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x40;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x0A;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[2] = 0x28;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 3;
    ret = spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }
    
    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x41;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x0A;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[2] = 0x00;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 3;
    ret = spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    printf("*********************** SPI Received Bytes ***************************\n");
    for(i=0; i<spi_controller_ioctl_param.transfer_message_param.in.tx_len ; i++) {
        printf("0x%x  ", spi_controller_ioctl_param.transfer_message_param.out.rx_buf[i]);
    }
    printf("\n***********************************************************************\n");

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x40;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x00;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[2] = 0x00;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 3;
    ret = spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x40;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x01;
    spi_controller_ioctl_param.transfer_message_param.in.tx_buf[2] = 0x00;
    spi_controller_ioctl_param.transfer_message_param.in.tx_len = 3;
    ret = spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    while(1) {

        if(output) {
            output = 0;
        } else {
            output = 0xff;
        }

        printf("set output to 0x%x\n", output);

        cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x40;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x12;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[2] = output;
        spi_controller_ioctl_param.transfer_message_param.in.tx_len = 3;
        spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);

        cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[0] = 0x40;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[1] = 0x13;
        spi_controller_ioctl_param.transfer_message_param.in.tx_buf[2] = output;
        spi_controller_ioctl_param.transfer_message_param.in.tx_len = 3;
        spi_controller_ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);

        sleep(5);
    }

    spi_controller_deinit(&spi_controller_ctx);

    spi_controller_list_resources(&spi_controller_ctx);

    return ret;
}

#endif
