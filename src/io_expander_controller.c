/**
* @file     io_expander_controller.c
* @brief    io_expander_controller.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     12/11/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "log_controller.h"
#include "io_expander_controller.h"
#include "hal_io_expander.h"

#include "debug.h"

#ifdef DATA_FLOW_DEBUG
#include "spi_controller.h"
#endif

#include "spi_controller.h"


void io_expander_controller_list_resources(io_expander_controller_context *io_expander_controller_ctx)
{
    int _nb_io_expander_devices, index, _nb_io_ports, j;
    io_expander_context *_io_expander_ctx = NULL;
    io_expander_controller_context *_controller_ctx = io_expander_controller_ctx;

    if(!_controller_ctx) {
        printf("io_expander_controller_list_resources fail, incorrect parameters\n");
        return ;
    }

    _io_expander_ctx = _controller_ctx->io_expander_ctx;
    _nb_io_expander_devices = _controller_ctx->nb_io_expander_devices;
    

    printf("***********************************IO Expander Resources***********************************\n");
    for(index=0; index<_nb_io_expander_devices; index++) {
        if(_io_expander_ctx[index].in_use) {
     
            printf("Index: %d\n", index);
            printf("Bus Type: %s\n", _io_expander_ctx[index].bus_type == IO_EXPANDER_BUS_TYPE_SPI?"SPI":"I2C");
            printf("IO Expander Signature: %s\n", _io_expander_ctx[index].signature);

            printf("--------------------------------------------------------------------------------------\n");
            for(j=0; j<_io_expander_ctx[index].nb_io_ports; j++) {
                printf("Terminal: %s\n", _io_expander_ctx[index].io_ports[j].terminal);
                printf("Direction: %s\n", _io_expander_ctx[index].io_ports[j].direction==IO_EXPANDER_DIRECTION_OUT?"Output":"Input");
                printf("config_data1: 0x%x\n", _io_expander_ctx[index].io_ports[j].config_data1);
                printf("config_data2: 0x%x\n", _io_expander_ctx[index].io_ports[j].config_data2);
                printf("config_data3: 0x%x\n", _io_expander_ctx[index].io_ports[j].config_data3);
            }
            printf("--------------------------------------------------------------------------------------\n");
        }
    }
    printf("********************************************************************************************\n");

    
}

int io_expander_controller_init(io_expander_controller_context *io_expander_controller_ctx, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param)
{
    int ret = 0, _nb_io_expander_devices = NB_IO_EXPANDER_DEVICES, i;
    io_expander_context *_io_expander_ctx = NULL;
    io_expander_controller_context *_controller_ctx = io_expander_controller_ctx;
    log_controller_context *_log_controller_ctx = log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param = log_controller_ioctl_param;

    if(!_controller_ctx || !_log_controller_ctx || !_log_controller_ioctl_param) {
        printf("io_expander_controller_init fail, incorrect parameters\n");
        return -1;
    }

    _controller_ctx->count = 0;
    _controller_ctx->_log_controller_ctx = _log_controller_ctx;
    _controller_ctx->_log_controller_ioctl_param = _log_controller_ioctl_param;
    _controller_ctx->nb_io_expander_devices = _nb_io_expander_devices;

    _io_expander_ctx = _controller_ctx->io_expander_ctx;
    
    // we'd better initialize all the important variables here    
    for(i=0; i<_nb_io_expander_devices; i++) {
        _io_expander_ctx[i].index = i;
        _io_expander_ctx[i].in_use = 0;
        _io_expander_ctx[i].gpio_a = 0;
        _io_expander_ctx[i].gpio_b = 0;
        _io_expander_ctx[i].spi_controller_ctx = NULL;
        _io_expander_ctx[i].spi_controller_ioctl_param = NULL;
        _io_expander_ctx[i].bus_type = IO_EXPANDER_BUS_TYPE_UNKNOWN;
        memset(_io_expander_ctx[i].signature, 0, MAX_LENGTH_OF_IO_EXPANDER_SIGNATURE);
    }
    
    _controller_ctx->ops.init = io_expander_controller_init;
    _controller_ctx->ops.register_context = io_expander_controller_register_io_expander;
    _controller_ctx->ops.unregister_context = io_expander_controller_unregister_io_expander;
    _controller_ctx->ops.list_resources = io_expander_controller_list_resources;
    _controller_ctx->ops.ioctl = io_expander_controller_ioctl;
    _controller_ctx->ops.deinit = io_expander_controller_deinit;
    
    _controller_ctx->io_expander_ops.init = hal_io_expander_init;
    _controller_ctx->io_expander_ops.ioctl = hal_io_expander_ioctl;
    _controller_ctx->io_expander_ops.deinit = hal_io_expander_deinit;

    return ret;
}

void io_expander_controller_deinit(io_expander_controller_context *io_expander_controller_ctx)
{
    int ret = 0, index;
    int _nb_io_expander_devices;
    io_expander_context *_io_expander_ctx = NULL;
    io_expander_ops *_io_expander_ops = NULL;
    io_expander_controller_context *_controller_ctx = io_expander_controller_ctx;
    unregister_io_expander_parameters io_expander_param;

    if(!_controller_ctx) {
        printf("gpio_controller_deinit fail, incorrect parameters\n");
        return ;
    }

    _io_expander_ops = &_controller_ctx->io_expander_ops;
    _io_expander_ctx = _controller_ctx->io_expander_ctx; 
    _nb_io_expander_devices = _controller_ctx->nb_io_expander_devices;

    for(index=0; index<_nb_io_expander_devices; index++) {
        if(_io_expander_ctx[index].in_use) {
            //printf("spi_controller_deinit: index %d is in use\n", index);
            memset(io_expander_param.in.signature, 0, MAX_LENGTH_OF_IO_EXPANDER_SIGNATURE);
            strcpy(io_expander_param.in.signature, _io_expander_ctx[index].signature);
            ret = _controller_ctx->ops.unregister_context(_controller_ctx, &io_expander_param);
            if(ret < 0) {
                printf("io_expander_controller_deinit->unregister_io_expander fail, device: %s\n", _io_expander_ctx[index].signature);
                return ;
            }
        }
    }

    return ;
}

int io_expander_controller_register_io_expander(io_expander_controller_context *io_expander_controller_ctx, register_io_expander_parameters *io_expander_param)
{
    int ret = 0, i, done, _bus_type, j, _nb_io_ports;
    char *_terminal_signature = NULL;
    uint32_t _nb_io_expander_devices;
    char *_io_expander_signatrue = NULL;
    io_expander_context *_io_expander_ctx = NULL;
    io_expander_ops *_io_expander_ops = NULL;
    register_io_expander_parameters *_io_expander_param = io_expander_param;
    io_expander_controller_context *_controller_ctx = io_expander_controller_ctx;

    if(!_controller_ctx || !_io_expander_param) {
        printf("io_expander_controller_register_io_expander fail, incorrect parameters\n");
        return -1;
    }

    _nb_io_expander_devices = _controller_ctx->nb_io_expander_devices;
    _io_expander_ctx = _controller_ctx->io_expander_ctx;
    _io_expander_ops = &_controller_ctx->io_expander_ops;
    _io_expander_signatrue = _io_expander_param->in.signature;
    _bus_type = _io_expander_param->in.bus_type;

    if(_bus_type != IO_EXPANDER_BUS_TYPE_SPI) {
        printf("we only support spi kind io expander\n");
        return -1;
    }

    if(_controller_ctx->count + 1 > _nb_io_expander_devices) {
        printf("io expander context in controller is full, there might be something wrong\n");
        return -1;
    }

    if(_io_expander_param->in.nb_io_ports > MAX_NB_OF_IO_EXPANDER_PORTS) {
        printf("io_expander_controller_register_io_expander fail, exceeding the max. amount of io ports");
        return -1;
    }

    ret = _io_expander_ops->init(_io_expander_param);
    if(ret < 0) {
        printf("io_expander_controller_register_io_expander->io_ops->init fail\n");
        return -1;
    }

    // first, find a free context 
    for(i=0, done=0; i<_nb_io_expander_devices; i++) {

        if(_io_expander_ctx[i].in_use == 0) {

            _nb_io_ports = _io_expander_param->in.nb_io_ports;
           
            // record input parameters 
            _io_expander_ctx[i].bus_type = _bus_type;
            _io_expander_ctx[i].nb_io_ports = _nb_io_ports;
            if(_bus_type == IO_EXPANDER_BUS_TYPE_SPI) {
                _io_expander_ctx[i].spi_controller_ioctl_param = _io_expander_param->in.spi_controller_ioctl_param;
                _io_expander_ctx[i].spi_controller_ctx = _io_expander_param->in.spi_controller_ctx;
            }
            strcpy(_io_expander_ctx[i].signature, _io_expander_signatrue);

            // record output parameters
            for(j=0; j<_nb_io_ports; j++) {
                _terminal_signature = _io_expander_param->in_out.io_ports[j].terminal;  
                _io_expander_ctx[i].io_ports[j].direction = _io_expander_param->in_out.io_ports[j].direction;
                _io_expander_ctx[i].io_ports[j].config_data1 = _io_expander_param->in_out.io_ports[j].config_data1;
                _io_expander_ctx[i].io_ports[j].config_data2 = _io_expander_param->in_out.io_ports[j].config_data2;
                _io_expander_ctx[i].io_ports[j].config_data3 = _io_expander_param->in_out.io_ports[j].config_data3;
                strcpy(_io_expander_ctx[i].io_ports[j].terminal, _terminal_signature);
            }

            // change the status of this gpio_context
            _io_expander_ctx[i].in_use = 1;

            done = 1; // all the related contents are recoreded in gpio_context

            break;
        }
    }

    if(!done) {
        printf("io_expander_controller_register_io_expander fail, can't record port %s\n", _io_expander_signatrue);
        return -1;
    }

    _controller_ctx->count++; /* if done, increase the object counter */

    return ret;
}

int io_expander_controller_unregister_io_expander(io_expander_controller_context *io_expander_controller_ctx, unregister_io_expander_parameters *io_expander_param)
{
    int ret = 0, index, done, i, same, find;
    uint32_t _nb_io_expander_devices;
    char *_io_expander_signatrue = NULL;
    io_expander_context *_io_expander_ctx = NULL;
    io_expander_ops *_io_expander_ops = NULL;
    unregister_io_expander_parameters *_io_expander_param = io_expander_param;
    io_expander_controller_context *_controller_ctx = io_expander_controller_ctx;

    if(!_controller_ctx || !_io_expander_param) {
        printf("io_expander_controller_unregister_io_expander fail, incorrect parameters\n");
        return -1;
    }

    _nb_io_expander_devices = _controller_ctx->nb_io_expander_devices;
    _io_expander_ctx = _controller_ctx->io_expander_ctx;
    _io_expander_ops = &_controller_ctx->io_expander_ops;
    _io_expander_signatrue = _io_expander_param->in.signature;

    if((_controller_ctx->count - 1) < 0) {
        printf("io expander context in controller is empty, there might be something wrong\n");
        return -1;
    }

    // check if we can find this signature in gpio_context
    for(index=0, find=0; index<_nb_io_expander_devices; index++) {

        same = !strcmp(_io_expander_ctx[index].signature, _io_expander_signatrue);
        if(same) {

            find = 1;

            break;
        }
    }

    if(!find) {
        printf("signature %s doesn't match any recorded signature in io_expander_context of io_expander_controller\n", _io_expander_signatrue);
        return -1;
    }
    
    ret = _io_expander_ops->deinit(_io_expander_param);
    if(ret < 0) {
        printf("io_expander_controller_unregister_io_expander->io_expander_ops->deinit fail\n");
        return -1;
    }

    _io_expander_ctx[index].in_use = 0;

    return ret;
}

int io_expander_controller_ioctl(io_expander_controller_context *io_expander_controller_ctx, uint16_t cmd, io_expander_controller_ioctl_parameters *io_expander_controller_ioctl_param)
{
    int ret = 0, same, done, index, _nb_io_expander_devices;
    char *_io_expander_signature = NULL;
    uint16_t _cmd = cmd;    
    io_expander_ops *_io_expander_ops = NULL;
    io_expander_context *_io_expander_ctx = NULL;
    io_expander_ioctl_parameters io_expander_ioctl_param;
    io_expander_controller_context *_controller_ctx = io_expander_controller_ctx;
    io_expander_controller_ioctl_parameters *_controller_ioctl_param = io_expander_controller_ioctl_param;
    
    if(!_controller_ctx || !_controller_ioctl_param || _cmd < IO_EXPANDER_CONTROLLER_IOCTL_CMD_NB_BEGIN || _cmd > IO_EXPANDER_CONTROLLER_IOCTL_CMD_NB_END) {
        printf("io_expander_controller_ioctl incorrect parameters\n");
        return -1;
    }

    _nb_io_expander_devices = _controller_ctx->nb_io_expander_devices;
    _io_expander_ops = &_controller_ctx->io_expander_ops;
    
    switch(_cmd) {

        case IO_EXPANDER_CONTROLLER_IOCTL_CMD_ACQUIRE_IO_EXPANDER_CONTEX:
             _io_expander_ctx = _controller_ctx->io_expander_ctx;
             _io_expander_signature = _controller_ioctl_param->acquire_io_expander_ctx_param.in.signature;         
             for(index=0, done=0; index<_nb_io_expander_devices; index++) {
                 same = !strcmp(_io_expander_signature, _io_expander_ctx[index].signature);
                 if(same) {
    
                    _controller_ioctl_param->temp.index = index;
    
                    _controller_ioctl_param->temp.io_expander_ctx = &_io_expander_ctx[index];   
    
                    done = 1;
    
                    break;
                 }
             }
    
             if(done == 0) {
                 printf("io_expander_controller_ioctl->acquire_io_expander_context fail\n");
                 return -1;
             }

             return 0; // acquire io_expander context done!!!
        break;

        case IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL:
             index = _controller_ioctl_param->temp.index;
             if(index < 0 || index >= _nb_io_expander_devices) {
                 printf("io_expander_controller_ioctl, incorrect index = %d\n", index);
                 return -1;
             }

             _io_expander_ctx = _controller_ioctl_param->temp.io_expander_ctx; 
             if(_io_expander_ctx != &_controller_ctx->io_expander_ctx[index]) {
                 printf("io_expander_controller_ioctl, incorrect ioctl parameters\n");
                 return -1;
             }
             
             io_expander_ioctl_param.cmd = IO_EXPANDER_IOCTL_CMD_SET_LEVEL;
             io_expander_ioctl_param.io_expander_set_level_param.in._gpio_a = &_io_expander_ctx->gpio_a;
             io_expander_ioctl_param.io_expander_set_level_param.in._gpio_b = &_io_expander_ctx->gpio_b;
             io_expander_ioctl_param.io_expander_set_level_param.in._nb_io_ports = _io_expander_ctx->nb_io_ports;
             io_expander_ioctl_param.io_expander_set_level_param.in._terminal = _controller_ioctl_param->set_level_param.in.terminal;
             io_expander_ioctl_param.io_expander_set_level_param.in._level = &_controller_ioctl_param->set_level_param.in.level;
             io_expander_ioctl_param.io_expander_set_level_param.in._bus_type = &_io_expander_ctx->bus_type;
             io_expander_ioctl_param.io_expander_set_level_param.in._io_ports = _io_expander_ctx->io_ports;
             io_expander_ioctl_param.io_expander_set_level_param.in._spi_controller_ctx = _io_expander_ctx->spi_controller_ctx;
             io_expander_ioctl_param.io_expander_set_level_param.in._spi_controller_ioctl_param = _io_expander_ctx->spi_controller_ioctl_param;
             ret = _io_expander_ops->ioctl(&io_expander_ioctl_param);
             if(ret < 0) {
                 printf("io_expander_controller_ioctl->io_expander_ops->ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        default:
             printf("io_expander_controller_ioctl, unknown cmd %d\n", _cmd);
             return -1;
        break;
    }

    
    return ret;
}

#if 0

int main(void)
{
    int ret = 0, cmd, bus_type, index;
    logging_ctx log_ctx;
    spi_controller_context spi_controller_ctx;
    register_spi_bus_parameters spi_bus_param_bus0_node1;
    io_expander_controller_context io_expander_controller_ctx;
    io_expander_controller_ioctl_parameters io_expander_controller_ioctl_param;
    register_io_expander_parameters io_expander_param;
    spi_controller_ioctl_parameters spi_controller_ioctl_param;

    printf("io_expander_controller.c\n");

    ret = spi_controller_init(&spi_controller_ctx, &log_ctx);
    if(ret < 0) {
        printf("spi_controller_init fail\n");
        return -1;
    }

    strcpy(spi_bus_param_bus0_node1.in.signature, "/dev/spidev0.1");
    spi_bus_param_bus0_node1.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node1.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node1.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node1.in.bits_per_word = SPI_BPW_08BITS;
    ret = spi_controller_ctx.ops.register_context(&spi_controller_ctx, &spi_bus_param_bus0_node1);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX;
    strcpy(spi_controller_ioctl_param.acquire_spi_ctx_param.in.signature, "/dev/spidev0.1");
    ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->acquire_spi_context fail\n");
        return -1;
    }
    
    spi_controller_ctx.ops.list_resources(&spi_controller_ctx);

    ret = io_expander_controller_init(&io_expander_controller_ctx, &log_ctx);
    if(ret < 0) {
        printf("io_expander_controller_init fail\n");
        return -1;
    }

    strcpy(io_expander_param.in.signature, "U1");
    io_expander_param.in.bus_type = IO_EXPANDER_BUS_TYPE_SPI;
    io_expander_param.in.spi_controller_ctx = &spi_controller_ctx;
    io_expander_param.in.spi_controller_ioctl_param = &spi_controller_ioctl_param;
    strcpy(io_expander_param.in_out.io_ports[0].terminal, "KZ_H1");
    io_expander_param.in_out.io_ports[0].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[1].terminal, "KZ_M1");
    io_expander_param.in_out.io_ports[1].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[2].terminal, "KZ_L1");
    io_expander_param.in_out.io_ports[2].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[3].terminal, "JC1");
    io_expander_param.in_out.io_ports[3].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[4].terminal, "CF1");
    io_expander_param.in_out.io_ports[4].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[5].terminal, "CK1");
    io_expander_param.in_out.io_ports[5].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[6].terminal, "CLED1");
    io_expander_param.in_out.io_ports[6].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[7].terminal, "FLED1");
    io_expander_param.in_out.io_ports[7].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[8].terminal, "KZ_H2");
    io_expander_param.in_out.io_ports[8].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[9].terminal, "KZ_M2");
    io_expander_param.in_out.io_ports[9].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[10].terminal, "KZ_L2");
    io_expander_param.in_out.io_ports[10].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[11].terminal, "JC2");
    io_expander_param.in_out.io_ports[11].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[12].terminal, "CF2");
    io_expander_param.in_out.io_ports[12].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[13].terminal, "CK2");
    io_expander_param.in_out.io_ports[13].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[14].terminal, "CLED2");
    io_expander_param.in_out.io_ports[14].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[15].terminal, "FLED2");
    io_expander_param.in_out.io_ports[15].direction = IO_EXPANDER_DIRECTION_OUT;
    io_expander_param.in.nb_io_ports = 16;
    ret = io_expander_controller_ctx.ops.register_context(&io_expander_controller_ctx, &io_expander_param);
    if(ret < 0) {
        printf("io_expander_controller_register_io_expander fail\n");
        return -1;
    }

    io_expander_controller_ctx.ops.list_resources(&io_expander_controller_ctx);

    cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_ACQUIRE_IO_EXPANDER_CONTEX;
    strcpy(io_expander_controller_ioctl_param.acquire_io_expander_ctx_param.in.signature, "U1"); 
    ret = io_expander_controller_ctx.ops.ioctl(&io_expander_controller_ctx, cmd, &io_expander_controller_ioctl_param);
    if(ret < 0) {
        printf("io_expander_controller_ioctl fail\n");
        return -1;
    }

    //bus_type = io_expander_controller_ioctl_param.temp.io_expander_ctx->bus_type;
    //index = io_expander_controller_ioctl_param.temp.io_expander_ctx->index;
    //printf("index %d bus_type %d\n", index, bus_type);

#if 0
    cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
    strcpy(io_expander_controller_ioctl_param.set_level_param.in.terminal, "FLED1");
    io_expander_controller_ioctl_param.set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_HIGH;
    ret = io_expander_controller_ctx.ops.ioctl(&io_expander_controller_ctx, cmd, &io_expander_controller_ioctl_param);
    if(ret < 0) {
        printf("io_expander_controller_ioctl fail\n");
        return -1;
    }
#endif

    while(1) {

        cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
        strcpy(io_expander_controller_ioctl_param.set_level_param.in.terminal, "KZ_H1");
        io_expander_controller_ioctl_param.set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_HIGH;
        ret = io_expander_controller_ctx.ops.ioctl(&io_expander_controller_ctx, cmd, &io_expander_controller_ioctl_param);
        if(ret < 0) {
            printf("io_expander_controller_ioctl fail\n");
            return -1;
        }

        sleep(5);

        cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
        strcpy(io_expander_controller_ioctl_param.set_level_param.in.terminal, "KZ_H1");
        io_expander_controller_ioctl_param.set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_LOW;
        ret = io_expander_controller_ctx.ops.ioctl(&io_expander_controller_ctx, cmd, &io_expander_controller_ioctl_param);
        if(ret < 0) {
            printf("io_expander_controller_ioctl fail\n");
            return -1;
        }

        sleep(5);

    }

    io_expander_controller_ctx.ops.deinit(&io_expander_controller_ctx);

    io_expander_controller_ctx.ops.list_resources(&io_expander_controller_ctx);

    spi_controller_ctx.ops.deinit(&spi_controller_ctx);

    return ret;
}
#endif

