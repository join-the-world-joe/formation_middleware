/**
* @file     da_controller.c
* @brief    da_controller.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     13/11/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "log_controller.h"
#include "dac_controller.h"
#include "hal_dac.h"

#include "debug.h"

#ifdef DATA_FLOW_DEBUG
#include "spi_controller.h"
#endif
#include "spi_controller.h"

void dac_controller_list_resources(dac_controller_context *dac_controller_ctx)
{
    int _nb_dac_devices, index, _nb_terminals, j;
    dac_context *_dac_ctx = NULL;
    dac_controller_context *_controller_ctx = dac_controller_ctx;

    if(!_controller_ctx) {
        printf("dac_controller_list_resources fail, incorrect parameters\n");
        return ;
    }

    _dac_ctx = _controller_ctx->dac_ctx;
    _nb_dac_devices = _controller_ctx->nb_dac_devices;
    

    printf("***********************************DAC Resources***********************************\n");
    for(index=0; index<_nb_dac_devices; index++) {
        if(_dac_ctx[index].in_use) {
     
            printf("Index: %d\n", index);
            printf("Bus Type: %s\n", _dac_ctx[index].bus_type == DAC_BUS_TYPE_SPI?"SPI":"I2C");
            printf("DAC Signature: %s\n", _dac_ctx[index].signature);
            printf("--------------------------------------------------------------------------------------\n");
            for(j=0; j<_dac_ctx[index].nb_terminals; j++) {
                printf("Terminal: %s\n", _dac_ctx[index].analog_outputs[j].terminal);
                printf("Address Bits: 0x%x\n", _dac_ctx[index].analog_outputs[j].config_data1);
            }
            printf("--------------------------------------------------------------------------------------\n");
        }
    }
    printf("************************************************************************************\n");
}

int dac_controller_init(dac_controller_context *dac_controller_ctx, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param)
{
    int ret = 0, _nb_dac_devices = NB_DAC_DEVICES, i;
    dac_context *_da_ctx = NULL;
    dac_controller_context *_controller_ctx = dac_controller_ctx;
    log_controller_context *_log_controller_ctx = log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param = log_controller_ioctl_param; 

    if(!_controller_ctx || !_log_controller_ctx || !_log_controller_ioctl_param) {
        printf("dac_controller_init fail, incorrect parameters\n");
        return -1;
    }

    _controller_ctx->count = 0;
    _controller_ctx->_log_controller_ctx = _log_controller_ctx;
    _controller_ctx->_log_controller_ioctl_param = _log_controller_ioctl_param;
    _controller_ctx->nb_dac_devices = _nb_dac_devices;

    _da_ctx = _controller_ctx->dac_ctx;
    
    // we'd better initialize all the important variables here    
    for(i=0; i<_nb_dac_devices; i++) {
        _da_ctx[i].index = i;
        _da_ctx[i].in_use = 0;
        _da_ctx[i].spi_controller_ctx = NULL;
        _da_ctx[i].spi_controller_ioctl_param = NULL;
        _da_ctx[i].bus_type = DAC_BUS_TYPE_UNKNOWN;
        memset(_da_ctx[i].signature, 0, MAX_LENGTH_OF_DAC_SIGNATURE);
    }
    
    _controller_ctx->ops.init = dac_controller_init;
    _controller_ctx->ops.register_context = dac_controller_register_dac;
    _controller_ctx->ops.unregister_context = dac_controller_unregister_dac;
    _controller_ctx->ops.list_resources = dac_controller_list_resources;
    _controller_ctx->ops.ioctl = dac_controller_ioctl;
    _controller_ctx->ops.deinit = dac_controller_deinit;
    
    _controller_ctx->da_ops.init = hal_dac_init;
    _controller_ctx->da_ops.ioctl = hal_dac_ioctl;
    _controller_ctx->da_ops.deinit = hal_dac_deinit;

    return ret;
}

void dac_controller_deinit(dac_controller_context *dac_controller_ctx)
{
    int ret = 0, index;
    int _nb_dac_devices;
    dac_context *_dac_ctx = NULL;
    dac_ops *_da_ops = NULL;
    dac_controller_context *_controller_ctx = dac_controller_ctx;
    unregister_dac_parameters dac_param;

    if(!_controller_ctx) {
        printf("dac_controller_deinit fail, incorrect parameters\n");
        return ;
    }

    _da_ops = &_controller_ctx->da_ops;
    _dac_ctx = _controller_ctx->dac_ctx; 
    _nb_dac_devices = _controller_ctx->nb_dac_devices;

    for(index=0; index<_nb_dac_devices; index++) {
        if(_dac_ctx[index].in_use) {
            //printf("spi_controller_deinit: index %d is in use\n", index);
            memset(dac_param.in.signature, 0, MAX_LENGTH_OF_DAC_SIGNATURE);
            strcpy(dac_param.in.signature, _dac_ctx[index].signature);
            ret = _controller_ctx->ops.unregister_context(_controller_ctx, &dac_param);
            if(ret < 0) {
                printf("dac_controller_deinit->unregister_dac fail, device: %s\n", _dac_ctx[index].signature);
                return ;
            }
        }
    }

    return ;
}

int dac_controller_register_dac(dac_controller_context *dac_controller_ctx, register_dac_parameters* dac_param)
{
    int ret = 0, i, done, _bus_type, j, _nb_terminals;
    char *_terminal_signature = NULL;
    uint32_t _nb_dac_devices;
    char *_dac_signatrue = NULL;
    dac_context *_dac_ctx = NULL;
    dac_ops *_dac_ops = NULL;
    register_dac_parameters *_dac_param = dac_param;
    dac_controller_context *_controller_ctx = dac_controller_ctx;

    if(!_controller_ctx || !_dac_param) {
        printf("dac_controller_register_dac fail, incorrect parameters\n");
        return -1;
    }

    _nb_dac_devices = _controller_ctx->nb_dac_devices;
    _dac_ctx = _controller_ctx->dac_ctx;
    _dac_ops = &_controller_ctx->da_ops;
    _dac_signatrue = _dac_param->in.signature;
    _bus_type = _dac_param->in.bus_type;

    if(_bus_type != DAC_BUS_TYPE_SPI) {
        printf("we only support spi kind DAC\n");
        return -1;
    }

    if(_controller_ctx->count + 1 > _nb_dac_devices) {
        printf("dac context in controller is full, there might be something wrong\n");
        return -1;
    }

    if(_dac_param->in.nb_terminals > MAX_NB_OF_DAC_ANALOG_OUTPUT_TERMINALS) {
        printf("dac_controller_register_dac fail, exceeding the max. amount of io ports");
        return -1;
    }

    ret = _dac_ops->init(_dac_param);
    if(ret < 0) {
        printf("dac_controller_register_dac->da_ops->init fail\n");
        return -1;
    }

    // first, find a free context 
    for(i=0, done=0; i<_nb_dac_devices; i++) {

        if(_dac_ctx[i].in_use == 0) {

            _nb_terminals = _dac_param->in.nb_terminals;
           
            // record input parameters 
            _dac_ctx[i].bus_type = _bus_type;
            _dac_ctx[i].nb_terminals = _nb_terminals;
            if(_bus_type == DAC_BUS_TYPE_SPI) {
                _dac_ctx[i].spi_controller_ioctl_param = _dac_param->in.spi_controller_ioctl_param;
                _dac_ctx[i].spi_controller_ctx = _dac_param->in.spi_controller_ctx;
            }
            strcpy(_dac_ctx[i].signature, _dac_signatrue);

            // record output parameters
            for(j=0; j<_nb_terminals; j++) {
                _terminal_signature = _dac_param->in_out.analog_outputs[j].terminal;  
                _dac_ctx[i].analog_outputs[j].config_data1 = _dac_param->in_out.analog_outputs[j].config_data1;
                _dac_ctx[i].analog_outputs[j].config_data2 = _dac_param->in_out.analog_outputs[j].config_data2;
                _dac_ctx[i].analog_outputs[j].config_data3 = _dac_param->in_out.analog_outputs[j].config_data3;
                strcpy(_dac_ctx[i].analog_outputs[j].terminal, _terminal_signature);
            }

            // change the status of this gpio_context
            _dac_ctx[i].in_use = 1;

            done = 1; // all the related contents are recoreded in gpio_context

            break;
        }
    }

    if(!done) {
        printf("dac_controller_register_dac fail, can't record port %s\n", _dac_signatrue);
        return -1;
    }

    _controller_ctx->count++; /* if done, increase the object counter */

    return ret;
}

int dac_controller_unregister_dac(dac_controller_context *dac_controller_ctx, unregister_dac_parameters* dac_param)
{
    int ret = 0, index, done, i, same, find;
    uint32_t _nb_dac_devices;
    char *_dac_signatrue = NULL;
    dac_context *_dac_ctx = NULL;
    dac_ops *_da_ops = NULL;
    unregister_dac_parameters *_dac_param = dac_param;
    dac_controller_context *_controller_ctx = dac_controller_ctx;

    if(!_controller_ctx || !_dac_param) {
        printf("dac_controller_unregister_dac fail, incorrect parameters\n");
        return -1;
    }

    _nb_dac_devices = _controller_ctx->nb_dac_devices;
    _dac_ctx = _controller_ctx->dac_ctx;
    _da_ops = &_controller_ctx->da_ops;
    _dac_signatrue = _dac_param->in.signature;

    if((_controller_ctx->count - 1) < 0) {
        printf("dac context in controller is empty, there might be something wrong\n");
        return -1;
    }

    // check if we can find this signature in gpio_context
    for(index=0, find=0; index<_nb_dac_devices; index++) {

        same = !strcmp(_dac_ctx[index].signature, _dac_signatrue);
        if(same) {

            find = 1;

            break;
        }
    }

    if(!find) {
        printf("signature %s doesn't match any recorded signature in io_expander_context of io_expander_controller\n", _dac_signatrue);
        return -1;
    }
    
    ret = _da_ops->deinit(_dac_param);
    if(ret < 0) {
        printf("io_expander_controller_unregister_io_expander->io_expander_ops->deinit fail\n");
        return -1;
    }

    _dac_ctx[index].in_use = 0;

    return ret;
}


int dac_controller_ioctl(dac_controller_context *dac_controller_ctx, uint16_t cmd, dac_controller_ioctl_parameters *dac_controller_ioctl_param)
{
    int ret = 0, same, done, index, _nb_dac_devices;
    char *_dac_signature = NULL;
    uint16_t _cmd = cmd;    
    dac_ops *_dac_ops = NULL;
    dac_context *_dac_ctx = NULL;
    dac_ioctl_parameters dac_ioctl_param;
    dac_controller_context *_controller_ctx = dac_controller_ctx;
    dac_controller_ioctl_parameters *_controller_ioctl_param = dac_controller_ioctl_param;
    
    if(!_controller_ctx || !_controller_ioctl_param || _cmd < DAC_CONTROLLER_IOCTL_CMD_NB_BEGIN || _cmd > DAC_CONTROLLER_IOCTL_CMD_NB_END) {
        printf("dac_controller_ioctl incorrect parameters\n");
        return -1;
    }

    _nb_dac_devices = _controller_ctx->nb_dac_devices;
    _dac_ops = &_controller_ctx->da_ops;
    
    switch(_cmd) {

        case DAC_CONTROLLER_IOCTL_CMD_ACQUIRE_DAC_CONTEX:
             _dac_ctx = _controller_ctx->dac_ctx;
             _dac_signature = _controller_ioctl_param->acquire_dac_ctx_param.in.signature;         
             for(index=0, done=0; index<_nb_dac_devices; index++) {
                 same = !strcmp(_dac_signature, _dac_ctx[index].signature);
                 if(same) {
    
                    _controller_ioctl_param->temp.index = index;
    
                    _controller_ioctl_param->temp.dac_ctx = &_dac_ctx[index];   
                    
                    done = 1;
    
                    break;
                 }
             }
    
             if(done == 0) {
                 printf("dac_controller_ioctl->acquire_io_expander_context fail\n");
                 return -1;
             }

             return 0; // acquire io_expander context done!!!
        break;

        case DAC_CONTROLLER_IOCTL_CMD_WRITE_AND_UPDATE_CHANNEL:
             index = _controller_ioctl_param->temp.index;
             if(index < 0 || index >= _nb_dac_devices) {
                 printf("dac_controller_ioctl, incorrect index = %d\n", index);
                 return -1;
             }
             
             _dac_ctx = _controller_ioctl_param->temp.dac_ctx; 
             if(_dac_ctx != &_controller_ctx->dac_ctx[index]) {
                 printf("dac_controller_ioctl, incorrect ioctl parameters\n");
                 return -1;
             }

             dac_ioctl_param.cmd = DAC_IOCTL_CMD_WRITE_AND_UPDATE_CHANNEL;
             dac_ioctl_param.write_and_update_channel_param.temp._signature = _dac_ctx->signature;
             dac_ioctl_param.write_and_update_channel_param.temp._nb_terminals = _dac_ctx->nb_terminals;
             dac_ioctl_param.write_and_update_channel_param.temp._analog_outputs = _dac_ctx->analog_outputs;
             dac_ioctl_param.write_and_update_channel_param.temp.spi_controller_ctx = _dac_ctx->spi_controller_ctx;
             dac_ioctl_param.write_and_update_channel_param.temp.spi_controller_ioctl_param = _dac_ctx->spi_controller_ioctl_param;
             dac_ioctl_param.write_and_update_channel_param.in.voltage = _controller_ioctl_param->write_and_update_channel_param.in.voltage;
             dac_ioctl_param.write_and_update_channel_param.in._terminal = _controller_ioctl_param->write_and_update_channel_param.in.terminal;
             ret = _dac_ops->ioctl(&dac_ioctl_param);
             if(ret < 0) {
                 printf("dac_controller_ioctl->io_expander_ops->ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        default:
             printf("dac_controller_ioctl, unknown cmd %d\n", _cmd);
             return -1;
        break;
    }

    
    return ret;
}


#if 0

int main(void) 
{
    int ret = 0, cmd;
    logging_ctx log_ctx;
    spi_controller_context spi_controller_ctx;
    register_spi_bus_parameters spi_bus_param_bus0_node6;
    spi_controller_ioctl_parameters spi_controller_ioctl_param;
    dac_controller_context dac_controller_ctx;
    register_dac_parameters register_dac_param;
    unregister_dac_parameters unregister_dac_param;
    dac_controller_ioctl_parameters dac_controller_ioctl_param;

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
    ret = spi_controller_ctx.ops.register_context(&spi_controller_ctx, &spi_bus_param_bus0_node6);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX;
    strcpy(spi_controller_ioctl_param.acquire_spi_ctx_param.in.signature, "/dev/spidev0.6");
    ret = spi_controller_ctx.ops.ioctl(&spi_controller_ctx, cmd, &spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->acquire_spi_context fail\n");
        return -1;
    }
    
    spi_controller_ctx.ops.list_resources(&spi_controller_ctx);
#if 0
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
#endif
    ret = dac_controller_init(&dac_controller_ctx, &log_ctx);
    if(ret < 0) {
        printf("dac_controller_init fail\n");
        return -1;
    }

    strcpy(register_dac_param.in.signature, "U2");
    register_dac_param.in.bus_type = DAC_BUS_TYPE_SPI;
    register_dac_param.in.nb_terminals = 8;
    register_dac_param.in.spi_controller_ctx = &spi_controller_ctx;
    register_dac_param.in.spi_controller_ioctl_param = &spi_controller_ioctl_param;
    strcpy(register_dac_param.in_out.analog_outputs[0].terminal,  "VI1");
    strcpy(register_dac_param.in_out.analog_outputs[1].terminal,  "VI2");
    strcpy(register_dac_param.in_out.analog_outputs[2].terminal,  "VI3");
    strcpy(register_dac_param.in_out.analog_outputs[3].terminal,  "VI4");
    strcpy(register_dac_param.in_out.analog_outputs[4].terminal,  "VU1");
    strcpy(register_dac_param.in_out.analog_outputs[5].terminal,  "VU2");
    strcpy(register_dac_param.in_out.analog_outputs[6].terminal,  "VU3");
    strcpy(register_dac_param.in_out.analog_outputs[7].terminal,  "VU4");
    ret = dac_controller_ctx.ops.register_context(&dac_controller_ctx, &register_dac_param);
    if(ret < 0) {
        printf("dac_controller_register_dac fail\n");
        return -1;
    }

    //dac_controller_list_resources(&dac_controller_ctx);

    cmd = DAC_CONTROLLER_IOCTL_CMD_ACQUIRE_DAC_CONTEX;
    strcpy(dac_controller_ioctl_param.acquire_dac_ctx_param.in.signature, "U2");
    ret = dac_controller_ctx.ops.ioctl(&dac_controller_ctx, cmd, &dac_controller_ioctl_param);
    if(ret < 0) {
        printf("dac_controller_ioctl fail\n");
        return -1;
    }
   
    cmd = DAC_CONTROLLER_IOCTL_CMD_WRITE_AND_UPDATE_CHANNEL;
    strcpy(dac_controller_ioctl_param.write_and_update_channel_param.in.terminal, "VU1");
    dac_controller_ioctl_param.write_and_update_channel_param.in.voltage = 5.0;
    ret = dac_controller_ctx.ops.ioctl(&dac_controller_ctx, cmd, &dac_controller_ioctl_param);
    if(ret < 0) {
        printf("write and update channel fail\n");
        return -1;
    }

    dac_controller_ctx.ops.deinit(&dac_controller_ctx);

    dac_controller_ctx.ops.list_resources(&dac_controller_ctx);

    spi_controller_ctx.ops.deinit(&spi_controller_ctx);

    return ret;
}

#endif
