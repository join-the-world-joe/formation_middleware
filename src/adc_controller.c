/**
* @file     adc_controller.c
* @brief    adc_controller.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     14/11/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "log_controller.h"
#include "adc_controller.h"
#include "hal_adc.h"

#include "debug.h"

#ifdef DATA_FLOW_DEBUG
#include "spi_controller.h"
#endif

#include "spi_controller.h"

#include "gpio_controller.h"

void adc_controller_list_resources(adc_controller_context *adc_controller_ctx)
{
    int _nb_adc_devices, index, _nb_terminals, j;
    adc_context *_adc_ctx = NULL;
    adc_controller_context *_controller_ctx = adc_controller_ctx;

    if(!_controller_ctx) {
        printf("adc_controller_list_resources fail, incorrect parameters\n");
        return ;
    }

    _adc_ctx = _controller_ctx->adc_ctx;
    _nb_adc_devices = _controller_ctx->nb_adc_devices;
    
    printf("***********************************ADC Resources***************************************\n");
    for(index=0; index<_nb_adc_devices; index++) {
        if(_adc_ctx[index].in_use) {
     
            printf("Index: %d\n", index);
            printf("Bus Type: %s\n", _adc_ctx[index].bus_type == ADC_BUS_TYPE_SPI?"SPI":"I2C");
            printf("ADC Signature: %s\n", _adc_ctx[index].signature);
            printf("---------------------------------------------------------------------------------------\n");
            for(j=0; j<_adc_ctx[index].nb_terminals; j++) {
                printf("Terminal: %s\n", _adc_ctx[index].analog_inputs[j].terminal);
                printf("Channel Bit Mask 0x%x\n", _adc_ctx[index].analog_inputs[j].config_data2);
                printf("Single-ended Input Select Register: 0x%x\n", _adc_ctx[index].analog_inputs[j].config_data1);
            }
            printf("----------------------------------------------------------------------------------------\n");
        }
    }
    printf("****************************************************************************************\n");
}

int adc_controller_init(adc_controller_context *adc_controller_ctx, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param)
{
    int ret = 0, _nb_adc_devices = NB_ADC_DEVICES, i;
    adc_context *_ad_ctx = NULL;
    adc_controller_context *_controller_ctx = adc_controller_ctx;
    log_controller_context *_log_controller_ctx = log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param = log_controller_ioctl_param; 

    if(!_controller_ctx || !_log_controller_ctx || !_log_controller_ioctl_param) {
        printf("adc_controller_init fail, incorrect parameters\n");
        return -1;
    }

    _controller_ctx->count = 0;
    _controller_ctx->_log_controller_ctx = _log_controller_ctx;
    _controller_ctx->_log_controller_ioctl_param = _log_controller_ioctl_param;
    _controller_ctx->nb_adc_devices = _nb_adc_devices;

    _ad_ctx = _controller_ctx->adc_ctx;
    
    // we'd better initialize all the important variables here    
    for(i=0; i<_nb_adc_devices; i++) {
        _ad_ctx[i].index = i;
        _ad_ctx[i].in_use = 0;
        _ad_ctx[i].spi_controller_ctx = NULL;
        _ad_ctx[i].spi_controller_ioctl_param = NULL;
        _ad_ctx[i].bus_type = ADC_BUS_TYPE_UNKNOWN;
        memset(_ad_ctx[i].signature, 0, MAX_LENGTH_OF_ADC_SIGNATURE);
    }
    
    _controller_ctx->ops.init = adc_controller_init;
    _controller_ctx->ops.register_context = adc_controller_register_adc;
    _controller_ctx->ops.unregister_context = adc_controller_unregister_adc;
    _controller_ctx->ops.list_resources = adc_controller_list_resources;
    _controller_ctx->ops.ioctl = adc_controller_ioctl;
    _controller_ctx->ops.deinit = adc_controller_deinit;
    
    _controller_ctx->ad_ops.init = hal_adc_init;
    _controller_ctx->ad_ops.ioctl = hal_adc_ioctl;
    _controller_ctx->ad_ops.deinit = hal_adc_deinit;

    return ret;
}

void adc_controller_deinit(adc_controller_context *adc_controller_ctx)
{
    int ret = 0, index;
    int _nb_adc_devices;
    adc_context *_adc_ctx = NULL;
    adc_ops *_ad_ops = NULL;
    adc_controller_context *_controller_ctx = adc_controller_ctx;
    unregister_adc_parameters adc_param;

    if(!_controller_ctx) {
        printf("adc_controller_deinit fail, incorrect parameters\n");
        return ;
    }

    _ad_ops = &_controller_ctx->ad_ops;
    _adc_ctx = _controller_ctx->adc_ctx; 
    _nb_adc_devices = _controller_ctx->nb_adc_devices;

    for(index=0; index<_nb_adc_devices; index++) {
        if(_adc_ctx[index].in_use) {
            //printf("spi_controller_deinit: index %d is in use\n", index);
            memset(adc_param.in.signature, 0, MAX_LENGTH_OF_ADC_SIGNATURE);
            strcpy(adc_param.in.signature, _adc_ctx[index].signature);
            ret = _controller_ctx->ops.unregister_context(_controller_ctx, &adc_param);
            if(ret < 0) {
                printf("adc_controller_deinit->unregister_adc fail, device: %s\n", _adc_ctx[index].signature);
                return ;
            }
        }
    }

    return ;
}

int adc_controller_register_adc(adc_controller_context *adc_controller_ctx, register_adc_parameters* adc_param)
{
    int ret = 0, i, done, _bus_type, j, _nb_terminals;
    char *_terminal_signature = NULL;
    uint32_t _nb_adc_devices;
    char *_adc_signatrue = NULL;
    adc_context *_adc_ctx = NULL;
    adc_ops *_adc_ops = NULL;
    register_adc_parameters *_adc_param = adc_param;
    adc_controller_context *_controller_ctx = adc_controller_ctx;

    if(!_controller_ctx || !_adc_param) {
        printf("adc_controller_register_adc fail, incorrect parameters\n");
        return -1;
    }

    _nb_adc_devices = _controller_ctx->nb_adc_devices;
    _adc_ctx = _controller_ctx->adc_ctx;
    _adc_ops = &_controller_ctx->ad_ops;
    _adc_signatrue = _adc_param->in.signature;
    _bus_type = _adc_param->in.bus_type;

    if(_bus_type != ADC_BUS_TYPE_SPI) {
        printf("we only support spi kind ADC\n");
        return -1;
    }

    if(_controller_ctx->count + 1 > _nb_adc_devices) {
        printf("dac context in controller is full, there might be something wrong\n");
        return -1;
    }

    if(_adc_param->in.nb_terminals > MAX_NB_OF_ADC_ANALOG_INPUT_TERMINALS) {
        printf("adc_controller_register_dac fail, exceeding the max. amount of AD devices");
        return -1;
    }

    ret = _adc_ops->init(_adc_param);
    if(ret < 0) {
        printf("adc_controller_register_adc->ad_ops->init fail\n");
        return -1;
    }

    // first, find a free context 
    for(i=0, done=0; i<_nb_adc_devices; i++) {

        if(_adc_ctx[i].in_use == 0) {

            _nb_terminals = _adc_param->in.nb_terminals;
           
            // record input parameters 
            _adc_ctx[i].bus_type = _bus_type;
            _adc_ctx[i].nb_terminals = _nb_terminals;
            if(_bus_type == ADC_BUS_TYPE_SPI) {
                _adc_ctx[i].spi_controller_ioctl_param = _adc_param->in.spi_controller_ioctl_param;
                _adc_ctx[i].spi_controller_ctx = _adc_param->in.spi_controller_ctx;
                _adc_ctx[i].gpio_controller_ctx = _adc_param->in.gpio_controller_ctx;
                _adc_ctx[i].gpio_controller_ioctl_param_drdy = _adc_param->in.gpio_controller_ioctl_param_drdy;
            }
            strcpy(_adc_ctx[i].signature, _adc_signatrue);

            // record output parameters
            for(j=0; j<_nb_terminals; j++) {
                _terminal_signature = _adc_param->in_out.analog_inputs[j].terminal;  
                _adc_ctx[i].analog_inputs[j].config_data1 = _adc_param->in_out.analog_inputs[j].config_data1;
                _adc_ctx[i].analog_inputs[j].config_data2 = _adc_param->in_out.analog_inputs[j].config_data2;
                _adc_ctx[i].analog_inputs[j].config_data3 = _adc_param->in_out.analog_inputs[j].config_data3;
                strcpy(_adc_ctx[i].analog_inputs[j].terminal, _terminal_signature);
            }

            // change the status of this gpio_context
            _adc_ctx[i].in_use = 1;

            done = 1; // all the related contents are recoreded in gpio_context

            break;
        }
    }

    if(!done) {
        printf("adc_controller_register_adc fail, can't record ad device %s\n", _adc_signatrue);
        return -1;
    }

    _controller_ctx->count++; /* if done, increase the object counter */

    return ret;
}

int adc_controller_unregister_adc(adc_controller_context *adc_controller_ctx, unregister_adc_parameters* adc_param)
{
    int ret = 0, index, done, i, same, find;
    uint32_t _nb_adc_devices;
    char *_adc_signatrue = NULL;
    adc_context *_adc_ctx = NULL;
    adc_ops *_ad_ops = NULL;
    unregister_adc_parameters *_adc_param = adc_param;
    adc_controller_context *_controller_ctx = adc_controller_ctx;

    if(!_controller_ctx || !_adc_param) {
        printf("adc_controller_unregister_adc fail, incorrect parameters\n");
        return -1;
    }

    _nb_adc_devices = _controller_ctx->nb_adc_devices;
    _adc_ctx = _controller_ctx->adc_ctx;
    _ad_ops = &_controller_ctx->ad_ops;
    _adc_signatrue = _adc_param->in.signature;

    if((_controller_ctx->count - 1) < 0) {
        printf("adc context in controller is empty, there might be something wrong\n");
        return -1;
    }

    // check if we can find this signature in gpio_context
    for(index=0, find=0; index<_nb_adc_devices; index++) {

        same = !strcmp(_adc_ctx[index].signature, _adc_signatrue);
        if(same) {

            find = 1;

            break;
        }
    }

    if(!find) {
        printf("signature %s doesn't match any recorded signature in adc_context of adc_controller\n", _adc_signatrue);
        return -1;
    }
    
    ret = _ad_ops->deinit(_adc_param);
    if(ret < 0) {
        printf("adc_controller_unregister_adc->ad_ops->deinit fail\n");
        return -1;
    }

    _adc_ctx[index].in_use = 0;

    return ret;
}


int adc_controller_ioctl(adc_controller_context *adc_controller_ctx, uint16_t cmd, adc_controller_ioctl_parameters *adc_controller_ioctl_param)
{
    int ret = 0, same, done, index, _nb_adc_devices;
    char *_adc_signature = NULL;
    uint16_t _cmd = cmd;    
    adc_ops *_adc_ops = NULL;
    adc_context *_adc_ctx = NULL;
    adc_ioctl_parameters adc_ioctl_param;
    adc_controller_context *_controller_ctx = adc_controller_ctx;
    adc_controller_ioctl_parameters *_controller_ioctl_param = adc_controller_ioctl_param;
    
    if(!_controller_ctx || !_controller_ioctl_param || _cmd < ADC_CONTROLLER_IOCTL_CMD_NB_BEGIN || _cmd > ADC_CONTROLLER_IOCTL_CMD_NB_END) {
        printf("adc_controller_ioctl incorrect parameters\n");
        return -1;
    }

    _nb_adc_devices = _controller_ctx->nb_adc_devices;
    _adc_ops = &_controller_ctx->ad_ops;
    
    switch(_cmd) {

        case ADC_CONTROLLER_IOCTL_CMD_ACQUIRE_ADC_CONTEX:
             _adc_ctx = _controller_ctx->adc_ctx;
             _adc_signature = _controller_ioctl_param->acquire_adc_ctx_param.in.signature;         
             for(index=0, done=0; index<_nb_adc_devices; index++) {
                 same = !strcmp(_adc_signature, _adc_ctx[index].signature);
                 if(same) {
    
                    if(_adc_ctx[index].in_use) {

                        _controller_ioctl_param->temp.index = index;
    
                        _controller_ioctl_param->temp.adc_ctx = &_adc_ctx[index];   
                    
                        done = 1;
    
                        break;
                    }
                 }
             }
    
             if(done == 0) {
                 printf("adc_controller_ioctl->acquire_adc_context fail\n");
                 return -1;
             }

             return 0; // acquire adc context done!!!
        break;

        case ADC_CONTROLLER_IOCTL_CMD_PULSE_CONVERSION:
             index = _controller_ioctl_param->temp.index;
             if(index < 0 || index >= _nb_adc_devices) {
                 printf("adc_controller_ioctl, incorrect index = %d\n", index);
                 return -1;
             }
            
             _adc_ctx = _controller_ioctl_param->temp.adc_ctx; 
             if(_adc_ctx != &_controller_ctx->adc_ctx[index]) {
                 printf("adc_controller_ioctl, incorrect ioctl parameters\n");
                 return -1;
             }

             adc_ioctl_param.cmd = ADC_IOCTL_CMD_PULSE_CONVERSION;             
             adc_ioctl_param.pulse_conversion_param.temp._bus_type = _adc_ctx->bus_type;
             adc_ioctl_param.pulse_conversion_param.temp._signature = _adc_ctx->signature;
             adc_ioctl_param.pulse_conversion_param.temp._nb_terminals = _adc_ctx->nb_terminals;
             adc_ioctl_param.pulse_conversion_param.temp._analog_inputs = _adc_ctx->analog_inputs;
             adc_ioctl_param.pulse_conversion_param.temp._spi_controller_ctx = _adc_ctx->spi_controller_ctx;      
             adc_ioctl_param.pulse_conversion_param.out._voltage = &_controller_ioctl_param->pulse_conversion_param.out.voltage;
             adc_ioctl_param.pulse_conversion_param.temp._spi_controller_ioctl_param = _adc_ctx->spi_controller_ioctl_param;
             
             adc_ioctl_param.pulse_conversion_param.in._terminal = _controller_ioctl_param->pulse_conversion_param.in.terminal;
             ret = _adc_ops->ioctl(&adc_ioctl_param);
             if(ret < 0) {
                 printf("adc_controller_ioctl->adc_ops->ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        case ADC_CONTROLLER_IOCTL_CMD_STM32_CONVERSION:
             index = _controller_ioctl_param->temp.index;
             if(index < 0 || index >= _nb_adc_devices) {
                 printf("adc_controller_ioctl, incorrect index = %d\n", index);
                 return -1;
             }

             _adc_ctx = _controller_ioctl_param->temp.adc_ctx; 
             if(_adc_ctx != &_controller_ctx->adc_ctx[index]) {
                 printf("adc_controller_ioctl, incorrect ioctl parameters\n");
                 return -1;
             }

             adc_ioctl_param.cmd = ADC_IOCTL_CMD_STM32_CONVERSION;
             adc_ioctl_param.stm32_conversion_param.temp._bus_type = _adc_ctx->bus_type;
             adc_ioctl_param.stm32_conversion_param.temp._signature = _adc_ctx->signature;
             adc_ioctl_param.stm32_conversion_param.temp._nb_terminals = _adc_ctx->nb_terminals;
#if 0
             adc_ioctl_param.stm32_conversion_param.temp._analog_inputs = _adc_ctx->analog_inputs;
#endif
             adc_ioctl_param.stm32_conversion_param.out._nb_samples = &_controller_ioctl_param->stm32_conversion_param.out.nb_samples;
             adc_ioctl_param.stm32_conversion_param.out._samples = _controller_ioctl_param->stm32_conversion_param.out.samples;
             adc_ioctl_param.stm32_conversion_param.temp._spi_controller_ctx = _adc_ctx->spi_controller_ctx;      
             adc_ioctl_param.stm32_conversion_param.temp._spi_controller_ioctl_param = _adc_ctx->spi_controller_ioctl_param;
             adc_ioctl_param.stm32_conversion_param.temp._gpio_controller_ctx = _adc_ctx->gpio_controller_ctx;
             adc_ioctl_param.stm32_conversion_param.temp._gpio_controller_ioctl_param_drdy = _adc_ctx->gpio_controller_ioctl_param_drdy;
             ret = _adc_ops->ioctl(&adc_ioctl_param);
             if(ret < 0) {
                 printf("adc_controller_ioctl->adc_ops->ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        default:
             printf("adc_controller_ioctl, unknown cmd %d\n", _cmd);
             return -1;
        break;
    }

    
    return ret;
}

#if 0
int main(void)
{
    int ret = 0, cmd, count = 1000, i, nb_samples, channel_index;
    double voltage;
    logging_ctx log_ctx;
    spi_controller_context spi_controller_ctx;
    register_spi_bus_parameters spi_bus_param_bus0_node6;
    spi_controller_ioctl_parameters spi_controller_ioctl_param;
    adc_controller_context adc_controller_ctx;
    register_adc_parameters register_adc_param;
    //unregister_dac_parameters unregister_dac_param;
    adc_controller_ioctl_parameters adc_controller_ioctl_param;
    gpio_controller_context gpio_controller_ctx;
    gpio_controller_ioctl_parameters gpio_controller_ioctl_param_rdry;
    register_gpio_port_parameters gpio_port_ad_rdry;    

    ret = gpio_controller_init(&gpio_controller_ctx, &log_ctx);
    if(ret < 0) {
        printf("gpio_controller_init fail\n");
        return -1;
    }

    strcpy(gpio_port_ad_rdry.in.signature, "AD-DRDY");
    gpio_port_ad_rdry.in.direction = GPIO_DIRECTION_IN;
    ret = gpio_controller_ctx.ops.register_context(&gpio_controller_ctx, &gpio_port_ad_rdry);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_ad_rdry.in.signature);
        return -1;
    }

    strcpy(gpio_controller_ioctl_param_rdry.acquire_gpio_ctx_param.in.signature, "AD-DRDY");
    ret = gpio_controller_ctx.ops.ioctl(&gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_ACQUIRE_GPIO_CONTEX, &gpio_controller_ioctl_param_rdry);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to acquire gpio context\n");
        return -1;
    }

    ret = spi_controller_init(&spi_controller_ctx, &log_ctx);
    if(ret < 0) {
        printf("spi_controller_init fail\n");
        return -1;
    }

    strcpy(spi_bus_param_bus0_node6.in.signature, "/dev/spidev0.0");
    spi_bus_param_bus0_node6.in.mode = SPI_MODE_00;
    //spi_bus_param_bus0_node6.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node6.in.speed = SPI_SPEED_16MHZ;
    spi_bus_param_bus0_node6.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node6.in.bits_per_word = SPI_BPW_08BITS;
    ret = spi_controller_ctx.ops.register_context(&spi_controller_ctx, &spi_bus_param_bus0_node6);
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

    ret = adc_controller_init(&adc_controller_ctx, &log_ctx);
    if(ret < 0) {
        printf("adc_controller_init fail\n");
        return -1;
    }

    strcpy(register_adc_param.in.signature, "U1");
    register_adc_param.in.bus_type = ADC_BUS_TYPE_SPI;
    register_adc_param.in.spi_controller_ctx = &spi_controller_ctx;
    register_adc_param.in.spi_controller_ioctl_param = &spi_controller_ioctl_param;
    register_adc_param.in.gpio_controller_ctx = &gpio_controller_ctx;
    register_adc_param.in.gpio_controller_ioctl_param_drdy = &gpio_controller_ioctl_param_rdry;
    strcpy(register_adc_param.in_out.analog_inputs[0].terminal,  "BV1");
    strcpy(register_adc_param.in_out.analog_inputs[1].terminal,  "CV1");
    strcpy(register_adc_param.in_out.analog_inputs[2].terminal,  "BV2");
    strcpy(register_adc_param.in_out.analog_inputs[3].terminal,  "CV2");
    strcpy(register_adc_param.in_out.analog_inputs[4].terminal,  "BV3");
    strcpy(register_adc_param.in_out.analog_inputs[5].terminal,  "CV3");
    strcpy(register_adc_param.in_out.analog_inputs[6].terminal,  "BV4");
    strcpy(register_adc_param.in_out.analog_inputs[7].terminal,  "CV4");
    strcpy(register_adc_param.in_out.analog_inputs[8].terminal,  "BV5");
    strcpy(register_adc_param.in_out.analog_inputs[9].terminal,  "CV5");
    strcpy(register_adc_param.in_out.analog_inputs[10].terminal,  "BV6");
    strcpy(register_adc_param.in_out.analog_inputs[11].terminal,  "CV6");
    strcpy(register_adc_param.in_out.analog_inputs[12].terminal,  "BV7");
    strcpy(register_adc_param.in_out.analog_inputs[13].terminal,  "CV7");
    strcpy(register_adc_param.in_out.analog_inputs[14].terminal,  "BV8");
    strcpy(register_adc_param.in_out.analog_inputs[15].terminal,  "CV8");
    register_adc_param.in.nb_terminals = 16;
    ret = adc_controller_ctx.ops.register_context(&adc_controller_ctx, &register_adc_param);
    if(ret < 0) {
        printf("dac_controller_register_dac fail\n");
        return -1;
    }

    //adc_controller_list_resources(&adc_controller_ctx);

    cmd = ADC_CONTROLLER_IOCTL_CMD_ACQUIRE_ADC_CONTEX;
    strcpy(adc_controller_ioctl_param.acquire_adc_ctx_param.in.signature, "U1"); 
    ret = adc_controller_ctx.ops.ioctl(&adc_controller_ctx, cmd, &adc_controller_ioctl_param);
    if(ret < 0) {
        printf("adc_controller_ioctl fail\n");
        return -1;
    }

    //while(count--) {
    while(1) {
        cmd = ADC_CONTROLLER_IOCTL_CMD_STM32_CONVERSION;
        //strcpy(adc_controller_ioctl_param.pulse_conversion_param.in.terminal, "CV1");
        ret = adc_controller_ctx.ops.ioctl(&adc_controller_ctx, cmd, &adc_controller_ioctl_param);
        if(ret < 0) {
            printf("adc_controller_ioctl fail\n");
            //return -1;
        }

        nb_samples = adc_controller_ioctl_param.stm32_conversion_param.out.nb_samples;

        for(i=0; i<nb_samples; i++) {
            channel_index = adc_controller_ioctl_param.stm32_conversion_param.out.samples[i].index;
            voltage = adc_controller_ioctl_param.stm32_conversion_param.out.samples[i].voltage;
            
            //printf("channel_index %x voltage %f\n", channel_index, voltage);

        }

        //sleep(1);
    }

    return ret;
}
#endif
