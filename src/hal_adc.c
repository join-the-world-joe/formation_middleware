/**
* @file     hal_adc.c
* @brief    hal_adc.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     15/11/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "log_controller.h"
#include "adc_controller.h"
#include "hal_adc.h"
//#include "spi_controller.h"

#include "debug.h"

#include "adc_ti1258.h"

int hal_adc_init(register_adc_parameters *adc_param)
{
    int ret = 0, count, i, _nb_terminals;
    char *_adc_signature = NULL, *_terminal_signature;
    void *spi_controller_ctx = NULL;
    void *spi_controller_ioctl_param = NULL;
    register_adc_parameters *_adc_param = adc_param;
    analog_input *_analog_inputs = NULL;

    if(!_adc_param) {
        printf("hal_adc_init fail, incorrect parameters\n");
        return -1;
    }

    spi_controller_ctx = _adc_param->in.spi_controller_ctx;
    if(!spi_controller_ctx) {
        printf("hal_adc_init fail, incorrect controller_ctx\n");
        return -1;
    }

    spi_controller_ioctl_param = _adc_param->in.spi_controller_ioctl_param;
    if(!spi_controller_ioctl_param) {
        printf("hal_adc_init fail, incorrect controller_ioctl_param\n");
        return -1;
    }
    
    _nb_terminals = _adc_param->in.nb_terminals;
    _adc_signature = _adc_param->in.signature;
    _analog_inputs = _adc_param->in_out.analog_inputs;

    if(!strcmp(_adc_signature, ADC1_DEVICE_SIGNATURE)) {
        for(i=0, count=0; i<_nb_terminals; i++) {
            _terminal_signature = _analog_inputs[i].terminal;
            if(!strcmp(_terminal_signature, "BV1")) {
                _analog_inputs[i].config_data1 = TERMINAL_BV1_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_BV1_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "CV1")) {
                _analog_inputs[i].config_data1 = TERMINAL_CV1_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_CV1_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "BV2")) {
                _analog_inputs[i].config_data1 = TERMINAL_BV2_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_BV2_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "CV2")) {
                _analog_inputs[i].config_data1 = TERMINAL_CV2_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_CV2_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "BV3")) {
                _analog_inputs[i].config_data1 = TERMINAL_BV3_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_BV3_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "CV3")) {
                _analog_inputs[i].config_data1 = TERMINAL_CV3_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_CV3_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "BV4")) {
                _analog_inputs[i].config_data1 = TERMINAL_BV4_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_BV4_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "CV4")) {
                _analog_inputs[i].config_data1 = TERMINAL_CV4_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_CV4_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "BV5")) {
                _analog_inputs[i].config_data1 = TERMINAL_BV5_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_BV5_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "CV5")) {
                _analog_inputs[i].config_data1 = TERMINAL_CV5_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_CV5_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "BV6")) {
                _analog_inputs[i].config_data1 = TERMINAL_BV6_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_BV6_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "CV6")) {
                _analog_inputs[i].config_data1 = TERMINAL_CV6_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_CV6_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "BV7")) {
                _analog_inputs[i].config_data1 = TERMINAL_BV7_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_BV7_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "CV7")) {
                _analog_inputs[i].config_data1 = TERMINAL_CV7_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_CV7_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "BV8")) {
                _analog_inputs[i].config_data1 = TERMINAL_BV8_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_BV8_CONFIG_DATA2;
                count++;
            } else if(!strcmp(_terminal_signature, "CV8")) {
                _analog_inputs[i].config_data1 = TERMINAL_CV8_CONFIG_DATA1;
                _analog_inputs[i].config_data2 = TERMINAL_CV8_CONFIG_DATA2;
                count++;
            } else {
                printf("unknow terminal %s\n", _terminal_signature);
                return -1;
            }
        }

    } else {
        printf("hal_adc_init fail, un-support adc device: %s\n", _adc_signature);
        return -1;
    }

    if(count != _nb_terminals) {
        printf("hal_adc_init fail, count != nb_terminals\n");
        return -1;
    }

    ret = adc_ti1258_init(_adc_param);
    if(ret < 0) {
        printf("hal_adc_init->adc_ti1258_init fail\n");
        return -1;
    }

    return ret;
}

int hal_adc_ioctl(adc_ioctl_parameters *adc_ioctl_param)
{
    char *_terminal = NULL, *_signature = NULL;
    int ret = 0, cmd, _cmd, _nb_terminals, _bus_type, i, same, find;
    analog_input *_analog_inputs = NULL, *_analog_input = NULL;
    void *spi_controller_ctx = NULL;
    adc_ioctl_parameters *_adc_ioctl_param = adc_ioctl_param;
    void *controller_ioctl_param = NULL;
    void *_gpio_controller_ctx = NULL;
    void *_gpio_controller_ioctl_param_drdy = NULL;
    int *_nb_samples = NULL;
    adc_sample *_samples = NULL;

    if(!_adc_ioctl_param) {
        printf("adc_ti1258_ioctl fail, incorrect parameters\n");
        return -1;
    }

    _cmd = _adc_ioctl_param->cmd;

    switch(_cmd) {

        case ADC_IOCTL_CMD_PULSE_CONVERSION:
             
             spi_controller_ctx = _adc_ioctl_param->pulse_conversion_param.temp._spi_controller_ctx;
             if(!spi_controller_ctx) {
                 printf("hal_adc_ioctl->pulse_conversion fail, incorrect parameters\n");
                 return -1;
             }

             controller_ioctl_param = _adc_ioctl_param->pulse_conversion_param.temp._spi_controller_ioctl_param;
             if(!controller_ioctl_param) {
                 printf("hal_adc_ioctl->pulse_conversion fail, incorrect parameters\n");
                 return -1;
             }

             _signature = _adc_ioctl_param->pulse_conversion_param.temp._signature;
             if(strlen(_signature) > MAX_LENGTH_OF_ADC_SIGNATURE) {
                 printf("hal_adc_ioctl fail, Length of terminal exceeds MAX_LENGTH_OF_ADC_TERMINAL_SIGNATURE\n");
                 return -1;
             }
        
             _nb_terminals = _adc_ioctl_param->pulse_conversion_param.temp._nb_terminals;
             if(_nb_terminals > MAX_NB_OF_ADC_ANALOG_INPUT_TERMINALS) {
                 printf("hal_adc_ioctl->pulse_conversion fail, incorrect parameters\n");
                 return -1;
             }

             _bus_type = _adc_ioctl_param->pulse_conversion_param.temp._bus_type;
             if(_bus_type != ADC_BUS_TYPE_SPI) {
                 printf("hal_adc_ioctl->pulse_conversion fail, incorrect parameters\n");
                 return -1;
             }

             _analog_inputs = _adc_ioctl_param->pulse_conversion_param.temp._analog_inputs;
             if(!_analog_inputs) {
                 printf("hal_adc_ioctl->pulse_conversion fail, incorrect parameters\n");
                 return -1;
             }

             _terminal = _adc_ioctl_param->pulse_conversion_param.in._terminal;
             if(strlen(_terminal) > MAX_LENGTH_OF_ADC_TERMINAL_SIGNATURE) {
                 printf("hal_adc_ioctl fail, Length of terminal exceeds MAX_LENGTH_OF_ADC_TERMINAL_SIGNATURE\n");
                 return -1;
             }

             for(i=0, find=0; i<_nb_terminals; i++) {
                 same = !strcmp(_terminal, _analog_inputs[i].terminal);
                 //printf("check if terminal == %s\n", _analog_inputs[i].terminal);
                 if(same) {
                     //_analog_input = &_analog_inputs[i];
                     _adc_ioctl_param->pulse_conversion_param.temp._analog_input = &_analog_inputs[i];
                     find = 1;
                     break;
                 }
             }

             if(!find) {
                 printf("hal_adc_ioctl fail, \n");
                 return -1;
             }
             
             ret = adc_ti1258_ioctl(_adc_ioctl_param);
             if(ret < 0) {
                 printf("hal_adc_ioctl->hal_adc_ioctl\n");
                 return -1;
             }

             return 0;
        break;

        case ADC_IOCTL_CMD_STM32_CONVERSION:

             spi_controller_ctx = _adc_ioctl_param->stm32_conversion_param.temp._spi_controller_ctx;
             if(!spi_controller_ctx) {
                 printf("hal_adc_ioctl->stm32_conversion_param fail, incorrect parameters 1\n");
                 return -1;
             }

             controller_ioctl_param = _adc_ioctl_param->stm32_conversion_param.temp._spi_controller_ioctl_param;
             if(!controller_ioctl_param) {
                 printf("hal_adc_ioctl->stm32_conversion_param fail, incorrect parameters 2\n");
                 return -1;
             }

             _signature = _adc_ioctl_param->stm32_conversion_param.temp._signature;
             if(strlen(_signature) > MAX_LENGTH_OF_ADC_SIGNATURE) {
                 printf("hal_adc_ioctl fail, Length of terminal exceeds MAX_LENGTH_OF_ADC_TERMINAL_SIGNATURE 3\n");
                 return -1;
             }

             _nb_terminals = _adc_ioctl_param->stm32_conversion_param.temp._nb_terminals;
             if(_nb_terminals > MAX_NB_OF_ADC_ANALOG_INPUT_TERMINALS) {
                 printf("hal_adc_ioctl->stm32_conversion_param fail, incorrect parameters 4\n");
                 return -1;
             }

             _bus_type = _adc_ioctl_param->stm32_conversion_param.temp._bus_type;
             if(_bus_type != ADC_BUS_TYPE_SPI) {
                 printf("hal_adc_ioctl->stm32_conversion_param fail, incorrect parameters 5\n");
                 return -1;
             }
#if 0
             _analog_inputs = _adc_ioctl_param->stm32_conversion_param.temp._analog_inputs;
             if(!_analog_inputs) {
                 printf("hal_adc_ioctl->stm32_conversion_param fail, incorrect parameters 6\n");
                 return -1;
             }
#endif
             _gpio_controller_ctx = _adc_ioctl_param->stm32_conversion_param.temp._gpio_controller_ctx;
             if(!_gpio_controller_ctx) {
                 printf("hal_adc_ioctl->stm32_conversion_param fail, incorrect parameters 7\n");
                 return -1;
             }

             _gpio_controller_ioctl_param_drdy = _adc_ioctl_param->stm32_conversion_param.temp._gpio_controller_ioctl_param_drdy;
             if(!_gpio_controller_ioctl_param_drdy) {
                 printf("hal_adc_ioctl->stm32_conversion_param fail, incorrect parameters 8\n");
                 return -1;
             }

             _nb_samples = _adc_ioctl_param->stm32_conversion_param.out._nb_samples;
             if(!_nb_samples) {
                 printf("hal_adc_ioctl fail, incorrect parameters, _nb_samples NULL\n");
                 return -1;
             }  

             _samples = _adc_ioctl_param->stm32_conversion_param.out._samples;
             if(!_samples) {
                 printf("hal_adc_ioctl fail, incorrect parameters, _samples NULL\n");
                 return -1;
             }  

             //printf("gpio_controller %p, ad_start %p\n", _gpio_controller_ctx, _gpio_controller_ioctl_param_ad_start);

             ret = adc_ti1258_ioctl(_adc_ioctl_param);
             if(ret < 0) {
                 printf("hal_stm32_ad_ioctl->hal_stm32_ad_ioctl\n");
                 return -1;
             }
             return 0;
        break;

        default:
             printf("hal_adc_ioctl fail, un-known cmd %d\n", _cmd);
             return -1;
        break;
    }

    return ret;
}


int hal_adc_deinit(unregister_adc_parameters *adc_param)
{
    int ret;

    printf("hal_adc_deinit\n");

    return ret;
}

