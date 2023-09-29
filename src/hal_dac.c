/**
* @file     hal_da.c
* @brief    hal_da.c
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

#include "dac_ti8568.h"

#include "debug.h"

int hal_dac_init(register_dac_parameters *dac_param)
{
    int ret = 0, count, i, _nb_terminals;
    char *_dac_signature = NULL, *_terminal_signature;
    void *spi_controller_ctx = NULL;
    void *spi_controller_ioctl_param = NULL;
    register_dac_parameters *_dac_param = dac_param;
    analog_output *analog_outputs = NULL;

    if(!_dac_param) {
        printf("hal_dac_init fail, incorrect parameters\n");
        return -1;
    }

    spi_controller_ctx = _dac_param->in.spi_controller_ctx;
    if(!spi_controller_ctx) {
        printf("hal_dac_init fail, incorrect controller_ctx\n");
        return -1;
    }

    spi_controller_ioctl_param = _dac_param->in.spi_controller_ioctl_param;
    if(!spi_controller_ioctl_param) {
        printf("hal_dac_init fail, incorrect controller_ioctl_param\n");
        return -1;
    }

    _nb_terminals = _dac_param->in.nb_terminals;
    _dac_signature = _dac_param->in.signature;
    analog_outputs = _dac_param->in_out.analog_outputs;

    if(!strcmp(_dac_signature, DAC1_DEVICE_SIGNATURE)) {
        for(i=0, count=0; i<_nb_terminals; i++) {
            _terminal_signature = analog_outputs[i].terminal;
            if(!strcmp(_terminal_signature, DAC1_VOUTA_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC1_VOUTA_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC1_VOUTB_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC1_VOUTB_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC1_VOUTC_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC1_VOUTC_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC1_VOUTD_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC1_VOUTD_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC1_VOUTE_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC1_VOUTE_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC1_VOUTF_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC1_VOUTF_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC1_VOUTG_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC1_VOUTG_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC1_VOUTH_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC1_VOUTH_ADDRESS_BITS;
                count++;
            } else {
                printf("hal_dac_init fail, dac_signature: %s, unsupport terminal: %s\n", _dac_signature, _terminal_signature);
                return -1;
            }
        }
    } else if(!strcmp(_dac_signature, DAC2_DEVICE_SIGNATURE)) {
            for(i=0, count=0; i<_nb_terminals; i++) {
            _terminal_signature = analog_outputs[i].terminal;
            if(!strcmp(_terminal_signature, DAC2_VOUTA_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC2_VOUTA_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC2_VOUTB_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC2_VOUTB_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC2_VOUTC_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC2_VOUTC_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC2_VOUTD_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC2_VOUTD_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC2_VOUTE_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC2_VOUTE_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC2_VOUTF_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC2_VOUTF_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC2_VOUTG_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC2_VOUTG_ADDRESS_BITS;
                count++;
            } else if(!strcmp(_terminal_signature, DAC2_VOUTH_TERMINAL_SIGNATURE)) {
                _dac_param->in_out.analog_outputs[i].config_data1 = DAC2_VOUTH_ADDRESS_BITS;
                count++;
            } else {
                printf("hal_dac_init fail, dac_signature: %s, unsupport terminal: %s\n", _dac_signature, _terminal_signature);
                return -1;
            }
        }
    } else {
        printf("dac_ti8568_init fail, unknown dac_signature: %s\n", _dac_signature);
        return -1;
    }

    if(count != _nb_terminals) {
            printf("hal_dac_init fail, count != _nb_terminals\n");
            return -1;
    }

    ret = dac_ti8568_init(dac_param);
    if(ret < 0) {
        printf("hal_dac_init->dac_ti8568_init fail\n");
        return -1;
    }

    return ret;
}

int hal_dac_ioctl(dac_ioctl_parameters *dac_ioctl_param)
{
    int ret = 0, _cmd, i, _nb_terminals, find;
    analog_output *_analog_outputs;
    uint16_t pcm_code;
    double _voltage = 0.0, vref;
    char *_terminal = NULL, *_dac_signature = NULL, *_terminal_in;
    dac_ioctl_parameters *_dac_ioctl_param = dac_ioctl_param;
    
    if(!_dac_ioctl_param) {
        printf("hal_dac_ioctl fail, incorrect parameters\n");
        return -1;
    }

    _cmd = _dac_ioctl_param->cmd;

    switch(_cmd) {

        case DAC_IOCTL_CMD_WRITE_AND_UPDATE_CHANNEL:
             _voltage = _dac_ioctl_param->write_and_update_channel_param.in.voltage;
             if(_voltage<0 || _voltage>5) {
                 printf("hal_dac_ioctl->write_and_update_channel fail, incorrect voltage %f\n", _voltage);
                 return -1;
             }

             _terminal_in = _dac_ioctl_param->write_and_update_channel_param.in._terminal;
             _analog_outputs = _dac_ioctl_param->write_and_update_channel_param.temp._analog_outputs;
             _dac_signature = _dac_ioctl_param->write_and_update_channel_param.temp._signature;
             _nb_terminals = _dac_ioctl_param->write_and_update_channel_param.temp._nb_terminals;
             
             for(i=0, find=0; i<_nb_terminals; i++) {
                 _terminal = _analog_outputs[i].terminal;
                 if(!strcmp(_terminal_in, _terminal)) {
                     _dac_ioctl_param->write_and_update_channel_param.temp._address_bits = _analog_outputs[i].config_data1; /* address bits */
                     find = 1;
                 }
             }

             if(!find) {
                 printf("hal_dac_ioctl->write_and_update_channel fail, un-known channel %s\n", _terminal_in);
                 return -1;
             }

             // determine the Vref
             if(!strcmp(_dac_signature, DAC1_DEVICE_SIGNATURE)) {
                 vref = DAC1_EXTERNAL_VREF;
             } else if(!strcmp(_dac_signature, DAC2_DEVICE_SIGNATURE)){
                 vref = DAC2_EXTERNAL_VREF;
             }

             // calculate PCM Code
             pcm_code = (_voltage/vref)*0xFFFF;
             printf("pcm_code = 0x%x\n", pcm_code);
             _dac_ioctl_param->write_and_update_channel_param.out.pcm_code = pcm_code;
             _dac_ioctl_param->write_and_update_channel_param.temp.pcm_code = pcm_code;
             ret = dac_ti8568_ioctl(dac_ioctl_param);
                 if(ret < 0) {
                 printf("hal_dac_ioctl->dac_ti8568_ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        default:
             printf("hal_dac_ioctl fail, un-support cmd %d\n", _cmd);
             return -1;
        break;
    }


    

    return ret;
}


int hal_dac_deinit(unregister_dac_parameters *dac_param)
{
    int ret = 0;

    ret = dac_ti8568_deinit(dac_param);
    if(ret < 0) {
        printf("hal_dac_deinit->dac_ti8568_deinit fail\n");
        return -1;
    }

    return 0;
}


