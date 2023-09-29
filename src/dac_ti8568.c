/**
* @file     dac_ti8568.c
* @brief    dac_ti8568.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     13/11/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "log_controller.h"
#include "dac_controller.h"

#include "debug.h"

#include "spi_controller.h"

int dac_ti8568_init(register_dac_parameters *dac_param)
{
    int ret = 0, cmd;
    uint16_t data_bits; 
    spi_controller_context *_controller_ctx = NULL;
    register_dac_parameters *_dac_param = dac_param;
    uint8_t prefix_bits, control_bits, address_bits, feature_bits;
    spi_controller_ioctl_parameters *_controller_ioctl_param = NULL;
    
    _controller_ctx = _dac_param->in.spi_controller_ctx;
    _controller_ioctl_param = (spi_controller_ioctl_parameters *)_dac_param->in.spi_controller_ioctl_param;

    // external reference voltage
    prefix_bits = 0;
    control_bits = 0x09;
    address_bits = 0;
    data_bits = 0xc0;
    feature_bits = 0;
    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    _controller_ioctl_param->transfer_message_param.in.tx_buf[0] = (prefix_bits << 4) | control_bits;
    _controller_ioctl_param->transfer_message_param.in.tx_buf[1] = (address_bits << 4) | (data_bits >> 12);
    _controller_ioctl_param->transfer_message_param.in.tx_buf[2] =  data_bits >> 4;
    _controller_ioctl_param->transfer_message_param.in.tx_buf[3] = (data_bits << 4) | feature_bits;
    _controller_ioctl_param->transfer_message_param.in.tx_len = 4;
    
    ret = spi_controller_ioctl(_controller_ctx, cmd, _controller_ioctl_param);
    if(ret < 0) {
        printf("dac_ti8568_init->spi_controller_ioctl transfer fail\n\n");
        return -1;
    }
    
    return ret;
}

int dac_ti8568_ioctl(dac_ioctl_parameters *dac_ioctl_param)
{
    int ret = 0, cmd, _cmd;
    spi_controller_context *_controller_ctx = NULL;
    dac_ioctl_parameters *_dac_ioctl_param = dac_ioctl_param;
    spi_controller_ioctl_parameters *_controller_ioctl_param = NULL;
    uint8_t prefix_bits, control_bits, address_bits, feature_bits;
    uint16_t data_bits;
    
    _cmd = _dac_ioctl_param->cmd;

    switch(_cmd) {

        case DAC_IOCTL_CMD_WRITE_AND_UPDATE_CHANNEL:
             data_bits = _dac_ioctl_param->write_and_update_channel_param.temp.pcm_code;
             printf("data_bits 0x%x\n", data_bits);
             prefix_bits = 0;
             control_bits = 0x03;
             address_bits = _dac_ioctl_param->write_and_update_channel_param.temp._address_bits;
             feature_bits = 0;
             _controller_ctx = _dac_ioctl_param->write_and_update_channel_param.temp.spi_controller_ctx;
             _controller_ioctl_param = _dac_ioctl_param->write_and_update_channel_param.temp.spi_controller_ioctl_param;
             
             cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
             _controller_ioctl_param->transfer_message_param.in.tx_buf[0] = (prefix_bits << 4) | control_bits;
             _controller_ioctl_param->transfer_message_param.in.tx_buf[1] = (address_bits << 4) | (data_bits >> 12);
             _controller_ioctl_param->transfer_message_param.in.tx_buf[2] =  data_bits >> 4;
             _controller_ioctl_param->transfer_message_param.in.tx_buf[3] = (data_bits << 4) | feature_bits;
             _controller_ioctl_param->transfer_message_param.in.tx_len = 4;
             ret = spi_controller_ioctl(_controller_ctx, cmd, _controller_ioctl_param);
             if(ret < 0) {
                 printf("dac_ti8568_init->spi_controller_ioctl transfer fail\n\n");
                 return -1;
             }
             return 0;
        break;

        default:
             printf("dac_ti8568_ioctl fail, un-known cmd %d\n", _cmd);
             return -1;
        break;
    }

    return ret;
}

int dac_ti8568_deinit(unregister_dac_parameters *dac_param)
{
    int ret = 0;

    return ret;
}

