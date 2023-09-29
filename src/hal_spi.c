/**
* @file     hal_spi.c
* @brief    hal_spi.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     09/11/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "log_controller.h"
#include "spi_controller.h"
#include "hal_spi.h"

#include "spi_raspberry_pi.h"

#include "debug.h"

/*
@param[in]      signature
                The signature of gpio port
@param[out]     dev_fd
                The file descriptor of gpio port
*/
int hal_spi_init(register_spi_bus_parameters *spi_bus_param)
{
    int ret = 0, length, spi_bus_number, virtual_dev_fd;
    uint8_t _mode;
    uint32_t _speed;
    uint32_t _cs_polarity;
    uint8_t _bits_per_word;
    char *_signature = NULL;
    register_spi_bus_parameters *_spi_bus_param = spi_bus_param;

    if(!_spi_bus_param) {
        printf("hal_spi_init incorrect parameters\n");
        return -1;
    }

    _signature = _spi_bus_param->in.signature;
    length = strlen(_signature);
    if(length == 0 || length > MAX_LENGTH_OF_SPI_BUS_SIGNATURE) {
        printf("hal_spi_init incorrect parameters\n");
        return -1;
    }

    _mode = _spi_bus_param->in.mode;
    if(_mode!=SPI_MODE_00 && _mode!=SPI_MODE_01 && _mode!=SPI_MODE_10 && _mode!=SPI_MODE_11) {
        printf("hal_spi_init incorrect mode %d\n", _mode);
        return -1;
    }

    _cs_polarity = _spi_bus_param->in.cs_polarity;
    if(_cs_polarity!=SPI_CS_POLARITY_HIGH && _cs_polarity!=SPI_CS_POLARITY_LOW) {
        printf("hal_spi_init incorrect cs_polarity %d\n", _cs_polarity);
        return -1;
    }

    _bits_per_word = _spi_bus_param->in.bits_per_word;
    if(_bits_per_word!=SPI_BPW_08BITS && _bits_per_word!=SPI_BPW_16BITS) {
        printf("hal_spi_init incorrect bits_per_word %d\n", _bits_per_word);
        return -1;
    }

#ifdef DATA_FLOW_DEBUG
    if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE0)) {
        virtual_dev_fd = SPI_BUS_FD_FOR_BUS0_NODE0;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE1)) {
        virtual_dev_fd = SPI_BUS_FD_FOR_BUS0_NODE1;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE2)) {
        virtual_dev_fd = SPI_BUS_FD_FOR_BUS0_NODE2;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE3)) {
        virtual_dev_fd = SPI_BUS_FD_FOR_BUS0_NODE3;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE4)) {
        virtual_dev_fd = SPI_BUS_FD_FOR_BUS0_NODE4;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE5)) {
        virtual_dev_fd = SPI_BUS_FD_FOR_BUS0_NODE5;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE6)) {
        virtual_dev_fd = SPI_BUS_FD_FOR_BUS0_NODE6;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE7)) {
        virtual_dev_fd = SPI_BUS_FD_FOR_BUS0_NODE7;
    } else {
        printf("unknown signature %s\n", _signature);
        return -1;
    }

    _spi_bus_param->out.dev_fd = virtual_dev_fd;
    printf("DATA_FOW_DEBUG(SPI): hal_spi_init, signature:%s, dev_fd:%d\n", _signature, _spi_bus_param->out.dev_fd);
    return 0;
#endif

    ret = raspberry_pi_spi_init(_spi_bus_param);
    if(ret < 0) {
        printf("hal_spi_init->raspberry_pi_spi_init fail\n");
        return -1;
    }

    return ret;
}

/*
@param[in]      signature
                The signature of gpio port
@param[out]     dev_fd
                The file descriptor of gpio port
*/
int hal_spi_deinit(unregister_spi_bus_parameters *spi_bus_param)
{
    char *_signature = NULL;
    int ret = 0, _dev_fd;
    unregister_spi_bus_parameters *_spi_bus_param = spi_bus_param;
  
    if(!_spi_bus_param) {
        printf("hal_spi_deinit incorrect parameters\n");
        return -1;
    }
    
    _dev_fd = _spi_bus_param->temp.dev_fd;
    if(_dev_fd < 0) {
        printf("hal_spi_deinit incorrect device file descriptor\n");
        return -1;
    }
    
#ifdef DATA_FLOW_DEBUG
    _signature = _spi_bus_param->in.signature;
    printf("DATA_FLOW_DEBUG(SPI): hal_spi_deinit, signature = %s dev_fd = %d\n", _signature, _dev_fd);
    return 0;
#endif

    ret = raspberry_pi_spi_deinit(_spi_bus_param);
    if(ret < 0) {
        printf("hal_spi_deinit->raspberry_pi_spi_deinit fail\n");
        return -1;
    }

    return ret;
}

int hal_spi_ioctl(spi_ioctl_parameters *spi_ioctl_param)
{
    int ret = 0, _cmd, i;
    uint8_t *_tx_buf, *_rx_buf;
    uint32_t *_tx_len, *_rx_len;
    int32_t _dev_fd;
    
    spi_ioctl_parameters *_spi_ioctl_param = spi_ioctl_param;

    if(!_spi_ioctl_param) {
        printf("hal_spi_ioctl fail, incorrect parameters\n");
        return -1;
    }

    _cmd = _spi_ioctl_param->cmd;

    switch(_cmd) {

        case SPI_IOCTL_CMD_TRANSFER_MESSAGE:
             _dev_fd = _spi_ioctl_param->transfer_message_param.in.dev_fd;
             if(_dev_fd < 0) {
                printf("hal_spi_ioctl fail, incorrect dev_fd %d\n", _dev_fd);
                return -1;
             }
             
#ifdef DATA_FLOW_DEBUG
             _tx_buf = _spi_ioctl_param->transfer_message_param.in._tx_buf;
             _tx_len = _spi_ioctl_param->transfer_message_param.in._tx_len;
             _rx_buf = _spi_ioctl_param->transfer_message_param.in_out._rx_buf;
             _rx_len = _spi_ioctl_param->transfer_message_param.in_out._rx_len;
             printf("DATA_FOW_DEBUG(SPI): hal_spi_ioctl.transfer_message, dev_fd:%d tx_len:%d\n", _dev_fd, *_tx_len);
             printf("***************************************Message***************************************\n");
             for(i=0; i<(*_tx_len); i++) {
                 printf("0x%x  ", _tx_buf[i]);
                 _rx_buf[i] = i;
             }
             *_rx_len = *_tx_len;
             printf("\n");
             printf("*************************************************************************************\n");
             return 0; 
#endif             

             ret = raspberry_pi_spi_transfer(_spi_ioctl_param);
             if(ret < 0) {
                 printf("raspberry_pi_spi_transfer fail\n");
                 return -1;
             }

             return 0;
        break;

        default:
             printf("hal_spi_ioctl fail, un-supported cmd %d", _cmd);
             return -1;
        break;

    }

    return ret;
}


