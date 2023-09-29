/**
* @file     spi_raspberry_pi.c
* @brief    spi_raspberry_pi.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     09/11/2018
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <pthread.h>

#include "log_controller.h"
#include "spi_controller.h"
#include "spi_raspberry_pi.h"

int raspberry_pi_spi_init(register_spi_bus_parameters *spi_bus_param)
{
    int ret = 0, spi_node_number, dev_fd;
    char path[512] = {0};
    register_spi_bus_parameters *_spi_bus_param = spi_bus_param;
    char *_signature = _spi_bus_param->in.signature;
    uint8_t _mode;
    uint32_t _speed;
    uint32_t _cs_polarity;
    uint8_t _bits_per_word;

    if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE0)) {
        spi_node_number = SPI_BUS_FD_FOR_BUS0_NODE0;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE1)) {
        spi_node_number = SPI_BUS_FD_FOR_BUS0_NODE1;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE2)) {
        spi_node_number = SPI_BUS_FD_FOR_BUS0_NODE2;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE3)) {
        spi_node_number = SPI_BUS_FD_FOR_BUS0_NODE3;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE4)) {
        spi_node_number = SPI_BUS_FD_FOR_BUS0_NODE4;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE5)) {
        spi_node_number = SPI_BUS_FD_FOR_BUS0_NODE5;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE6)) {
        spi_node_number = SPI_BUS_FD_FOR_BUS0_NODE6;
    } else if(!strcmp(_signature, SPI_BUS_SIGNATURE_BUS0_NODE7)) {
        spi_node_number = SPI_BUS_FD_FOR_BUS0_NODE7;
    } else {
        printf("unknown signature %s\n", _signature);
        return -1;
    }

    snprintf(path, 255, "/dev/spidev0.%d", spi_node_number);

    dev_fd = open(path, O_RDWR);
    if(dev_fd < 0) {
        printf("raspberry_pi_spi_init->open fail\n");
        return -1;
    }

    _mode = _spi_bus_param->in.mode;
    _speed = _spi_bus_param->in.speed;
    _cs_polarity = _spi_bus_param->in.cs_polarity;
    _bits_per_word = _spi_bus_param->in.bits_per_word;
    
    ret = ioctl(dev_fd, SPI_IOC_WR_MODE, &_mode);
    if(ret < 0)
    {
        printf("set write mode %d fail\n", _mode);
        close(dev_fd);
        return -1;
    }

    ret = ioctl(dev_fd, SPI_IOC_RD_MODE, &_mode);
    if(ret < 0)
    {
        printf("set read mode %d fail\n", _mode);
        close(dev_fd);
        return -1;
    }

    ret = ioctl(dev_fd, SPI_IOC_WR_BITS_PER_WORD, &_bits_per_word);
    if(ret < 0)
    {
      printf("set bits per word for write operation fail, bpw = %d\n", _bits_per_word);
      close(dev_fd);
      return -1;
    }

    ret = ioctl(dev_fd, SPI_IOC_RD_BITS_PER_WORD, &_bits_per_word);
    if(ret < 0)
    {
      printf("set bits per word for read operation fail, bpw = %d\n", _bits_per_word);
      close(dev_fd);
      return -1;
    }

    ret = ioctl(dev_fd, SPI_IOC_WR_MAX_SPEED_HZ, &_speed);
    if(ret < 0)
    {
      printf("set max speed %d for read operation, fail\n", _speed);
      close(dev_fd);
      return -1;
    }

    ret = ioctl(dev_fd, SPI_IOC_RD_MAX_SPEED_HZ, &_speed);
    if(ret < 0)
    {
      printf("set max speed %d for read operation, fail\n", _speed);
      close(dev_fd);
      return -1;
    }

    _spi_bus_param->out.dev_fd = dev_fd; /* set spi_fd */

    return ret;
}

int raspberry_pi_spi_transfer(spi_ioctl_parameters *spi_ioctl_param)
{
    int ret = 0;
    int32_t _dev_fd;
    uint8_t *_tx_buf, *_rx_buf;
    uint32_t *_tx_len, *_rx_len;
    uint32_t _speed;
    uint8_t _bits_per_word;
    struct spi_ioc_transfer spi_transfer_param;
    spi_ioctl_parameters *_spi_ioctl_param = spi_ioctl_param;
    
    _dev_fd = _spi_ioctl_param->transfer_message_param.in.dev_fd;
    _tx_buf = _spi_ioctl_param->transfer_message_param.in._tx_buf;
    _tx_len = _spi_ioctl_param->transfer_message_param.in._tx_len;
    _rx_buf = _spi_ioctl_param->transfer_message_param.in_out._rx_buf;
    _rx_len = _spi_ioctl_param->transfer_message_param.in_out._rx_len;
    _speed = _spi_ioctl_param->transfer_message_param.in.speed;
    _bits_per_word = _spi_ioctl_param->transfer_message_param.in.bits_per_word;

    memset(&spi_transfer_param, 0, sizeof(spi_transfer_param));
    spi_transfer_param.tx_buf        = (unsigned long)(_tx_buf);
    spi_transfer_param.rx_buf        = (unsigned long)(_rx_buf);
    spi_transfer_param.len           = *_tx_len;
    spi_transfer_param.delay_usecs   = 0 ;
    spi_transfer_param.speed_hz      = _speed;
    spi_transfer_param.bits_per_word = _bits_per_word;
    spi_transfer_param.cs_change = 0;

    ret = ioctl(_dev_fd, SPI_IOC_MESSAGE(1), &spi_transfer_param) ;
    if(ret < 0) {
        printf("raspberry_pi_spi_transfer -> ioctl fail, %d\n", ret);
        return -1;
    }

    return ret;
}

int raspberry_pi_spi_deinit(unregister_spi_bus_parameters *spi_bus_param)
{
    int ret = 0, _dev_fd;
    unregister_spi_bus_parameters *_spi_bus_param = spi_bus_param;

    _dev_fd = _spi_bus_param->temp.dev_fd;

    close(_dev_fd);

    return ret;
}

