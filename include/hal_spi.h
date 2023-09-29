/**
* @file     hal_spi.h
* @brief    hal_spi.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     09/11/2018
*/

#ifndef __HAL_SPI_H__
#define __HAL_SPI_H__

int hal_spi_init(register_spi_bus_parameters *);
int hal_spi_ioctl(spi_ioctl_parameters *);
int hal_spi_deinit(unregister_spi_bus_parameters *);

#endif // __HAL_SPI_H__