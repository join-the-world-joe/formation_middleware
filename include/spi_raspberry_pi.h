/**
* @file     spi_raspberry_pi.h
* @brief    spi_raspberry_pi.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     09/11/2018
*/

#ifndef __SPI_RASPBERRY_PI_H__
#define __SPI_RASPBERRY_PI_H__

#define SPI_BUS_SIGNATURE_BUS0_NODE0    "/dev/spidev0.0"
#define SPI_BUS_SIGNATURE_BUS0_NODE1    "/dev/spidev0.1"
#define SPI_BUS_SIGNATURE_BUS0_NODE2    "/dev/spidev0.2"
#define SPI_BUS_SIGNATURE_BUS0_NODE3    "/dev/spidev0.3"
#define SPI_BUS_SIGNATURE_BUS0_NODE4    "/dev/spidev0.4"
#define SPI_BUS_SIGNATURE_BUS0_NODE5    "/dev/spidev0.5"
#define SPI_BUS_SIGNATURE_BUS0_NODE6    "/dev/spidev0.6"
#define SPI_BUS_SIGNATURE_BUS0_NODE7    "/dev/spidev0.7"

#define SPI_BUS_FD_FOR_BUS0_NODE0   0
#define SPI_BUS_FD_FOR_BUS0_NODE1   1
#define SPI_BUS_FD_FOR_BUS0_NODE2   2
#define SPI_BUS_FD_FOR_BUS0_NODE3   3
#define SPI_BUS_FD_FOR_BUS0_NODE4   4
#define SPI_BUS_FD_FOR_BUS0_NODE5   5
#define SPI_BUS_FD_FOR_BUS0_NODE6   6
#define SPI_BUS_FD_FOR_BUS0_NODE7   7

int raspberry_pi_spi_init(register_spi_bus_parameters *);
int raspberry_pi_spi_transfer(spi_ioctl_parameters *);
int raspberry_pi_spi_deinit(unregister_spi_bus_parameters *);


#endif // __SPI_RASPBERRY_PI_H__
