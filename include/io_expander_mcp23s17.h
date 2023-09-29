/**
* @file     io_expander_mcp23s17.h
* @brief    io_expander_mcp23s17.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     12/11/2018
*/

#ifndef __IO_EXPANDER_MCP23S17_H__
#define __IO_EXPANDER_MCP23S17_H__

#define NB_IO_EXPANDER_PORTS 64

#define IO_EXPANDER1_DEVICE_SIGNATURE "U1"
#define IO_EXPANDER2_DEVICE_SIGNATURE "U2"
#define IO_EXPANDER3_DEVICE_SIGNATURE "U3"
#define IO_EXPANDER4_DEVICE_SIGNATURE "U4"

#define IO_EXPANDER1_IO_PORTS 16
#define IO_EXPANDER2_IO_PORTS 16
#define IO_EXPANDER3_IO_PORTS 16
#define IO_EXPANDER4_IO_PORTS 16

#define MCP23S17_SPI_READ_OPERATION  1
#define MCP23S17_SPI_WRITE_OPERATION 0

#define IODIRA 0x00  // I/O DIRECTION REGISTER A, Port7-Port0
#define IODIRB 0x01  // I/O DIRECTION REGISTER B, Port15-Port8 

#define GPIOA 0x12  // PORT REGISTER A, Port7-Port0
#define GPIOB 0x13  // PORT REGISTER B, Port15-Port8

#define PORT_NB_KZ_H1   8 
#define PORT_NB_KZ_M1   9
#define PORT_NB_KZ_L1   10
#define PORT_NB_JC1     11
#define PORT_NB_CF1     12
#define PORT_NB_CK1     13
#define PORT_NB_CLED1   14
#define PORT_NB_FLED1   15

#define PORT_NB_KZ_H2   0
#define PORT_NB_KZ_M2   1
#define PORT_NB_KZ_L2   2
#define PORT_NB_JC2     3
#define PORT_NB_CF2     4
#define PORT_NB_CK2     5
#define PORT_NB_CLED2   6
#define PORT_NB_FLED2   7

#define PORT_NB_KZ_H3   8
#define PORT_NB_KZ_M3   9
#define PORT_NB_KZ_L3   10
#define PORT_NB_JC3     11
#define PORT_NB_CF3     12
#define PORT_NB_CK3     13
#define PORT_NB_CLED3   14
#define PORT_NB_FLED3   15

#define PORT_NB_KZ_H4   0
#define PORT_NB_KZ_M4   1
#define PORT_NB_KZ_L4   2
#define PORT_NB_JC4     3
#define PORT_NB_CF4     4
#define PORT_NB_CK4     5
#define PORT_NB_CLED4   6
#define PORT_NB_FLED4   7

#define PORT_NB_KZ_H5   8
#define PORT_NB_KZ_M5   9
#define PORT_NB_KZ_L5   10
#define PORT_NB_JC5     11
#define PORT_NB_CF5     12
#define PORT_NB_CK5     13
#define PORT_NB_CLED5   14
#define PORT_NB_FLED5   15

#define PORT_NB_KZ_H6   0
#define PORT_NB_KZ_M6   1
#define PORT_NB_KZ_L6   2
#define PORT_NB_JC6     3
#define PORT_NB_CF6     4
#define PORT_NB_CK6     5
#define PORT_NB_CLED6   6
#define PORT_NB_FLED6   7

#define PORT_NB_KZ_H7   8
#define PORT_NB_KZ_M7   9
#define PORT_NB_KZ_L7   10
#define PORT_NB_JC7     11
#define PORT_NB_CF7     12
#define PORT_NB_CK7     13
#define PORT_NB_CLED7   14
#define PORT_NB_FLED7   15

#define PORT_NB_KZ_H8   0
#define PORT_NB_KZ_M8   1
#define PORT_NB_KZ_L8   2
#define PORT_NB_JC8     3
#define PORT_NB_CF8     4
#define PORT_NB_CK8     5
#define PORT_NB_CLED8   6  
#define PORT_NB_FLED8   7


typedef struct _signature_pin_number_mapping {
    char signature[MAX_LENGTH_OF_IO_EXPANDER_SIGNATURE];
    int port_number;
} signature_pin_number_mapping;

int mcp23s17_init(register_io_expander_parameters *);
int mcp23s17_ioctl(io_expander_ioctl_parameters *);
int mcp23s17_deinit(unregister_io_expander_parameters *);


#endif // __IO_EXPANDER_MCP23S17_H__
