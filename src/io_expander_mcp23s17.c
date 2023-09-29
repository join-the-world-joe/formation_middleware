/**
* @file     io_expander_mcp23s17.c
* @brief    io_expander_mcp23s17.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     12/11/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "log_controller.h"
#include "spi_controller.h"
#include "io_expander_controller.h"
#include "io_expander_mcp23s17.h"

signature_pin_number_mapping mapping[NB_IO_EXPANDER_PORTS] = {
    {"KZ_H1", PORT_NB_KZ_H1},
    {"KZ_M1", PORT_NB_KZ_M1},
    {"KZ_L1", PORT_NB_KZ_L1},
    {"JC1", PORT_NB_JC1},
    {"CF1", PORT_NB_CF1},
    {"CK1", PORT_NB_CK1},
    {"CLED1", PORT_NB_CLED1},
    {"FLED1", PORT_NB_FLED1},
        {"KZ_H2", PORT_NB_KZ_H2},
        {"KZ_M2", PORT_NB_KZ_M2},
        {"KZ_L2", PORT_NB_KZ_L2},
        {"JC2", PORT_NB_JC2},
        {"CF2", PORT_NB_CF2},
        {"CK2", PORT_NB_CK2},
        {"CLED2", PORT_NB_CLED2},
        {"FLED2", PORT_NB_FLED2},
    {"KZ_H3", PORT_NB_KZ_H3},
    {"KZ_M3", PORT_NB_KZ_M3},
    {"KZ_L3", PORT_NB_KZ_L3},
    {"JC3", PORT_NB_JC3},
    {"CF3", PORT_NB_CF3},
    {"CK3", PORT_NB_CK3},
    {"CLED3", PORT_NB_CLED3},
    {"FLED3", PORT_NB_FLED3},
        {"KZ_H4", PORT_NB_KZ_H4},
        {"KZ_M4", PORT_NB_KZ_M4},
        {"KZ_L4", PORT_NB_KZ_L4},
        {"JC4", PORT_NB_JC4},
        {"CF4", PORT_NB_CF4},
        {"CK4", PORT_NB_CK4},
        {"CLED4", PORT_NB_CLED4},
        {"FLED4", PORT_NB_FLED4},
    {"KZ_H5", PORT_NB_KZ_H5},
    {"KZ_M5", PORT_NB_KZ_M5},
    {"KZ_L5", PORT_NB_KZ_L5},
    {"JC5", PORT_NB_JC5},
    {"CF5", PORT_NB_CF5},
    {"CK5", PORT_NB_CK5},
    {"CLED5", PORT_NB_CLED5},
    {"FLED5", PORT_NB_FLED5},
        {"KZ_H6", PORT_NB_KZ_H6},
        {"KZ_M6", PORT_NB_KZ_M6},
        {"KZ_L6", PORT_NB_KZ_L6},
        {"JC6", PORT_NB_JC6},
        {"CF6", PORT_NB_CF6},
        {"CK6", PORT_NB_CK6},
        {"CLED6", PORT_NB_CLED6},
        {"FLED6", PORT_NB_FLED6},
    {"KZ_H7", PORT_NB_KZ_H7},
    {"KZ_M7", PORT_NB_KZ_M7},
    {"KZ_L7", PORT_NB_KZ_L7},
    {"JC7", PORT_NB_JC7},
    {"CF7", PORT_NB_CF7},
    {"CK7", PORT_NB_CK7},
    {"CLED7", PORT_NB_CLED7},
    {"FLED7", PORT_NB_FLED7},
        {"KZ_H8", PORT_NB_KZ_H8},
        {"KZ_M8", PORT_NB_KZ_M8},
        {"KZ_L8", PORT_NB_KZ_L8},
        {"JC8", PORT_NB_JC8},
        {"CF8", PORT_NB_CF8},
        {"CK8", PORT_NB_CK8},
        {"CLED8", PORT_NB_CLED8},
        {"FLED8", PORT_NB_FLED8}
};

uint8_t mcp23s17_get_device_opcode(uint8_t rw_bit, uint8_t hw_addr)
{
    hw_addr = (hw_addr << 1) & 0x0E;
    rw_bit &= 0x01; 
    return 0x40 | hw_addr | rw_bit;
}

int mcp23s17_readRegister(spi_controller_context *spi_controller_ctx, spi_controller_ioctl_parameters *spi_controller_ioctl_param, uint8_t reg, uint8_t *data, uint8_t hw_addr)
{
    int ret = 0;
    uint8_t tx_buf[3] = {0}, _bus_type, rx_buf[3] = {0};
    uint8_t device_opcode = 0;
    spi_controller_context *_spi_controller_ctx = spi_controller_ctx;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param = spi_controller_ioctl_param;
    
    device_opcode = mcp23s17_get_device_opcode(MCP23S17_SPI_READ_OPERATION, hw_addr);    
    
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = device_opcode; /* opcode */
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = reg;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[2] = 0;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 3;
    ret = spi_controller_ioctl(_spi_controller_ctx, SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("mcp23s17_readRegister->mcp23s17_readRegister fail\n");
        return -1;
    }        
    
    *data = _spi_controller_ioctl_param->transfer_message_param.out.rx_buf[2];

    return 0;
}

int mcp23s17_writeRegister(spi_controller_context *spi_controller_ctx, spi_controller_ioctl_parameters *spi_controller_ioctl_param, uint8_t reg, uint8_t data, uint8_t hw_addr)
{
    int ret = 0;
    uint8_t tx_buf[3] = {0}, _bus_type, rx_buf[3] = {0};
    uint8_t device_opcode = 0;
    spi_controller_context *_spi_controller_ctx = spi_controller_ctx;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param = spi_controller_ioctl_param;

     device_opcode = mcp23s17_get_device_opcode(MCP23S17_SPI_WRITE_OPERATION, hw_addr);    
    
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = device_opcode; /* opcode */
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = reg;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[2] = data;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 3;
    ret = spi_controller_ioctl(_spi_controller_ctx, SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("mcp23s17_readRegister->mcp23s17_readRegister fail\n");
        return -1;
    }   
    
    return ret;
}

int mcp23s17_init(register_io_expander_parameters *io_expander_param)
{
    int ret = 0, _bus_type, count, i, j, find, same, cmd;
    int _nb_io_ports;
    io_port *_io_ports = NULL;
    uint8_t _port_number, port_number;
    uint8_t reg_iodira, reg_iodirb, reg_gpioa, reg_gpiob, bit_nb; 
    register_io_expander_parameters *_io_expander_param = io_expander_param;
    spi_controller_context *spi_controller_ctx;
    spi_controller_ioctl_parameters *spi_controller_ioctl_param;

    _io_ports = _io_expander_param->in_out.io_ports;
    _nb_io_ports = _io_expander_param->in.nb_io_ports;

    _bus_type = _io_expander_param->in.bus_type;
    if(_bus_type == IO_EXPANDER_BUS_TYPE_SPI) {
        spi_controller_ctx = _io_expander_param->in.spi_controller_ctx;
        spi_controller_ioctl_param = _io_expander_param->in.spi_controller_ioctl_param;        
    } else {
        printf("mcp23s17_init fail, un-support bus type %d\n", _bus_type);
        return -1;
    }

    count = 0;
    _io_ports = _io_expander_param->in_out.io_ports;
    for(i=0; i<_nb_io_ports; i++) {

        for(j=0, find=0; j<NB_IO_EXPANDER_PORTS; j++) {
            same = !strcmp(_io_ports[i].terminal, mapping[j].signature);
            if(same) {
                _port_number = mapping[j].port_number;
                if(_port_number > 7) {
                    _io_ports[i].config_data1 = mapping[j].port_number;
                    _io_ports[i].config_data2 = IODIRB;
                    _io_ports[i].config_data3 = GPIOB;
                    
                } else {
                    _io_ports[i].config_data1 = mapping[j].port_number;
                    _io_ports[i].config_data2 = IODIRA;
                    _io_ports[i].config_data3 = GPIOA;
                }
                find = 1;
            }
        }
        if(find) {
            count++;
        }
    }
    
    if(count != _nb_io_ports) {
        printf("mcp23s17_init fail, count != _nb_io_ports");
        return -1;
    }

    // calculate the contents in reg_iodira, reg_iodirb
    reg_iodira = 0; /* 0-Output, 1-Input */
    reg_iodirb = 0; /* 0-Output, 1-Input */
    for(i=0; i<_nb_io_ports; i++) {
        port_number = i;
        for(j=0; j<_nb_io_ports; j++) {
           
            if(port_number == _io_ports[j].config_data1) {
                if(port_number > 7) { /* Region B */
                    if(_io_ports[j].direction == IO_EXPANDER_DIRECTION_IN) {
                        bit_nb = port_number - 8;
                        reg_iodirb |= 1<<bit_nb;
                    } 
                } else { /* Region A */
                    if(_io_ports[j].direction == IO_EXPANDER_DIRECTION_IN) {
                        bit_nb = port_number;
                        reg_iodira |= 1<<bit_nb;
                    } 
                }
            }
        }
    }

    spi_controller_ctx = spi_controller_ctx;
    spi_controller_ioctl_param = spi_controller_ioctl_param;
    //printf("DATA_FOW_DEBUG(IO_EXPANDER): hal_io_expander_init, signature:%s, reg_iodira:0x%x, reg_iodirb:0x%x\n", _io_expander_signatrue, reg_iodira, reg_iodirb);
    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x40;
    spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = 0x0a;
    spi_controller_ioctl_param->transfer_message_param.in.tx_buf[2] = 0x28;
    spi_controller_ioctl_param->transfer_message_param.in.tx_len = 3;
    ret = spi_controller_ioctl(spi_controller_ctx, cmd, spi_controller_ioctl_param);
    if(ret < 0) {
        printf("mcp23s17_init->spi_controller_ioctl fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x40;
    spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = 0x00;
    spi_controller_ioctl_param->transfer_message_param.in.tx_buf[2] = reg_iodira;
    spi_controller_ioctl_param->transfer_message_param.in.tx_len = 3;
    ret = spi_controller_ioctl(spi_controller_ctx, cmd, spi_controller_ioctl_param);
    if(ret < 0) {
        printf("mcp23s17_init->spi_controller_ioctl fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x40;
    spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = 0x01;
    spi_controller_ioctl_param->transfer_message_param.in.tx_buf[2] = reg_iodirb;
    spi_controller_ioctl_param->transfer_message_param.in.tx_len = 3;
    ret = spi_controller_ioctl(spi_controller_ctx, cmd, spi_controller_ioctl_param);
    if(ret < 0) {
        printf("mcp23s17_init->spi_controller_ioctl fail\n");
        return -1;
    }
    
    return ret;
}

int mcp23s17_ioctl(io_expander_ioctl_parameters *io_expander_ioctl_param)
{
    int ret = 0, _cmd, cmd;
    io_port *_io_port;
    uint8_t _config_data1, _config_data2, _config_data3, reg_val, reg, hw_addr = 0, bit_nb;
    uint8_t reg_gpio_ab;
    uint8_t *_level, *_gpio_a, *_gpio_b;
    io_expander_ioctl_parameters *_io_expander_ioctl_param = io_expander_ioctl_param;
    spi_controller_context *_spi_controller_ctx;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param;

    _io_port = _io_expander_ioctl_param->io_expander_set_level_param.temp._io_port;
    _cmd = _io_expander_ioctl_param->cmd;
    _config_data1 = _io_port->config_data1;
    _config_data2 = _io_port->config_data2;
    _config_data3 = _io_port->config_data3;

    _spi_controller_ctx = _io_expander_ioctl_param->io_expander_set_level_param.in._spi_controller_ctx;
    _spi_controller_ioctl_param = _io_expander_ioctl_param->io_expander_set_level_param.in._spi_controller_ioctl_param;   

    switch(_cmd) {

        case IO_EXPANDER_IOCTL_CMD_SET_LEVEL:
#if 0
             reg = _config_data3;
             ret = mcp23s17_readRegister(_spi_controller_ctx, _spi_controller_ioctl_param, reg, &reg_val, hw_addr);
             if(ret < 0) {
                 printf("mcp23s17_ioctl->mcp23s17_readRegister fail\n");
                 return -1;
             }

#endif
             _gpio_a = _io_expander_ioctl_param->io_expander_set_level_param.in._gpio_a;
             _gpio_b = _io_expander_ioctl_param->io_expander_set_level_param.in._gpio_b;

             printf("*************************GPIO Group Begin****************************\n\n\n");
             printf("gpioa 0x%x\n", *_gpio_a);
             printf("gpiob 0x%x\n", *_gpio_b);
             printf("*************************GPIO Group Begin****************************\n\n\n");

             _level = _io_expander_ioctl_param->io_expander_set_level_param.in._level;
             if(*_level==IO_EXPANDER_PORT_LEVEL_HIGH) {
                if(_config_data1 > 7) { /* Region B */
                    bit_nb = _config_data1 - 8;    
                    reg_val = *_gpio_b;
                    reg_val |= (1<<bit_nb);
                    *_gpio_b = reg_val;
                } else { /* Region A */
                    bit_nb = _config_data1;
                    reg_val = *_gpio_a;
                    reg_val |= (1<<bit_nb);
                    *_gpio_a = reg_val;
                }
             } else {
                if(_config_data1 > 7) { /* Region B */
                    bit_nb = _config_data1 - 8;          
                    reg_val = *_gpio_b;
                    reg_val &= ~(1<<bit_nb);
                    *_gpio_b = reg_val;
                } else { /* Region A */
                    bit_nb = _config_data1;
                    reg_val = *_gpio_a;
                    reg_val &= ~(1<<bit_nb);
                    *_gpio_a = reg_val;
                }
             }

             if(_config_data3 == GPIOA) {
                 reg_val = *_gpio_a;
             } else {
                 reg_val = *_gpio_b;
             } 

             printf("*************************GPIO Group End****************************\n\n\n");
             printf("gpioa 0x%x\n", *_gpio_a);
             printf("gpiob 0x%x\n", *_gpio_b);
             printf("*************************GPIO Group End****************************\n\n\n");

             printf("**********************************************************\n\n");
             printf("reg_val 0x%x bit_nb %d\n", reg_val, bit_nb);
             printf("**********************************************************\n\n");

             cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x40;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = _config_data3;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[2] = reg_val;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 3;
             ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
             if(ret < 0) {
                 printf("mcp23s17_ioctl->spi_controller_ioctl fail\n");
                 return -1;
             }
             return 0;
        break;

    }

    return ret;
}


int mcp23s17_deinit(unregister_io_expander_parameters *io_expander_param)
{
    int ret = 0;

    printf("mcp23s17_deinit done!!\n");

    return ret;
}




