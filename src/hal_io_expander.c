/**
* @file     hal_io_expander.c
* @brief    hal_io_expander.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     12/11/2018
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "log_controller.h"
#include "io_expander_controller.h"
#include "hal_io_expander.h"
#include "io_expander_mcp23s17.h"

#include "debug.h"

#ifdef DATA_FLOW_DEBUG
#include "spi_controller.h"
#endif

extern signature_pin_number_mapping mapping[NB_IO_EXPANDER_PORTS];

int hal_io_expander_init(register_io_expander_parameters *io_expander_param)
{
    int ret = 0, _bus_type, i, j, same, count, find, cmd, _nb_io_ports;
    uint8_t _port_number, port_number;
    uint8_t reg_iodira, reg_iodirb, reg_gpioa, reg_gpiob, bit_nb; 
    void *spi_controller_ctx;
    void *spi_controller_ioctl_param;
    char *_io_expander_signatrue = NULL;
    io_port *_io_ports = NULL;
    register_io_expander_parameters *_io_expander_param = io_expander_param;

#ifdef DATA_FLOW_DEBUG
    spi_controller_context *_spi_controller_ctx;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param;
#endif

    if(!_io_expander_param) {
        printf("hal_io_expander_init fail, incorrect parameters\n");
        return -1;
    }

    _nb_io_ports = _io_expander_param->in.nb_io_ports;
    _io_expander_signatrue = _io_expander_param->in.signature;

    if(!strcmp(IO_EXPANDER1_DEVICE_SIGNATURE, _io_expander_signatrue)) {
        if(_nb_io_ports > IO_EXPANDER1_IO_PORTS) {
            printf("hal_io_expander_init fail, the amount of io ports is incorrect, signature %s nb_io_port %d\n", _io_expander_signatrue, _nb_io_ports);
            return -1;
        }
    } else if(!strcmp(IO_EXPANDER2_DEVICE_SIGNATURE, _io_expander_signatrue)) {
        if(_nb_io_ports > IO_EXPANDER2_IO_PORTS) {
            printf("hal_io_expander_init fail, the amount of io ports is incorrect, signature %s nb_io_port %d\n", _io_expander_signatrue, _nb_io_ports);
            return -1;
        }
    } else if(!strcmp(IO_EXPANDER3_DEVICE_SIGNATURE, _io_expander_signatrue)) {
        if(_nb_io_ports > IO_EXPANDER3_IO_PORTS) {
            printf("hal_io_expander_init fail, the amount of io ports is incorrect, signature %s nb_io_port %d\n", _io_expander_signatrue, _nb_io_ports);
            return -1;
        }
    } else if(!strcmp(IO_EXPANDER4_DEVICE_SIGNATURE, _io_expander_signatrue)) {
        if(_nb_io_ports > IO_EXPANDER4_IO_PORTS) {
            printf("hal_io_expander_init fail, the amount of io ports is incorrect, signature %s nb_io_port %d\n", _io_expander_signatrue, _nb_io_ports);
            return -1;
        }
    } else {
        printf("hal_io_expander_init fail, un-support io_expander_signature %s\n", _io_expander_signatrue);
        return -1;
    }
    
    _bus_type = _io_expander_param->in.bus_type;
    if(_bus_type == IO_EXPANDER_BUS_TYPE_SPI) {

        spi_controller_ctx = _io_expander_param->in.spi_controller_ctx;
        if(!spi_controller_ctx) {
            printf("hal_io_expander_init fail, incorrect spi_controller_ctx\n");
            return -1;
        }

        spi_controller_ioctl_param = _io_expander_param->in.spi_controller_ioctl_param;
        if(!spi_controller_ioctl_param) {
            printf("hal_io_expander_init fail, incorrect spi_controller_ioctl_param\n");
            return -1;
        }
        
    } else {
        printf("hal_io_expander_init fail, un-support bus type %d\n", _bus_type);
        return -1;
    }

#ifdef DATA_FLOW_DEBUG
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
                //printf("config1 0x%x\n", _io_ports[i].config_data1);
                //printf("config2 0x%x\n", _io_ports[i].config_data2);
                //printf("config3 0x%x\n", _io_ports[i].config_data3);
                find = 1;
            }
        }
        if(find) {
            count++;
        }
    }
    
    if(count != _nb_io_ports) {
        printf("hal_io_expander_init fail, count != _nb_io_ports");
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

    _spi_controller_ctx = spi_controller_ctx;
    _spi_controller_ioctl_param = spi_controller_ioctl_param;
    //printf("DATA_FOW_DEBUG(IO_EXPANDER): hal_io_expander_init, signature:%s, reg_iodira:0x%x, reg_iodirb:0x%x\n", _io_expander_signatrue, reg_iodira, reg_iodirb);
    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x40;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = 0x0a;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[2] = 0x28;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 3;
    ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("hal_io_expander_init->spi_controller_ioctl fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x40;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = 0x00;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[2] = reg_iodira;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 3;
    ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("hal_io_expander_init->spi_controller_ioctl fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x40;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = 0x01;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[2] = reg_iodirb;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 3;
    ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("hal_io_expander_init->spi_controller_ioctl fail\n");
        return -1;
    }

    return 0;

#endif

    ret = mcp23s17_init(_io_expander_param);
    if(ret < 0) {
        printf("hal_io_expander_init->mcp23s17_init fail\n");
        return -1;
    }

    return ret;
}


int hal_io_expander_ioctl(io_expander_ioctl_parameters *io_expander_ioctl_param)
{
    int ret = 0, _cmd, i, j, length, same, find;   
    int *_bus_type; 
    int _nb_io_ports;
    uint8_t _direction;
    uint8_t *_level;
    char *_terminal_signature;
    io_port *_io_ports, *_io_port;
    void *spi_controller_ctx;
    void *spi_controller_ioctl_param;
    io_expander_ioctl_parameters *_io_expander_ioctl_param = io_expander_ioctl_param;

#ifdef DATA_FLOW_DEBUG
    int cmd;
    uint8_t config_data1, config_data2, config_data3, reg_val = 0;
    uint8_t reg_iodira, reg_iodirb, reg_gpioa, reg_gpiob, bit_nb;
    spi_controller_context *_spi_controller_ctx;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param;
#endif

    if(!_io_expander_ioctl_param) {
        printf("hal_io_expander_ioctl fail, incorrect parameters\n");
        return -1;
    }

    _cmd = _io_expander_ioctl_param->cmd;
    
    switch(_cmd) {

        case IO_EXPANDER_IOCTL_CMD_SET_LEVEL:
             _terminal_signature = _io_expander_ioctl_param->io_expander_set_level_param.in._terminal;
             length = strlen(_terminal_signature);
             if(length > MAX_LENGTH_OF_IO_EXPANDER_TERMINAL_SIGNATURE) {
                 printf("hal_io_expander_ioctl fail, incorrect signature length %d\n", length);
                 return -1;
             }

             _nb_io_ports = _io_expander_ioctl_param->io_expander_set_level_param.in._nb_io_ports;
             if(_nb_io_ports > MAX_NB_OF_IO_EXPANDER_PORTS) {
                 printf("hal_io_expander_ioctl fail, incorrect nb_io_ports %d\n", _nb_io_ports);
                 return -1;
             }

             _io_ports = _io_expander_ioctl_param->io_expander_set_level_param.in._io_ports;
             for(i=0, find=0; i<_nb_io_ports; i++) {
                 same = !strcmp(_terminal_signature, _io_ports[i].terminal);
                 if(same) {
                     _io_port = &_io_ports[i];
                     _io_expander_ioctl_param->io_expander_set_level_param.temp._io_port = _io_port;
                     find = 1;
                     break;
                 }
             }
             
             if(!find) {
                 printf("hal_io_expander_ioctl fail, can't mach terminal %s\n", _terminal_signature);
                 return -1;
             }

             _direction = _io_port->direction;
             if(_direction!=IO_EXPANDER_DIRECTION_OUT) {
                 printf("hal_io_expander_ioctl fail, incorrect direction %d\n", _direction);
                 return -1;
             }

             _level = _io_expander_ioctl_param->io_expander_set_level_param.in._level;
             if(*_level!=IO_EXPANDER_PORT_LEVEL_HIGH && *_level!=IO_EXPANDER_PORT_LEVEL_LOW) {
                 printf("hal_io_expander_ioctl fail, incorrect level %d\n", *_level);
                 return -1;
             }

             _bus_type = _io_expander_ioctl_param->io_expander_set_level_param.in._bus_type;
            
             if(*_bus_type == IO_EXPANDER_BUS_TYPE_SPI) {
                 spi_controller_ctx = _io_expander_ioctl_param->io_expander_set_level_param.in._spi_controller_ctx;
                 spi_controller_ioctl_param = _io_expander_ioctl_param->io_expander_set_level_param.in._spi_controller_ioctl_param;
             }

             if(!spi_controller_ctx || !spi_controller_ioctl_param) {
                 printf("hal_io_expander_ioctl fail, incorrect spi parameters\n");
                 return -1;
             }
             
#ifdef DATA_FLOW_DEBUG
             config_data1 = _io_port->config_data1;
             config_data2 = _io_port->config_data2;
             config_data3 = _io_port->config_data3;
             printf("DATA_FOW_DEBUG(IO_Expander): hal_io_expander_ioctl.set_level, terminal:%s level:%s\n", _terminal_signature, *_level==IO_EXPANDER_PORT_LEVEL_HIGH?"High":"Low");
             printf("DATA_FOW_DEBUG(IO_Expander): hal_io_expander_ioctl.set_level config_data1 0x%x config_data2 0x%x config_data3 0x%x\n", config_data1, config_data2, config_data3);
             
             /* read gpio register pointing by config_data3 */
             
             /* check if level is High, then set mask bit; if level is Low, then clear bit */
 
             if(*_level==IO_EXPANDER_PORT_LEVEL_HIGH) {
                reg_val = 0;
                if(config_data1 > 7) { /* Region B */
                    bit_nb = config_data1 - 8;
                    reg_val |= 1<<bit_nb;
                    
                } else { /* Region A */
                    bit_nb = config_data1;
                    reg_val |= 1<<bit_nb;
                }
             } else {
                reg_val = 0xFF;
                if(config_data1 > 7) { /* Region B */
                    bit_nb = config_data1 - 8;
                    reg_val |= 1<<bit_nb;
                    
                } else { /* Region A */
                    bit_nb = config_data1;
                    reg_val |= 1<<bit_nb;
                }
                reg_val &= 0xff ^ (1 << bit_nb);
             }

             _spi_controller_ctx = spi_controller_ctx;
             _spi_controller_ioctl_param = spi_controller_ioctl_param;
             printf("DATA_FOW_DEBUG(IO_Expander): hal_io_expander_ioctl->set_level, bit_nb:0x%x, reg_val 0x%x\n", bit_nb, reg_val);
             cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x40;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = config_data3;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[2] = reg_val;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 3;
             ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
             if(ret < 0) {
                 printf("hal_io_expander_init->spi_controller_ioctl fail\n");
                 return -1;
             }

             return 0; 
#endif             

             ret = mcp23s17_ioctl(_io_expander_ioctl_param);
             if(ret < 0) {
                 printf("mcp23s17_ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        default:
             printf("hal_io_expander_ioctl fail, un-supported cmd %d", _cmd);
             return -1;
        break;

    }

    return ret;
}


int hal_io_expander_deinit(unregister_io_expander_parameters *io_expander_param)
{
    char *_io_expander_signature = NULL;
    int ret = 0;
    unregister_io_expander_parameters *_io_expander_param = io_expander_param;
  
    if(!_io_expander_param) {
        printf("hal_io_expander_deinit incorrect parameters\n");
        return -1;
    }
    
#ifdef DATA_FLOW_DEBUG
    _io_expander_signature = _io_expander_param->in.signature;
    printf("DATA_FLOW_DEBUG(IO_EXPANDER): hal_io_expander_deinit, signature = %s\n", _io_expander_signature);
    return 0;
#endif

    ret = mcp23s17_deinit(_io_expander_param);
    if(ret < 0) {
        printf("hal_spi_deinit->mcp23s17_deinit fail\n");
        return -1;
    }

    return ret;
}

