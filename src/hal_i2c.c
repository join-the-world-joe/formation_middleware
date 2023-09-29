/**
* @file     hal_i2c.c
* @brief    hal_i2c.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     01/12/2018
*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "log_controller.h"
#include "i2c_controller.h"
#include "hal_i2c.h"

#include "i2c_linux.h"

#include "debug.h"

int hal_i2c_init(register_i2c_bus_parameters *_i2c_bus_param)
{
    int ret = 0, _nb_i2c_bus = NB_I2C_BUS, length;
    char *_i2c_bus_signature = NULL, *_path = NULL;
    register_i2c_bus_parameters *i2c_bus_param = _i2c_bus_param;

    printf("hal_i2c_init, bus: %s\n", i2c_bus_param->in.signature);

    if(!_i2c_bus_param) {
        printf("hal_i2c_init fail, incorrect parameters\n");
        return -1;
    }

    _path = i2c_bus_param->temp.path;
    _i2c_bus_signature = i2c_bus_param->in.signature;
    
#ifdef DATA_FLOW_DEBUG
        
        if(!strcmp(_i2c_bus_signature, I2C_BUS_1_SIGNATURE)) {
            
            memset(_path, 0, MAX_LENGTH_OF_I2C_DEV_PATH);

            snprintf(_path, 255, "/dev/%s", I2C_BUS_1_SIGNATURE);

            i2c_bus_param->out.dev_fd = 1;

            printf("Data Flow Debug: %s supported\n", _i2c_bus_signature);
            printf("Data Flow Debug: path: %s\n", _path);
            printf("Data Flow Debug: FD [1]\n");

            return 0;
        } else {
            printf("hal_i2c_init fail, un-support i2c_bus_signature %s\n", _i2c_bus_signature);
            return -1;
        }
    
#endif

    if(!strcmp(_i2c_bus_signature, I2C_BUS_1_SIGNATURE)) {
        memset(_path, 0, MAX_LENGTH_OF_I2C_DEV_PATH);
        snprintf(_path, 255, "/dev/%s", I2C_BUS_1_SIGNATURE);
    } else {
        printf("hal_i2c_init fail, un-support i2c_bus_signature %s\n", _i2c_bus_signature);
        return -1;
    }
    
    ret = i2c_linux_init(i2c_bus_param);
    if(ret < 0) {
        printf("i2c_linux_init fail\n");
        return -1;
    }

    return ret;
}

int hal_i2c_ioctl(i2c_ioctl_parameters *_i2c_ioctl_param)
{
    int ret = 0, _cmd, _dev_fd, i, _wr_length, _rd_length;
    uint8_t _i2c_dev_addr, *_wr_data, *_rd_data;
    i2c_ioctl_parameters *i2c_ioctl_param = _i2c_ioctl_param;

    if(!i2c_ioctl_param) {
        printf("hal_i2c_ioctl fail\n");
        return -1;
    }

    _cmd = i2c_ioctl_param->cmd;
    
    

    switch(_cmd) {

        case I2C_IOCTL_CMD_WRITE_MULTI_BYTES:
             _dev_fd = i2c_ioctl_param->write_multi_bytes_param.in._dev_fd;
             _i2c_dev_addr = *i2c_ioctl_param->write_multi_bytes_param.in._i2c_dev_addr;
             _wr_data = i2c_ioctl_param->write_multi_bytes_param.in._data;
             _wr_length = *i2c_ioctl_param->write_multi_bytes_param.in._length;

#ifdef DATA_FLOW_DEBUG
            
             printf("I2C Data Flow Debug(hal_i2c_ioctl, I2C_IOCTL_CMD_WRITE_MULTI_BYTES): dev_fd %d\n", _dev_fd);
             printf("I2C Data Flow Debug(hal_i2c_ioctl, I2C_IOCTL_CMD_WRITE_MULTI_BYTES): i2c_dev_addr 0x%x\n", _i2c_dev_addr);
             //printf("I2C Data Flow Debug: Data 0x%x\n", _data);

             for(i=0; i<_wr_length; i++) {
                 printf("data[%d]: 0x%x\n", i, _wr_data[i]);
             }

             return 0;
#endif

             if(_dev_fd < 0) {
                 printf("hal_i2c_ioctl->write_multi_bytes fail\n");
                 return -1;
             }

             if(!_wr_data) {
                 printf("hal_i2c_ioctl fail, data pointer incorrect\n");
                 return -1;
             }

             if(_wr_length > MAX_LENGTH_OF_I2C_DATA || 0 > _wr_length) {
                 printf("hal_i2c_ioctl fail, data length exceeds the MAX_LENGTH_OF_I2C_DATA\n");
                 return -1;
             }

             ret = i2c_linux_ioctl(i2c_ioctl_param);
             if(ret < 0) {
                 printf("hal_i2c_ioctl->i2c_linux_ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        case I2C_IOCTL_CMD_READ_MULTI_BYTES:
             _dev_fd = i2c_ioctl_param->read_multi_bytes_param.in._dev_fd;
             _i2c_dev_addr = *i2c_ioctl_param->read_multi_bytes_param.in._i2c_dev_addr;
             _wr_data = i2c_ioctl_param->read_multi_bytes_param.in._wr_data;
             _wr_length = *i2c_ioctl_param->read_multi_bytes_param.in._wr_length;
             _rd_length = *i2c_ioctl_param->read_multi_bytes_param.in._rd_length;
             _rd_data = i2c_ioctl_param->read_multi_bytes_param.out._rd_data;
             
#ifdef DATA_FLOW_DEBUG
             printf("I2C Data Flow Debug(hal_i2c_ioctl, I2C_IOCTL_CMD_READ_MULTI_BYTES): dev_fd %d\n", _dev_fd);
             printf("I2C Data Flow Debug(hal_i2c_ioctl, I2C_IOCTL_CMD_READ_MULTI_BYTES): i2c_dev_addr 0x%x\n", _i2c_dev_addr);
             printf("I2C Data Flow Debug(hal_i2c_ioctl, I2C_IOCTL_CMD_READ_MULTI_BYTES): rd_length %d\n", _rd_length);
             for(i=0; i<_wr_length; i++) {
                 printf("data[%d]: 0x%x\n", i, _wr_data[i]);
             }             

             return 0;
#endif

             if(_dev_fd < 0) {
                 printf("hal_i2c_ioctl->write_multi_bytes fail\n");
                 return -1;
             }

             if(!_wr_data) {
                 printf("hal_i2c_ioctl fail, data pointer NULL\n");
                 return -1;
             }

             if(!_rd_data) {
                 printf("hal_i2c_ioctl fail, data pointer NULL\n");
                 return -1;
             }

             if(_wr_length > MAX_LENGTH_OF_I2C_DATA || 0 > _wr_length) {
                 printf("hal_i2c_ioctl fail, data length exceeds the MAX_LENGTH_OF_I2C_DATA\n");
                 return -1;
             }

             if(_rd_length > MAX_LENGTH_OF_I2C_DATA || 0 > _rd_length) {
                 printf("hal_i2c_ioctl fail, data length exceeds the MAX_LENGTH_OF_I2C_DATA\n");
                 return -1;
             }

             ret = i2c_linux_ioctl(i2c_ioctl_param);
             if(ret < 0) {
                 printf("hal_i2c_ioctl->i2c_linux_ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        default:
             return -1;
        break;
    }

    return ret;
}

int hal_i2c_deinit(unregister_i2c_bus_parameters *_i2c_bus_param)
{
    int ret = 0, _dev_fd;
    char *_i2c_bus_signature = NULL;
    unregister_i2c_bus_parameters *i2c_bus_param = _i2c_bus_param;

    if(!i2c_bus_param) {
        printf("hal_i2c_deinit fail\n");
        return -1;
    }

    _dev_fd = i2c_bus_param->temp.dev_fd;
    _i2c_bus_signature = i2c_bus_param->in.signature;

#ifdef DATA_FLOW_DEBUG
    printf("I2C Data Flow Debug(hal_i2c_deinit): bus %s, dev_fd %d\n", _i2c_bus_signature, _dev_fd);
    return 0;
#endif


    return ret;
}


