/**
* @file     i2c_linux.c
* @brief    i2c_linux.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     01/12/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <pthread.h>

#include "log_controller.h"
#include "i2c_controller.h"
#include "i2c_linux.h"

int i2c_linux_init(register_i2c_bus_parameters *_i2c_bus_param)
{
    int ret = 0, i2c_fd = -1;
    char *_i2c_bus_signature = NULL;
    char path[512] = {0};
    register_i2c_bus_parameters *i2c_bus_param = _i2c_bus_param;

    _i2c_bus_signature = i2c_bus_param->in.signature;

    if(!strcmp(_i2c_bus_signature, I2C_BUS_1_SIGNATURE)) {
        snprintf(path, 255, "/dev/%s", I2C_BUS_1_SIGNATURE);
        i2c_fd = open(path, O_RDWR);
        if(i2c_fd < 0) {
            printf("i2c_linux_init open %s fail\n", _i2c_bus_signature);
            return -1;
        }
    } else {
        return -1;
    }


    //ioctl(i2c_fd, I2C_TIMEOUT, 1);
    //ioctl(i2c_fd, I2C_RETRIES, 2);

    i2c_bus_param->out.dev_fd = i2c_fd;

    return ret;
}

int i2c_linux_ioctl(i2c_ioctl_parameters *_i2c_ioctl_param)
{
    int ret = 0, _cmd, _dev_fd, i, _wr_length, _rd_length;
    uint8_t _i2c_dev_addr, *_wr_data, *_rd_data;
    struct i2c_msg msg[2];
    struct i2c_rdwr_ioctl_data i2c_ioctl_data;
    i2c_ioctl_parameters *i2c_ioctl_param = _i2c_ioctl_param;

    _cmd = i2c_ioctl_param->cmd;

    switch(_cmd) {

        case I2C_IOCTL_CMD_WRITE_MULTI_BYTES:
             _dev_fd = i2c_ioctl_param->write_multi_bytes_param.in._dev_fd;
             _i2c_dev_addr = *i2c_ioctl_param->write_multi_bytes_param.in._i2c_dev_addr;
             _wr_data = i2c_ioctl_param->write_multi_bytes_param.in._data;
             _wr_length = *i2c_ioctl_param->write_multi_bytes_param.in._length;

             msg[0].len = _wr_length;
             msg[0].addr = _i2c_dev_addr;
             msg[0].flags = 0;
             msg[0].buf = _wr_data;

             i2c_ioctl_data.msgs = msg;
             i2c_ioctl_data.nmsgs = 1;

             ret = ioctl(_dev_fd, I2C_RDWR, &i2c_ioctl_data);
             if(ret < 0) {
                 printf("i2c_linux_ioctl fail, write_multi_bytes\n");
                 return -1;
             }

             return 0;
        break;

        case I2C_IOCTL_CMD_READ_MULTI_BYTES:
             _dev_fd = i2c_ioctl_param->read_multi_bytes_param.in._dev_fd;
             _i2c_dev_addr = *i2c_ioctl_param->read_multi_bytes_param.in._i2c_dev_addr;
             _rd_data = i2c_ioctl_param->read_multi_bytes_param.out._rd_data;
             _rd_length = *i2c_ioctl_param->read_multi_bytes_param.in._rd_length;
             _wr_data = i2c_ioctl_param->read_multi_bytes_param.in._wr_data;
             _wr_length = *i2c_ioctl_param->read_multi_bytes_param.in._wr_length;
             
#if 1
             msg[0].addr = _i2c_dev_addr;
             msg[0].flags = 0;
             msg[0].len = _wr_length;             
             msg[0].buf = _wr_data;

             msg[1].addr = _i2c_dev_addr;
             //msg[1].flags = I2C_M_RD | I2C_M_NOSTART;
             msg[1].flags = I2C_M_RD;
             msg[1].len = _rd_length;             
             msg[1].buf = _rd_data;
#endif

            _rd_data[0] = 0;
             


             printf("_i2c_dev_addr: 0x%x\n", _i2c_dev_addr);
             printf("_rd_length %d _wr_length %d\n", _rd_length, _wr_length);
             printf("_wr_data[0]: %d\n", _wr_data[0]);

             i2c_ioctl_data.msgs = msg;
             i2c_ioctl_data.nmsgs = 2;

             ret = ioctl(_dev_fd, I2C_RDWR, &i2c_ioctl_data);
             if(ret < 0) {
                 printf("i2c_linux_ioctl fail, read_multi_bytes\n");
                 return -1;
             }
             
             return 0;
        break;

        default:
             printf("i2c_linux_ioctl fail, unknown cmd %d\n", _cmd);
             return -1;
        break;

    }

    return ret;
}

int i2c_linux_deinit(unregister_i2c_bus_parameters *i2c_bus_param)
{
    int ret = 0, _dev_fd;
    unregister_i2c_bus_parameters *_i2c_bus_param = i2c_bus_param;

    if(!_i2c_bus_param) {
        printf("i2c_linux_deinit fail, incorrect parameters 1\n");
        return -1;
    }

    _dev_fd = _i2c_bus_param->temp.dev_fd;

    if(_dev_fd < 0) {
        printf("i2c_linux_deinit fail, incorrect parameters 2\n");
        return -1;
    }

    close(_dev_fd);

    return ret;
}



