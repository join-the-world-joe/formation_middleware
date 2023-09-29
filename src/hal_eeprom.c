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

#include "log_controller.h"
#include "eeprom_controller.h"
#include "hal_eeprom.h"
#include "eeprom_24c0x.h"

#include "debug.h"

#ifdef DATA_FLOW_DEBUG
#include "i2c_controller.h"
#endif

int hal_eeprom_init(register_eeprom_parameters *eeprom_param)
{
    int ret, length, _nb_bytes = MAX_MEMORY_SIZE_IN_BYTES_OF_EEPROM;
    char *_eeprom_signature;
    void *_i2c_controller_ctx, *_i2c_controller_ioctl_param;
    register_eeprom_parameters *_eeprom_param = eeprom_param;

    if(!_eeprom_param) {
        printf("hal_eeprom_init fail, incorrect parameters 1\n");
        return -1;
    }

    _nb_bytes = _eeprom_param->in.nb_bytes;
    _eeprom_signature = _eeprom_param->in.signature;
    _i2c_controller_ctx = _eeprom_param->in.i2c_controller_ctx;
    _i2c_controller_ioctl_param = _eeprom_param->in.i2c_controller_ioctl_param;
    
    if(!_i2c_controller_ctx) {
        printf("hal_eeprom_init fail, incorrect parameters 2\n");
        return -1;
    }

    if(!_i2c_controller_ioctl_param) {
        printf("hal_eeprom_init fail, incorrect parameters 3\n");
        return -1;
    }

    if(!_eeprom_signature) {
        printf("hal_eeprom_init fail, incorrect parameters 3\n");
        return -1;
    }

    length = strlen(_eeprom_signature);
    if(length == 0 || length >= MAX_LENGTH_OF_EEPROM_SIGNATURE ) {
        printf("hal_eeprom_init fail, incorrect parameters 4\n");
        return -1;
    }

    if(_nb_bytes > MAX_MEMORY_SIZE_IN_BYTES_OF_EEPROM) {
        printf("hal_eeprom_init fail, incorrect parameters 5\n");
        return -1;
    }

#ifdef DATA_FLOW_DEBUG

    if(!strcmp(_eeprom_signature, EEPROM_SIGNATURE_24C0X_U6)) {

        printf("EEPROM Data Flow Debug: signature %s\n", _eeprom_signature);
        
        _eeprom_param->out.dev_addr = EEPROM_SIGNATURE_24C0X_U6_I2C_SLAVE_ADDR;

        return 0;
    } else {
        printf("EEPROM Data Flow Debug: unknow signature %s\n", _eeprom_signature);
        return -1;
    }

#endif

    ret = eeprom_24c0x_init(_eeprom_param);
    if(ret < 0) {
        printf("eeprom_24c0x_init fail\n");
        return -1;
    }

    return 0;
}

int hal_eeprom_ioctl(eeprom_ioctl_parameters *eeprom_param)
{

    
    return 0;
}


int hal_eeprom_deinit(unregister_eeprom_parameters *eeprom_param)
{
    char *_eeprom_signature = NULL;
    int ret = 0;
    unregister_eeprom_parameters *_eeprom_param = eeprom_param;
  
    if(!_eeprom_param) {
        printf("hal_eeprom_deinit incorrect parameters\n");
        return -1;
    }
    
#ifdef DATA_FLOW_DEBUG
    _eeprom_signature = _eeprom_param->in.signature;
    printf("DATA_FLOW_DEBUG(EEPROM): hal_eeprom_deinit, signature = %s\n", _eeprom_signature);
    return 0;
#endif

    ret = eeprom_24c0x_deinit(_eeprom_param);
    if(ret < 0) {
        printf("hal_eeprom_deinit->eeprom_24c0x_deinit fail\n");
        return -1;
    }

    return ret;
}



