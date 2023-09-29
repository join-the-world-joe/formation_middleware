/**
* @file     eeprom_24c0x.c
* @brief    eeprom_24c0x.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     12/11/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>


#include "log_controller.h"
#include "eeprom_controller.h"
#include "hal_eeprom.h"

#include "debug.h"

#ifdef DATA_FLOW_DEBUG

#include "i2c_controller.h"

#endif

int eeprom_24c0x_init(register_eeprom_parameters *eeprom_param)
{
    printf("eeprom_24c0x_init \n");
    return 0;
}

int eeprom_24c0x_deinit(unregister_eeprom_parameters *eeprom_param)
{
    printf("eeprom_24c0x_deinit\n");
    return 0;
}


