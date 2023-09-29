/**
* @file     configuration_logic.c
* @brief    configuration_logic.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     12/12/2018
*/

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <time.h>

#include "channel.h"
#include "log_controller.h"
#include "tcp.h"
#include "registers.h"
#include "modbus_tcp.h"
#include "spi_controller.h"
#include "i2c_controller.h"
#include "gpio_controller.h"
#include "dac_controller.h"
#include "io_expander_controller.h"
#include "adc_controller.h"
#include "ssd1306_oled_controller.h"
#include "manager.h"
#include "modbus_tcp.h"
#include "macros.h"
#include "platform_endian.h"

#include "register_offset.h"
#include "register_values.h"
#include "error_no.h"

#include "logical_channel.h"

int configuration_reset(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Data_Model *data_model = NULL;
    uint16_t temp1, temp2, temp3;
    int ret = 0, logical_channel_id = channel_id, i;

    if(!manager || logical_channel_id < LOGICAL_CHANNEL_1_ID || logical_channel_id >= NB_LOGIC_CHANNEL) {
        printf("do_formation_logic fail, incorrect parameters, channel %d\n", logical_channel_id);
        return -1;
    }

    data_model = manager->modbus_ctx._register.data_model;

    reset_one_logical_channel(manager, logical_channel_id);

    reset_step_parameter_register(manager, logical_channel_id);

    return ret;
}


