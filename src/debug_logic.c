
/**
* @file     debug_logic.c
* @brief    debug_logic.c
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
#include "spi_controller.h"
#include "i2c_controller.h"
#include "gpio_controller.h"
#include "dac_controller.h"
#include "io_expander_controller.h"
#include "adc_controller.h"
#include "ssd1306_oled_controller.h"
#include "registers.h"
#include "modbus_tcp.h"

#include "manager.h"
#include "logical_channel.h"
#include "physical_channel.h"
#include "step_timming_parameters.h"
#include "registers.h"


#include "modbus_tcp.h"
#include "macros.h"
#include "platform_endian.h"

#include "register_offset.h"
#include "register_values.h"
#include "error_no.h"

#include "debug_logic.h"

void debug_reset(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    uint16_t temp1, temp2, temp3;
    int ret = 0, logical_channel_id = channel_id, i;

    if(!manager || logical_channel_id < LOGICAL_CHANNEL_1_ID || logical_channel_id >= NB_LOGIC_CHANNEL) {
        printf("do_formation_logic fail, incorrect parameters, channel %d\n", logical_channel_id);
    }

    reset_all_logical_channels(manager);
    reset_all_physical_channel_to_default_state(manager);
    reset_step_parameter_register(manager, logical_channel_id);

}

int do_debug_logic(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    double temp;
    int ret = 0, physical_channel_id, cmd;
    uint16_t *vi, *vu, *kz_h, *kz_m, *kz_l, *jc, *cf, *ck, *cled, *fled, *bell, *rdry;
    uint16_t *last_vi, *last_vu, *last_kz_h, *last_kz_m, *last_kz_l;
    uint16_t *last_jc, *last_cf, *last_ck, *last_cled, *last_fled, *last_bell, *last_rdry;
    char *_da_terminal = NULL, *_io_expander_terminal = NULL;
    char format[8][15] = {"KZ_H%d", "KZ_M%d", "KZ_L%d", "JC%d", "CF%d", "CK%d", "CLED%d", "FLED%d"};
    dac_controller_context *_dac_controller_ctx = NULL;
    io_expander_controller_context *_io_expander_controller_ctx = NULL;
    dac_controller_ioctl_parameters *_dac_controller_ioctl_param = NULL;
    io_expander_controller_ioctl_parameters *_io_expander_controller_ioctl_param = NULL;


    // so far, do nothing

    //_register = &manager->modbus_ctx._register;

    
    return 0;
}


