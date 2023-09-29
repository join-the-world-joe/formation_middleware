/**
* @file     formation_logic.c
* @brief    formation_logic.c
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
#include <inttypes.h>  

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

#include "misc.h"

static int check_general_parameters(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    _4paired_register_region *accumulated_logic_capacity_region, *last_accumulated_logic_capacity_sampling_time_region;
    uint64_t accumulated_logic_capacity_unsign;
    uint64_t last_accumulated_logic_capacity_sampling_time_unsign;

    _register = &manager->modbus_ctx._register;

    accumulated_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_logic_capacity_region;
    last_accumulated_logic_capacity_sampling_time_region =  &_register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region;
    
    accumulated_logic_capacity_unsign = combine_4paired_register_region(accumulated_logic_capacity_region);
    last_accumulated_logic_capacity_sampling_time_unsign = combine_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region);

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    // check step counter register to determine the process index
    if(*_register->logical_channel[logical_channel_id].step_pointer != 0) {
        return ERROR_STEP_POINTER_NONZERO;
    }

    // check if step quantity is correct
    if(*_register->logical_channel[logical_channel_id].step_quantity == 0) {
        return ERROR_STEP_QUANTITY_ZERO;
    }

    if(*_register->logical_channel[logical_channel_id].step_quantity > MAX_STEP_QUANTITY) {
        return ERROR_STEP_QUANTITY_EXCESS;
    }

    if(accumulated_logic_capacity_unsign != 0) {
        return ERROR_ACCUMULATED_LOGIC_CAPACITY_NONZERO;
    }

    if(last_accumulated_logic_capacity_sampling_time_unsign != 0) {
        return ERROR_LAST_ACCUMULATED_LOGIC_CAPACITY_SAMPLING_TIME_NONZERO;
    }

    return 0;
}

void init_REST_registers(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint64_t sampling_interval;

    _4paired_register_region *accumulated_step_logic_capacity_region, *last_accumulated_logic_capacity_sampling_time_region;
    _4paired_register_region *holding_time_reference_region, *holding_time_region, *sampling_interval_region;

    _register = &manager->modbus_ctx._register;

    // set step holding time reference to zero
    holding_time_reference_region = &_register->logical_channel[logical_channel_id].temp.holding_time_reference_region;
    update_4paired_register_region(holding_time_reference_region, 0);

    
    // set step holding time to zero
    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    update_4paired_register_region(holding_time_region, 0);

    // set step accumulated capacity to zero
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;
    update_4paired_register_region(accumulated_step_logic_capacity_region, 0);

    // set last accumulated  capacity sampling time to zero
    last_accumulated_logic_capacity_sampling_time_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region;
    update_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region, 0);

    // check sampling interval
    sampling_interval_region = &_register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region;
    sampling_interval = combine_4paired_register_region(sampling_interval_region);
    if(sampling_interval == 0) {
        update_4paired_register_region(sampling_interval_region, DEFAULT_STEP_INFO_COMMON_SAMPLING_INTERVAL_IN_MS);
    }

}

void init_CRD_registers(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;

    _4paired_register_region *accumulated_step_logic_capacity_region, *last_accumulated_logic_capacity_sampling_time_region;
    _4paired_register_region *holding_time_reference_region, *holding_time_region, *sampling_interval_region;
    uint16_t *calc_accumulated_capacity_time_interval_reg;
    uint64_t sampling_interval;

    _register = &manager->modbus_ctx._register;

    // set step holding time reference to zero
    holding_time_reference_region = &_register->logical_channel[logical_channel_id].temp.holding_time_reference_region;
    update_4paired_register_region(holding_time_reference_region, 0);

    
    // set step holding time to zero
    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    update_4paired_register_region(holding_time_region, 0);

    // set step accumulated capacity to zero
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;
    update_4paired_register_region(accumulated_step_logic_capacity_region, 0);

    // set last accumulated  capacity sampling time to zero
    last_accumulated_logic_capacity_sampling_time_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region;
    update_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region, 0);

    // set last accumulated  capacity sampling time to zero
    last_accumulated_logic_capacity_sampling_time_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region;
    update_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region, 0);

    // check calc accumulated capacity time inter
    calc_accumulated_capacity_time_interval_reg = _register->logical_channel[logical_channel_id].timing.calc_accumulated_capacity_time_interval;
    if(*calc_accumulated_capacity_time_interval_reg == 0) {
        *calc_accumulated_capacity_time_interval_reg = DEFAULT_CALC_ACCUMULATED_CAPACITY_TIME_INTERVAL_IN_MS;
    };

    // check sampling interval
    sampling_interval_region = &_register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region;
    sampling_interval = combine_4paired_register_region(sampling_interval_region);
    if(sampling_interval == 0) {
        update_4paired_register_region(sampling_interval_region, DEFAULT_STEP_INFO_COMMON_SAMPLING_INTERVAL_IN_MS);
    }

}

void init_CPC_registers(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;

    _4paired_register_region *accumulated_step_logic_capacity_region, *last_accumulated_logic_capacity_sampling_time_region;
    _4paired_register_region *holding_time_reference_region, *holding_time_region, *sampling_interval_region;
    uint16_t *calc_accumulated_capacity_time_interval_reg;
    uint64_t sampling_interval;

    _register = &manager->modbus_ctx._register;

    // set step holding time reference to zero
    holding_time_reference_region = &_register->logical_channel[logical_channel_id].temp.holding_time_reference_region;
    update_4paired_register_region(holding_time_reference_region, 0);

    
    // set step holding time to zero
    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    update_4paired_register_region(holding_time_region, 0);

    // set step accumulated capacity to zero
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;
    update_4paired_register_region(accumulated_step_logic_capacity_region, 0);

    // set step accumulated capacity to zero
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;
    update_4paired_register_region(accumulated_step_logic_capacity_region, 0);

    // set last accumulated  capacity sampling time to zero
    last_accumulated_logic_capacity_sampling_time_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region;
    update_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region, 0);

    // check calc accumulated capacity time inter
    calc_accumulated_capacity_time_interval_reg = _register->logical_channel[logical_channel_id].timing.calc_accumulated_capacity_time_interval;
    if(*calc_accumulated_capacity_time_interval_reg == 0) {
        *calc_accumulated_capacity_time_interval_reg = DEFAULT_CALC_ACCUMULATED_CAPACITY_TIME_INTERVAL_IN_MS;
    };

    // check sampling interval
    sampling_interval_region = &_register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region;
    sampling_interval = combine_4paired_register_region(sampling_interval_region);
    if(sampling_interval == 0) {
        update_4paired_register_region(sampling_interval_region, DEFAULT_STEP_INFO_COMMON_SAMPLING_INTERVAL_IN_MS);
    }

}


void init_CPD_registers(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint16_t *calc_accumulated_capacity_time_interval_reg;
    uint64_t sampling_interval;

    _4paired_register_region *accumulated_step_logic_capacity_region, *last_accumulated_logic_capacity_sampling_time_region;
    _4paired_register_region *holding_time_reference_region, *holding_time_region, *sampling_interval_region;

    _register = &manager->modbus_ctx._register;

    // set step holding time reference to zero
    holding_time_reference_region = &_register->logical_channel[logical_channel_id].temp.holding_time_reference_region;
    update_4paired_register_region(holding_time_reference_region, 0);

    
    // set step holding time to zero
    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    update_4paired_register_region(holding_time_region, 0);

    // set step accumulated capacity to zero
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;
    update_4paired_register_region(accumulated_step_logic_capacity_region, 0);

    // set last accumulated  capacity sampling time to zero
    last_accumulated_logic_capacity_sampling_time_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region;
    update_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region, 0);

    // check calc accumulated capacity time inter
    calc_accumulated_capacity_time_interval_reg = _register->logical_channel[logical_channel_id].timing.calc_accumulated_capacity_time_interval;
    if(*calc_accumulated_capacity_time_interval_reg == 0) {
        *calc_accumulated_capacity_time_interval_reg = DEFAULT_CALC_ACCUMULATED_CAPACITY_TIME_INTERVAL_IN_MS;
    };

    // check sampling interval
    sampling_interval_region = &_register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region;
    sampling_interval = combine_4paired_register_region(sampling_interval_region);
    if(sampling_interval == 0) {
        update_4paired_register_region(sampling_interval_region, DEFAULT_STEP_INFO_COMMON_SAMPLING_INTERVAL_IN_MS);
    }

}

void init_CCDCVD_registers(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint16_t *calc_accumulated_capacity_time_interval_reg;
    uint64_t sampling_interval;

    _4paired_register_region *accumulated_step_logic_capacity_region, *last_accumulated_logic_capacity_sampling_time_region;
    _4paired_register_region *holding_time_reference_region, *holding_time_region, *sampling_interval_region;

    _register = &manager->modbus_ctx._register;

    // set step holding time reference to zero
    holding_time_reference_region = &_register->logical_channel[logical_channel_id].temp.holding_time_reference_region;
    update_4paired_register_region(holding_time_reference_region, 0);

    
    // set step holding time to zero
    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    update_4paired_register_region(holding_time_region, 0);

    // set step accumulated capacity to zero
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;
    update_4paired_register_region(accumulated_step_logic_capacity_region, 0);

    // set last accumulated  capacity sampling time to zero
    last_accumulated_logic_capacity_sampling_time_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region;
    update_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region, 0);

    // check calc accumulated capacity time inter
    calc_accumulated_capacity_time_interval_reg = _register->logical_channel[logical_channel_id].timing.calc_accumulated_capacity_time_interval;
    if(*calc_accumulated_capacity_time_interval_reg == 0) {
        *calc_accumulated_capacity_time_interval_reg = DEFAULT_CALC_ACCUMULATED_CAPACITY_TIME_INTERVAL_IN_MS;
    };

    // check sampling interval
    sampling_interval_region = &_register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region;
    sampling_interval = combine_4paired_register_region(sampling_interval_region);
    if(sampling_interval == 0) {
        update_4paired_register_region(sampling_interval_region, DEFAULT_STEP_INFO_COMMON_SAMPLING_INTERVAL_IN_MS);
    }

}

void init_CCCCVC_registers(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint16_t *calc_accumulated_capacity_time_interval_reg;
    uint64_t sampling_interval;

    _4paired_register_region *accumulated_step_logic_capacity_region, *last_accumulated_logic_capacity_sampling_time_region;
    _4paired_register_region *holding_time_reference_region, *holding_time_region, *sampling_interval_region;

    _register = &manager->modbus_ctx._register;

    // set step holding time reference to zero
    holding_time_reference_region = &_register->logical_channel[logical_channel_id].temp.holding_time_reference_region;
    update_4paired_register_region(holding_time_reference_region, 0);

    
    // set step holding time to zero
    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    update_4paired_register_region(holding_time_region, 0);

    // set step accumulated capacity to zero
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;
    update_4paired_register_region(accumulated_step_logic_capacity_region, 0);

    // set last accumulated  capacity sampling time to zero
    last_accumulated_logic_capacity_sampling_time_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region;
    update_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region, 0);

    // check calc accumulated capacity time inter
    calc_accumulated_capacity_time_interval_reg = _register->logical_channel[logical_channel_id].timing.calc_accumulated_capacity_time_interval;
    if(*calc_accumulated_capacity_time_interval_reg == 0) {
        *calc_accumulated_capacity_time_interval_reg = DEFAULT_CALC_ACCUMULATED_CAPACITY_TIME_INTERVAL_IN_MS;
    };

    // check sampling interval
    sampling_interval_region = &_register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region;
    sampling_interval = combine_4paired_register_region(sampling_interval_region);
    if(sampling_interval == 0) {
        update_4paired_register_region(sampling_interval_region, DEFAULT_STEP_INFO_COMMON_SAMPLING_INTERVAL_IN_MS);
    }

}

void init_CCD_registers(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint16_t *calc_accumulated_capacity_time_interval_reg;
    uint64_t sampling_interval;

    _4paired_register_region *accumulated_step_logic_capacity_region, *last_accumulated_logic_capacity_sampling_time_region;
    _4paired_register_region *holding_time_reference_region, *holding_time_region, *sampling_interval_region;

    _register = &manager->modbus_ctx._register;

    // set step holding time reference to zero
    holding_time_reference_region = &_register->logical_channel[logical_channel_id].temp.holding_time_reference_region;
    update_4paired_register_region(holding_time_reference_region, 0);

    
    // set step holding time to zero
    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    update_4paired_register_region(holding_time_region, 0);

    // set step accumulated capacity to zero
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;
    update_4paired_register_region(accumulated_step_logic_capacity_region, 0);

    // set last accumulated  capacity sampling time to zero
    last_accumulated_logic_capacity_sampling_time_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region;
    update_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region, 0);

    // check calc accumulated capacity time inter
    calc_accumulated_capacity_time_interval_reg = _register->logical_channel[logical_channel_id].timing.calc_accumulated_capacity_time_interval;
    if(*calc_accumulated_capacity_time_interval_reg == 0) {
        *calc_accumulated_capacity_time_interval_reg = DEFAULT_CALC_ACCUMULATED_CAPACITY_TIME_INTERVAL_IN_MS;
    };

    // check sampling interval
    sampling_interval_region = &_register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region;
    sampling_interval = combine_4paired_register_region(sampling_interval_region);
    if(sampling_interval == 0) {
        update_4paired_register_region(sampling_interval_region, DEFAULT_STEP_INFO_COMMON_SAMPLING_INTERVAL_IN_MS);
    }

}

void init_CCC_registers(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint16_t *calc_accumulated_capacity_time_interval_reg;
    uint64_t sampling_interval;

    _4paired_register_region *accumulated_step_logic_capacity_region, *last_accumulated_logic_capacity_sampling_time_region;
    _4paired_register_region *holding_time_reference_region, *holding_time_region, *sampling_interval_region;

    _register = &manager->modbus_ctx._register;

    // set step holding time reference to zero
    holding_time_reference_region = &_register->logical_channel[logical_channel_id].temp.holding_time_reference_region;
    update_4paired_register_region(holding_time_reference_region, 0);

    
    // set step holding time to zero
    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    update_4paired_register_region(holding_time_region, 0);

    // set step accumulated capacity to zero
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;
    update_4paired_register_region(accumulated_step_logic_capacity_region, 0);

    // set last accumulated  capacity sampling time to zero
    last_accumulated_logic_capacity_sampling_time_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region;
    update_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region, 0);

    // check calc accumulated capacity time inter
    calc_accumulated_capacity_time_interval_reg = _register->logical_channel[logical_channel_id].timing.calc_accumulated_capacity_time_interval;
    if(*calc_accumulated_capacity_time_interval_reg == 0) {
        *calc_accumulated_capacity_time_interval_reg = DEFAULT_CALC_ACCUMULATED_CAPACITY_TIME_INTERVAL_IN_MS;
    };

    // check sampling interval
    sampling_interval_region = &_register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region;
    sampling_interval = combine_4paired_register_region(sampling_interval_region);
    if(sampling_interval == 0) {
        update_4paired_register_region(sampling_interval_region, DEFAULT_STEP_INFO_COMMON_SAMPLING_INTERVAL_IN_MS);
    }

}


int check_REST_parameters(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;

    uint64_t jump_condiction_time;
    _4paired_register_region *jump_condiction_time_region;

    _register = &manager->modbus_ctx._register;

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    // check if jump condiction enable zero
    if(*_register->logical_channel[logical_channel_id].step_param.jump.enable == 0) {
        return ERROR_JUMP_CONDICTION_ENABLE_ZERO;
    }

    if(((*_register->logical_channel[logical_channel_id].step_param.jump.enable) & STEP_JUMP_CONDICTION_TIMEOUT) == 0) {
        return ERROR_JUMP_CONDICTION_TIMEOUT_NOT_SET;
    }

    // check if step time interval is incorrect
    jump_condiction_time_region = &_register->logical_channel[logical_channel_id].step_param.jump.time_region;
    jump_condiction_time = combine_4paired_register_region(jump_condiction_time_region);
    if(jump_condiction_time == 0) {
        return ERROR_JUMP_CONDICTION_TIME_ZERO;
    }

    return 0;
}

int check_CRD_parameters(Manager *_manager, int channel_id)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = channel_id;
    uint64_t amplitude_mask = 0x7fffffffffffffff, logic_voltage, logic_current;
    _4paired_register_region *level_1_region, *level_2_region, *level_3_region, *jump_time_region;
    uint64_t level_1, level_2, level_3, maximum_source_current, common_resistance, jump_time;
    _4paired_register_region *common_resistance_region, *maximum_source_current_region, *logic_voltage_region;
    double temp1, temp2, temp3;

    _register = &manager->modbus_ctx._register;

    level_1_region = &_register->physical_channel[logical_channel_id].source_current_level_1_region;
    level_2_region = &_register->physical_channel[logical_channel_id].source_current_level_2_region;
    level_3_region = &_register->physical_channel[logical_channel_id].source_current_level_3_region;

    level_1 = combine_4paired_register_region(level_1_region);
    level_2 = combine_4paired_register_region(level_2_region);
    level_3 = combine_4paired_register_region(level_3_region);

    jump_time_region = &_register->logical_channel[logical_channel_id].step_param.jump.time_region;
    jump_time = combine_4paired_register_region(jump_time_region);

    logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;
    logic_voltage = combine_4paired_register_region(logic_voltage_region);

    maximum_source_current_region = &_register->logical_channel[logical_channel_id].maximum_source_current_region;
    maximum_source_current = combine_4paired_register_region(maximum_source_current_region);
    
    common_resistance_region = &_register->logical_channel[logical_channel_id].step_param.info.common.resistance_region;
    common_resistance = combine_4paired_register_region(common_resistance_region);

    printf("common resistance word1 0x%x\n", *common_resistance_region->word1);
    printf("common resistance word2 0x%x\n", *common_resistance_region->word2);
    printf("common resistance word3 0x%x\n", *common_resistance_region->word3);
    printf("common resistance word4 0x%x\n", *common_resistance_region->word4);

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    // check if jump time is incorrect
    if(jump_time == 0) {
        return ERROR_JUMP_CONDICTION_TIME_ZERO;
    }

    // check if resistance is zero
    if(common_resistance == 0) {
        return ERROR_CRD_RESISTANCE_ZERO;
    }

    // check if jump condiction is zero
    if(*_register->logical_channel[logical_channel_id].step_param.jump.enable == 0) {
        return ERROR_JUMP_CONDICTION_ENABLE_ZERO;
    }

    // check if jump condiction TIMEOUT is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) == 0) {
        return ERROR_JUMP_CONDICTION_TIMEOUT_NOT_SET;
    }

    // check if jump condiction VOLTAGE is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_VOLTAGE) == 0) {
        return ERROR_JUMP_CONDICTION_VOLTAGE_NOT_SET;
    }

    // check if source current capacity level 1 is zero
    if(level_1 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 1;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 2 is zero
    if(level_2 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 2;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 3 is zero
    if(level_3 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 3;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    temp1 = logic_voltage;
    temp2 = common_resistance;
    temp3 = (temp1) / (temp2);

    logic_current = temp3 * UNIT_FACTOR_BETWEEN_A_AND_01UA;

    printf("temp1: %.15f, temp2:%.15f, temp3: %.15f\n", temp1, temp2, temp3);
    printf("logic_current %" PRIu64 "\n", logic_current);

    if(logic_current > maximum_source_current) {
        printf("logic_current %"PRIu64 "maximum_source_current %"PRIu64"\n", logic_current, maximum_source_current);
        return ERROR_CONSTANT_CURRENT_LIMIT_EXCEEDED;
    }

    return 0;
}

int check_CPC_parameters(Manager *_manager, int channel_id)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = channel_id;
    uint64_t amplitude_mask = 0x7fffffffffffffff, logic_voltage, logic_current;
    _4paired_register_region *level_1_region, *level_2_region, *level_3_region, *jump_time_region;
    uint64_t level_1, level_2, level_3, maximum_source_current, common_power, maximum_power, minimum_power, jump_time;
    _4paired_register_region *common_power_region, *maximum_source_current_region, *logic_voltage_region;
    double temp1, temp2, temp3;

    _register = &manager->modbus_ctx._register;

    level_1_region = &_register->physical_channel[logical_channel_id].source_current_level_1_region;
    level_2_region = &_register->physical_channel[logical_channel_id].source_current_level_2_region;
    level_3_region = &_register->physical_channel[logical_channel_id].source_current_level_3_region;

    level_1 = combine_4paired_register_region(level_1_region);
    level_2 = combine_4paired_register_region(level_2_region);
    level_3 = combine_4paired_register_region(level_3_region);

    jump_time_region = &_register->logical_channel[logical_channel_id].step_param.jump.time_region;
    jump_time = combine_4paired_register_region(jump_time_region);

    logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;
    logic_voltage = combine_4paired_register_region(logic_voltage_region);

    maximum_source_current_region = &_register->logical_channel[logical_channel_id].maximum_source_current_region;
    maximum_source_current = combine_4paired_register_region(maximum_source_current_region);
    
    common_power_region = &_register->logical_channel[logical_channel_id].step_param.info.common.power_region;
    common_power = combine_4paired_register_region(common_power_region);

    //printf("common power word1 0x%x\n", *common_power_region->word1);
    //printf("common power word2 0x%x\n", *common_power_region->word2);
    //printf("common power word3 0x%x\n", *common_power_region->word3);
    //printf("common power word4 0x%x\n", *common_power_region->word4);

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    // check if jump time is incorrect
    if(jump_time == 0) {
        return ERROR_JUMP_CONDICTION_TIME_ZERO;
    }

    // check if jump power is zero
    if(common_power == 0) {
        return ERROR_CPC_POWER_ZERO;
    }

    // check if jump condiction is zero
    if(*_register->logical_channel[logical_channel_id].step_param.jump.enable == 0) {
        return ERROR_JUMP_CONDICTION_ENABLE_ZERO;
    }

    // check if jump condiction TIMEOUT is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) == 0) {
        return ERROR_JUMP_CONDICTION_TIMEOUT_NOT_SET;
    }

    // check if jump condiction VOLTAGE is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_VOLTAGE) == 0) {
        return ERROR_JUMP_CONDICTION_VOLTAGE_NOT_SET;
    }

    // check if source current capacity level 1 is zero
    if(level_1 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 1;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 2 is zero
    if(level_2 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 2;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 3 is zero
    if(level_3 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 3;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    //printf("common_power %"PRIu64"\n", common_power);
    if((common_power & amplitude_mask) == 0) {
        printf("discharging current is zero\n");
        return ERROR_CPC_POWER_ZERO;
    }

    // check if power setting is correct
    minimum_power = 0.001 * UNIT_FACTOR_BETWEEN_1MW_AND_10PW;
    //printf("minimum_power %"PRIu64"\n", minimum_power);
    if(common_power < minimum_power) {
        return ERROR_POWER_LIMIT_EXCEEDED;
    }

    // check if power setting is correct
    maximum_power = (uint64_t)25 * (uint64_t)UNIT_FACTOR_BETWEEN_1MW_AND_10PW ; /* 10 mW */
    //printf("maximum_power %"PRIu64"\n", maximum_power);
    if(common_power > maximum_power) {
        printf("Constant power %"PRIu64" maximum_power %"PRIu64"\n", common_power, maximum_power);
        return ERROR_POWER_LIMIT_EXCEEDED;
    }

    //printf("Constant power %"PRIu64" maximum_power %"PRIu64"\n", common_power, maximum_power);

    // check if current setting is proper 
    temp1 = logic_voltage*UNIT_FACTOR_BETWEEN_10NV_AND_10PV;
    temp2 = common_power;
    temp3 = (temp2 / temp1);

    logic_current = temp3 * UNIT_FACTOR_BETWEEN_A_AND_01UA;
 
    //printf("temp1: %.15f, temp2:%.15f, temp3: %.15f\n", temp1, temp2, temp3);

    if(logic_current > maximum_source_current) {
        printf("logic_current %"PRIu64 "maximum_source_current %"PRIu64"\n", logic_current, maximum_source_current);
        return ERROR_CONSTANT_CURRENT_LIMIT_EXCEEDED;
    }

    //printf("logic_current %"PRIu64 "maximum_source_current %"PRIu64"\n", logic_current, maximum_source_current);

    return 0;
}

int check_CPD_parameters(Manager *_manager, int channel_id)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = channel_id;
    uint64_t amplitude_mask = 0x7fffffffffffffff, logic_voltage, logic_current;
    _4paired_register_region *level_1_region, *level_2_region, *level_3_region, *jump_time_region;
    uint64_t level_1, level_2, level_3, maximum_source_current, common_power, maximum_power, minimum_power, jump_time;
    _4paired_register_region *common_power_region, *maximum_source_current_region, *logic_voltage_region;
    double temp1, temp2, temp3;

    _register = &manager->modbus_ctx._register;

    level_1_region = &_register->physical_channel[logical_channel_id].source_current_level_1_region;
    level_2_region = &_register->physical_channel[logical_channel_id].source_current_level_2_region;
    level_3_region = &_register->physical_channel[logical_channel_id].source_current_level_3_region;

    level_1 = combine_4paired_register_region(level_1_region);
    level_2 = combine_4paired_register_region(level_2_region);
    level_3 = combine_4paired_register_region(level_3_region);

    jump_time_region = &_register->logical_channel[logical_channel_id].step_param.jump.time_region;
    jump_time = combine_4paired_register_region(jump_time_region);

    logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;
    logic_voltage = combine_4paired_register_region(logic_voltage_region);

    maximum_source_current_region = &_register->logical_channel[logical_channel_id].maximum_source_current_region;
    maximum_source_current = combine_4paired_register_region(maximum_source_current_region);
    
    common_power_region = &_register->logical_channel[logical_channel_id].step_param.info.common.power_region;
    common_power = combine_4paired_register_region(common_power_region);

    //printf("common power word1 0x%x\n", *common_power_region->word1);
    //printf("common power word2 0x%x\n", *common_power_region->word2);
    //printf("common power word3 0x%x\n", *common_power_region->word3);
    //printf("common power word4 0x%x\n", *common_power_region->word4);

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    // check if jump time is incorrect
    if(jump_time == 0) {
        return ERROR_JUMP_CONDICTION_TIME_ZERO;
    }

    // check if jump power is zero
    if(common_power == 0) {
        return ERROR_CPC_POWER_ZERO;
    }

    // check if jump condiction is zero
    if(*_register->logical_channel[logical_channel_id].step_param.jump.enable == 0) {
        return ERROR_JUMP_CONDICTION_ENABLE_ZERO;
    }

    // check if jump condiction TIMEOUT is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) == 0) {
        return ERROR_JUMP_CONDICTION_TIMEOUT_NOT_SET;
    }

    // check if jump condiction VOLTAGE is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_VOLTAGE) == 0) {
        return ERROR_JUMP_CONDICTION_VOLTAGE_NOT_SET;
    }

    // check if source current capacity level 1 is zero
    if(level_1 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 1;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 2 is zero
    if(level_2 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 2;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 3 is zero
    if(level_3 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 3;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    //printf("common_power %"PRIu64"\n", common_power);
    if((common_power & amplitude_mask) == 0) {
        printf("discharging current is zero\n");
        return ERROR_CPD_POWER_ZERO;
    }

    // check if power setting is correct
    minimum_power = 0.001 * UNIT_FACTOR_BETWEEN_1MW_AND_10PW;
    //printf("minimum_power %"PRIu64"\n", minimum_power);
    if(common_power < minimum_power) {
        return ERROR_POWER_LIMIT_EXCEEDED;
    }

    // check if power setting is correct
    maximum_power = (uint64_t)25 * (uint64_t)UNIT_FACTOR_BETWEEN_1MW_AND_10PW ; /* 10 mW */
    //printf("maximum_power %"PRIu64"\n", maximum_power);
    if(common_power > maximum_power) {
        printf("Constant power %"PRIu64" maximum_power %"PRIu64"\n", common_power, maximum_power);
        return ERROR_POWER_LIMIT_EXCEEDED;
    }

    //printf("Constant power %"PRIu64" maximum_power %"PRIu64"\n", common_power, maximum_power);

    // check if current setting is proper 
    temp1 = logic_voltage*UNIT_FACTOR_BETWEEN_10NV_AND_10PV;
    temp2 = common_power;
    temp3 = (temp2 / temp1);

    logic_current = temp3 * UNIT_FACTOR_BETWEEN_A_AND_01UA;
 
    //printf("temp1: %.15f, temp2:%.15f, temp3: %.15f\n", temp1, temp2, temp3);

    if(logic_current > maximum_source_current) {
        printf("logic_current %"PRIu64 "maximum_source_current %"PRIu64"\n", logic_current, maximum_source_current);
        return ERROR_CONSTANT_CURRENT_LIMIT_EXCEEDED;
    }

    //printf("logic_current %"PRIu64 "maximum_source_current %"PRIu64"\n", logic_current, maximum_source_current);

    return 0;
}


int check_CCDCVD_parameters(Manager *_manager, int channel_id)
{
    int logical_channel_id = channel_id;
    Manager *manager = _manager;
    Register *_register = NULL;
    uint64_t jump_current, amplitude_mask = 0x7fffffffffffffff;
    _4paired_register_region *level_1_region, *level_2_region, *level_3_region, *jump_time_region;
    uint64_t level_1, level_2, level_3, maximum_source_current, common_current, jump_time;
    _4paired_register_region *jump_current_region, *common_current_region, *maximum_source_current_region;

    _register = &manager->modbus_ctx._register;

    level_1_region = &_register->physical_channel[logical_channel_id].source_current_level_1_region;
    level_2_region = &_register->physical_channel[logical_channel_id].source_current_level_2_region;
    level_3_region = &_register->physical_channel[logical_channel_id].source_current_level_3_region;
    
    level_1 = combine_4paired_register_region(level_1_region);
    level_2 = combine_4paired_register_region(level_2_region);
    level_3 = combine_4paired_register_region(level_3_region);

    jump_time_region = &_register->logical_channel[logical_channel_id].step_param.jump.time_region;
    jump_time = combine_4paired_register_region(jump_time_region);

    maximum_source_current_region = &_register->logical_channel[logical_channel_id].maximum_source_current_region;
    maximum_source_current = combine_4paired_register_region(maximum_source_current_region);

    common_current_region = &_register->logical_channel[logical_channel_id].step_param.info.common.current_region;
    common_current = combine_4paired_register_region(common_current_region);

    jump_current_region = &_register->logical_channel[logical_channel_id].step_param.jump.current_region;
    jump_current = combine_4paired_register_region(jump_current_region);

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    // check if jump time is incorrect
    if(jump_time == 0) {
        return ERROR_JUMP_CONDICTION_TIME_ZERO;
    }  

    // check if jump current is zero
    if(jump_current == 0) {
        return ERROR_JUMP_CONDICTION_CURRENT_ZERO;
    }   
    
    // check if jump condiction is zero
    if(*_register->logical_channel[logical_channel_id].step_param.jump.enable == 0) {
        return ERROR_JUMP_CONDICTION_ENABLE_ZERO;
    }

    // check if jump condiction current is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_CURRENT) == 0) {
        return ERROR_JUMP_CONDICTION_CURRENT_NOT_SET;
    }

    // check if jump condiction TIMEOUT is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) == 0) {
        return ERROR_JUMP_CONDICTION_TIMEOUT_NOT_SET;
    }

    // check if source current capacity level 1 is zero
    if(level_1 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 1;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 2 is zero
    if(level_2 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 2;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 3 is zero
    if(level_3 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 3;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if charging current is zero
    if((common_current & amplitude_mask) == 0) {
        printf("discharging current is zero\n");
        return ERROR_CCDCVD_CURRENT_ZERO;
    }

    // check if current setting is correct
    if((common_current & amplitude_mask) > maximum_source_current) {
        printf("Constant current 0x%"PRIu64" max_source_current %"PRIu64"\n", common_current, maximum_source_current);
        return ERROR_CONSTANT_CURRENT_LIMIT_EXCEEDED;
    }

    return 0;
}

int check_CCCCVC_parameters(Manager *_manager, int channel_id)
{
    int logical_channel_id = channel_id;
    Manager *manager = _manager;
    Register *_register = NULL;
    uint64_t jump_current;
    uint64_t amplitude_mask = 0x7fffffffffffffff;
    _4paired_register_region *jump_current_region, *common_current_region, *common_voltage_region, *jump_time_region;
    uint64_t level_1, level_2, level_3, maximum_source_current, common_current, common_voltage, jump_time;
    _4paired_register_region *level_1_region, *level_2_region, *level_3_region, *maximum_source_current_region;

    _register = &manager->modbus_ctx._register;

    level_1_region = &_register->physical_channel[logical_channel_id].source_current_level_1_region;
    level_2_region = &_register->physical_channel[logical_channel_id].source_current_level_2_region;
    level_3_region = &_register->physical_channel[logical_channel_id].source_current_level_3_region;
    
    level_1 = combine_4paired_register_region(level_1_region);
    level_2 = combine_4paired_register_region(level_2_region);
    level_3 = combine_4paired_register_region(level_3_region);

    jump_time_region = &_register->logical_channel[logical_channel_id].step_param.jump.time_region;
    jump_time = combine_4paired_register_region(jump_time_region);

    common_current_region = &_register->logical_channel[logical_channel_id].step_param.info.common.current_region;
    common_current = combine_4paired_register_region(common_current_region);

    common_voltage_region = &_register->logical_channel[logical_channel_id].step_param.info.common.voltage_region;
    common_voltage = combine_4paired_register_region(common_voltage_region);

    maximum_source_current_region =  &_register->logical_channel[logical_channel_id].maximum_source_current_region;
    maximum_source_current = combine_4paired_register_region(maximum_source_current_region);

    jump_current_region = &_register->logical_channel[logical_channel_id].step_param.jump.current_region;

    jump_current = combine_4paired_register_region(jump_current_region);

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    // check if jump time is incorrect
    if(jump_time == 0) {
        return ERROR_JUMP_CONDICTION_TIME_ZERO;
    }  

    // check if jump current is zero
    if(jump_current == 0) {
        return ERROR_JUMP_CONDICTION_CURRENT_ZERO;
    }   
    
    // check if jump voltage is zero
    if(common_voltage == 0) {
        return ERROR_JUMP_CONDICTION_VOLTAGE_ZERO;
    }

    // check if jump condiction is zero
    if(*_register->logical_channel[logical_channel_id].step_param.jump.enable == 0) {
        return ERROR_JUMP_CONDICTION_ENABLE_ZERO;
    }

    // check if jump condiction current is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_CURRENT) == 0) {
        return ERROR_JUMP_CONDICTION_CURRENT_NOT_SET;
    }

    // check if jump condiction TIMEOUT is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) == 0) {
        return ERROR_JUMP_CONDICTION_TIMEOUT_NOT_SET;
    }

    // check if source current capacity level 1 is zero
    if(level_1 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 1;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 2 is zero
    if(level_2 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 2;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 3 is zero
    if(level_3 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 3;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if charging current is zero
    if((common_current & amplitude_mask) == 0) {
        printf("charging current is zero\n");
        return ERROR_CCCCVC_CURRENT_ZERO;
    }

    // check if current setting is correct
    if((common_current & amplitude_mask) > maximum_source_current) {
        printf("Constant current %"PRIu64" max_source_current %"PRIu64"\n", common_current, maximum_source_current);
        return ERROR_CONSTANT_CURRENT_LIMIT_EXCEEDED;
    }

    return 0;
}

int check_CCD_parameters(Manager *_manager, int channel_id)
{
    int logical_channel_id = channel_id;
    Manager *manager = _manager;
    Register *_register = NULL;
    uint16_t *_cc = NULL, *_max_source_current = NULL;
    _4paired_register_region *jump_voltage_region;
    _4paired_register_region *level_1_region, *level_2_region, *level_3_region, *jump_time_region;
    uint64_t level_1, level_2, level_3, max_current, jump_time;

    _4paired_register_region *common_current_region, *maximum_source_current_region;
    uint64_t common_current, maximum_source_current, jump_voltage;

    _register = &manager->modbus_ctx._register;

    level_1_region = &_register->physical_channel[logical_channel_id].source_current_level_1_region;
    level_2_region = &_register->physical_channel[logical_channel_id].source_current_level_2_region;
    level_3_region = &_register->physical_channel[logical_channel_id].source_current_level_3_region;
    
    level_1 = combine_4paired_register_region(level_1_region);
    level_2 = combine_4paired_register_region(level_2_region);
    level_3 = combine_4paired_register_region(level_3_region);

    jump_time_region = &_register->logical_channel[logical_channel_id].step_param.jump.time_region;
    jump_time = combine_4paired_register_region(jump_time_region);

    //_cc = _register->logical_channel[logical_channel_id].step_param.info.common.current;
    common_current_region = &_register->logical_channel[logical_channel_id].step_param.info.common.current_region;
    common_current = combine_4paired_register_region(common_current_region);

    //_max_source_current = _register->logical_channel[logical_channel_id].maximum_source_current;
    maximum_source_current_region = &_register->logical_channel[logical_channel_id].maximum_source_current_region;
    maximum_source_current = combine_4paired_register_region(maximum_source_current_region);

    //_jc_voltage = _register->logical_channel[logical_channel_id].step_param.jump.voltage; /* lower limit voltage */
    jump_voltage_region = &_register->logical_channel[logical_channel_id].step_param.jump.voltage_region;
    jump_voltage = combine_4paired_register_region(jump_voltage_region);

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    // check if jump time is incorrect
    if(jump_time == 0) {
        return ERROR_JUMP_CONDICTION_TIME_ZERO;
    }   

    // check if jump voltag is zero
    if(jump_voltage == 0) {
        return ERROR_JUMP_CONDICTION_VOLTAGE_ZERO;
    }          

    // check if jump condiction is zero
    if(*_register->logical_channel[logical_channel_id].step_param.jump.enable == 0) {
        return ERROR_JUMP_CONDICTION_ENABLE_ZERO;
    }

    // check if jump condiction VOLTAGE is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_VOLTAGE) == 0) {
        return ERROR_JUMP_CONDICTION_VOLTAGE_NOT_SET;
    }

    // check if jump condiction TIMEOUT is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) == 0) {
        return ERROR_JUMP_CONDICTION_TIMEOUT_NOT_SET;
    }

    // check if source current capacity level 1 is zero
    if(level_1 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 1;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 2 is zero
    if(level_2 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 2;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 3 is zero
    if(level_3 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 3;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if charging current is zero
    if(common_current == 0) {
        printf("charging current is zero\n");
        return ERROR_CCD_CURRENT_ZERO;
    }

    // check if current setting is correct
    if(common_current > maximum_source_current) {
        //printf("Constant current %PRIu64 max_source_current %PRIu64\n", common_current, maximum_source_current);
        printf("common_current %"PRIu64"\n", common_current);
        printf("maximum_source_current %"PRIu64"\n", maximum_source_current);
        return ERROR_CONSTANT_CURRENT_LIMIT_EXCEEDED;
    }

    return 0;
}


int check_CCC_parameters(Manager *_manager, int channel_id)
{
    int logical_channel_id = channel_id;
    Manager *manager = _manager;
    Register *_register = NULL;
    _4paired_register_region *jump_voltage_region;
    _4paired_register_region *level_1_region, *level_2_region, *level_3_region, *jump_time_region;
    uint64_t level_1, level_2, level_3, jump_time;

    _4paired_register_region *common_current_region, *maximum_source_current_region;
    uint64_t common_current, maximum_source_current, jump_voltage;
  

    _register = &manager->modbus_ctx._register;

    level_1_region = &_register->physical_channel[logical_channel_id].source_current_level_1_region;
    level_2_region = &_register->physical_channel[logical_channel_id].source_current_level_2_region;
    level_3_region = &_register->physical_channel[logical_channel_id].source_current_level_3_region;
    
    level_1 = combine_4paired_register_region(level_1_region);
    level_2 = combine_4paired_register_region(level_2_region);
    level_3 = combine_4paired_register_region(level_3_region);

    jump_time_region = &_register->logical_channel[logical_channel_id].step_param.jump.time_region;
    jump_time = combine_4paired_register_region(jump_time_region);

    //_cc = _register->logical_channel[logical_channel_id].step_param.info.common.current;
    common_current_region = &_register->logical_channel[logical_channel_id].step_param.info.common.current_region;
    common_current = combine_4paired_register_region(common_current_region);

    //_max_source_current = _register->logical_channel[logical_channel_id].maximum_source_current;
    maximum_source_current_region = &_register->logical_channel[logical_channel_id].maximum_source_current_region;
    maximum_source_current = combine_4paired_register_region(maximum_source_current_region);

    //_jc_voltage = _register->logical_channel[logical_channel_id].step_param.jump.voltage; /* upper limit voltage */
    jump_voltage_region = &_register->logical_channel[logical_channel_id].step_param.jump.voltage_region;
    jump_voltage = combine_4paired_register_region(jump_voltage_region);

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    // check if jump time is incorrect
    if(jump_time == 0) {
        return ERROR_JUMP_CONDICTION_TIME_ZERO;
    }   

    // check if jump voltag is zero
    if(jump_voltage == 0) {
        return ERROR_JUMP_CONDICTION_VOLTAGE_ZERO;
    }          

    // check if jump condiction is zero
    if(*_register->logical_channel[logical_channel_id].step_param.jump.enable == 0) {
        return ERROR_JUMP_CONDICTION_ENABLE_ZERO;
    }

    // check if jump condiction VOLTAGE is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_VOLTAGE) == 0) {
        return ERROR_JUMP_CONDICTION_VOLTAGE_NOT_SET;
    }

    // check if jump condiction TIMEOUT is not set
    if((*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) == 0) {
        return ERROR_JUMP_CONDICTION_TIMEOUT_NOT_SET;
    }

    // check if source current capacity level 1 is zero
    if(level_1 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 1;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 2 is zero
    if(level_2 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 2;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if source current capacity level 3 is zero
    if(level_3 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message = 3;
        return ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    // check if charging current is zero
    if(common_current == 0) {
        printf("charging current is zero\n");
        return ERROR_CCC_CURRENT_ZERO;
    }

    // check if current setting is correct
    if(common_current > maximum_source_current) {
        //printf("Constant current %PRIu64 max_source_current %PRIu64\n", common_current, maximum_source_current);
        printf("common_current %"PRIu64"\n", common_current);
        printf("maximum_source_current %"PRIu64"\n", maximum_source_current);
        return ERROR_CONSTANT_CURRENT_LIMIT_EXCEEDED;
    }

    return 0;
}

void reset_step_parameter_register(Manager * _manager, int channel_id)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    Data_Model *data_model = NULL;
    uint16_t *step_definition_reg = NULL;
    int i, j, logical_channel_id = channel_id;
    
    _register =  &manager->modbus_ctx._register;
    data_model = manager->modbus_ctx._register.data_model;

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_1_655;
    clear_holding_registers(step_definition_reg, NB_HOLDING_REGISTERS/HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET);

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_656_1310;
    clear_holding_registers(step_definition_reg, NB_HOLDING_REGISTERS/HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET);

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_1311_1965;
    clear_holding_registers(step_definition_reg, NB_HOLDING_REGISTERS/HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET);

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_1966_2620;
    clear_holding_registers(step_definition_reg, NB_HOLDING_REGISTERS/HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET);

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_2621_3275;
    clear_holding_registers(step_definition_reg, NB_HOLDING_REGISTERS/HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET);

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_3276_3930;
    clear_holding_registers(step_definition_reg, NB_HOLDING_REGISTERS/HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET);

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_3931_4585;
    clear_holding_registers(step_definition_reg, NB_HOLDING_REGISTERS/HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET);

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_4586_5240;
    clear_holding_registers(step_definition_reg, NB_HOLDING_REGISTERS/HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET);

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_5241_5895;
    clear_holding_registers(step_definition_reg, NB_HOLDING_REGISTERS/HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET);

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_5896_6550;
    clear_holding_registers(step_definition_reg, NB_HOLDING_REGISTERS/HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET);

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_6551_7205;
    clear_holding_registers(step_definition_reg, NB_HOLDING_REGISTERS/HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET);

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_7206_7860;
    clear_holding_registers(step_definition_reg, NB_HOLDING_REGISTERS/HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET);

    step_definition_reg = _register->logical_channel[logical_channel_id].step_param_pointers.nb_1_655;
    update_step_parameter_register_mapping(manager, logical_channel_id, step_definition_reg);
    
}

void calculate_and_update_accumulated_logic_capacity_and_accumulated_step_logic_capacity(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    
    _4paired_register_region *last_accumulated_logic_capacity_sampling_time_region, *right_boundary_time_region, *logic_current_region;
    _4paired_register_region *accumulated_logic_capacity_region, *accumulated_step_logic_capacity_region;
    uint16_t *calc_accumulated_capacity_time_interval_reg;
    
    uint64_t last_accumulated_logic_capacity_sampling_time, right_boundary_time, pass_time;
    uint64_t logic_voltage_unsign, logic_current_unsign;
    int64_t logic_current_sign;
    uint64_t accumulated_logic_capacity_unsign, accumulated_step_logic_capacity_unsign;
    int64_t delta_capacity_sign, accumulated_logic_capacity_sign, accumulated_step_logic_capacity_sign;

    _register = &manager->modbus_ctx._register;

    calc_accumulated_capacity_time_interval_reg = _register->logical_channel[logical_channel_id].timing.calc_accumulated_capacity_time_interval;
    last_accumulated_logic_capacity_sampling_time_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region;
    right_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.right_boundary_time_region;
    logic_current_region = &_register->logical_channel[logical_channel_id].logic_current_region;
    accumulated_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_logic_capacity_region;
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;
    
    last_accumulated_logic_capacity_sampling_time = combine_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region);
    right_boundary_time = combine_4paired_register_region(right_boundary_time_region);
    logic_current_unsign = combine_4paired_register_region(logic_current_region);
    accumulated_logic_capacity_unsign = combine_4paired_register_region(accumulated_logic_capacity_region);
    accumulated_step_logic_capacity_unsign = combine_4paired_register_region(accumulated_step_logic_capacity_region);

    if(last_accumulated_logic_capacity_sampling_time != 0) {

        pass_time = right_boundary_time - last_accumulated_logic_capacity_sampling_time;

        if(pass_time >= *calc_accumulated_capacity_time_interval_reg) {

            //printf("calculate_and_update_accumulated_logic_capacity_and_accumulated_step_logic_capacity timing for test\n");

            if(logic_current_unsign & 0x8000000000000000){ //negative
                logic_current_sign = (logic_current_unsign & 0x7fffffffffffffff) * -1;
            } else {
                logic_current_sign = logic_current_unsign;
            }
            
            if(accumulated_logic_capacity_unsign & 0x8000000000000000){ //negative
                accumulated_logic_capacity_sign = (accumulated_logic_capacity_unsign & 0x7fffffffffffffff) * -1;
            } else {
                accumulated_logic_capacity_sign = accumulated_logic_capacity_unsign;
            }
            
            if(accumulated_step_logic_capacity_unsign & 0x8000000000000000){ //negative
                accumulated_step_logic_capacity_sign = (accumulated_step_logic_capacity_unsign & 0x7fffffffffffffff) * -1;
            } else {
                accumulated_step_logic_capacity_sign = accumulated_step_logic_capacity_unsign;
            }

            delta_capacity_sign = pass_time * logic_current_sign;
            accumulated_logic_capacity_sign = accumulated_logic_capacity_sign + delta_capacity_sign;
            accumulated_step_logic_capacity_sign = accumulated_step_logic_capacity_sign + delta_capacity_sign;

            if(accumulated_logic_capacity_sign < 0) {
                accumulated_logic_capacity_unsign = accumulated_logic_capacity_sign * -1;
                accumulated_logic_capacity_unsign = accumulated_logic_capacity_unsign | 0x8000000000000000;
            } else {
                accumulated_logic_capacity_unsign = accumulated_logic_capacity_sign;
            }

            if(accumulated_step_logic_capacity_sign < 0) {
                accumulated_step_logic_capacity_unsign = accumulated_step_logic_capacity_sign * -1;
                accumulated_step_logic_capacity_unsign = accumulated_step_logic_capacity_unsign | 0x8000000000000000;
            } else {
                accumulated_step_logic_capacity_unsign = accumulated_step_logic_capacity_sign;
            }

            update_4paired_register_region(accumulated_logic_capacity_region, accumulated_logic_capacity_unsign);

            update_4paired_register_region(accumulated_step_logic_capacity_region, accumulated_step_logic_capacity_unsign);

            copy_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region, right_boundary_time_region);
        }
    }
    else {
        copy_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region, right_boundary_time_region);
    }

}


int calculate_and_update_logical_voltage_and_current(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    int logical_channel_id = channel_id, i, voltage_updata_flag;
    uint16_t *binding;
    Register *_register = NULL;
    _4paired_register_region *sample_voltage_region, *sample_current_region;
    uint64_t unsign_voltage_amplitude, unsigned_calc_current_amplitude;
    uint64_t sample_voltage, sample_current;    
    _4paired_register_region *logic_current_region, *logic_voltage_region;
    _4paired_register_region *physical_current_region, *physical_voltage_region;
    uint64_t logic_current, logic_voltage, channel_current, channel_voltage, current_sign, voltage_sign;
    uint64_t amplitude_mask = 0x7fffffffffffffff, current_amplitude;
    int operation;
    uint16_t *current_source_level;
    _4paired_register_region *c_cv_b_region, *d_cv_b_region, *bv_b_region;
    uint16_t *c_cv_k_reg, *d_cv_k_reg, *bv_k_reg;
    int64_t c_cv_b_sign, d_cv_b_sign, bv_b_sign;
    uint64_t c_cv_b_unsign, d_cv_b_unsign, bv_b_unsign;
    int64_t calc_current_amplitude, temp_channel_current, calc_voltage_amplitude;
    double c_cv_k, d_cv_k, bv_k;

    _register = &manager->modbus_ctx._register;

    binding = _register->logical_channel[logical_channel_id].binding;

    if(*binding == 0) {
        printf("calculate_and_update_logical_voltage_and_current fail, binding = 0\n");
        return 0;
    }

    logic_current_region = &_register->logical_channel[logical_channel_id].logic_current_region;
    logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;

    sample_voltage_region = &_register->physical_channel[logical_channel_id].sample.voltage_region;
    sample_current_region = &_register->physical_channel[logical_channel_id].sample.current_region;
    
    sample_voltage = combine_4paired_register_region(sample_voltage_region);
    sample_current = combine_4paired_register_region(sample_current_region);

    for(i=0, voltage_updata_flag=0, calc_current_amplitude=0; i<NB_PHYSICAL_CHANNEL; i++) {

        if(((( *binding )>>i) & 0x0001) > 0) {

            current_source_level = _register->physical_channel[i].current_source_level;

            if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1) {
                c_cv_k_reg = _register->physical_channel[i].kb.c_cv_k_level_1;
                d_cv_k_reg = _register->physical_channel[i].kb.d_cv_k_level_1;
                c_cv_b_region = &_register->physical_channel[i].kb.c_cv_b_level1_region;
                d_cv_b_region = &_register->physical_channel[i].kb.d_cv_b_level1_region;
            } else if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2) {
                c_cv_k_reg = _register->physical_channel[i].kb.c_cv_k_level_2;
                d_cv_k_reg = _register->physical_channel[i].kb.d_cv_k_level_2;
                c_cv_b_region = &_register->physical_channel[i].kb.c_cv_b_level2_region;
                d_cv_b_region = &_register->physical_channel[i].kb.d_cv_b_level2_region;
            } else if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3) {
                c_cv_k_reg = _register->physical_channel[i].kb.c_cv_k_level_3;
                d_cv_k_reg = _register->physical_channel[i].kb.d_cv_k_level_3;
                c_cv_b_region = &_register->physical_channel[i].kb.c_cv_b_level3_region;
                d_cv_b_region = &_register->physical_channel[i].kb.d_cv_b_level3_region;
            } else {
                printf("unknow physical channel current source level\n");
                return ERROR_UNKNOWN_PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL;
            }

            c_cv_b_unsign = combine_4paired_register_region(c_cv_b_region);
            d_cv_b_unsign = combine_4paired_register_region(d_cv_b_region);
            
            if(c_cv_b_unsign & 0x8000000000000000){ //negative
                c_cv_b_sign = (c_cv_b_unsign & 0x7fffffffffffffff) * -1;
            } else {
                c_cv_b_sign = c_cv_b_unsign;
            }

            if(d_cv_b_unsign & 0x8000000000000000){ //negative
                d_cv_b_sign = (d_cv_b_unsign & 0x7fffffffffffffff) * -1;
            } else {
                d_cv_b_sign = d_cv_b_unsign;
            }

            c_cv_k = (double)(*c_cv_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);
            d_cv_k = (double)(*d_cv_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);
    
            physical_current_region = &_register->physical_channel[i].sample.current_region;
            physical_voltage_region = &_register->physical_channel[i].sample.voltage_region;

            channel_current = combine_4paired_register_region(physical_current_region);
            channel_voltage = combine_4paired_register_region(physical_voltage_region);

            if(channel_current & 0x8000000000000000){//negative
                operation = -1; /* for substraction operation */
            } else {//positive 
                operation = 1; /* for addition operation*/
            }

            current_amplitude = channel_current & amplitude_mask;

            temp_channel_current = current_amplitude * operation;

            if(operation > 0) { /* plus */
                temp_channel_current = (double)(temp_channel_current) * c_cv_k + c_cv_b_sign;
            } else { /* minus */
                temp_channel_current = (double)(temp_channel_current) * d_cv_k + d_cv_b_sign;
            }

            calc_current_amplitude = calc_current_amplitude + temp_channel_current;

            if(voltage_updata_flag == 0) {

                voltage_updata_flag = 1;

                bv_k_reg = _register->physical_channel[i].kb.bv_k;
                bv_k = (double)(*bv_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;

                bv_b_region = &_register->physical_channel[i].kb.bv_b_region;
                bv_b_unsign = combine_4paired_register_region(bv_b_region);
                if(bv_b_unsign & 0x8000000000000000){ //negative
                    bv_b_sign = (bv_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    bv_b_sign = bv_b_unsign;
                }

                if(channel_voltage & 0x8000000000000000) {
                    unsign_voltage_amplitude = channel_voltage & 0x7fffffffffffffff;
                    calc_voltage_amplitude = unsign_voltage_amplitude * -1;
                } else {
                    unsign_voltage_amplitude = channel_voltage & 0x7fffffffffffffff;
                    calc_voltage_amplitude = unsign_voltage_amplitude;
                }

                calc_voltage_amplitude = (double)calc_voltage_amplitude * bv_k + (double)bv_b_sign;
            }
        }
    }

    if(calc_current_amplitude > 0) {
        current_sign = 0x0000000000000000;
    } else {
        current_sign = 0x8000000000000000;
    }

    if(calc_voltage_amplitude > 0){
        voltage_sign = 0x0000000000000000;
    } else {
        voltage_sign = 0x8000000000000000;
    }

    unsigned_calc_current_amplitude = llabs(calc_current_amplitude);
    unsign_voltage_amplitude = llabs(calc_voltage_amplitude);

    logic_current = (unsigned_calc_current_amplitude & 0x7fffffffffffffff) | current_sign;
    update_4paired_register_region(logic_current_region, logic_current);

    logic_voltage = (unsign_voltage_amplitude & 0x7fffffffffffffff) | voltage_sign;
    update_4paired_register_region(logic_voltage_region, logic_voltage);

    return 0;
}

int check_if_exception(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    int logical_channel_id = channel_id;

    return 0;
}

int check_if_meet_jump_condiction(Manager *manager, int channel_id)
{
    int ret, logical_channel_id = channel_id, constant_voltage_stage_flag, current_drop_flag;
    Register *_register = NULL;    
    uint16_t exp_code;
    uint16_t *exp_code_reg, *mode_reg, *l_status_reg, *exp_message_reg, *step_type_reg;
    uint16_t *step_pointer_reg, *step_quantity_reg;
    uint16_t *common_stauts_reg;
    _4paired_register_region *jump_current_region, *jump_voltage_region, *jump_capacity_region;
    _4paired_register_region *info_common_current_region, *info_common_voltage_region;
    _4paired_register_region *logic_current_region, *logic_voltage_region, *accumulated_step_logic_capacity_region;
    _4paired_register_region *constant_voltage_stage_current_drop_permillage_region;
    _4paired_register_region *constant_voltage_stage_confirmation_tolerance_region;
    _4paired_register_region *common_current_region;
    _4paired_register_region *common_voltage_region;
    uint64_t jump_current, jump_voltage, jump_capacity, temp, common_current, common_voltage;
    uint64_t amplitude_mask = 0x7fffffffffffffff, logic_current_amplitude, logic_voltage_amplitude, accumulated_step_logic_capacity_amplitude;
    uint64_t constant_voltage_stage_confirmation_tolerance;
    uint64_t constant_voltage_stage_current_drop_permillage;

    _4paired_register_region *holding_time_region, *jump_time_region, *jump_time_reference_region;
    uint64_t holding_time, jump_time, jump_time_reference, calc_jump_time;

    _register = &manager->modbus_ctx._register;
    
    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    jump_time_region = &_register->logical_channel[logical_channel_id].step_param.jump.time_region;
    jump_time_reference_region = &_register->logical_channel[logical_channel_id].temp.jump_time_reference_region;
    holding_time = combine_4paired_register_region(holding_time_region);
    jump_time = combine_4paired_register_region(jump_time_region);
    jump_time_reference = combine_4paired_register_region(jump_time_reference_region);
    calc_jump_time = jump_time + jump_time_reference;

    jump_current_region = &_register->logical_channel[logical_channel_id].step_param.jump.current_region;
    jump_voltage_region = &_register->logical_channel[logical_channel_id].step_param.jump.voltage_region;
    jump_capacity_region = &_register->logical_channel[logical_channel_id].step_param.jump.capacity_region;

    info_common_current_region = &_register->logical_channel[logical_channel_id].step_param.info.common.current_region;
    info_common_voltage_region = &_register->logical_channel[logical_channel_id].step_param.info.common.voltage_region;

    logic_current_region = &_register->logical_channel[logical_channel_id].logic_current_region;
    logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;

    step_type_reg = _register->logical_channel[logical_channel_id].step_param.step_type;
    
    common_stauts_reg = _register->logical_channel[logical_channel_id].step_param.info.common.status;

    constant_voltage_stage_current_drop_permillage_region = &_register->logical_channel[logical_channel_id].tolerance.constant_voltage_stage_current_drop_confirmation_tolerance_region;
    constant_voltage_stage_current_drop_permillage = combine_4paired_register_region(constant_voltage_stage_current_drop_permillage_region);

    constant_voltage_stage_confirmation_tolerance_region = &_register->logical_channel[logical_channel_id].tolerance.constant_voltage_stage_confirmation_tolerance_region;
    constant_voltage_stage_confirmation_tolerance = combine_4paired_register_region(constant_voltage_stage_confirmation_tolerance_region);

    common_voltage_region = &_register->logical_channel[logical_channel_id].step_param.info.common.voltage_region;
    common_voltage = combine_4paired_register_region(common_voltage_region);

    common_current_region = &_register->logical_channel[logical_channel_id].step_param.info.common.current_region;
    common_current = combine_4paired_register_region(common_current_region);

    temp = combine_4paired_register_region(jump_current_region);
    jump_current = temp & amplitude_mask;

    temp = combine_4paired_register_region(jump_voltage_region);
    jump_voltage = temp & amplitude_mask;

    temp = combine_4paired_register_region(jump_capacity_region);
    jump_capacity = temp & amplitude_mask;

    temp = combine_4paired_register_region(info_common_current_region);
    common_current = temp & amplitude_mask;

    temp = combine_4paired_register_region(info_common_voltage_region);
    common_voltage = temp & amplitude_mask;

    temp = combine_4paired_register_region(logic_current_region);
    logic_current_amplitude = temp & amplitude_mask;

    temp = combine_4paired_register_region(logic_voltage_region);
    logic_voltage_amplitude = temp & amplitude_mask;

    temp = combine_4paired_register_region(accumulated_step_logic_capacity_region);
    accumulated_step_logic_capacity_amplitude = temp & amplitude_mask;

    switch(*step_type_reg) {

        case FORMATION_STEP_TYPE_REST:
             // check if timeout
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) {
                 if(holding_time >= calc_jump_time) {
                     return JUMP_CONDICTION_TIME;
                 }
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCC:
             // check if timeout
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) {
                 if(holding_time >= calc_jump_time) {
                     return JUMP_CONDICTION_TIME;
                 }
             }

             // check if voltage upper limit reached
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_VOLTAGE) {
                 if(logic_voltage_amplitude >= jump_voltage) {
                     return JUMP_CONDICTION_VOLTAGE;
                 }
             }

             // check if capacity limit reached
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_CAPACITY) {
                 if(accumulated_step_logic_capacity_amplitude >= jump_capacity) {
                     return JUMP_CONDICTION_CAPACITY;
                 }
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCD:
             // check if timeout
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) {
                 if(holding_time >= calc_jump_time) {
                     return JUMP_CONDICTION_TIME;
                 }
             }

             // check if voltage lower limit reached
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_VOLTAGE) {
                 if(logic_voltage_amplitude <= jump_voltage) {
                     return JUMP_CONDICTION_VOLTAGE;
                 }
             }

             // check if capacity limit reached
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_CAPACITY) {
                 if(accumulated_step_logic_capacity_amplitude >= jump_capacity) {
                     return JUMP_CONDICTION_CAPACITY;
                 }
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCCCVC:
             // check if CCXCVX
             constant_voltage_stage_flag = (logic_voltage_amplitude >= (common_voltage - constant_voltage_stage_confirmation_tolerance));
             current_drop_flag = (  ( ((double)common_current - logic_current_amplitude)*1000 / common_current ) >= constant_voltage_stage_current_drop_permillage);
             if (*common_stauts_reg == STEP_INFO_COMMON_STATUS_NORMAL){
                 if (constant_voltage_stage_flag & current_drop_flag){
                     *common_stauts_reg = STEP_INFO_COMMON_STATUS_CCXCVX_VOLTAGE_LIMIT_REACHED;
                 }
             }

             // check if timeout
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) {    
                 if(holding_time >= calc_jump_time) {
                     return JUMP_CONDICTION_TIME;
                 }
             }

             // check if current lower limit reached
             if (*common_stauts_reg == STEP_INFO_COMMON_STATUS_CCXCVX_VOLTAGE_LIMIT_REACHED){
                 if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_CURRENT) {
                     if(logic_current_amplitude <= jump_current) {
                         *common_stauts_reg = STEP_INFO_COMMON_STATUS_CCXCVX_CURRENT_LOWER_LIMIT_REACHED;
                         return JUMP_CONDICTION_CURRENT;
                     }
                 }
             }

             // check if capacity limit reached
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_CAPACITY) {
                 if(accumulated_step_logic_capacity_amplitude >= jump_capacity) {
                     return JUMP_CONDICTION_CAPACITY;
                 }
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCDCVD:
             // check if CCXCVX
             constant_voltage_stage_flag = (logic_voltage_amplitude <= (common_voltage + constant_voltage_stage_confirmation_tolerance));
             current_drop_flag = (  ( ((double)common_current - logic_current_amplitude)*1000 / common_current ) >= constant_voltage_stage_current_drop_permillage );
             if (*common_stauts_reg == STEP_INFO_COMMON_STATUS_NORMAL){
                 if (constant_voltage_stage_flag & current_drop_flag){
                     *common_stauts_reg = STEP_INFO_COMMON_STATUS_CCXCVX_VOLTAGE_LIMIT_REACHED;
                 }
             }

             // check if timeout
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) {    
                 if(holding_time >= calc_jump_time) {
                     return JUMP_CONDICTION_TIME;
                 }
             }

             // check if current lower limit reached
             if (*common_stauts_reg == STEP_INFO_COMMON_STATUS_CCXCVX_VOLTAGE_LIMIT_REACHED){
                 if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_CURRENT) {
                     if(logic_current_amplitude <= jump_current) {
                         *common_stauts_reg = STEP_INFO_COMMON_STATUS_CCXCVX_CURRENT_LOWER_LIMIT_REACHED;
                         return JUMP_CONDICTION_CURRENT;
                     }
                 }
             }

             // check if capacity limit reached
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_CAPACITY) {
                 if(accumulated_step_logic_capacity_amplitude >= jump_capacity) {
                     return JUMP_CONDICTION_CAPACITY;
                 }
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CPC:
             // check if timeout
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) {    
                 if(holding_time >= calc_jump_time) {
                     return JUMP_CONDICTION_TIME;
                 }
             }

             // check if voltage upper limit reached
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_VOLTAGE) {
                 if(logic_voltage_amplitude >= jump_voltage) {
                     return JUMP_CONDICTION_VOLTAGE;
                 }
             }

             // check if capacity limit reached
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_CAPACITY) {
                 if(accumulated_step_logic_capacity_amplitude >= jump_capacity) {
                     return JUMP_CONDICTION_CAPACITY;
                 }
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CPD:
             // check if timeout
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) {    
                 if(holding_time >= calc_jump_time) {
                     return JUMP_CONDICTION_TIME;
                 }
             }

             // check if voltage upper limit reached
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_VOLTAGE) {
                 if(logic_voltage_amplitude <= jump_voltage) {
                     return JUMP_CONDICTION_VOLTAGE;
                 }
             }

             // check if capacity limit reached
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_CAPACITY) {
                 if(accumulated_step_logic_capacity_amplitude >= jump_capacity) {
                     return JUMP_CONDICTION_CAPACITY;
                 }
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CRD:
             // check if timeout
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_TIMEOUT) {    
                 if(holding_time >= calc_jump_time) {
                     return JUMP_CONDICTION_TIME;
                 }
             }

             // check if voltage upper limit reached
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_VOLTAGE) {
                 if(logic_voltage_amplitude <= jump_voltage) {
                     return JUMP_CONDICTION_VOLTAGE;
                 }
             }

             // check if capacity limit reached
             if(*_register->logical_channel[logical_channel_id].step_param.jump.enable & STEP_JUMP_CONDICTION_CAPACITY) {
                 if(accumulated_step_logic_capacity_amplitude >= jump_capacity) {
                     return JUMP_CONDICTION_CAPACITY;
                 }
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CYCLE:
             // jump now
             return JUMP_CONDICTION_CYCLE;
        break;

        case FORMATION_STEP_TYPE_GOTO:
             // jump now
             return JUMP_CONDICTION_GOTO;
        break;

        default:
             return ERROR_UNKNOWN_STEP_TYPE;
        break;
    }
    
    return 0;
}


int formation_reset(Manager *_manager, int channel_id)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    Data_Model *data_model = NULL;
    int logical_channel_id = channel_id;
    uint16_t *exp_code_reg, *action_reg, *mode_reg, *l_status_reg;
    uint16_t *step_pointer_reg, *step_quantity_reg, *nb_binding;    

    if(!manager || logical_channel_id < LOGICAL_CHANNEL_1_ID || logical_channel_id >= NB_LOGIC_CHANNEL) {
        printf("formation_reset fail, incorrect parameters, channel %d\n", logical_channel_id);
        return ERROR_INTERNEL_ERRORS;
    }

    _register = &manager->modbus_ctx._register;

    mode_reg = _register->logical_channel[logical_channel_id].mode;
    action_reg = _register->logical_channel[logical_channel_id].action;
    l_status_reg = _register->logical_channel[logical_channel_id].status;
    exp_code_reg = _register->logical_channel[logical_channel_id].exception_code;
    step_pointer_reg = _register->logical_channel[logical_channel_id].step_pointer;
    nb_binding = _register->logical_channel[logical_channel_id].nb_binding;

    switch(*l_status_reg) {

        case LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_IDEL:
             reset_one_logical_channel(manager, logical_channel_id);
             reset_step_parameter_register(manager, logical_channel_id);
             return 0;
        break;

        case LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING:
        case LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_PAUSING:
        case LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED:
        case LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_GOTO:
        case LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_TRANSFORMING:
        case LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_STOPPING:
             reset_bound_physical_channels(manager, logical_channel_id);
             reset_one_logical_channel(manager, logical_channel_id);
             reset_step_parameter_register(manager, logical_channel_id);
             return 0;
        break;

        case LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_EXCEPTION:
             if(*nb_binding != 0) {
                 reset_bound_physical_channels(manager, logical_channel_id);
             } 

             reset_one_logical_channel(manager, logical_channel_id);
             reset_step_parameter_register(manager, logical_channel_id);
             return 0;
        break;

        default:
             return ERROR_INTERNEL_ERRORS;
        break;
    }

    return ERROR_INTERNEL_ERRORS;
}

int formation_resume(Manager *manager, int channel_id)
{
    int ret = 0, logical_channel_id = channel_id;
    Register *_register = NULL;

    if(!manager || logical_channel_id < LOGICAL_CHANNEL_1_ID || logical_channel_id >= NB_LOGIC_CHANNEL) {
        printf("do_formation_logic fail, incorrect parameters, channel %d\n", logical_channel_id);
        return -1;
    }

    _register = &manager->modbus_ctx._register;

    switch(*_register->logical_channel[logical_channel_id].step_param.step_type) {

        case FORMATION_STEP_TYPE_REST:

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             update_right_boundary_and_holding_time(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CRD:
        case FORMATION_STEP_TYPE_CPD:
        case FORMATION_STEP_TYPE_CPC:
        case FORMATION_STEP_TYPE_CCDCVD:
        case FORMATION_STEP_TYPE_CCCCVC:
        case FORMATION_STEP_TYPE_CCD:
        case FORMATION_STEP_TYPE_CCC:

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_DISABLE);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             update_right_boundary_and_holding_time(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CYCLE:
        case FORMATION_STEP_TYPE_GOTO:

             return 0;
    }

    return ERROR_UNKNOWN_STEP_TYPE;
}

int formation_pause(Manager *manager, int channel_id)
{
    int ret = 0, logical_channel_id = channel_id;
    uint16_t temp1, temp2, temp3;
    Register *_register = NULL;

    if(!manager || logical_channel_id < LOGICAL_CHANNEL_1_ID || logical_channel_id >= NB_LOGIC_CHANNEL) {
        printf("do_formation_logic fail, incorrect parameters, channel %d\n", logical_channel_id);
        return -1;
    }

    _register = &manager->modbus_ctx._register;

    switch(*_register->logical_channel[logical_channel_id].step_param.step_type) {

        case FORMATION_STEP_TYPE_REST:

             update_right_boundary_and_holding_time(manager, logical_channel_id);

             update_holding_time_reference(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CRD:
        case FORMATION_STEP_TYPE_CPD:
        case FORMATION_STEP_TYPE_CPC:
        case FORMATION_STEP_TYPE_CCDCVD:
        case FORMATION_STEP_TYPE_CCCCVC:
        case FORMATION_STEP_TYPE_CCD:
        case FORMATION_STEP_TYPE_CCC:

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_ENABLE);

             update_right_boundary_and_holding_time(manager, logical_channel_id);

             update_holding_time_reference(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CYCLE:
        case FORMATION_STEP_TYPE_GOTO:

             return 0;
        break;
    }
}

int formation_start(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int ret = 0, logical_channel_id = channel_id;
    
    if(!manager || logical_channel_id < LOGICAL_CHANNEL_1_ID || logical_channel_id >= NB_LOGIC_CHANNEL) {
        printf("formation_start fail, incorrect parameters, channel %d\n", logical_channel_id);
        return -1;
    }

    _register = &manager->modbus_ctx._register;

    // check if general parameters correct!!!
    ret = check_general_parameters(manager, logical_channel_id);
    if(ret < 0) {
        printf("formation_start->check_general_parameters fail\n");
        return ret;
    }

    // check if all physical channels for this logical channel are ready
    ret = check_physical_channels_and_update_info(manager, logical_channel_id);
    if(ret < 0) {
        printf("formation_start fail, physical channels are not idel\n");
        return ret;
    }

    update_bound_physical_channel_status_to_running(manager, logical_channel_id);

    calculate_and_update_logical_voltage_and_current(manager, logical_channel_id);

    //printf("step type %d\n", *_register->logical_channel[logical_channel_id].step_param.step_type);

    switch(*_register->logical_channel[logical_channel_id].step_param.step_type) {
        
        case FORMATION_STEP_TYPE_REST:
             
             ret = check_REST_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check_REST_parameters fail\n");
                 return ret;
             }

             //put time killer before update established_time
             clear_one_logical_channel_log(manager, logical_channel_id);

             init_REST_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("formation_start->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCC:

             // check CCC parameters
             ret = check_CCC_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CCC parameters fail\n");
                 return ret;
             }

             //put time killer before update established_time
             clear_one_logical_channel_log(manager, logical_channel_id);

             init_CCC_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("formation_start->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCD:
        
             // check CCD parameters
             ret = check_CCD_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CCD parameters fail\n");
                 return ret;
             }

             //put time killer before update established_time
             clear_one_logical_channel_log(manager, logical_channel_id);

             init_CCD_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("formation_start->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCCCVC:

             // check CCCCVC parameters
             ret = check_CCCCVC_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CCCCVC parameters fail\n");
                 return ret;
             }

             //put time killer before update established_time
             clear_one_logical_channel_log(manager, logical_channel_id);

             init_CCCCVC_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("formation_start->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCDCVD:

             // check CCDCVD parameters
             ret = check_CCDCVD_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CCC parameters fail\n");
                 return ret;
             }

             //put time killer before update established_time
             clear_one_logical_channel_log(manager, logical_channel_id);

             init_CCDCVD_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("formation_start->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CPC:

             // check CPC parameters
             ret = check_CPC_parameters(manager, logical_channel_id);
             if(ret < 0) {
                printf("check CCC parameters fail\n");
                return ret;
             }

             //put time killer before update established_time
             clear_one_logical_channel_log(manager, logical_channel_id);

             init_CPC_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("formation_start->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CPD:
             
             // check CPD parameters
             ret = check_CPD_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CPD parameters fail\n");
                 return ret;
             }

             //put time killer before update established_time
             clear_one_logical_channel_log(manager, logical_channel_id);

             init_CPD_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("formation_start->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CRD:

             // check CRD parameters
             ret = check_CRD_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CRD parameters fail\n");
                 return ret;
             }

             //put time killer before update established_time
             clear_one_logical_channel_log(manager, logical_channel_id);

             init_CRD_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("formation_start->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CYCLE:
             // errors occur, the first step can't be cycle
             return ERROR_FIRST_STEP_IS_CYCLE;
        break;

        case FORMATION_STEP_TYPE_GOTO:
             // errors occur, the first step can't be GOTO
             return ERROR_FIRST_STEP_IS_GOTO;
        break;

        default:
             printf("formation_start, unknown step type!!!\n");
             return ERROR_UNKNOWN_STEP_TYPE;
        break;
    }

    return ret;
}

int formation_run(Manager *manager, int channel_id)
{
    int ret = 0, logical_channel_id = channel_id, ready;
    Register *_register = NULL;

    if(!manager || logical_channel_id < LOGICAL_CHANNEL_1_ID || logical_channel_id >= NB_LOGIC_CHANNEL) {
        printf("do_formation_logic fail, incorrect parameters, channel %d\n", logical_channel_id);
        return -1;
    }

    _register = &manager->modbus_ctx._register;

    switch(*_register->logical_channel[logical_channel_id].step_param.step_type) {

        case FORMATION_STEP_TYPE_REST:
             update_right_boundary_and_holding_time(manager, logical_channel_id);

             ret = check_if_voltage_difference_of_all_physical_channels_in_tolerance(manager, logical_channel_id);
             if(ret < 0) {
                 printf("formation_run->check_if_voltage_difference_of_all_physical_channels_in_tolerance fail\n");
                 update_step_abolished_time(manager, logical_channel_id);
                 return ret;
             }

             ret = calculate_and_update_logical_voltage_and_current(manager, logical_channel_id);
             if(ret < 0) {
                 printf("formation_run->calculate_and_update_logical_voltage_and_current fail\n");
                 update_step_abolished_time(manager, logical_channel_id);
                 return ret;
             }

             ret = check_record_one_logical_channel_currve_log(manager, logical_channel_id);
             if(ret < 0){
                update_step_abolished_time(manager, logical_channel_id);
                return ret;
             }

             ret = check_if_exception(manager, logical_channel_id);
             if(ret < 0) {
                 update_step_abolished_time(manager, logical_channel_id);
                 return ret;
             }

             // check if meet jump condiction
             ret = check_if_meet_jump_condiction(manager, logical_channel_id);
             if(ret > 0) {
                 update_step_abolished_time(manager, logical_channel_id);
                 record_one_logical_channel_sampling(manager, logical_channel_id);
                 return ret;
             }

             ret = check_if_do_channel_regulation(manager, logical_channel_id);
             if(ret < 0) {
                 update_step_abolished_time(manager, logical_channel_id);
                 return ret;
             }

             return 0;
        break;
 
        case FORMATION_STEP_TYPE_CRD:
        case FORMATION_STEP_TYPE_CPD:
        case FORMATION_STEP_TYPE_CPC:
        case FORMATION_STEP_TYPE_CCC:
        case FORMATION_STEP_TYPE_CCD:
        case FORMATION_STEP_TYPE_CCCCVC:
        case FORMATION_STEP_TYPE_CCDCVD:
             update_right_boundary_and_holding_time(manager, logical_channel_id);

             // check if physical channel ready???
             ready = check_if_all_physical_channels_ready(manager, logical_channel_id);
             if(ready) {

                 ret = check_if_voltage_difference_of_all_physical_channels_in_tolerance(manager, logical_channel_id);
                 if(ret < 0) {
                     printf("formation_run->check_if_voltage_difference_of_all_physical_channels_in_tolerance fail\n");
                     return ret;
                 }

                 ret = calculate_and_update_logical_voltage_and_current(manager, logical_channel_id);
                 if(ret < 0) {
                     printf("formation_run->calculate_and_update_logical_voltage_and_current fail\n");
                     return ret;
                 }

                 calculate_and_update_accumulated_logic_capacity_and_accumulated_step_logic_capacity(manager, logical_channel_id);

                 ret = check_record_one_logical_channel_currve_log(manager, logical_channel_id);
                 if(ret < 0){
                    update_step_abolished_time(manager, logical_channel_id);
                    return ret;
                 }

                 // check if exception occurs
                 ret = check_if_exception(manager, logical_channel_id);
                 if(ret < 0) {
                     printf("CCC exception occurs\n");
                     update_step_abolished_time(manager, logical_channel_id);
                     return ret;
                 }

                 // check if meet jump condiction
                 ret = check_if_meet_jump_condiction(manager, logical_channel_id);
                 if(ret > 0) {
                     update_step_abolished_time(manager, logical_channel_id);
                     record_one_logical_channel_sampling(manager, logical_channel_id);
                     return ret;
                 }

                 // check if regulate channel parameters
                 ret = check_if_do_channel_regulation(manager, logical_channel_id);
                 if(ret < 0) {
                     update_step_abolished_time(manager, logical_channel_id);
                     return ret;
                 }

             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_GOTO:
        case FORMATION_STEP_TYPE_CYCLE:
             // check if meet jump condiction
             ret = check_if_meet_jump_condiction(manager, logical_channel_id);
             if(ret > 0) {
                 return ret;
             }

             return 0;
        break;

        default:
             return ERROR_UNKNOWN_STEP_TYPE;
        break;

    }

    return ret;
}


