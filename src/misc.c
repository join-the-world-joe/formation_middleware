/**
* @file     misc.c
* @brief    misc.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     01/15/2018
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

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

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
#include "ssd1306_oled_raspberry_pi.h"
#include "registers.h"
#include "modbus_tcp.h"

#include "manager.h"
#include "logical_channel.h"
#include "physical_channel.h"
#include "step_timming_parameters.h"
#include "registers.h"

#include "log_controller.h"
#include "modbus_tcp.h"
#include "macros.h"
#include "platform_endian.h"

#include "register_offset.h"
#include "register_values.h"
#include "error_no.h"

#include "debug_logic.h"

#include "misc.h"
#include "set_get_ip.h"




void clear_holding_registers(uint16_t *start_address, uint16_t _quantity)
{
    int i;
    uint16_t *holding_register_address = start_address, quantity = _quantity;

    for(i=0; i<quantity; i++) {
        *(holding_register_address + i) = 0;
    }
}

int accquire_physical_channel_mask(Manager *_manager, int _physical_channel_id)
{
    int ret;
    int physical_channel_id = _physical_channel_id;
    
    switch(physical_channel_id) {

        case PHYSICAL_CHANNEL_1_ID:
             return LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH1_MASK_BIT;
        break;

        case PHYSICAL_CHANNEL_2_ID:
             return LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH2_MASK_BIT;
        break;

        case PHYSICAL_CHANNEL_3_ID:
             return LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH3_MASK_BIT;
        break;

        case PHYSICAL_CHANNEL_4_ID:
             return LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH4_MASK_BIT;
        break;

        case PHYSICAL_CHANNEL_5_ID:
             return LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH5_MASK_BIT;
        break;

        case PHYSICAL_CHANNEL_6_ID:
             return LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH6_MASK_BIT;
        break;

        case PHYSICAL_CHANNEL_7_ID:
             return LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH7_MASK_BIT;
        break;

        case PHYSICAL_CHANNEL_8_ID:
             return LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH8_MASK_BIT;
        break;

    }

    return ERROR_INTERNEL_ERRORS;
}

int bind_physical_channel_and_update_info(Manager *_manager, int _logical_channel_id, int _physical_channel_id)
{
    int ret, mask;
    Register *_register = NULL;
    Manager *manager = _manager;
    uint16_t *physical_channel_status;
    uint64_t level_1, level_2, level_3, temp, maximum_source_current = 0;
    int logical_channel_id = _logical_channel_id, physical_channel_id = _physical_channel_id;
    
    _4paired_register_region *maximum_source_current_region;
    _4paired_register_region *level_1_region, *level_2_region, *level_3_region;

    _register = &manager->modbus_ctx._register;

    physical_channel_status = _register->physical_channel[physical_channel_id].status;

    // check if binding is non-zero
    if(*_register->logical_channel[logical_channel_id].binding != 0) {
        printf("physical channels are bound for this logical channel now!\n");
        return ERROR_BINDING_REGISTER_NONZERO;
    }

    // check if physical_channel is busy
    mask = accquire_physical_channel_mask(manager, physical_channel_id);
    if(*physical_channel_status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
        *_register->logical_channel[logical_channel_id].exception_message |= mask;
        ret = ERROR_PHYSICAL_CHANNEL_BUSY;
    }

    
    

    // clear the magnitude of maxium source current
    maximum_source_current_region = &_register->logical_channel[logical_channel_id].maximum_source_current_region;
    update_4paired_register_region(maximum_source_current_region, 0);

    // check if configuration is correct
    level_1_region = &_register->physical_channel[physical_channel_id].source_current_level_1_region;
    level_2_region = &_register->physical_channel[physical_channel_id].source_current_level_2_region;
    level_3_region = &_register->physical_channel[physical_channel_id].source_current_level_3_region;
    
    level_1 = combine_4paired_register_region(level_1_region);
    level_2 = combine_4paired_register_region(level_2_region);
    level_3 = combine_4paired_register_region(level_3_region);

    if(level_1 == 0 || level_2 == 0 || level_3 == 0) {
        *_register->logical_channel[logical_channel_id].exception_message |= mask;
        ret = ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
    }

    maximum_source_current = level_3;

    *_register->logical_channel[logical_channel_id].nb_binding = 1;

    //update maxium source current
    update_4paired_register_region(maximum_source_current_region, maximum_source_current);

    return 0;
}


int bell_enable(Manager *_manager)
{
    int ret;
    Manager *manager = _manager;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param_bell = NULL;

    _gpio_controller_ctx = &manager->resources.gpio_controller_ctx;
    _gpio_controller_ioctl_param_bell = &manager->resources.gpio_controller_ioctl_param_bell;

    _gpio_controller_ioctl_param_bell->set_level_param.in.level = GPIO_LEVEL_HIGH;
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL, _gpio_controller_ioctl_param_bell);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        return ret;
    }

    return 0;
}

int bell_disable(Manager *_manager)
{
    int ret;
    Manager *manager = _manager;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param_bell = NULL;

    _gpio_controller_ctx = &manager->resources.gpio_controller_ctx;
    _gpio_controller_ioctl_param_bell = &manager->resources.gpio_controller_ioctl_param_bell;

    _gpio_controller_ioctl_param_bell->set_level_param.in.level = GPIO_LEVEL_LOW;
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL, _gpio_controller_ioctl_param_bell);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        return ret;
    }

    return 0;
}

int led_ready_enable(Manager *_manager)
{
    int ret;
    Manager *manager = _manager;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param = NULL;

    _gpio_controller_ctx = &manager->resources.gpio_controller_ctx;
    _gpio_controller_ioctl_param = &manager->resources.gpio_controller_ioctl_param_led_ready;

    _gpio_controller_ioctl_param->set_level_param.in.level = GPIO_LEVEL_HIGH;
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        return ret;
    }
    return 0;
}

int led_ready_disable(Manager *_manager)
{
    int ret;
    Manager *manager = _manager;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param = NULL;

    _gpio_controller_ctx = &manager->resources.gpio_controller_ctx;
    _gpio_controller_ioctl_param = &manager->resources.gpio_controller_ioctl_param_led_ready;

    _gpio_controller_ioctl_param->set_level_param.in.level = GPIO_LEVEL_LOW;
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        return ret;
    }
    return 0;
}

int led_run_enable(Manager *_manager)
{
    int ret;
    Manager *manager = _manager;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param = NULL;

    _gpio_controller_ctx = &manager->resources.gpio_controller_ctx;
    _gpio_controller_ioctl_param = &manager->resources.gpio_controller_ioctl_param_led_run;

    _gpio_controller_ioctl_param->set_level_param.in.level = GPIO_LEVEL_HIGH;
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        return ret;
    }
    return 0;
}

int led_run_disable(Manager *_manager)
{
    int ret;
    Manager *manager = _manager;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param = NULL;

    _gpio_controller_ctx = &manager->resources.gpio_controller_ctx;
    _gpio_controller_ioctl_param = &manager->resources.gpio_controller_ioctl_param_led_run;

    _gpio_controller_ioctl_param->set_level_param.in.level = GPIO_LEVEL_LOW;
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        return ret;
    }
    return 0;
}

int led_error_enable(Manager *_manager)
{
    int ret;
    Manager *manager = _manager;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param = NULL;

    _gpio_controller_ctx = &manager->resources.gpio_controller_ctx;
    _gpio_controller_ioctl_param = &manager->resources.gpio_controller_ioctl_param_led_error;

    _gpio_controller_ioctl_param->set_level_param.in.level = GPIO_LEVEL_HIGH;
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        return ret;
    }
    return 0;
}

int led_error_disable(Manager *_manager)
{
    int ret;
    Manager *manager = _manager;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param = NULL;

    _gpio_controller_ctx = &manager->resources.gpio_controller_ctx;
    _gpio_controller_ioctl_param = &manager->resources.gpio_controller_ioctl_param_led_error;

    _gpio_controller_ioctl_param->set_level_param.in.level = GPIO_LEVEL_LOW;
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        return ret;
    }
    return 0;
}

int clear_one_logical_channel_log(Manager * _manager, int channel_id)
{
    Manager *manager = _manager;
    int logical_channel_id = channel_id, cmd, ret;
    log_controller_context *_log_controller_ctx;

    log_controller_ioctl_parameters *log_controller_ioctl_param;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel0;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel1;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel2;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel3;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel4;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel5;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel6;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel7;

    _log_controller_ctx = &manager->log_controller_ctx;
    log_controller_ioctl_param_channel0 = &manager->sampling_log.log_controller_ioctl_param_channel0;
    log_controller_ioctl_param_channel1 = &manager->sampling_log.log_controller_ioctl_param_channel1;
    log_controller_ioctl_param_channel2 = &manager->sampling_log.log_controller_ioctl_param_channel2;
    log_controller_ioctl_param_channel3 = &manager->sampling_log.log_controller_ioctl_param_channel3;
    log_controller_ioctl_param_channel4 = &manager->sampling_log.log_controller_ioctl_param_channel4;
    log_controller_ioctl_param_channel5 = &manager->sampling_log.log_controller_ioctl_param_channel5;
    log_controller_ioctl_param_channel6 = &manager->sampling_log.log_controller_ioctl_param_channel6;
    log_controller_ioctl_param_channel7 = &manager->sampling_log.log_controller_ioctl_param_channel7;

    switch (logical_channel_id) {
        case PHYSICAL_CHANNEL_01_UNIT_IDENTIFIER:
            log_controller_ioctl_param = log_controller_ioctl_param_channel0;
        break;
        case PHYSICAL_CHANNEL_02_UNIT_IDENTIFIER:
            log_controller_ioctl_param = log_controller_ioctl_param_channel1;
        break;
        case PHYSICAL_CHANNEL_03_UNIT_IDENTIFIER:
            log_controller_ioctl_param = log_controller_ioctl_param_channel2;
        break;
        case PHYSICAL_CHANNEL_04_UNIT_IDENTIFIER:
            log_controller_ioctl_param = log_controller_ioctl_param_channel3;
        break;
        case PHYSICAL_CHANNEL_05_UNIT_IDENTIFIER:
            log_controller_ioctl_param = log_controller_ioctl_param_channel4;
        break;
        case PHYSICAL_CHANNEL_06_UNIT_IDENTIFIER:
            log_controller_ioctl_param = log_controller_ioctl_param_channel5;
        break;
        case PHYSICAL_CHANNEL_07_UNIT_IDENTIFIER:
            log_controller_ioctl_param = log_controller_ioctl_param_channel6;
        break;
        case PHYSICAL_CHANNEL_08_UNIT_IDENTIFIER:
            log_controller_ioctl_param = log_controller_ioctl_param_channel7;
        break;
        default:
            printf("clean_one_logical_channel_log unknow logical_channel_id\n");
            return ERROR_INTERNEL_ERRORS;
    }
    
    cmd = LOG_CONTROLLER_IOCTL_CMD_REMOVE_LOG_CONTENT;
    log_controller_ioctl_param->log_message_param.in.level = LOG_LEVEL_DEBUG;
    ret = _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, log_controller_ioctl_param);
    if(ret < 0) {
        printf("clean_one_logical_channel_log rmmove log content fail\n");
        return ret;
    }
    return 0;
}

int record_one_logical_channel_sampling(Manager * _manager, int channel_id)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = channel_id, cmd, ret, sign;
    log_controller_context *_log_controller_ctx;
    //struct milli_time now_time;
    uint64_t source_logical_voltage, source_logical_current, source_logical_voltage_amplitude, source_logical_current_amplitude;
    double present_logical_voltage, present_logical_current;
    _4paired_register_region *right_boundary_time_region, *holding_time_region, *accumulated_logic_capacity_region, *accumulated_step_logic_capacity_region;
    struct tm *now_time;
    time_t unix_timestamp;
    uint16_t now_time_millisecond, *step_pointer_reg;
    uint64_t right_boundary_time, holding_time;
    uint64_t accumulated_logic_capacity_unsign, accumulated_logic_capacity_sign, accumulated_step_logic_capacity_unsign, accumulated_step_logic_capacity_sign;
    uint64_t accumulated_logic_capacity_amplitude, accumulated_step_logic_capacity_amplitude;
    uint16_t *cycle_count_reg, *step_type_reg;

    log_controller_ioctl_parameters *log_controller_ioctl_param;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel0;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel1;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel2;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel3;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel4;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel5;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel6;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel7;

    _register = &manager->modbus_ctx._register;
    _log_controller_ctx = &manager->log_controller_ctx;
    log_controller_ioctl_param_channel0 = &manager->sampling_log.log_controller_ioctl_param_channel0;
    log_controller_ioctl_param_channel1 = &manager->sampling_log.log_controller_ioctl_param_channel1;
    log_controller_ioctl_param_channel2 = &manager->sampling_log.log_controller_ioctl_param_channel2;
    log_controller_ioctl_param_channel3 = &manager->sampling_log.log_controller_ioctl_param_channel3;
    log_controller_ioctl_param_channel4 = &manager->sampling_log.log_controller_ioctl_param_channel4;
    log_controller_ioctl_param_channel5 = &manager->sampling_log.log_controller_ioctl_param_channel5;
    log_controller_ioctl_param_channel6 = &manager->sampling_log.log_controller_ioctl_param_channel6;
    log_controller_ioctl_param_channel7 = &manager->sampling_log.log_controller_ioctl_param_channel7;

    switch (logical_channel_id) {
        case PHYSICAL_CHANNEL_01_UNIT_IDENTIFIER:
            source_logical_voltage = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_01_UNIT_IDENTIFIER].logic_voltage_region);
            source_logical_current = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_01_UNIT_IDENTIFIER].logic_current_region);
            log_controller_ioctl_param = log_controller_ioctl_param_channel0;
        break;
        case PHYSICAL_CHANNEL_02_UNIT_IDENTIFIER:
            source_logical_voltage = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_02_UNIT_IDENTIFIER].logic_voltage_region);
            source_logical_current = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_02_UNIT_IDENTIFIER].logic_current_region);
            log_controller_ioctl_param = log_controller_ioctl_param_channel1;
        break;
        case PHYSICAL_CHANNEL_03_UNIT_IDENTIFIER:
            source_logical_voltage = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_03_UNIT_IDENTIFIER].logic_voltage_region);
            source_logical_current = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_03_UNIT_IDENTIFIER].logic_current_region);
            log_controller_ioctl_param = log_controller_ioctl_param_channel2;
        break;
        case PHYSICAL_CHANNEL_04_UNIT_IDENTIFIER:
            source_logical_voltage = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_04_UNIT_IDENTIFIER].logic_voltage_region);
            source_logical_current = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_04_UNIT_IDENTIFIER].logic_current_region);
            log_controller_ioctl_param = log_controller_ioctl_param_channel3;
        break;
        case PHYSICAL_CHANNEL_05_UNIT_IDENTIFIER:
            source_logical_voltage = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_05_UNIT_IDENTIFIER].logic_voltage_region);
            source_logical_current = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_05_UNIT_IDENTIFIER].logic_current_region);
            log_controller_ioctl_param = log_controller_ioctl_param_channel4;
        break;
        case PHYSICAL_CHANNEL_06_UNIT_IDENTIFIER:
            source_logical_voltage = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_06_UNIT_IDENTIFIER].logic_voltage_region);
            source_logical_current = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_06_UNIT_IDENTIFIER].logic_current_region);
            log_controller_ioctl_param = log_controller_ioctl_param_channel5;
        break;
        case PHYSICAL_CHANNEL_07_UNIT_IDENTIFIER:
            source_logical_voltage = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_07_UNIT_IDENTIFIER].logic_voltage_region);
            source_logical_current = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_07_UNIT_IDENTIFIER].logic_current_region);
            log_controller_ioctl_param = log_controller_ioctl_param_channel6;
        break;
        case PHYSICAL_CHANNEL_08_UNIT_IDENTIFIER:
            source_logical_voltage = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_08_UNIT_IDENTIFIER].logic_voltage_region);
            source_logical_current = combine_4paired_register_region(&_register->logical_channel[PHYSICAL_CHANNEL_08_UNIT_IDENTIFIER].logic_current_region);
            log_controller_ioctl_param = log_controller_ioctl_param_channel7;
        break;
        default:
            printf("clean_one_logical_channel_log unknow logical_channel_id\n");
            return ERROR_INTERNEL_ERRORS;
    }

    right_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.right_boundary_time_region;
    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    step_pointer_reg = _register->logical_channel[logical_channel_id].step_pointer;
    cycle_count_reg = _register->logical_channel[logical_channel_id].step_param.info.cycle.count;
    step_type_reg = _register->logical_channel[logical_channel_id].step_param.step_type;
    accumulated_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_logic_capacity_region;
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;
    
    right_boundary_time = combine_4paired_register_region(right_boundary_time_region);
    holding_time = combine_4paired_register_region(holding_time_region);
    now_time_millisecond = right_boundary_time % 1000;
    unix_timestamp = right_boundary_time / 1000;
    now_time = localtime(&unix_timestamp);
    accumulated_logic_capacity_unsign = combine_4paired_register_region(accumulated_logic_capacity_region);
    accumulated_step_logic_capacity_unsign = combine_4paired_register_region(accumulated_step_logic_capacity_region);


    if(source_logical_voltage & 0x8000000000000000){ //negative
        sign = -1;
        source_logical_voltage_amplitude = source_logical_voltage & 0x7fffffffffffffff;
    } else {
        sign = 1;
        source_logical_voltage_amplitude = source_logical_voltage;
    }
    present_logical_voltage = (double)source_logical_voltage_amplitude * sign;
    
    if(source_logical_current & 0x8000000000000000){ //negative
        sign = -1;
        source_logical_current_amplitude = source_logical_current & 0x7fffffffffffffff;
    } else {
        sign = 1;
        source_logical_current_amplitude = source_logical_current;
    }
    present_logical_current = (double)source_logical_current_amplitude * sign;

    if(accumulated_logic_capacity_unsign & 0x8000000000000000){ //negative
        sign = -1;
        accumulated_logic_capacity_amplitude = accumulated_logic_capacity_unsign & 0x7fffffffffffffff;
    } else {
        sign = 1;
        accumulated_logic_capacity_amplitude = accumulated_logic_capacity_unsign;
    }
    accumulated_logic_capacity_sign = (double)accumulated_logic_capacity_amplitude * sign;

    if(accumulated_step_logic_capacity_unsign & 0x8000000000000000){ //negative
        sign = -1;
        accumulated_step_logic_capacity_amplitude = accumulated_step_logic_capacity_unsign & 0x7fffffffffffffff;
    } else {
        sign = 1;
        accumulated_step_logic_capacity_amplitude = accumulated_step_logic_capacity_unsign;
    }
    accumulated_step_logic_capacity_sign = (double)accumulated_step_logic_capacity_amplitude * sign;


    cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
    log_controller_ioctl_param->log_message_param.in.level = LOG_LEVEL_DEBUG;
    //2019-1-1 10:10:00 999,cycle=0,step=0,step_type=0,worktime=0.1,voltage=4000.2,current=2.001,accumulated_logic_capacity=,energy=0,tempture=0.0,stop_flag=0,stop_msg=0, accumulated_step_logic_capacity\n
                                                                           //YY-MM-DD hh:mm:ss ms,cy,st,st,worktime ,volt,curr,accumulate,e,t,s,st,accumulat\n
    snprintf(log_controller_ioctl_param->log_message_param.in.message, 255, "%d-%d-%d %d:%d:%d %d,%d,%d,%d,%"PRIu64",%.5f,%.4f,%"PRIu64",0,0,0,"",%"PRIu64"\n",
        now_time->tm_year+1900, now_time->tm_mon, now_time->tm_mday, now_time->tm_hour, now_time->tm_min, now_time->tm_sec,
        now_time_millisecond,
        *cycle_count_reg,
        *step_pointer_reg,
        *step_type_reg,
        holding_time,
        present_logical_voltage/100000, present_logical_current/10000, accumulated_logic_capacity_sign,
        accumulated_step_logic_capacity_sign
    );
    ret = _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, log_controller_ioctl_param);
    if(ret < 0) {
        printf("channel log message fail\n");
        return ret;
    }

#if 0
    printf("%d-%d-%d %d:%d:%d %d,0,%d,0,%"PRIu64",%.3f,%.4f,0,0,0,0,""\n",
        now_time->tm_year, now_time->tm_mon, now_time->tm_mday, now_time->tm_hour, now_time->tm_min, now_time->tm_sec,
        now_time_millisecond,
        *step_pointer_reg,
        holding_time,
        present_logical_voltage/100000, present_logical_current/10000
    );
#endif
    //set_4paired_register_region(debug_log_holding_time_reg, (uint64_t)(now_time.tm_epoch_miliseconds - channel_start_log_timestampt_reg[i]));
    //channel_last_sample_timestampt_reg[i] = now_time.tm_epoch_miliseconds;
    return 0;
}

int check_record_one_logical_channel_currve_log(Manager * _manager, int channel_id)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = channel_id, ret;
    uint64_t last_curver_log_record_time, curver_log_record_deley, right_boundary_time, passby_time;
    _4paired_register_region *last_curver_log_record_time_region, *step_info_common_curver_log_record_deley_region, *right_boundary_time_region;

    switch (logical_channel_id) {
        case PHYSICAL_CHANNEL_01_UNIT_IDENTIFIER:
        break;
        case PHYSICAL_CHANNEL_02_UNIT_IDENTIFIER:
        break;
        case PHYSICAL_CHANNEL_03_UNIT_IDENTIFIER:
        break;
        case PHYSICAL_CHANNEL_04_UNIT_IDENTIFIER:
        break;
        case PHYSICAL_CHANNEL_05_UNIT_IDENTIFIER:
        break;
        case PHYSICAL_CHANNEL_06_UNIT_IDENTIFIER:
        break;
        case PHYSICAL_CHANNEL_07_UNIT_IDENTIFIER:
        break;
        case PHYSICAL_CHANNEL_08_UNIT_IDENTIFIER:
        break;
        default:
            printf("check_record_one_logical_channel_currve_log unknow logical_channel_id\n");
            return ERROR_INTERNEL_ERRORS;
    }

    _register = &manager->modbus_ctx._register;

    last_curver_log_record_time_region = &_register->logical_channel[logical_channel_id].temp.last_sampling_record_time_region;
    step_info_common_curver_log_record_deley_region = &_register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region;
    right_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.right_boundary_time_region;

    last_curver_log_record_time = combine_4paired_register_region(last_curver_log_record_time_region);
    curver_log_record_deley = combine_4paired_register_region(step_info_common_curver_log_record_deley_region);
    right_boundary_time = combine_4paired_register_region(right_boundary_time_region);

    passby_time = right_boundary_time - last_curver_log_record_time;

    if(passby_time < curver_log_record_deley){
        return 0;
    }

    ret = record_one_logical_channel_sampling(manager, logical_channel_id);
    if(ret < 0){
        printf("check_record_one_logical_channel_currve_log->record_one_logical_channel_currve_log fail\n");
        return ret;
    }
    update_4paired_register_region(last_curver_log_record_time_region, right_boundary_time);
    return 0;
}

void init_system_log(Manager *manager)
{
    int ret, cmd;
    Manager *_manager = manager;
    register_log_parameters register_log_param;
    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param;

    _log_controller_ctx = &_manager->log_controller_ctx;
    _log_controller_ioctl_param = &_manager->log_controller_ioctl_param;

    memset(_manager->log_file_name, '\0', LENGTH_OF_LOG_FILE_NAME);

    strcpy(_manager->log_file_name, "system.log");

    ret = log_controller_init(_log_controller_ctx);
    if(ret < 0) {
        printf("log_controller_init fail\n");
        exit(-1);
    }

    strcpy(register_log_param.in.path, "./");
    strcpy(register_log_param.in.signature, "system.log");
    ret = _log_controller_ctx->ops.register_context(_log_controller_ctx, &register_log_param);
    if(ret < 0) {
        printf("register_log fail\n");
        exit(-1);
    }

    _log_controller_ctx->ops.list_resources(_log_controller_ctx);

    cmd = LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX;
    strcpy(_log_controller_ioctl_param->acquire_log_ctx_param.in.signature, "system.log");
    ret = _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, _log_controller_ioctl_param);
    if(ret < 0) {
        printf("acquire log context fail\n");
        exit(-1);
    }

#if 0
    cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
    strcpy(_log_controller_ioctl_param->log_message_param.in.message, "Hello, World!!!");
    _log_controller_ioctl_param->log_message_param.in.level = LOG_LEVEL_DEBUG;
    ret = _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, _log_controller_ioctl_param);
    if(ret < 0) {
        printf("log message fail\n");
        exit(-1);
    }
#endif

}

void init_sampling_log(Manager *_manager)
{
    int ret, cmd;
    Manager *manager = _manager;
    log_controller_context *_log_controller_ctx;
    register_log_parameters register_log_parameters_channel0;
    register_log_parameters register_log_parameters_channel1;
    register_log_parameters register_log_parameters_channel2;
    register_log_parameters register_log_parameters_channel3;
    register_log_parameters register_log_parameters_channel4;
    register_log_parameters register_log_parameters_channel5;
    register_log_parameters register_log_parameters_channel6;
    register_log_parameters register_log_parameters_channel7;

    log_controller_ioctl_parameters *log_controller_ioctl_param_channel0;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel1;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel2;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel3;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel4;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel5;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel6;
    log_controller_ioctl_parameters *log_controller_ioctl_param_channel7;

    log_controller_ioctl_param_channel0 = &_manager->sampling_log.log_controller_ioctl_param_channel0;
    log_controller_ioctl_param_channel1 = &_manager->sampling_log.log_controller_ioctl_param_channel1;
    log_controller_ioctl_param_channel2 = &_manager->sampling_log.log_controller_ioctl_param_channel2;
    log_controller_ioctl_param_channel3 = &_manager->sampling_log.log_controller_ioctl_param_channel3;
    log_controller_ioctl_param_channel4 = &_manager->sampling_log.log_controller_ioctl_param_channel4;
    log_controller_ioctl_param_channel5 = &_manager->sampling_log.log_controller_ioctl_param_channel5;
    log_controller_ioctl_param_channel6 = &_manager->sampling_log.log_controller_ioctl_param_channel6;
    log_controller_ioctl_param_channel7 = &_manager->sampling_log.log_controller_ioctl_param_channel7;

    _log_controller_ctx = &manager->log_controller_ctx;
    
    // Initialize LOG Controller
    ret = log_controller_init(_log_controller_ctx);
    if(ret < 0) {
        printf("log_controller_init fail\n");
        exit(-1);
    }

    strcpy(register_log_parameters_channel0.in.path, "/home/pi/hf_formation/run/data");
    strcpy(register_log_parameters_channel0.in.signature, "0000.dat");
    ret = _log_controller_ctx->ops.register_context(_log_controller_ctx, &register_log_parameters_channel0);
    if(ret < 0) {
        printf("register_log fail\n");
        exit(-1);
    }

    strcpy(register_log_parameters_channel1.in.path, "/home/pi/hf_formation/run/data");
    strcpy(register_log_parameters_channel1.in.signature, "0001.dat");
    ret = _log_controller_ctx->ops.register_context(_log_controller_ctx, &register_log_parameters_channel1);
    if(ret < 0) {
        printf("register_log fail\n");
        exit(-1);
    }

    strcpy(register_log_parameters_channel2.in.path, "/home/pi/hf_formation/run/data");
    strcpy(register_log_parameters_channel2.in.signature, "0002.dat");
    ret = _log_controller_ctx->ops.register_context(_log_controller_ctx, &register_log_parameters_channel2);
    if(ret < 0) {
        printf("register_log fail\n");
        exit(-1);
    }

    strcpy(register_log_parameters_channel3.in.path, "/home/pi/hf_formation/run/data");
    strcpy(register_log_parameters_channel3.in.signature, "0003.dat");
    ret = _log_controller_ctx->ops.register_context(_log_controller_ctx, &register_log_parameters_channel3);
    if(ret < 0) {
        printf("register_log fail\n");
        exit(-1);
    }

    strcpy(register_log_parameters_channel4.in.path, "/home/pi/hf_formation/run/data");
    strcpy(register_log_parameters_channel4.in.signature, "0004.dat");
    ret = _log_controller_ctx->ops.register_context(_log_controller_ctx, &register_log_parameters_channel4);
    if(ret < 0) {
        printf("register_log fail\n");
        exit(-1);
    }

    strcpy(register_log_parameters_channel5.in.path, "/home/pi/hf_formation/run/data");
    strcpy(register_log_parameters_channel5.in.signature, "0005.dat");
    ret = _log_controller_ctx->ops.register_context(_log_controller_ctx, &register_log_parameters_channel5);
    if(ret < 0) {
        printf("register_log fail\n");
        exit(-1);
    }

    strcpy(register_log_parameters_channel6.in.path, "/home/pi/hf_formation/run/data");
    strcpy(register_log_parameters_channel6.in.signature, "0006.dat");
    ret = _log_controller_ctx->ops.register_context(_log_controller_ctx, &register_log_parameters_channel6);
    if(ret < 0) {
        printf("register_log fail\n");
        exit(-1);
    }

    strcpy(register_log_parameters_channel7.in.path, "/home/pi/hf_formation/run/data");
    strcpy(register_log_parameters_channel7.in.signature, "0007.dat");
    ret = _log_controller_ctx->ops.register_context(_log_controller_ctx, &register_log_parameters_channel7);
    if(ret < 0) {
        printf("register_log fail\n");
        exit(-1);
    }

    cmd = LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX;
    strcpy(log_controller_ioctl_param_channel0->acquire_log_ctx_param.in.signature, "0000.dat");
    ret = _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, log_controller_ioctl_param_channel0);
    if(ret < 0) {
        printf("acquire log context fail\n");
        exit(-1);
    }

    cmd = LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX;
    strcpy(log_controller_ioctl_param_channel1->acquire_log_ctx_param.in.signature, "0001.dat");
    ret = _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, log_controller_ioctl_param_channel1);
    if(ret < 0) {
        printf("acquire log context fail\n");
        exit(-1);
    }

    cmd = LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX;
    strcpy(log_controller_ioctl_param_channel2->acquire_log_ctx_param.in.signature, "0002.dat");
    ret = _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, log_controller_ioctl_param_channel2);
    if(ret < 0) {
        printf("acquire log context fail\n");
        exit(-1);
    }

    cmd = LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX;
    strcpy(log_controller_ioctl_param_channel3->acquire_log_ctx_param.in.signature, "0003.dat");
    ret = _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, log_controller_ioctl_param_channel3);
    if(ret < 0) {
        printf("acquire log context fail\n");
        exit(-1);
    }

    cmd = LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX;
    strcpy(log_controller_ioctl_param_channel4->acquire_log_ctx_param.in.signature, "0004.dat");
    ret = _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, log_controller_ioctl_param_channel4);
    if(ret < 0) {
        printf("acquire log context fail\n");
        exit(-1);
    }

    cmd = LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX;
    strcpy(log_controller_ioctl_param_channel5->acquire_log_ctx_param.in.signature, "0005.dat");
    ret = _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, log_controller_ioctl_param_channel5);
    if(ret < 0) {
        printf("acquire log context fail\n");
        exit(-1);
    }

    cmd = LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX;
    strcpy(log_controller_ioctl_param_channel6->acquire_log_ctx_param.in.signature, "0006.dat");
    ret = _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, log_controller_ioctl_param_channel6);
    if(ret < 0) {
        printf("acquire log context fail\n");
        exit(-1);
    }

    cmd = LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX;
    strcpy(log_controller_ioctl_param_channel7->acquire_log_ctx_param.in.signature, "0007.dat");
    ret = _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, log_controller_ioctl_param_channel7);
    if(ret < 0) {
        printf("acquire log context fail\n");
        exit(-1);
    }

    _log_controller_ctx->ops.list_resources(_log_controller_ctx);
}

void uint16_t_to_uint8_t(uint16_t conver_value, uint8_t *byte_array)
{
    uint16_t value = conver_value;
    uint8_t L_byte, H_byte;
    
    L_byte = (value >> 0) & 0xFF;
    H_byte = (value >> 8) & 0xFF;
    
    byte_array[0] = L_byte;
    byte_array[1] = H_byte;
}

void uint32_t_to_uint8_t(uint32_t conver_value, uint8_t *byte_array)
{
    uint32_t value = conver_value;
    uint8_t LL_byte, LH_byte, HL_byte, HH_byte;
    
    LL_byte = (value >>  0) & 0xFF;
    LH_byte = (value >>  8) & 0xFF;
    HL_byte = (value >> 16) & 0xFF;
    HH_byte = (value >> 24) & 0xFF;
    
    byte_array[0] = LL_byte;
    byte_array[1] = LH_byte;
    byte_array[2] = HL_byte;
    byte_array[3] = HH_byte;
}

uint16_t combine_uint8_t_to_uint16_t(uint8_t *byte_array)
{
    uint16_t value = 0;
    uint16_t L_byte, H_byte;

    L_byte = byte_array[0];
    H_byte = byte_array[1];
    
    value = value | (L_byte << 0);
    value = value | (H_byte << 8);
    
    return value;
}

uint32_t combine_uint8_t_to_uint32_t(uint8_t *byte_array)
{
    uint32_t value = 0;
    uint32_t LL_byte, LH_byte, HL_byte, HH_byte;

    LL_byte = byte_array[0];
    LH_byte = byte_array[1];
    HL_byte = byte_array[2];
    HH_byte = byte_array[3];
    
    value = value | (LL_byte << 0);
    value = value | (LH_byte << 8);
    value = value | (HL_byte << 16);
    value = value | (HH_byte << 24);
    
    return value;

}

int creat_default_info_file()
{
    FILE *KB_file;
    int ret, i, j, byte_count = 4, channel_count = 8;
    uint8_t byte_array[byte_count];
    uint32_t host_ip=0xC0A8000E;//192.168.0.14
    byte_array[0] = 0x10;
    byte_array[1] = 0x27;
    byte_array[2] = 0;
    byte_array[3] = 0;

    KB_file = fopen("/home/pi/KB_file.binary","ab+");
    if(KB_file == NULL){
        printf("creat_default_KB_file creat file fail\n");
        return ERROR_INTERNEL_ERRORS;
    }

    for(j=0;j<channel_count;j++){
        for(i=0;i<14;i++){
            ret = fwrite(byte_array, 1, byte_count, KB_file);
            if(ret != byte_count){
                printf("creat_default_KB_file write KB_file error\n");
                return ERROR_INTERNEL_ERRORS;
            }
        }
    }

    uint32_t_to_uint8_t(host_ip, byte_array);
    byte_count = 4;

    ret = fwrite(byte_array, 1, byte_count, KB_file);
    if(ret != byte_count){
        printf("creat_default_KB_file write KB_file error\n");
        return ERROR_INTERNEL_ERRORS;
    }


    fclose(KB_file);
    
    return 0;
}

int write_info_to_file(uint8_t *byte_array, uint16_t byte_count, uint16_t byte_offset_count)
{
    FILE *KB_file;
    int ret;

    KB_file = fopen("/home/pi/KB_file.binary","rb+");
    if(KB_file == NULL){
        ret = creat_default_info_file();
        if(ret != 0){
            printf("write_info_to_file->creat_default_KB_file fail\n");
            return ERROR_INTERNEL_ERRORS;
        }
        
        KB_file = fopen("/home/pi/KB_file.binary","wb");
        if(KB_file == NULL){
            printf("write_info_to_file loss KB file\n");
            return ERROR_INTERNEL_ERRORS;
        }
    }

    ret = fseek(KB_file, byte_offset_count, SEEK_SET);
    if(ret != 0){
        printf("write_info_to_file seek KB_file error\n");
        return ERROR_INTERNEL_ERRORS;
    }

    ret = fwrite(byte_array, 1, byte_count, KB_file);
    if(ret != byte_count){
        printf("write_info_to_file write KB_file error\n");
        return ERROR_INTERNEL_ERRORS;
    }

    ret = fclose(KB_file);
    if(ret != 0){
        printf("write_info_to_file close KB_file error\n");
        return ERROR_INTERNEL_ERRORS;
    }

    return 0;
}

int read_info_from_file(uint8_t *byte_array, uint16_t byte_count, uint16_t byte_offset_count)
{
    FILE *KB_file;
    int ret;

    KB_file = fopen("/home/pi/KB_file.binary","rb");
    if(KB_file == NULL){
        ret = creat_default_info_file();
        if(ret != 0){
            printf("read_info_from_file->creat_default_KB_file fail\n");
            return ERROR_INTERNEL_ERRORS;
        }
        
        KB_file = fopen("/home/pi/KB_file.binary","rb");
        if(KB_file == NULL){
            printf("read_info_from_file loss KB file\n");
            return ERROR_INTERNEL_ERRORS;
        }
    }

    ret = fseek(KB_file, byte_offset_count, SEEK_SET);
    if(ret != 0){
        printf("read_info_from_file seek KB_file error\n");
        return ERROR_INTERNEL_ERRORS;
    }

    ret = fread(byte_array, 1, byte_count, KB_file);
    if(ret != byte_count){
        printf("read_info_from_file read KB_file error\n");
        return ERROR_INTERNEL_ERRORS;
    }

    ret = fclose(KB_file);
    if(ret != 0){
        printf("read_info_from_file close KB_file error\n");
        return ERROR_INTERNEL_ERRORS;
    }

    return 0;
}






int save_info_to_ROM(Manager *_manager, int channel_id, int _info_type)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int ret,physical_channel_id, info_type;
    
    uint16_t data_start_address, data_byte_count, channel_base_address_offset, suboffset;
    uint16_t B_value_sign;
    uint8_t byte_array[4];

    uint16_t *K_value_reg;
    _4paired_register_region *B_value_region;
    uint16_t K_value, unit_convert_B_value;
    uint64_t B_value, B_value_amplitude;

    _2paired_register_region *device_host_region;
    uint32_t device_host_value;
    
    physical_channel_id = channel_id;
    info_type = _info_type;

    _register = &manager->modbus_ctx._register;

    switch (physical_channel_id) {
        case PHYSICAL_CHANNEL_1_ID:
            channel_base_address_offset = 0 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_2_ID:
            channel_base_address_offset = 1 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_3_ID:
            channel_base_address_offset = 2 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_4_ID:
            channel_base_address_offset = 3 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_5_ID:
            channel_base_address_offset = 4 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_6_ID:
            channel_base_address_offset = 5 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_7_ID:
            channel_base_address_offset = 6 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_8_ID:
            channel_base_address_offset = 7 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        default:
            printf("save_info_to_ROM unknow physical channel\n");
            return ERROR_INTERNEL_ERRORS;
        break;
    }

    switch(info_type){
        
        case INFO_TYPE_BV_K:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_BV_K_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.bv_k;
            
            K_value = *K_value_reg;
            uint16_t_to_uint8_t(K_value, byte_array);
        break;
        
        case INFO_TYPE_VU_K:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_VU_K_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.vu_k;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;

        case INFO_TYPE_C_VI_K_LEVEL_1:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_C_VI_K_LEVEL_1_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.c_vi_k_level_1;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;
        
        case INFO_TYPE_C_VI_K_LEVEL_2:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_C_VI_K_LEVEL_2_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.c_vi_k_level_2;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;

        case INFO_TYPE_C_VI_K_LEVEL_3:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_C_VI_K_LEVEL_3_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.c_vi_k_level_3;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;

        case INFO_TYPE_D_VI_K_LEVEL_1:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_D_VI_K_LEVEL_1_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.d_vi_k_level_1;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;
        
        case INFO_TYPE_D_VI_K_LEVEL_2:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_D_VI_K_LEVEL_2_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.d_vi_k_level_2;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;

        case INFO_TYPE_D_VI_K_LEVEL_3:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_D_VI_K_LEVEL_3_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.d_vi_k_level_3;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;

        case INFO_TYPE_C_CV_K_LEVEL_1:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_C_CV_K_LEVEL_1_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.c_cv_k_level_1;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;

        case INFO_TYPE_C_CV_K_LEVEL_2:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_C_CV_K_LEVEL_2_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.c_cv_k_level_2;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;

        case INFO_TYPE_C_CV_K_LEVEL_3:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_C_CV_K_LEVEL_3_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.c_cv_k_level_3;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;

        case INFO_TYPE_D_CV_K_LEVEL_1:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_D_CV_K_LEVEL_1_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.d_cv_k_level_1;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;

        case INFO_TYPE_D_CV_K_LEVEL_2:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_D_CV_K_LEVEL_2_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.d_cv_k_level_2;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;

        case INFO_TYPE_D_CV_K_LEVEL_3:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_D_CV_K_LEVEL_3_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.d_cv_k_level_3;
            
            K_value = *K_value_reg;
            
            uint16_t_to_uint8_t(K_value, byte_array);
        break;




        case INFO_TYPE_BV_B:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_BV_B_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.bv_b_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude / UNIT_FACTOR_BETWEEN_01MV_AND_10NV;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;
        
        case INFO_TYPE_VU_B:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_VU_B_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.vu_b_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude / UNIT_FACTOR_BETWEEN_01MV_AND_10NV;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;

        case INFO_TYPE_C_VI_B_LEVEL_1:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_C_VI_B_LEVEL_1_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.c_vi_b_level1_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;

        case INFO_TYPE_C_VI_B_LEVEL_2:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_C_VI_B_LEVEL_2_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.c_vi_b_level2_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;

        case INFO_TYPE_C_VI_B_LEVEL_3:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_C_VI_B_LEVEL_3_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.c_vi_b_level3_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;

        case INFO_TYPE_D_VI_B_LEVEL_1:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_D_VI_B_LEVEL_1_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.d_vi_b_level1_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;

        case INFO_TYPE_D_VI_B_LEVEL_2:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_D_VI_B_LEVEL_2_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.d_vi_b_level2_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;

        case INFO_TYPE_D_VI_B_LEVEL_3:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_D_VI_B_LEVEL_3_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.d_vi_b_level3_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;

        case INFO_TYPE_C_CV_B_LEVEL_1:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_C_CV_B_LEVEL_1_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.c_cv_b_level1_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;

        case INFO_TYPE_C_CV_B_LEVEL_2:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_C_CV_B_LEVEL_2_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.c_cv_b_level2_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;

        case INFO_TYPE_C_CV_B_LEVEL_3:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_C_CV_B_LEVEL_3_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.c_cv_b_level3_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;

        case INFO_TYPE_D_CV_B_LEVEL_1:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_D_CV_B_LEVEL_1_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.d_cv_b_level1_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;

        case INFO_TYPE_D_CV_B_LEVEL_2:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_D_CV_B_LEVEL_2_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.d_cv_b_level2_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;

        case INFO_TYPE_D_CV_B_LEVEL_3:
            
            data_byte_count = 2;
            suboffset = PHYSICAL_CHANNEL_KB_D_CV_B_LEVEL_3_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.d_cv_b_level3_region;
            
            B_value = combine_4paired_register_region(B_value_region);
            
            if(B_value & 0x8000000000000000){ //negative
                B_value_sign = 0x8000;
                B_value_amplitude = B_value & 0x7fffffffffffffff;
            } else {
                B_value_sign = 0x0000;
                B_value_amplitude = B_value;
            }
            
            unit_convert_B_value = B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            uint16_t_to_uint8_t(unit_convert_B_value, byte_array);
        break;


        case INFO_TYPE_DEVICE_HOST:
            
            data_byte_count = 4;
            suboffset = PHYSICAL_CHANNEL8_EXTRA_DEVICE_HOST_SUBOFFSET_BYTE;
            // for lock data channel offset
            channel_base_address_offset = 7 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
            device_host_region = &_register->physical_channel[PHYSICAL_CHANNEL_1_ID].device_host;
            
            device_host_value = combine_2paired_register_region(device_host_region);

            uint32_t_to_uint8_t(device_host_value, byte_array);
        break;


        default:
            printf("save_info_to_ROM unknow info_type\n");
            return ERROR_INTERNEL_ERRORS;
        break;
    }
    data_start_address = channel_base_address_offset + suboffset;
    ret = write_info_to_file(byte_array, data_byte_count, data_start_address);
    if(ret != 0){
        printf("save_info_to_ROM->write_info_to_file fail\n");
        return ret;
    }
    return 0;
}

int reload_info_from_ROM(Manager *_manager, int channel_id, int _info_type)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int ret,physical_channel_id, info_type;
    
    uint16_t data_start_address, data_byte_count, channel_base_address_offset, suboffset;
    uint8_t byte_array[4];

    uint16_t *K_value_reg;
    _4paired_register_region *B_value_region;
    uint16_t K_value, B_value, B_value_amplitude;
    uint64_t unit_convert_B_value,B_value_sign;

    _2paired_register_region *device_host_region;
    uint32_t device_host_value;

    physical_channel_id = channel_id;
    info_type = _info_type;

    _register = &manager->modbus_ctx._register;

    switch (physical_channel_id) {

        case PHYSICAL_CHANNEL_1_ID:
            channel_base_address_offset = 0 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_2_ID:
            channel_base_address_offset = 1 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_3_ID:
            channel_base_address_offset = 2 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_4_ID:
            channel_base_address_offset = 3 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_5_ID:
            channel_base_address_offset = 4 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_6_ID:
            channel_base_address_offset = 5 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_7_ID:
            channel_base_address_offset = 6 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        case PHYSICAL_CHANNEL_8_ID:
            channel_base_address_offset = 7 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
        break;

        default:
            printf("reload_info_from_ROM unknow physical channel\n");
            return ERROR_INTERNEL_ERRORS;
        break;
    }

    switch(info_type){
        
        case INFO_TYPE_BV_K:
            
            suboffset = PHYSICAL_CHANNEL_KB_BV_K_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.bv_k;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_BV_K fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;
        
        case INFO_TYPE_VU_K:
            
            suboffset = PHYSICAL_CHANNEL_KB_VU_K_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.vu_k;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_VU_K fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;

        case INFO_TYPE_C_VI_K_LEVEL_1:
            
            suboffset = PHYSICAL_CHANNEL_KB_C_VI_K_LEVEL_1_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.c_vi_k_level_1;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_C_VI_K_LEVEL_1 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;

        case INFO_TYPE_C_VI_K_LEVEL_2:
            
            suboffset = PHYSICAL_CHANNEL_KB_C_VI_K_LEVEL_2_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.c_vi_k_level_2;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_C_VI_K_LEVEL_2 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;

        case INFO_TYPE_C_VI_K_LEVEL_3:
            
            suboffset = PHYSICAL_CHANNEL_KB_C_VI_K_LEVEL_3_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.c_vi_k_level_3;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_C_VI_K_LEVEL_3 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;

        case INFO_TYPE_D_VI_K_LEVEL_1:
            
            suboffset = PHYSICAL_CHANNEL_KB_D_VI_K_LEVEL_1_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.d_vi_k_level_1;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_VI_K_LEVEL_1 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;

        case INFO_TYPE_D_VI_K_LEVEL_2:
            
            suboffset = PHYSICAL_CHANNEL_KB_D_VI_K_LEVEL_2_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.d_vi_k_level_2;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_VI_K_LEVEL_2 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;

        case INFO_TYPE_D_VI_K_LEVEL_3:
            
            suboffset = PHYSICAL_CHANNEL_KB_D_VI_K_LEVEL_3_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.d_vi_k_level_3;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_VI_K_LEVEL_3 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;

        case INFO_TYPE_C_CV_K_LEVEL_1:
            
            suboffset = PHYSICAL_CHANNEL_KB_C_CV_K_LEVEL_1_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.c_cv_k_level_1;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_C_CV_K_LEVEL_1 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;

        case INFO_TYPE_C_CV_K_LEVEL_2:
            
            suboffset = PHYSICAL_CHANNEL_KB_C_CV_K_LEVEL_2_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.c_cv_k_level_2;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_C_CV_K_LEVEL_2 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;

        case INFO_TYPE_C_CV_K_LEVEL_3:
            
            suboffset = PHYSICAL_CHANNEL_KB_C_CV_K_LEVEL_3_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.c_cv_k_level_3;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_C_CV_K_LEVEL_3 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;

        case INFO_TYPE_D_CV_K_LEVEL_1:
            
            suboffset = PHYSICAL_CHANNEL_KB_D_CV_K_LEVEL_1_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.d_cv_k_level_1;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_CV_K_LEVEL_1 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;

        case INFO_TYPE_D_CV_K_LEVEL_2:
            
            suboffset = PHYSICAL_CHANNEL_KB_D_CV_K_LEVEL_2_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.d_cv_k_level_2;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_CV_K_LEVEL_2 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;

        case INFO_TYPE_D_CV_K_LEVEL_3:
            
            suboffset = PHYSICAL_CHANNEL_KB_D_CV_K_LEVEL_3_SUBOFFSET_BYTE;
            K_value_reg = _register->physical_channel[physical_channel_id].kb.d_cv_k_level_3;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_CV_K_LEVEL_3 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            K_value = combine_uint8_t_to_uint16_t(byte_array);
            
            *K_value_reg = K_value;
        break;




        case INFO_TYPE_BV_B:
            
            suboffset = PHYSICAL_CHANNEL_KB_BV_B_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.bv_b_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_BV_B fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude * (uint64_t)UNIT_FACTOR_BETWEEN_01MV_AND_10NV;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;

        case INFO_TYPE_VU_B:
            
            suboffset = PHYSICAL_CHANNEL_KB_VU_B_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.vu_b_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_VU_B fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude * (uint64_t)UNIT_FACTOR_BETWEEN_01MV_AND_10NV;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;


        case INFO_TYPE_C_VI_B_LEVEL_1:
            
            suboffset = PHYSICAL_CHANNEL_KB_C_VI_B_LEVEL_1_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.c_vi_b_level1_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_C_VI_B_LEVEL_1 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;

        case INFO_TYPE_C_VI_B_LEVEL_2:
            
            suboffset = PHYSICAL_CHANNEL_KB_C_VI_B_LEVEL_2_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.c_vi_b_level2_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_C_VI_B_LEVEL_2 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;

        case INFO_TYPE_C_VI_B_LEVEL_3:
            
            suboffset = PHYSICAL_CHANNEL_KB_C_VI_B_LEVEL_3_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.c_vi_b_level3_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_C_VI_B_LEVEL_3 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;

        case INFO_TYPE_D_VI_B_LEVEL_1:
            
            suboffset = PHYSICAL_CHANNEL_KB_D_VI_B_LEVEL_1_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.d_vi_b_level1_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_VI_B_LEVEL_1 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;

        case INFO_TYPE_D_VI_B_LEVEL_2:
            
            suboffset = PHYSICAL_CHANNEL_KB_D_VI_B_LEVEL_2_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.d_vi_b_level2_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_VI_B_LEVEL_2 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;

        case INFO_TYPE_D_VI_B_LEVEL_3:
            
            suboffset = PHYSICAL_CHANNEL_KB_D_VI_B_LEVEL_3_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.d_vi_b_level3_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_VI_B_LEVEL_3 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;

        case INFO_TYPE_C_CV_B_LEVEL_1:
            
            suboffset = PHYSICAL_CHANNEL_KB_C_CV_B_LEVEL_1_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.c_cv_b_level1_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_C_CV_B_LEVEL_1 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;

        case INFO_TYPE_C_CV_B_LEVEL_2:
            
            suboffset = PHYSICAL_CHANNEL_KB_C_CV_B_LEVEL_2_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.c_cv_b_level2_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_C_CV_B_LEVEL_2 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;

        case INFO_TYPE_C_CV_B_LEVEL_3:
            
            suboffset = PHYSICAL_CHANNEL_KB_C_CV_B_LEVEL_3_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.c_cv_b_level3_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_C_CV_B_LEVEL_3 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;

        case INFO_TYPE_D_CV_B_LEVEL_1:
            
            suboffset = PHYSICAL_CHANNEL_KB_D_CV_B_LEVEL_1_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.d_cv_b_level1_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_CV_B_LEVEL_1 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;

        case INFO_TYPE_D_CV_B_LEVEL_2:
            
            suboffset = PHYSICAL_CHANNEL_KB_D_CV_B_LEVEL_2_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.d_cv_b_level2_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_CV_B_LEVEL_2 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;

        case INFO_TYPE_D_CV_B_LEVEL_3:
            
            suboffset = PHYSICAL_CHANNEL_KB_D_CV_B_LEVEL_3_SUBOFFSET_BYTE;
            B_value_region = &_register->physical_channel[physical_channel_id].kb.d_cv_b_level3_region;
            data_byte_count = 2;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_CV_B_LEVEL_3 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            B_value = combine_uint8_t_to_uint16_t(byte_array);
            
            if(B_value & 0x8000){ //negative
                B_value_amplitude = B_value & 0x7FFF;
                B_value_sign = 0x8000000000000000;
            } else {
                B_value_amplitude = B_value;
                B_value_sign = 0x0000000000000000;
            }
            
            unit_convert_B_value = (uint64_t)B_value_amplitude;
            unit_convert_B_value = unit_convert_B_value | B_value_sign;
            
            update_4paired_register_region(B_value_region, unit_convert_B_value);
        break;


        case INFO_TYPE_DEVICE_HOST:
            
            suboffset = PHYSICAL_CHANNEL8_EXTRA_DEVICE_HOST_SUBOFFSET_BYTE;
            // for lock data channel offset
            channel_base_address_offset = 7 * PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE;
            device_host_region = &_register->physical_channel[PHYSICAL_CHANNEL_1_ID].device_host;
            data_byte_count = 4;
            
            data_start_address = channel_base_address_offset + suboffset;
            
            ret = read_info_from_file(byte_array, data_byte_count, data_start_address);
            if(ret != 0){
                printf("reload_info_from_ROM->read_info_from_file INFO_TYPE_D_CV_B_LEVEL_3 fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
            
            device_host_value = combine_uint8_t_to_uint32_t(byte_array);
            
            update_2paired_register_region(device_host_region, device_host_value);
        break;


        default:
            printf("reload_info_from_ROM unknow info_type\n");
            return ERROR_INTERNEL_ERRORS;
        break;
    }
    return 0;
}


int save_one_physical_channel_KB_value(Manager *_manager, int channel_id, uint16_t calibration_step_type)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int ret, step_type, physical_channel_id, info_type;
    uint16_t *current_source_level_reg;

    physical_channel_id = channel_id;
    step_type = calibration_step_type;

    _register = &manager->modbus_ctx._register;

    switch (physical_channel_id) {
        case PHYSICAL_CHANNEL_1_ID:
        break;

        case PHYSICAL_CHANNEL_2_ID:
        break;

        case PHYSICAL_CHANNEL_3_ID:
        break;

        case PHYSICAL_CHANNEL_4_ID:
        break;

        case PHYSICAL_CHANNEL_5_ID:
        break;

        case PHYSICAL_CHANNEL_6_ID:
        break;

        case PHYSICAL_CHANNEL_7_ID:
        break;

        case PHYSICAL_CHANNEL_8_ID:
        break;

        default:
            printf("save_one_physical_channel_KB_value unknow physical channel\n");
            return ERROR_INTERNEL_ERRORS;
        break;
    }

    current_source_level_reg = _register->physical_channel[physical_channel_id].current_source_level;

    switch (step_type) {

        case CALIBRATION_STEP_TYPE_CCC:
            
            switch (*current_source_level_reg) {
                
                case PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1:
                    
                    info_type = INFO_TYPE_C_VI_K_LEVEL_1;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_C_VI_K_LEVEL_1 fail\n");
                        return ret;
                    }
                    
                    info_type = INFO_TYPE_C_VI_B_LEVEL_1;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_C_VI_B_LEVEL_1 fail\n");
                        return ret;
                    }

                    info_type = INFO_TYPE_C_CV_K_LEVEL_1;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_C_CV_K_LEVEL_1 fail\n");
                        return ret;
                    }

                    info_type = INFO_TYPE_C_CV_B_LEVEL_1;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_C_CV_B_LEVEL_1 fail\n");
                        return ret;
                    }
                    
                break;

                case PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2:
                    
                    info_type = INFO_TYPE_C_VI_K_LEVEL_2;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_C_VI_K_LEVEL_2 fail\n");
                        return ret;
                    }
                    
                    info_type = INFO_TYPE_C_VI_B_LEVEL_2;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_C_VI_B_LEVEL_2 fail\n");
                        return ret;
                    }

                    info_type = INFO_TYPE_C_CV_K_LEVEL_2;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_C_CV_K_LEVEL_2 fail\n");
                        return ret;
                    }

                    info_type = INFO_TYPE_C_CV_B_LEVEL_2;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_C_CV_B_LEVEL_2 fail\n");
                        return ret;
                    }
                    
                break;

                case PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3:
                    
                    info_type = INFO_TYPE_C_VI_K_LEVEL_3;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_C_VI_K_LEVEL_3 fail\n");
                        return ret;
                    }
                    
                    info_type = INFO_TYPE_C_VI_B_LEVEL_3;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_C_VI_B_LEVEL_3 fail\n");
                        return ret;
                    }

                    info_type = INFO_TYPE_C_CV_K_LEVEL_3;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_C_CV_K_LEVEL_3 fail\n");
                        return ret;
                    }

                    info_type = INFO_TYPE_C_CV_B_LEVEL_3;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_C_CV_B_LEVEL_3 fail\n");
                        return ret;
                    }
                    
                break;

                default:
                    
                    printf("save_one_physical_channel_KB_value %d channel unknow current_source_level\n",channel_id);
                    return ERROR_UNKNOWN_PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL;
                    
                break;
            }

        break;

        case CALIBRATION_STEP_TYPE_CCD:
            
            switch (*current_source_level_reg) {
                
                case PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1:
                    
                    info_type = INFO_TYPE_D_VI_K_LEVEL_1;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_D_VI_K_LEVEL_1 fail\n");
                        return ret;
                    }
                    
                    info_type = INFO_TYPE_D_VI_B_LEVEL_1;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_D_VI_B_LEVEL_1 fail\n");
                        return ret;
                    }
            
                    info_type = INFO_TYPE_D_CV_K_LEVEL_1;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_D_CV_K_LEVEL_1 fail\n");
                        return ret;
                    }
            
                    info_type = INFO_TYPE_D_CV_B_LEVEL_1;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_D_CV_B_LEVEL_1 fail\n");
                        return ret;
                    }
                    
                break;

                case PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2:
                    
                    info_type = INFO_TYPE_D_VI_K_LEVEL_2;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_D_VI_K_LEVEL_2 fail\n");
                        return ret;
                    }
                    
                    info_type = INFO_TYPE_D_VI_B_LEVEL_2;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_D_VI_B_LEVEL_2 fail\n");
                        return ret;
                    }
            
                    info_type = INFO_TYPE_D_CV_K_LEVEL_2;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_D_CV_K_LEVEL_2 fail\n");
                        return ret;
                    }
            
                    info_type = INFO_TYPE_D_CV_B_LEVEL_2;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_D_CV_B_LEVEL_2 fail\n");
                        return ret;
                    }
                    
                break;

                case PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3:
                    
                    info_type = INFO_TYPE_D_VI_K_LEVEL_3;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_D_VI_K_LEVEL_3 fail\n");
                        return ret;
                    }
                    
                    info_type = INFO_TYPE_D_VI_B_LEVEL_3;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_D_VI_B_LEVEL_3 fail\n");
                        return ret;
                    }
            
                    info_type = INFO_TYPE_D_CV_K_LEVEL_3;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_D_CV_K_LEVEL_3 fail\n");
                        return ret;
                    }
            
                    info_type = INFO_TYPE_D_CV_B_LEVEL_3;
                    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
                    if(ret != 0){
                        printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_D_CV_B_LEVEL_3 fail\n");
                        return ret;
                    }
                    
                break;


                default:
                    
                    printf("save_one_physical_channel_KB_value %d channel unknow current_source_level\n",channel_id);
                    return ERROR_UNKNOWN_PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL;
                    
                break;
            }
            
        break;

        case CALIBRATION_STEP_TYPE_CVAD:
            
            info_type = INFO_TYPE_BV_K;
            ret = save_info_to_ROM(manager, physical_channel_id, info_type);
            if(ret != 0){
                printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_BV_K fail\n");
                return ret;
            }

            info_type = INFO_TYPE_BV_B;
            ret = save_info_to_ROM(manager, physical_channel_id, info_type);
            if(ret != 0){
                printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_BV_B fail\n");
                return ret;
            }

        break;

        case CALIBRATION_STEP_TYPE_CVDA:
            
            info_type = INFO_TYPE_VU_K;
            ret = save_info_to_ROM(manager, physical_channel_id, info_type);
            if(ret != 0){
                printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_VU_K fail\n");
                return ret;
            }
            
            info_type = INFO_TYPE_VU_B;
            ret = save_info_to_ROM(manager, physical_channel_id, info_type);
            if(ret != 0){
                printf("save_one_physical_channel_KB_value->save_info_to_ROM INFO_TYPE_VU_B fail\n");
                return ret;
            }

        break;

        default:
            
            printf("save_one_physical_channel_KB_value unknow calibration step\n");
            return ERROR_UNKNOWN_CALIBRATION_STEP_TYPE;
            
        break;
    }
    return 0;
}


int reload_one_physical_channel_KB_value(Manager *_manager, int channel_id)
{
    int physical_channel_id = channel_id, i, ret;
    Manager *manager = _manager;

    for(i=0; i<INFO_TYPE_KB_COUNT; i++){

        ret = reload_info_from_ROM(manager, physical_channel_id, i);
        if(ret != 0){
            printf("reload_one_physical_channel_KB_value->reload_info_from_ROM %d channel reload %d type K_or_B data fail\n", physical_channel_id, i);
            return ERROR_INTERNEL_ERRORS;
        }
    }
    return 0;
}


int save_and_update_IP(Manager *_manager)
{
    Manager *manager = _manager;
    Register *_register = NULL;

    ssd1306_oled_controller_context *_ssd1306_oled_controller_ctx = NULL;
    ssd1306_oled_controller_ioctl_parameters *_ssd1306_oled_controller_ioctl_param = NULL;

    int info_type, physical_channel_id, host_string_lenth, ret, cmd, i;
    uint8_t ip_HH, ip_HL, ip_LH, ip_LL;

    _2paired_register_region *device_host_region;
    uint32_t device_host_value;
    char host[NI_MAXHOST];
    struct in_addr inp;
    in_addr_t in_ip;


    _register = &manager->modbus_ctx._register;
    _ssd1306_oled_controller_ctx = &manager->resources.ssd1306_oled_controller_ctx;
    _ssd1306_oled_controller_ioctl_param = &manager->resources.ssd1306_oled_controller_ioctl_param_ssd1306_oled_1;
    device_host_region = &_register->physical_channel[PHYSICAL_CHANNEL_1_ID].device_host;

    info_type = INFO_TYPE_DEVICE_HOST;
    physical_channel_id = 0;


    ip_HH = (*device_host_region->word2 & 0xFF00) >> 8;
    ip_HL = (*device_host_region->word2 & 0x00FF) >> 0;
    ip_LH = (*device_host_region->word1 & 0xFF00) >> 8;
    ip_LL = (*device_host_region->word1 & 0x00FF) >> 0;

    ret = snprintf(host, NI_MAXHOST, "%d.%d.%d.%d", ip_HH, ip_HL, ip_LH, ip_LL);
    if(ret < 1) {
        printf("save_and_update_IP error\n");
        return -1;
    }
    host_string_lenth = strlen(host);

    ret = inet_aton(host, &inp);
    if(ret != 1){
        printf("save_and_update_IP->inet_aton ip invaid\n");
        return -1;
    }
    in_ip = inp.s_addr;

    ret = set_ip(&in_ip);
    if(ret != 0) {
        printf("save_and_update_IP->set_ip fail\n");
        return -1;
    }

/*
    cmd = SSD1306_OLED_CONTROLLER_IOCTL_CMD_WRITE_CHAR;
    _ssd1306_oled_controller_ioctl_param->write_char_param.in.char_row_idx = HOST_IN_OLED_CHAR_ROW_IDX;
    
    for(i = 0;i < SSD1306_OLED_CHAR_MAX_COLUMN_IDX;i++) {
        _ssd1306_oled_controller_ioctl_param->write_char_param.in.char_column_idx = i;
        _ssd1306_oled_controller_ioctl_param->write_char_param.in._char = ' ';

        ret = _ssd1306_oled_controller_ctx->ops.ioctl(_ssd1306_oled_controller_ctx, cmd, _ssd1306_oled_controller_ioctl_param);
        if(ret < 0) {
            printf("reload_and_update_IP->_ssd1306_oled_controller_ctx->ioctl fail\n");
            return -1;
        }
    };

    for(i = 0;i < host_string_lenth;i++) {
        _ssd1306_oled_controller_ioctl_param->write_char_param.in.char_column_idx = i;
        _ssd1306_oled_controller_ioctl_param->write_char_param.in._char = host[i];

        ret = _ssd1306_oled_controller_ctx->ops.ioctl(_ssd1306_oled_controller_ctx, cmd, _ssd1306_oled_controller_ioctl_param);
        if(ret < 0) {
            printf("save_and_update_IP->_ssd1306_oled_controller_ctx->ioctl fail\n");
            return -1;
        }
    };

    ret = save_info_to_ROM(manager, physical_channel_id, info_type);
    if(ret != 0){
        printf("save_and_update_IP->save_info_to_ROM INFO_TYPE_DEVICE_HOST fail\n");
        return ret;
    }
*/

    return 0;
}


int reload_and_update_IP(Manager *_manager)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    ssd1306_oled_controller_context *_ssd1306_oled_controller_ctx = NULL;
    ssd1306_oled_controller_ioctl_parameters *_ssd1306_oled_controller_ioctl_param = NULL;
    int info_type, physical_channel_id, host_string_lenth, ret, cmd, i;
    uint8_t ip_HH, ip_HL, ip_LH, ip_LL;
    struct in_addr inp;
    in_addr_t in_ip;

    _2paired_register_region *device_host_region;
    uint32_t device_host_value;
    char host[NI_MAXHOST];

    _register = &manager->modbus_ctx._register;
    _ssd1306_oled_controller_ctx = &manager->resources.ssd1306_oled_controller_ctx;
    _ssd1306_oled_controller_ioctl_param = &manager->resources.ssd1306_oled_controller_ioctl_param_ssd1306_oled_1;
    device_host_region = &_register->physical_channel[PHYSICAL_CHANNEL_1_ID].device_host;

    info_type = INFO_TYPE_DEVICE_HOST;
    physical_channel_id = 0;

    ret = reload_info_from_ROM(manager, physical_channel_id, info_type);
    if(ret != 0){
        printf("reload_and_update_IP->reload_info_from_ROM INFO_TYPE_DEVICE_HOST fail\n");
        return ret;
    }

    ip_HH = (*device_host_region->word2 & 0xFF00) >> 8;
    ip_HL = (*device_host_region->word2 & 0x00FF) >> 0;
    ip_LH = (*device_host_region->word1 & 0xFF00) >> 8;
    ip_LL = (*device_host_region->word1 & 0x00FF) >> 0;

    ret = snprintf(host, NI_MAXHOST, "%d.%d.%d.%d", ip_HH, ip_HL, ip_LH, ip_LL);
    if(ret < 1) {
        printf("reload_and_update_IP error\n");
        return -1;
    }
    host_string_lenth = strlen(host);

    ret = inet_aton(host, &inp);
    if(ret != 1){
        printf("save_and_update_IP->inet_aton ip invaid\n");
        return -1;
    }
    in_ip = inp.s_addr;

    ret = set_ip(&in_ip);
    if(ret != 0) {
        printf("save_and_update_IP->set_ip fail\n");
        return -1;
    }

/*
    cmd = SSD1306_OLED_CONTROLLER_IOCTL_CMD_WRITE_CHAR;
    _ssd1306_oled_controller_ioctl_param->write_char_param.in.char_row_idx = HOST_IN_OLED_CHAR_ROW_IDX;

    for(i = 0;i < SSD1306_OLED_CHAR_MAX_COLUMN_IDX;i++) {
        _ssd1306_oled_controller_ioctl_param->write_char_param.in.char_column_idx = i;
        _ssd1306_oled_controller_ioctl_param->write_char_param.in._char = ' ';

        ret = _ssd1306_oled_controller_ctx->ops.ioctl(_ssd1306_oled_controller_ctx, cmd, _ssd1306_oled_controller_ioctl_param);
        if(ret < 0) {
            printf("reload_and_update_IP->_ssd1306_oled_controller_ctx->ioctl fail\n");
            return -1;
        }
    };

    for(i = 0;i < host_string_lenth;i++) {
        _ssd1306_oled_controller_ioctl_param->write_char_param.in.char_column_idx = i;
        _ssd1306_oled_controller_ioctl_param->write_char_param.in._char = host[i];

        ret = _ssd1306_oled_controller_ctx->ops.ioctl(_ssd1306_oled_controller_ctx, cmd, _ssd1306_oled_controller_ioctl_param);
        if(ret < 0) {
            printf("reload_and_update_IP->_ssd1306_oled_controller_ctx->ioctl fail\n");
            return -1;
        }
    };
*/

    return 0;

}

int get_IP_and_update_oled(Manager *_manager)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    ssd1306_oled_controller_context *_ssd1306_oled_controller_ctx = NULL;
    ssd1306_oled_controller_ioctl_parameters *_ssd1306_oled_controller_ioctl_param = NULL;
    int info_type, physical_channel_id, host_string_lenth, ret, cmd, i;

    struct in_addr host_ip_struct;
    in_addr_t old_ip, *manager_host_ip_reg, now_ip;

    //char host[NI_MAXHOST];
    char *host;

    _register = &manager->modbus_ctx._register;
    _ssd1306_oled_controller_ctx = &manager->resources.ssd1306_oled_controller_ctx;
    _ssd1306_oled_controller_ioctl_param = &manager->resources.ssd1306_oled_controller_ioctl_param_ssd1306_oled_1;
    manager_host_ip_reg = &manager->oled_host_ip;

    old_ip = *manager_host_ip_reg;

    ret = get_ip(&now_ip);
    if(ret < 0) {
        printf("get_IP_and_update_oled->get_ip fail\n");
        return -1;
    }
    if(now_ip != old_ip) {

        host_ip_struct.s_addr = now_ip;

        host = inet_ntoa(host_ip_struct);

        host_string_lenth = strlen(host);

        cmd = SSD1306_OLED_CONTROLLER_IOCTL_CMD_STOP_SCOLL;
        ret = _ssd1306_oled_controller_ctx->ops.ioctl(_ssd1306_oled_controller_ctx, cmd, _ssd1306_oled_controller_ioctl_param);
        if(ret < 0) {
            printf("reload_and_update_IP->_ssd1306_oled_controller_ctx->ioctl fail\n");
            return -1;
        }
        
        cmd = SSD1306_OLED_CONTROLLER_IOCTL_CMD_WRITE_CHAR;
        _ssd1306_oled_controller_ioctl_param->write_char_param.in.char_row_idx = HOST_IN_OLED_CHAR_ROW_IDX;

        for(i = 0;i < SSD1306_OLED_CHAR_MAX_COLUMN_IDX+1;i++) {
            _ssd1306_oled_controller_ioctl_param->write_char_param.in.char_column_idx = i;
            _ssd1306_oled_controller_ioctl_param->write_char_param.in._char = ' ';

            ret = _ssd1306_oled_controller_ctx->ops.ioctl(_ssd1306_oled_controller_ctx, cmd, _ssd1306_oled_controller_ioctl_param);
            if(ret < 0) {
                printf("reload_and_update_IP->_ssd1306_oled_controller_ctx->ioctl fail\n");
                return -1;
            }
        };

        for(i = 0;i < host_string_lenth;i++) {
            _ssd1306_oled_controller_ioctl_param->write_char_param.in.char_column_idx = i;
            _ssd1306_oled_controller_ioctl_param->write_char_param.in._char = host[i];

            ret = _ssd1306_oled_controller_ctx->ops.ioctl(_ssd1306_oled_controller_ctx, cmd, _ssd1306_oled_controller_ioctl_param);
            if(ret < 0) {
                printf("reload_and_update_IP->_ssd1306_oled_controller_ctx->ioctl fail\n");
                return -1;
            }
        };

        cmd = SSD1306_OLED_CONTROLLER_IOCTL_CMD_START_SCOLL;
        ret = _ssd1306_oled_controller_ctx->ops.ioctl(_ssd1306_oled_controller_ctx, cmd, _ssd1306_oled_controller_ioctl_param);
        if(ret < 0) {
            printf("reload_and_update_IP->_ssd1306_oled_controller_ctx->ioctl fail\n");
            return -1;
        }

        *manager_host_ip_reg = now_ip;

    }

    return 0;

}


int check_and_update_KB(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id, i, ret;
    uint16_t *binding, *kb_save_enable_reg, *step_type_reg;

    _register = &manager->modbus_ctx._register;
    
    binding = _register->logical_channel[logical_channel_id].binding;
    kb_save_enable_reg = _register->logical_channel[logical_channel_id].calibration_param.kb_save_enable;
    step_type_reg = _register->logical_channel[logical_channel_id].step_param.step_type;


    if(*binding == 0) {
        printf("check_and_update_KB fail, binding = 0\n");
        return -1;
    }

    if(*step_type_reg != CALIBRATION_STEP_TYPE_CCC & *step_type_reg != CALIBRATION_STEP_TYPE_CCD & *step_type_reg != CALIBRATION_STEP_TYPE_CVAD & *step_type_reg != CALIBRATION_STEP_TYPE_CVDA){
        printf("check_and_update_KB unknow calibration_step_type\n");
        return ERROR_UNKNOWN_CALIBRATION_STEP_TYPE;
    }

    if(*kb_save_enable_reg == 1){

        for(i=0; i<NB_PHYSICAL_CHANNEL; i++) {

            if(((( *binding )>>i) & 0x0001) > 0) {

                ret = save_one_physical_channel_KB_value(manager, i, *step_type_reg);
                if(ret != 0){
                    printf("check_and_update_KB->save_one_physical_channel_KB_value %d physical_channel_id save KB fail\n", i);
                    return -1;
                }

            }

        }

        *kb_save_enable_reg = 0;

    }
    return 0;
}

int check_and_update_IP(Manager *_manager)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int ret;
    uint16_t *device_host_update_enable_reg;

    _2paired_register_region *device_host_region;
    uint32_t device_host_value;
    in_addr_t in_ip;

    _register = &manager->modbus_ctx._register;

    device_host_region = &_register->physical_channel[PHYSICAL_CHANNEL_1_ID].device_host;
    
    device_host_update_enable_reg = _register->physical_channel[PHYSICAL_CHANNEL_1_ID].device_host_save_enable;

    if(*device_host_update_enable_reg == 1){

        ret = save_and_update_IP(_manager);
        if(ret != 0) {
            printf("check_and_update_IP->save_IP fail\n");
            return -1;
        }

        *device_host_update_enable_reg = 0;

    }
    return 0;
}

void check_and_update_tower_light(Manager *_manager)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id, idel_count, error_count, run_count;
    uint16_t *l_status_reg, *led_ready, *led_run, *led_error;

    _register = &manager->modbus_ctx._register;

    led_ready = _register->physical_channel[LOGICAL_CHANNEL_1_ID].led_ready;
    led_run = _register->physical_channel[LOGICAL_CHANNEL_1_ID].led_run;
    led_error = _register->physical_channel[LOGICAL_CHANNEL_1_ID].led_error;

    idel_count = 0;
    error_count = 0;
    run_count = 0;

    for(logical_channel_id=LOGICAL_CHANNEL_1_ID; logical_channel_id<NB_LOGIC_CHANNEL; logical_channel_id++) {
        l_status_reg = _register->logical_channel[logical_channel_id].status;

        if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_IDEL) {
            idel_count++;
        } else if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_EXCEPTION) {
            error_count++;
        } else {
            run_count++;
        }
    }

    if(error_count > 0) {
        
        *led_ready = LED_READY_DISABLE;
        *led_run = LED_RUN_DISABLE;
        *led_error = LED_ERROR_ENABLE;
        
    } else if(run_count > 0) {
        *led_ready = LED_READY_DISABLE;
        *led_run = LED_RUN_ENABLE;
        *led_error = LED_ERROR_DISABLE;
        
    } else {
        *led_ready = LED_READY_ENABLE;
        *led_run = LED_RUN_DISABLE;
        *led_error = LED_ERROR_DISABLE;
    }

}


