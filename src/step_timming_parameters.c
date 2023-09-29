/**
* @file     date.c
* @brief    date.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     21/12/2018
*/

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdint.h>
//#include <time.h>
#include <sys/time.h>
#include <pthread.h>

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
#include "registers.h"

#include "modbus_tcp.h"
#include "macros.h"
#include "platform_endian.h"

#include "register_offset.h"
#include "register_values.h"
#include "error_no.h"

void update_step_established_time(Manager *manager, int channel_id)
{
    int logical_channel_id = channel_id;
    
    _4paired_register_region *established_time_region;
    uint64_t established_time;
    struct timespec time_now;
    Register *_register = NULL;

    _register = &manager->modbus_ctx._register;

    // get current time
    established_time_region = &_register->logical_channel[logical_channel_id].step_param.timing.established_time_region;
    clock_gettime(CLOCK_REALTIME, &time_now);
    established_time = (uint64_t)time_now.tv_sec * UINT_FACTOR_BETWEEN_S_AND_MS + (uint64_t)time_now.tv_nsec / UINT_FACTOR_BETWEEN_MS_AND_NS;
    
    update_4paired_register_region(established_time_region, established_time);
    
#if 0
    time_t time_now;
    struct tm *timeinfo;
    time(&time_now);
    timeinfo = localtime(&time_now);

    *_register->logical_channel[logical_channel_id].step_param.timing.established_year = timeinfo->tm_year;
    *_register->logical_channel[logical_channel_id].step_param.timing.established_moon = timeinfo->tm_mon;
    *_register->logical_channel[logical_channel_id].step_param.timing.established_day = timeinfo->tm_mday;
    *_register->logical_channel[logical_channel_id].step_param.timing.established_hour = timeinfo->tm_hour;
    *_register->logical_channel[logical_channel_id].step_param.timing.established_minute = timeinfo->tm_min;
    *_register->logical_channel[logical_channel_id].step_param.timing.established_second = timeinfo->tm_sec;
#endif
}

void update_step_abolished_time(Manager *manager, int channel_id)
{
    int logical_channel_id = channel_id;

    _4paired_register_region *abolished_time_region;
    uint64_t abolished_time;
    struct timespec time_now;
    Register *_register = NULL;

    _register = &manager->modbus_ctx._register;

    // get current time
    abolished_time_region = &_register->logical_channel[logical_channel_id].step_param.timing.abolished_time_region;
    clock_gettime(CLOCK_REALTIME, &time_now);
    abolished_time= (uint64_t)time_now.tv_sec * UINT_FACTOR_BETWEEN_S_AND_MS + (uint64_t)time_now.tv_nsec / UINT_FACTOR_BETWEEN_MS_AND_NS;
    
    update_4paired_register_region(abolished_time_region, abolished_time);

#if 0
    time_t time_now;
    struct tm *timeinfo;
    time(&time_now);
    timeinfo = localtime(&time_now);

    *_register->logical_channel[logical_channel_id].step_param.timing.abolished_year = timeinfo->tm_year;
    *_register->logical_channel[logical_channel_id].step_param.timing.abolished_moon = timeinfo->tm_mon;
    *_register->logical_channel[logical_channel_id].step_param.timing.abolished_day = timeinfo->tm_mday;
    *_register->logical_channel[logical_channel_id].step_param.timing.abolished_hour = timeinfo->tm_hour;
    *_register->logical_channel[logical_channel_id].step_param.timing.abolished_minute = timeinfo->tm_min;
    *_register->logical_channel[logical_channel_id].step_param.timing.abolished_second = timeinfo->tm_sec;
#endif
}


void update_holding_time_reference(Manager *manager, int channel_id)
{
    int logical_channel_id = channel_id;
    _4paired_register_region *holding_time_reference_region, *holding_time_region;
    Register *_register = NULL;

    _register = &manager->modbus_ctx._register;

    holding_time_reference_region = &_register->logical_channel[logical_channel_id].temp.holding_time_reference_region;
    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    copy_4paired_register_region(holding_time_reference_region, holding_time_region);
    //*_register->logical_channel[logical_channel_id].temp.holding_time_reference = *_register->logical_channel[logical_channel_id].temp.holding_time;
}

void update_holding_time_left_boundary_now(Manager *manager, int channel_id)
{
    int logical_channel_id = channel_id;
    _4paired_register_region *left_boundary_time_region;
    uint64_t left_boundary_time;
    struct timespec time_now;

    Register *_register = NULL;

    _register = &manager->modbus_ctx._register;

    // get current time
    left_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.left_boundary_time_region;
    clock_gettime(CLOCK_REALTIME, &time_now);
    left_boundary_time = (uint64_t)time_now.tv_sec * UINT_FACTOR_BETWEEN_S_AND_MS + (uint64_t)time_now.tv_nsec / UINT_FACTOR_BETWEEN_MS_AND_NS;
    
    update_4paired_register_region(left_boundary_time_region, left_boundary_time);

#if 0
    time_t time_now;
    struct tm *timeinfo;
    time(&time_now);
    timeinfo = localtime(&time_now);

    *_register->logical_channel[logical_channel_id].temp.left_boundary_year = timeinfo->tm_year;
    *_register->logical_channel[logical_channel_id].temp.left_boundary_moon = timeinfo->tm_mon;
    *_register->logical_channel[logical_channel_id].temp.left_boundary_day = timeinfo->tm_mday;
    *_register->logical_channel[logical_channel_id].temp.left_boundary_hour = timeinfo->tm_hour;
    *_register->logical_channel[logical_channel_id].temp.left_boundary_minute = timeinfo->tm_min;
    *_register->logical_channel[logical_channel_id].temp.left_boundary_second = timeinfo->tm_sec;
#endif
}

void update_right_boundary_and_holding_time(Manager *manager, int channel_id)
{
    Register *_register = NULL;
    int logical_channel_id = channel_id;

    _4paired_register_region *holding_time_region, *holding_time_reference_region, *right_boundary_time_region, *left_boundary_time_region;
    uint64_t holding_time, holding_time_reference, lb_time_t, rb_time_t;
    struct timespec time_now;

    _register = &manager->modbus_ctx._register;

    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    holding_time_reference_region = &_register->logical_channel[logical_channel_id].temp.holding_time_reference_region;

    right_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.right_boundary_time_region;
    left_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.left_boundary_time_region;

    holding_time_reference = combine_4paired_register_region(holding_time_reference_region);
    clock_gettime(CLOCK_REALTIME, &time_now);
    rb_time_t = (uint64_t)time_now.tv_sec * UINT_FACTOR_BETWEEN_S_AND_MS + (uint64_t)time_now.tv_nsec / UINT_FACTOR_BETWEEN_MS_AND_NS;
    lb_time_t = combine_4paired_register_region(left_boundary_time_region);

    holding_time = (rb_time_t - lb_time_t) + holding_time_reference;

    update_4paired_register_region(right_boundary_time_region, rb_time_t);
    update_4paired_register_region(holding_time_region, holding_time);



    //get current time
#if 0
    time_t time_now;
    struct tm *timeinfo;
    struct tm lb_time = {0}, rb_time = {0};
    time_t lb_time_t, rb_time_t;

    time(&time_now);
    timeinfo = localtime(&time_now);

    rb_time.tm_year = *_register->logical_channel[logical_channel_id].temp.right_boundary_year = timeinfo->tm_year;
    rb_time.tm_mon  = *_register->logical_channel[logical_channel_id].temp.right_boundary_moon = timeinfo->tm_mon;
    rb_time.tm_mday = *_register->logical_channel[logical_channel_id].temp.right_boundary_day = timeinfo->tm_mday;
    rb_time.tm_hour = *_register->logical_channel[logical_channel_id].temp.right_boundary_hour = timeinfo->tm_hour;
    rb_time.tm_min  = *_register->logical_channel[logical_channel_id].temp.right_boundary_minute = timeinfo->tm_min;
    rb_time.tm_sec  = *_register->logical_channel[logical_channel_id].temp.right_boundary_second = timeinfo->tm_sec;
    //rb_time.tm_millisec  = *_register->logical_channel[logical_channel_id].temp.right_boundary_second = timeinfo->tm_millisec;

    lb_time.tm_year = *_register->logical_channel[logical_channel_id].temp.left_boundary_year;
    lb_time.tm_mon  = *_register->logical_channel[logical_channel_id].temp.left_boundary_moon;
    lb_time.tm_mday = *_register->logical_channel[logical_channel_id].temp.left_boundary_day;
    lb_time.tm_hour = *_register->logical_channel[logical_channel_id].temp.left_boundary_hour;
    lb_time.tm_min  = *_register->logical_channel[logical_channel_id].temp.left_boundary_minute;
    lb_time.tm_sec  = *_register->logical_channel[logical_channel_id].temp.left_boundary_second;
    //lb_time.tm_millisec  = *_register->logical_channel[logical_channel_id].temp.left_boundary_millisec;

    lb_time_t = mktime(&lb_time);
    rb_time_t = mktime(&rb_time);

    *_register->logical_channel[logical_channel_id].temp.holding_time = difftime(rb_time_t, lb_time_t) + *_register->logical_channel[logical_channel_id].temp.holding_time_reference;
#endif
}

