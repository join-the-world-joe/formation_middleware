/**
* @file     physical_channel.c
* @brief    physical_channel.c
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
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <inttypes.h> 

#include "log_controller.h"

#include "channel.h"
#include "spi_controller.h"
#include "i2c_controller.h"
#include "gpio_controller.h"
#include "dac_controller.h"
#include "io_expander_controller.h"
#include "adc_controller.h"
#include "ssd1306_oled_controller.h"
#include "tcp.h"
#include "registers.h"

#include "modbus_tcp.h"

#include "manager.h"

#include "logical_channel.h"

#include "modbus_tcp.h"
#include "macros.h"
#include "platform_endian.h"

#include "register_offset.h"
#include "register_values.h"
#include "error_no.h"
#include "error_msg.h"
#include "logical_channel.h"

#include "physical_channel.h"

#include "adc_ti1258.h"

#include "misc.h"

int check_physical_channels_and_update_info(Manager *manager, int channel_id)
{   
    uint16_t logical_channel_id = channel_id;
    int ret = 0, mask, nb_binding = 0;
    Register *_register = NULL;
    uint16_t *cc = NULL; 

    _4paired_register_region *level_1_region, *level_2_region, *level_3_region;
    _4paired_register_region *maximum_source_current_region;
    uint64_t level_1, level_2, level_3, temp, maximum_source_current;

    _register = &manager->modbus_ctx._register;

    maximum_source_current = 0;

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    *_register->logical_channel[logical_channel_id].nb_binding = 0;

    // clear the magnitude of maxium source current
    maximum_source_current_region = &_register->logical_channel[logical_channel_id].maximum_source_current_region;
    update_4paired_register_region(maximum_source_current_region, 0);
    
    // check if no physical channel is bound for this logical channel
    if(*_register->logical_channel[logical_channel_id].binding == 0) {
        printf("no physical channel is bound now!\n");
        return ERROR_BINDING_REGISTER_ZERO;
    }

    // check if physical channel 1 is in count
    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH1_MASK_BIT;
    if((*_register->logical_channel[logical_channel_id].binding) & mask) {
        if(*_register->physical_channel[PHYSICAL_CHANNEL_1_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %d\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        } else {
            // check if configuration is correct
            level_1_region = &_register->physical_channel[PHYSICAL_CHANNEL_1_ID].source_current_level_1_region;
            level_2_region = &_register->physical_channel[PHYSICAL_CHANNEL_1_ID].source_current_level_2_region;
            level_3_region = &_register->physical_channel[PHYSICAL_CHANNEL_1_ID].source_current_level_3_region;
    
            level_1 = combine_4paired_register_region(level_1_region);
            level_2 = combine_4paired_register_region(level_2_region);
            level_3 = combine_4paired_register_region(level_3_region);

            if(level_1 == 0 || level_2 == 0 || level_3 == 0) {
                *_register->logical_channel[logical_channel_id].exception_message |= mask;
                ret = ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
            }
        }

        nb_binding++;
        maximum_source_current += level_3;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH2_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_2_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %d\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        } else {
            // check if configuration is correct
            level_1_region = &_register->physical_channel[PHYSICAL_CHANNEL_2_ID].source_current_level_1_region;
            level_2_region = &_register->physical_channel[PHYSICAL_CHANNEL_2_ID].source_current_level_2_region;
            level_3_region = &_register->physical_channel[PHYSICAL_CHANNEL_2_ID].source_current_level_3_region;
    
            level_1 = combine_4paired_register_region(level_1_region);
            level_2 = combine_4paired_register_region(level_2_region);
            level_3 = combine_4paired_register_region(level_3_region);

            if(level_1 == 0 || level_2 == 0 || level_3 == 0) {
                *_register->logical_channel[logical_channel_id].exception_message |= mask;
                ret = ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
            }
        }

        nb_binding++;
        maximum_source_current += level_3;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH3_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_3_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %d\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        } else {
            // check if configuration is correct
            level_1_region = &_register->physical_channel[PHYSICAL_CHANNEL_3_ID].source_current_level_1_region;
            level_2_region = &_register->physical_channel[PHYSICAL_CHANNEL_3_ID].source_current_level_2_region;
            level_3_region = &_register->physical_channel[PHYSICAL_CHANNEL_3_ID].source_current_level_3_region;
    
            level_1 = combine_4paired_register_region(level_1_region);
            level_2 = combine_4paired_register_region(level_2_region);
            level_3 = combine_4paired_register_region(level_3_region);

            if(level_1 == 0 || level_2 == 0 || level_3 == 0) {
                *_register->logical_channel[logical_channel_id].exception_message |= mask;
                ret = ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
            }
        }

        nb_binding++;
        maximum_source_current += level_3;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH4_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_4_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %d\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        } else {
            // check if configuration is correct
            level_1_region = &_register->physical_channel[PHYSICAL_CHANNEL_4_ID].source_current_level_1_region;
            level_2_region = &_register->physical_channel[PHYSICAL_CHANNEL_4_ID].source_current_level_2_region;
            level_3_region = &_register->physical_channel[PHYSICAL_CHANNEL_4_ID].source_current_level_3_region;
    
            level_1 = combine_4paired_register_region(level_1_region);
            level_2 = combine_4paired_register_region(level_2_region);
            level_3 = combine_4paired_register_region(level_3_region);

            if(level_1 == 0 || level_2 == 0 || level_3 == 0) {
                *_register->logical_channel[logical_channel_id].exception_message |= mask;
                ret = ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
            }
        }

        nb_binding++;
        maximum_source_current += level_3;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH5_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_5_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %d\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        } else {
            // check if configuration is correct
            level_1_region = &_register->physical_channel[PHYSICAL_CHANNEL_5_ID].source_current_level_1_region;
            level_2_region = &_register->physical_channel[PHYSICAL_CHANNEL_5_ID].source_current_level_2_region;
            level_3_region = &_register->physical_channel[PHYSICAL_CHANNEL_5_ID].source_current_level_3_region;
    
            level_1 = combine_4paired_register_region(level_1_region);
            level_2 = combine_4paired_register_region(level_2_region);
            level_3 = combine_4paired_register_region(level_3_region);

            if(level_1 == 0 || level_2 == 0 || level_3 == 0) {
                *_register->logical_channel[logical_channel_id].exception_message |= mask;
                ret = ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
            }
        }

        nb_binding++;
        maximum_source_current += level_3;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH6_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_6_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %d\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        } else {
            // check if configuration is correct
            level_1_region = &_register->physical_channel[PHYSICAL_CHANNEL_6_ID].source_current_level_1_region;
            level_2_region = &_register->physical_channel[PHYSICAL_CHANNEL_6_ID].source_current_level_2_region;
            level_3_region = &_register->physical_channel[PHYSICAL_CHANNEL_6_ID].source_current_level_3_region;
    
            level_1 = combine_4paired_register_region(level_1_region);
            level_2 = combine_4paired_register_region(level_2_region);
            level_3 = combine_4paired_register_region(level_3_region);

            if(level_1 == 0 || level_2 == 0 || level_3 == 0) {
                *_register->logical_channel[logical_channel_id].exception_message |= mask;
                ret = ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
            }
        }

        nb_binding++;
        maximum_source_current += level_3;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH7_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_7_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %hu\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        } else {
            // check if configuration is correct
            level_1_region = &_register->physical_channel[PHYSICAL_CHANNEL_7_ID].source_current_level_1_region;
            level_2_region = &_register->physical_channel[PHYSICAL_CHANNEL_7_ID].source_current_level_2_region;
            level_3_region = &_register->physical_channel[PHYSICAL_CHANNEL_7_ID].source_current_level_3_region;
    
            level_1 = combine_4paired_register_region(level_1_region);
            level_2 = combine_4paired_register_region(level_2_region);
            level_3 = combine_4paired_register_region(level_3_region);

            if(level_1 == 0 || level_2 == 0 || level_3 == 0) {
                *_register->logical_channel[logical_channel_id].exception_message |= mask;
                ret = ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
            }
        }

        nb_binding++;
        maximum_source_current += level_3;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH8_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_8_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %hu\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        } else {
            // check if configuration is correct
            level_1_region = &_register->physical_channel[PHYSICAL_CHANNEL_8_ID].source_current_level_1_region;
            level_2_region = &_register->physical_channel[PHYSICAL_CHANNEL_8_ID].source_current_level_2_region;
            level_3_region = &_register->physical_channel[PHYSICAL_CHANNEL_8_ID].source_current_level_3_region;
    
            level_1 = combine_4paired_register_region(level_1_region);
            level_2 = combine_4paired_register_region(level_2_region);
            level_3 = combine_4paired_register_region(level_3_region);

            if(level_1 == 0 || level_2 == 0 || level_3 == 0) {
                *_register->logical_channel[logical_channel_id].exception_message |= mask;
                ret = ERROR_SOURCE_CURRENT_CAPACITY_ZERO;
            }
        }

        nb_binding++;
        maximum_source_current += level_3;
    }

    if(!ret) {
        printf("nb_binding %d\n", nb_binding);
        printf("maximum_source_current %"PRIu64"\n", maximum_source_current);
        //printf("nb_binding %d maximum_source_current %"PRIu64"\n", nb_binding, maximum_source_current);
        *_register->logical_channel[logical_channel_id].nb_binding = nb_binding;
        update_4paired_register_region(maximum_source_current_region, maximum_source_current);
    } 

    return ret;
}

int check_if_all_physical_channels_ready(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id, ready = 0;
    uint16_t *nb_binding, *binding, ok_count = 0, i;
    struct timespec time_now;
    _4paired_register_region *step_turnaround_delay_region, *ready_left_boundary_time_region, *ready_right_boundary_time_region, *jump_time_reference_region;
    uint64_t passby_time, step_turnaround_delay, ready_left_boundary_time, ready_right_boundary_time, jump_time_reference;

    _register = &manager->modbus_ctx._register;

    // check if all boundary channel is ready
    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;
    binding = _register->logical_channel[logical_channel_id].binding;
    nb_binding = _register->logical_channel[logical_channel_id].nb_binding;

    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        
        if((((*binding )>>i) & 0x0001) > 0) {
     
            if(*_register->physical_channel[i].ready == 1)
                ok_count++;
        }

    }

    if(ok_count == *nb_binding) {
        return 1;
    }

    //check and update all boudary physical channel ready status
    jump_time_reference_region = &_register->logical_channel[logical_channel_id].temp.jump_time_reference_region;
    step_turnaround_delay_region = &_register->logical_channel[logical_channel_id].timing.step_turnaround_delay_region;
    ready_left_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.ready_left_boundary_time_region;
    ready_right_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.ready_right_boundary_time_region;

    clock_gettime(CLOCK_REALTIME, &time_now);
    step_turnaround_delay = combine_4paired_register_region(step_turnaround_delay_region);
    ready_left_boundary_time = combine_4paired_register_region(ready_left_boundary_time_region);
    ready_right_boundary_time = combine_4paired_register_region(ready_right_boundary_time_region);
    jump_time_reference = combine_4paired_register_region(jump_time_reference_region);

    //update ready left boundary time
    if(ready_left_boundary_time == 0){
        ready_left_boundary_time = (uint64_t)time_now.tv_sec * UINT_FACTOR_BETWEEN_S_AND_MS + (uint64_t)time_now.tv_nsec / UINT_FACTOR_BETWEEN_MS_AND_NS;
        update_4paired_register_region(ready_left_boundary_time_region, ready_left_boundary_time);
    }

    //update ready right boundary time
    ready_right_boundary_time = (uint64_t)time_now.tv_sec * UINT_FACTOR_BETWEEN_S_AND_MS + (uint64_t)time_now.tv_nsec / UINT_FACTOR_BETWEEN_MS_AND_NS;
    update_4paired_register_region(ready_right_boundary_time_region, ready_right_boundary_time);

    passby_time = ready_right_boundary_time - ready_left_boundary_time;

    //check physical channle ready
    if(passby_time >= step_turnaround_delay){
        ready = 1;
    }

    //update boudary physical channel ready status
    ok_count = 0;
    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        if((((*binding )>>i) & 0x0001) > 0) {
            if(*_register->physical_channel[i].ready != ready){
                *_register->physical_channel[i].ready = ready;
                ok_count++;
            }
        }
    }

    if(ok_count == *nb_binding) {
        jump_time_reference = jump_time_reference + passby_time;
        update_4paired_register_region(jump_time_reference_region, jump_time_reference);
    }

    return 0;
}


int check_if_voltage_difference_of_all_physical_channels_in_tolerance(Manager *manager, int channel_id)
{
    int logical_channel_id = channel_id;
    Register *_register = NULL;
    uint16_t id, *nb_binding, check_count, *binding, i;
    uint64_t common_node_voltage_tolerance, temp_voltage1, temp_voltage2;
    _4paired_register_region *common_node_voltage_tolerance_region, *temp_sample_voltage_region;

    _register = &manager->modbus_ctx._register;

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    check_count = 0;
    temp_voltage1 = 0;
    temp_voltage2 = 0;
    binding = _register->logical_channel[logical_channel_id].binding;
    nb_binding = _register->logical_channel[logical_channel_id].nb_binding;
  
    common_node_voltage_tolerance_region = &_register->logical_channel[logical_channel_id].tolerance.common_node_voltage_tolerance_region;
    common_node_voltage_tolerance = combine_4paired_register_region(common_node_voltage_tolerance_region);

    //printf("common node voltage tolerance %"PRIu64", nb_binding %d\n", common_node_voltage_tolerance, *nb_binding);

    if(*nb_binding == 1) {
        //printf("we don't need to check common voltage so far!!!\n");
        return 0;
    }

    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        
        if((((*binding )>>i) & 0x0001) > 0) {
     
            if(check_count == 0) {
                temp_sample_voltage_region = &_register->physical_channel[i].sample.voltage_region;
                temp_voltage1 = combine_4paired_register_region(temp_sample_voltage_region);
            } else {
                temp_sample_voltage_region = &_register->physical_channel[i].sample.voltage_region;                
                temp_voltage2 = combine_4paired_register_region(temp_sample_voltage_region);

                //check if exception
#if 0
                if(abs(temp_voltage1 - temp_voltage2) > common_node_voltage_tolerance) {
                    //printf("> tolerance, exceptions occur\n");
                    return ERROR_COMMON_NODE_VOLTAGE_TOLERANCE_EXCEEDED;
                }
#endif

                if(temp_voltage1 > temp_voltage2) {
                    if((temp_voltage1 - temp_voltage2) > common_node_voltage_tolerance) {
                        return ERROR_COMMON_NODE_VOLTAGE_TOLERANCE_EXCEEDED;
                    }
                } else {
                    if((temp_voltage2 - temp_voltage1) > common_node_voltage_tolerance) {
                        return ERROR_COMMON_NODE_VOLTAGE_TOLERANCE_EXCEEDED;
                    }
                }

                temp_voltage1 = temp_voltage2;
            }

            check_count++;
        }

    }

    return 0;
}

void update_bound_physical_channel_status_to_running(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    int ret = 0, mask, logical_channel_id = channel_id;
    Register *_register = NULL;
    uint16_t *step_type_reg, *step_pointer_reg;

    _register = &manager->modbus_ctx._register;

    // check if physical channel 1 is in count
    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH1_MASK_BIT;
    if((*_register->logical_channel[logical_channel_id].binding) & mask) {
        *_register->physical_channel[LOGICAL_CHANNEL_1_ID].bound_relation = logical_channel_id;
        *_register->physical_channel[LOGICAL_CHANNEL_1_ID].status = PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH2_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        *_register->physical_channel[LOGICAL_CHANNEL_2_ID].bound_relation = logical_channel_id;
        *_register->physical_channel[LOGICAL_CHANNEL_2_ID].status = PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH3_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        *_register->physical_channel[LOGICAL_CHANNEL_3_ID].bound_relation = logical_channel_id;
        *_register->physical_channel[LOGICAL_CHANNEL_3_ID].status = PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH4_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        *_register->physical_channel[LOGICAL_CHANNEL_4_ID].bound_relation = logical_channel_id;
        *_register->physical_channel[LOGICAL_CHANNEL_4_ID].status = PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH5_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        *_register->physical_channel[LOGICAL_CHANNEL_5_ID].bound_relation = logical_channel_id;
        *_register->physical_channel[LOGICAL_CHANNEL_5_ID].status = PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH6_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        *_register->physical_channel[LOGICAL_CHANNEL_6_ID].bound_relation = logical_channel_id;
        *_register->physical_channel[LOGICAL_CHANNEL_6_ID].status = PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH7_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        *_register->physical_channel[LOGICAL_CHANNEL_7_ID].bound_relation = logical_channel_id;
        *_register->physical_channel[LOGICAL_CHANNEL_7_ID].status = PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH8_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        *_register->physical_channel[LOGICAL_CHANNEL_8_ID].bound_relation = logical_channel_id;
        *_register->physical_channel[LOGICAL_CHANNEL_8_ID].status = PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;
    }

}

int update_bound_physical_channel_vi_registers_by_calibration_current(Manager *_manager, int _channel_id, uint64_t _calibration_current)
{
    Manager *manager = _manager;
    int logical_channel_id = _channel_id, i;
    Register *_register = NULL;
    int64_t temp = 0, temp_current, temp_vi;
    uint64_t calibration_current = _calibration_current;
    uint64_t unit_scale, base_voltage, half_scale;
    uint16_t *current_source_level, *last_current_source_level, *kz_h, *kz_m, *kz_l, *binding;
    uint64_t current_amplitude, level_1, level_2, level_3, physical_channel_current, vi, sign;
    
    _4paired_register_region *vi_region, *level_1_region, *level_2_region, *level_3_region;
    _4paired_register_region *last_current_region, *current_region, *last_current;
    _4paired_register_region *c_vi_b_region, *d_vi_b_region;
    uint64_t c_vi_b_unsign, d_vi_b_unsign;
    int64_t c_vi_b_sign, d_vi_b_sign;
    double c_vi_k, d_vi_k;
    uint16_t *c_vi_k_reg, *d_vi_k_reg;
    uint64_t vi_max;

    printf("update_bound_physical_channel_vi_registers_by_calibration_current calibration_current:%"PRIu64"\n", calibration_current);

    _register = &manager->modbus_ctx._register;

    binding = _register->logical_channel[logical_channel_id].binding;

    if(calibration_current  & 0x8000000000000000) { /* Negative */
        sign = 0x8000000000000000;
        current_amplitude = calibration_current & 0x7fffffffffffffff;        
    } else { /* Positive */
        sign = 0;
        current_amplitude = calibration_current;
    }

    //printf("current_amplitude %"PRIu64" binding 0x%x\n", current_amplitude, *binding);

    current_source_level = _register->logical_channel[logical_channel_id].calibration_param.current_source_level; 

    for(i=PHYSICAL_CHANNEL_1_ID; i<NB_LOGIC_CHANNEL; i++) {

        if(((( *binding )>>i) & 0x0001) > 0) {
            
            vi_region = &_register->physical_channel[i].vi_region;

            kz_h = _register->physical_channel[i].kz_h;
            kz_m = _register->physical_channel[i].kz_m;
            kz_l = _register->physical_channel[i].kz_l;

            level_1_region = &_register->physical_channel[i].source_current_level_1_region;
            level_2_region = &_register->physical_channel[i].source_current_level_2_region;
            level_3_region = &_register->physical_channel[i].source_current_level_3_region;

            if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1) {

                level_1 = combine_4paired_register_region(level_1_region);

                *kz_h=0;
                *kz_m=0;
                *kz_l=1;

                c_vi_k_reg = _register->physical_channel[i].kb.c_vi_k_level_1;
                d_vi_k_reg = _register->physical_channel[i].kb.d_vi_k_level_1;
                c_vi_b_region = &_register->physical_channel[i].kb.c_vi_b_level1_region;
                d_vi_b_region = &_register->physical_channel[i].kb.d_vi_b_level1_region;

                c_vi_k = (double)(*c_vi_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);
                d_vi_k = (double)(*d_vi_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);

                c_vi_b_unsign = combine_4paired_register_region(c_vi_b_region);
                d_vi_b_unsign = combine_4paired_register_region(d_vi_b_region);

                if(c_vi_b_unsign & 0x8000000000000000){ //negative
                    c_vi_b_sign = (c_vi_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    c_vi_b_sign = c_vi_b_unsign;
                }

                if(d_vi_b_unsign & 0x8000000000000000){ //negative
                    d_vi_b_sign = (d_vi_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    d_vi_b_sign = d_vi_b_unsign;
                }

                c_vi_k = (double)(*c_vi_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;
                d_vi_k = (double)(*d_vi_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;

                if(sign) {
                    temp_current = -1 * temp_current;
                    temp = (double)temp_current * d_vi_k + (double)d_vi_b_sign;
                } else {
                    temp_current = temp_current;
                    temp = (double)temp_current * c_vi_k + (double)c_vi_b_sign;
                }

                half_scale = PHYSICAL_CHANNEL_VI_HALFSCALE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                base_voltage = PHYSICAL_CHANNEL_BASE_VOLTAGE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                unit_scale = (half_scale)/(level_1);
                temp_vi = base_voltage + temp * unit_scale;

                // check if vi is valid
                if(temp_vi < 0) {
                    printf("update_bound_physical_channel_vi_registers_by_calibration_current temp_vi<0\n");
                    return -1;
                }

                vi = base_voltage + temp * unit_scale;
                vi_max = PHYSICAL_CHANNEL_VI_MAX_IN_MV_UNIT * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                if(vi > vi_max){
                    printf("update_bound_physical_channel_vi_registers_by_calibration_current vi>vi_max\n");
                    return -1;
                }

                update_4paired_register_region(vi_region, vi);
            } else if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2) {

                level_2 = combine_4paired_register_region(level_2_region);

                *kz_h=0;
                *kz_m=1;
                *kz_l=0;

                c_vi_k_reg = _register->physical_channel[i].kb.c_vi_k_level_2;
                d_vi_k_reg = _register->physical_channel[i].kb.d_vi_k_level_2;
                c_vi_b_region = &_register->physical_channel[i].kb.c_vi_b_level2_region;
                d_vi_b_region = &_register->physical_channel[i].kb.d_vi_b_level2_region;

                c_vi_k = (double)(*c_vi_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);
                d_vi_k = (double)(*d_vi_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);

                c_vi_b_unsign = combine_4paired_register_region(c_vi_b_region);
                d_vi_b_unsign = combine_4paired_register_region(d_vi_b_region);

                if(c_vi_b_unsign & 0x8000000000000000){ //negative
                    c_vi_b_sign = (c_vi_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    c_vi_b_sign = c_vi_b_unsign;
                }

                if(d_vi_b_unsign & 0x8000000000000000){ //negative
                    d_vi_b_sign = (d_vi_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    d_vi_b_sign = d_vi_b_unsign;
                }

                c_vi_k = (double)(*c_vi_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;
                d_vi_k = (double)(*d_vi_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;

                if(sign) {
                    temp_current = -1 * temp_current;
                    temp = (double)temp_current * d_vi_k + (double)d_vi_b_sign;
                } else {
                    temp_current = temp_current;
                    temp = (double)temp_current * c_vi_k + (double)c_vi_b_sign;
                }

                half_scale = PHYSICAL_CHANNEL_VI_HALFSCALE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                base_voltage = PHYSICAL_CHANNEL_BASE_VOLTAGE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                unit_scale = (half_scale)/(level_2);
                temp_vi = base_voltage + temp * unit_scale;

                // check if vi is valid
                if(temp_vi < 0) {
                    printf("update_bound_physical_channel_vi_registers_by_calibration_current temp_vi<0\n");
                    return -1;
                }

                vi = base_voltage + temp * unit_scale;
                vi_max = PHYSICAL_CHANNEL_VI_MAX_IN_MV_UNIT * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                if(vi > vi_max){
                    printf("update_bound_physical_channel_vi_registers_by_calibration_current vi>vi_max\n");
                    return -1;
                }

                update_4paired_register_region(vi_region, vi);
            } else if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3) {

                level_3 = combine_4paired_register_region(level_3_region);

                *kz_h=0;
                *kz_m=0;
                *kz_l=0;

                c_vi_k_reg = _register->physical_channel[i].kb.c_vi_k_level_3;
                d_vi_k_reg = _register->physical_channel[i].kb.d_vi_k_level_3;
                c_vi_b_region = &_register->physical_channel[i].kb.c_vi_b_level3_region;
                d_vi_b_region = &_register->physical_channel[i].kb.d_vi_b_level3_region;

                c_vi_k = (double)(*c_vi_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);
                d_vi_k = (double)(*d_vi_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);

                c_vi_b_unsign = combine_4paired_register_region(c_vi_b_region);
                d_vi_b_unsign = combine_4paired_register_region(d_vi_b_region);

                if(c_vi_b_unsign & 0x8000000000000000){ //negative
                    c_vi_b_sign = (c_vi_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    c_vi_b_sign = c_vi_b_unsign;
                }

                if(d_vi_b_unsign & 0x8000000000000000){ //negative
                    d_vi_b_sign = (d_vi_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    d_vi_b_sign = d_vi_b_unsign;
                }

                c_vi_k = (double)(*c_vi_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;
                d_vi_k = (double)(*d_vi_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;

                if(sign) {
                    temp_current = -1 * temp_current;
                    temp = (double)temp_current * d_vi_k + (double)d_vi_b_sign;
                } else {
                    temp_current = temp_current;
                    temp = (double)temp_current * c_vi_k + (double)c_vi_b_sign;
                }

                half_scale = PHYSICAL_CHANNEL_VI_HALFSCALE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                base_voltage = PHYSICAL_CHANNEL_BASE_VOLTAGE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                unit_scale = (half_scale)/(level_3);
                temp_vi = base_voltage + temp * unit_scale;

                // check if vi is valid
                if(temp_vi < 0) {
                    printf("update_bound_physical_channel_vi_registers_by_calibration_current temp_vi<0\n");
                    return -1;
                }

                vi = base_voltage + temp * unit_scale;
                vi_max = PHYSICAL_CHANNEL_VI_MAX_IN_MV_UNIT * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                if(vi > vi_max){
                    printf("update_bound_physical_channel_vi_registers_by_calibration_current vi>vi_max\n");
                    return -1;
                }

                update_4paired_register_region(vi_region, vi);
            } else {
                // unknown errors occur
                return -1;
            }

#if 0
            printf("level 1 %"PRIu64"\n", level_1);
            printf("level 2 %"PRIu64"\n", level_2);
            printf("level 3 %"PRIu64"\n", level_3);
            printf("half_scale %"PRIu64"\n", half_scale);
            printf("base_voltage %"PRIu64"\n", base_voltage);
            printf("unit_scale %"PRIu64"\n", unit_scale);
            printf("temp %"PRIu64"\n", temp);
            printf("vi %"PRIu64"\n", vi);
            printf("phy current %"PRIu64"\n", physical_channel_current);

            if(logic_current  & 0x8000000000000000) {
                printf("channel_id %d negative current %"PRIu64"\n", i, temp_current);
            } else {
                printf("channel_id %d positive current %"PRIu64"\n", i, temp_current);
            }
#endif
        }
    }

    return 0;
}


int update_bound_physical_channel_current_source_and_vi_registers(Manager *_manager, int _channel_id, uint64_t _logic_current)
{
    Manager *manager = _manager;
    int logical_channel_id = _channel_id, i;
    Register *_register = NULL;
    int64_t temp = 0, temp_current, temp_vi;
    uint64_t logic_current = _logic_current;
    uint64_t unit_scale, base_voltage, half_scale;
    uint16_t *current_source_level, *last_current_source_level, *kz_h, *kz_m, *kz_l, *binding;
    uint64_t current_amplitude, calc_current, left, level_1, level_2, level_3, physical_channel_current, vi, sign;
    
    _4paired_register_region *vi_region, *level_1_region, *level_2_region, *level_3_region;
    _4paired_register_region *last_current_region, *current_region, *last_current;
    _4paired_register_region *c_vi_b_region, *d_vi_b_region;
    uint64_t c_vi_b_unsign, d_vi_b_unsign;
    int64_t c_vi_b_sign, d_vi_b_sign;
    double c_vi_k, d_vi_k;
    uint16_t *c_vi_k_reg, *d_vi_k_reg;
    uint64_t vi_max;

    //printf("update_bound_physical_channel_vi_registers %"PRIu64"\n", _logic_current);

    _register = &manager->modbus_ctx._register;

    binding = _register->logical_channel[logical_channel_id].binding;

    if(logic_current  & 0x8000000000000000) { /* Negative */
        sign = 0x8000000000000000;
        current_amplitude = logic_current & 0x7fffffffffffffff;
    } else { /* Positive */
        sign = 0;
        current_amplitude = logic_current;
    }

    //printf("current_amplitude %"PRIu64" binding 0x%x\n", current_amplitude, *binding);

    for(i=PHYSICAL_CHANNEL_1_ID, calc_current=0; i<NB_LOGIC_CHANNEL; i++) {

        if(((( *binding )>>i) & 0x0001) > 0) {
            
            vi_region = &_register->physical_channel[i].vi_region;

            kz_h = _register->physical_channel[i].kz_h;
            kz_m = _register->physical_channel[i].kz_m;
            kz_l = _register->physical_channel[i].kz_l;

            level_1_region = &_register->physical_channel[i].source_current_level_1_region;
            level_2_region = &_register->physical_channel[i].source_current_level_2_region;
            level_3_region = &_register->physical_channel[i].source_current_level_3_region;

            level_1 = combine_4paired_register_region(level_1_region);
            level_2 = combine_4paired_register_region(level_2_region);
            level_3 = combine_4paired_register_region(level_3_region);

            current_source_level = _register->physical_channel[i].current_source_level;
            last_current_source_level = _register->physical_channel[i].temp.last_current_source_level;

            last_current_region = &_register->physical_channel[i].temp.last_current_region;
            
            current_region = &_register->physical_channel[i].current_region;

            left = current_amplitude - calc_current;

            //printf("left %"PRIu64", channel_id %d\n", left, i);

            if((left > level_3) | ((left > level_2) && (left <= level_3)) ) {
                if(left >= level_3) {
                    calc_current += level_3;
                    temp_current = level_3;
                } else {
                    calc_current += left;
                    temp_current = left;
                }

                *kz_h=0;
                *kz_m=0;
                *kz_l=0;
                
                *current_source_level = PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3;
                *last_current_source_level = PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3;

                physical_channel_current = temp_current | sign;
                update_4paired_register_region(current_region, physical_channel_current);
                update_4paired_register_region(last_current_region, physical_channel_current);

                c_vi_k_reg = _register->physical_channel[i].kb.c_vi_k_level_3;
                d_vi_k_reg = _register->physical_channel[i].kb.d_vi_k_level_3;
                c_vi_b_region = &_register->physical_channel[i].kb.c_vi_b_level3_region;
                d_vi_b_region = &_register->physical_channel[i].kb.d_vi_b_level3_region;

                c_vi_k = (double)(*c_vi_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);
                d_vi_k = (double)(*d_vi_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);

                c_vi_b_unsign = combine_4paired_register_region(c_vi_b_region);
                d_vi_b_unsign = combine_4paired_register_region(d_vi_b_region);

                if(c_vi_b_unsign & 0x8000000000000000){ //negative
                    c_vi_b_sign = (c_vi_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    c_vi_b_sign = c_vi_b_unsign;
                }

                if(d_vi_b_unsign & 0x8000000000000000){ //negative
                    d_vi_b_sign = (d_vi_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    d_vi_b_sign = d_vi_b_unsign;
                }

                c_vi_k = (double)(*c_vi_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;
                d_vi_k = (double)(*d_vi_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;

                if(sign) {
                    temp_current = -1 * temp_current;
                    temp = (double)temp_current * d_vi_k + (double)d_vi_b_sign;
                } else {
                    temp_current = temp_current;
                    temp = (double)temp_current * c_vi_k + (double)c_vi_b_sign;
                }

                half_scale = PHYSICAL_CHANNEL_VI_HALFSCALE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                base_voltage = PHYSICAL_CHANNEL_BASE_VOLTAGE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                unit_scale = (half_scale)/(level_3);
                temp_vi = base_voltage + temp * unit_scale;

                // check if vi is valid
                if(temp_vi < 0) {
                    printf("update_bound_physical_channel_current_source_and_vi_registers temp_vi<0\n");
                    return -1;
                }

                vi = base_voltage + temp * unit_scale;
                vi_max = PHYSICAL_CHANNEL_VI_MAX_IN_MV_UNIT * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                if(vi > vi_max){
                    printf("update_bound_physical_channel_current_source_and_vi_registers vi>vi_max\n");
                    return -1;
                }

                update_4paired_register_region(vi_region, vi);
            } else if((left > level_1) && (left <= level_2)) {
                if(left == level_2) {
                    calc_current += level_2;
                    temp_current = level_2;
                } else {
                    calc_current += left;
                    temp_current = left;
                }

                *kz_h=0;
                *kz_m=1;
                *kz_l=0;
       
                *current_source_level = PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2;
                *last_current_source_level = PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2;

                physical_channel_current = temp_current | sign;
                update_4paired_register_region(current_region, physical_channel_current);
                update_4paired_register_region(last_current_region, physical_channel_current);

                c_vi_k_reg = _register->physical_channel[i].kb.c_vi_k_level_2;
                d_vi_k_reg = _register->physical_channel[i].kb.d_vi_k_level_2;
                c_vi_b_region = &_register->physical_channel[i].kb.c_vi_b_level2_region;
                d_vi_b_region = &_register->physical_channel[i].kb.d_vi_b_level2_region;

                c_vi_k = (double)(*c_vi_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);
                d_vi_k = (double)(*d_vi_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);

                c_vi_b_unsign = combine_4paired_register_region(c_vi_b_region);
                d_vi_b_unsign = combine_4paired_register_region(d_vi_b_region);

                if(c_vi_b_unsign & 0x8000000000000000){ //negative
                    c_vi_b_sign = (c_vi_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    c_vi_b_sign = c_vi_b_unsign;
                }

                if(d_vi_b_unsign & 0x8000000000000000){ //negative
                    d_vi_b_sign = (d_vi_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    d_vi_b_sign = d_vi_b_unsign;
                }

                c_vi_k = (double)(*c_vi_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;
                d_vi_k = (double)(*d_vi_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;

                if(sign) {
                    temp_current = -1 * temp_current;
                    temp = (double)temp_current * d_vi_k + (double)d_vi_b_sign;
                } else {
                    temp_current = temp_current;
                    temp = (double)temp_current * c_vi_k + (double)c_vi_b_sign;
                }

                half_scale = PHYSICAL_CHANNEL_VI_HALFSCALE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                base_voltage = PHYSICAL_CHANNEL_BASE_VOLTAGE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                unit_scale = (half_scale)/(level_2);
                temp_vi = base_voltage + temp * unit_scale;

                // check if vi is valid
                if(temp_vi < 0) {
                    printf("update_bound_physical_channel_current_source_and_vi_registers temp_vi<0\n");
                    return -1;
                }

                vi = base_voltage + temp * unit_scale;
                vi_max = PHYSICAL_CHANNEL_VI_MAX_IN_MV_UNIT * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                if(vi > vi_max){
                    printf("update_bound_physical_channel_current_source_and_vi_registers vi>vi_max\n");
                    return -1;
                }

                update_4paired_register_region(vi_region, vi);
            } else if(left <= level_1) {
                if(left == level_1) {
                    calc_current += level_1;
                    temp_current = level_1;
                } else {
                    calc_current += left;
                    temp_current = left;
                }

                *kz_h=0;
                *kz_m=0;
                *kz_l=1;

                *current_source_level = PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1;
                *last_current_source_level = PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1;

                physical_channel_current = temp_current | sign;
                update_4paired_register_region(current_region, physical_channel_current);
                update_4paired_register_region(last_current_region, physical_channel_current);

                c_vi_k_reg = _register->physical_channel[i].kb.c_vi_k_level_1;
                d_vi_k_reg = _register->physical_channel[i].kb.d_vi_k_level_1;
                c_vi_b_region = &_register->physical_channel[i].kb.c_vi_b_level1_region;
                d_vi_b_region = &_register->physical_channel[i].kb.d_vi_b_level1_region;

                c_vi_k = (double)(*c_vi_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);
                d_vi_k = (double)(*d_vi_k_reg) / (double)(UINT_FACTOR_BETWEEN_1_AND_00001);

                c_vi_b_unsign = combine_4paired_register_region(c_vi_b_region);
                d_vi_b_unsign = combine_4paired_register_region(d_vi_b_region);

                if(c_vi_b_unsign & 0x8000000000000000){ //negative
                    c_vi_b_sign = (c_vi_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    c_vi_b_sign = c_vi_b_unsign;
                }

                if(d_vi_b_unsign & 0x8000000000000000){ //negative
                    d_vi_b_sign = (d_vi_b_unsign & 0x7fffffffffffffff) * -1;
                } else {
                    d_vi_b_sign = d_vi_b_unsign;
                }

                c_vi_k = (double)(*c_vi_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;
                d_vi_k = (double)(*d_vi_k_reg) / (double)UINT_FACTOR_BETWEEN_1_AND_00001;

                if(sign) {
                    temp_current = -1 * temp_current;
                    temp = (double)temp_current * d_vi_k + (double)d_vi_b_sign;
                } else {
                    temp_current = temp_current;
                    temp = (double)temp_current * c_vi_k + (double)c_vi_b_sign;
                }

                half_scale = PHYSICAL_CHANNEL_VI_HALFSCALE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                base_voltage = PHYSICAL_CHANNEL_BASE_VOLTAGE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                unit_scale = (half_scale)/(level_1);
                temp_vi = base_voltage + temp * unit_scale;

                // check if vi is valid
                if(temp_vi < 0) {
                    printf("update_bound_physical_channel_current_source_and_vi_registers temp_vi<0\n");
                    return -1;
                }

                vi = base_voltage + temp * unit_scale;
                vi_max = PHYSICAL_CHANNEL_VI_MAX_IN_MV_UNIT * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
                if(vi > vi_max){
                    printf("update_bound_physical_channel_current_source_and_vi_registers vi>vi_max\n");
                    return -1;
                }

                update_4paired_register_region(vi_region, vi);
            }
#if 0
            printf("level 1 %"PRIu64"\n", level_1);
            printf("level 2 %"PRIu64"\n", level_2);
            printf("level 3 %"PRIu64"\n", level_3);
            printf("half_scale %"PRIu64"\n", half_scale);
            printf("base_voltage %"PRIu64"\n", base_voltage);
            printf("unit_scale %"PRIu64"\n", unit_scale);
            printf("temp %"PRIu64"\n", temp);
            printf("vi %"PRIu64"\n", vi);
            printf("phy current %"PRIu64"\n", physical_channel_current);

            if(logic_current  & 0x8000000000000000) {
                printf("channel_id %d negative current %"PRIu64"\n", i, temp_current);
            } else {
                printf("channel_id %d positive current %"PRIu64"\n", i, temp_current);
            }
#endif
        }
    }

    return 0;
}

int update_bound_physical_channel_vu_registers_by_calibration_voltage(Manager *_manager, int _channel_id, uint64_t _calibration_voltage)
{
    int i;
    uint16_t *binding = NULL;
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = _channel_id;
    uint64_t calibration_voltage = _calibration_voltage, vu, vu_max;
    _4paired_register_region *vu_region, *channel_voltage_region, *last_channel_voltage_region;
    double vu_k;
    uint16_t *vu_k_reg;
    _4paired_register_region *vu_b_region;
    int64_t vu_b_sign, temp_vu;
    uint64_t vu_b_unsign;

    printf("update_bound_physical_channel_vu_registers_by_calibration_voltage calibration_voltage: %"PRIu64"\n", calibration_voltage);

    _register = &manager->modbus_ctx._register;

    binding = _register->logical_channel[logical_channel_id].binding;

    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        if((((*binding )>>i) & 0x0001) > 0) {
            vu_region = &_register->physical_channel[i].vu_region;
            channel_voltage_region = &_register->physical_channel[i].voltage_region;
            last_channel_voltage_region = &_register->physical_channel[i].temp.last_voltage_region;

            vu_b_region = &_register->physical_channel[i].kb.vu_b_region;
            
            vu_b_unsign = combine_4paired_register_region(vu_b_region);
            if(vu_b_unsign & 0x8000000000000000){ //negative
                vu_b_sign = (vu_b_unsign & 0x7fffffffffffffff) * -1;
            } else {
                vu_b_sign = vu_b_unsign;
            }

            vu_k = (double)*_register->physical_channel[i].kb.vu_k / (double)UINT_FACTOR_BETWEEN_1_AND_00001;
            
            temp_vu = (double)calibration_voltage * vu_k + (double)vu_b_sign;
            if(temp_vu < 0){
                printf("update_bound_physical_channel_vu_registers_by_calibration_voltage temp_vu<0\n");
                return -1;
            }

            vu  = (double)calibration_voltage * vu_k + (double)vu_b_sign;
            vu_max = PHYSICAL_CHANNEL_VU_MAX_IN_MV_UNIT * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
            if(vu >  vu_max){
                printf("update_bound_physical_channel_vu_registers_by_calibration_voltage vu>vu_max\n");
                return -1;
            }

            update_4paired_register_region(vu_region, vu);
        }
    }

    return 0;
}


int update_bound_physical_channel_vu_registers(Manager *_manager, int _channel_id, uint64_t _logic_voltage)
{
    int i;
    uint16_t *binding = NULL;
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = _channel_id;
    uint64_t logic_voltage = _logic_voltage, vu, vu_max;
    _4paired_register_region *vu_region, *channel_voltage_region, *last_channel_voltage_region;
    double vu_k;
    uint16_t *vu_k_reg;
    _4paired_register_region *vu_b_region;
    int64_t vu_b_sign, temp_vu;
    uint64_t vu_b_unsign;

    printf("update_bound_physical_channel_vu_registers logic_voltage %"PRIu64"\n", logic_voltage);

    _register = &manager->modbus_ctx._register;

    binding = _register->logical_channel[logical_channel_id].binding;

    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        if((((*binding )>>i) & 0x0001) > 0) {
            vu_region = &_register->physical_channel[i].vu_region;
            channel_voltage_region = &_register->physical_channel[i].voltage_region;
            last_channel_voltage_region = &_register->physical_channel[i].temp.last_voltage_region;

            vu_b_region = &_register->physical_channel[i].kb.vu_b_region;
            
            vu_b_unsign = combine_4paired_register_region(vu_b_region);
            if(vu_b_unsign & 0x8000000000000000){ //negative
                vu_b_sign = (vu_b_unsign & 0x7fffffffffffffff) * -1;
            } else {
                vu_b_sign = vu_b_unsign;
            }

            vu_k = (double)*_register->physical_channel[i].kb.vu_k / (double)UINT_FACTOR_BETWEEN_1_AND_00001;
            
            temp_vu = (double)logic_voltage * vu_k + (double)vu_b_sign;
            if(temp_vu < 0){
                printf("update_bound_physical_channel_vu_registers temp_vu<0\n");
                return -1;
            }

            vu  = (double)logic_voltage * vu_k + (double)vu_b_sign;
            vu_max = PHYSICAL_CHANNEL_VU_MAX_IN_MV_UNIT * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
            if(vu >  vu_max){
                printf("update_bound_physical_channel_vu_registers vu>vu_max\n");
                return -1;
            }

            update_4paired_register_region(vu_region, vu);
            update_4paired_register_region(channel_voltage_region, logic_voltage);
            update_4paired_register_region(last_channel_voltage_region, logic_voltage);
        }
    }

    return 0;
}

int update_bound_physical_channel_jc_registers(Manager *_manager, int _channel_id, int _operation)
{
    Manager *manager = _manager;
    uint16_t *jc_reg = NULL, *binding = NULL, *bound_relation;
    int i;
    Register *_register = NULL;
    int physical_channel_id = _channel_id;
    int operation = _operation;
    int logical_channel_id = _channel_id;

    _register = &manager->modbus_ctx._register;

    binding = _register->logical_channel[logical_channel_id].binding;
    
    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        
        if((((*binding )>>i) & 0x0001) > 0) {
     
            bound_relation = _register->physical_channel[i].bound_relation;
            
            if(*bound_relation == logical_channel_id) {
                
                jc_reg = _register->physical_channel[i].jc;

                *jc_reg = operation;

            } else {
                printf("update_bound_physical_channel_jc_registers fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
        }
    }

    return 0;
}

int update_bound_physical_channel_info_io_registers(Manager *_manager, int _channel_id, uint16_t step_type_reg)
{
    Manager *manager = _manager;
    uint16_t *cf_reg = NULL, *cled_reg = NULL, *fled_reg = NULL, *binding = NULL, *bound_relation;
    int i;
    Register *_register = NULL;
    int physical_channel_id = _channel_id;
    int logical_channel_id = _channel_id;
    int cf_operation = CF_DISABLE, cled_operation = CLED_DISABLE, fled_operation = FLED_DISABLE;

    _register = &manager->modbus_ctx._register;

    binding = _register->logical_channel[logical_channel_id].binding;

    switch (step_type_reg) {
        
        case FORMATION_STEP_TYPE_REST:
        case CALIBRATION_STEP_TYPE_CVAD:
            cf_operation = CF_ENABLE;
            cled_operation = CLED_ENABLE;
            fled_operation = FLED_ENABLE;
        break;

        case FORMATION_STEP_TYPE_CCC:
        case FORMATION_STEP_TYPE_CCCCVC:
        case FORMATION_STEP_TYPE_CPC:
        case CALIBRATION_STEP_TYPE_CCC:
        case CALIBRATION_STEP_TYPE_CVDA:
            cf_operation = CF_ENABLE;
            cled_operation = CLED_ENABLE;
            fled_operation = FLED_DISABLE;
        break;

        case FORMATION_STEP_TYPE_CCD:
        case FORMATION_STEP_TYPE_CCDCVD:
        case FORMATION_STEP_TYPE_CPD:
        case FORMATION_STEP_TYPE_CRD:
        case CALIBRATION_STEP_TYPE_CCD:
            cf_operation = CF_DISABLE;
            cled_operation = CLED_DISABLE;
            fled_operation = FLED_ENABLE;
        break;

        default:
            printf("update_bound_physical_channel_info_io_registers unknow step_type\n");
            return ERROR_UNKNOWN_STEP_TYPE;
        break;

    }
    
    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        
        if((((*binding )>>i) & 0x0001) > 0) {
     
            bound_relation = _register->physical_channel[i].bound_relation;
            
            if(*bound_relation == logical_channel_id) {
                
                cf_reg = _register->physical_channel[i].cf;
                cled_reg = _register->physical_channel[i].cled;
                fled_reg = _register->physical_channel[i].fled;

                *cf_reg = cf_operation;
                *cled_reg = cled_operation;
                *fled_reg = cled_operation;
                
            } else {
                printf("update_bound_physical_channel_jc_registers fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
        }
    }

    return 0;
}



int check_if_do_channel_regulation(Manager *_manager, int channel_id) 
{
    int ret = 0;
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint16_t *step_type_reg, *exp_message_reg;    
    _4paired_register_region *common_power_region, *logic_voltage_region, *common_resistance_region;
    _4paired_register_region *vu_bias_region, *jump_voltage_region;
    uint64_t logic_voltage, logic_current, vu_bias, common_power, jump_voltage, common_resistance;
    double temp1, temp2, temp3;

    _register = &manager->modbus_ctx._register;

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    step_type_reg = _register->logical_channel[logical_channel_id].step_param.step_type;
    exp_message_reg = _register->logical_channel[logical_channel_id].exception_message;

    switch(*step_type_reg) {

        case FORMATION_STEP_TYPE_CCC:
        case FORMATION_STEP_TYPE_CCD:
        case FORMATION_STEP_TYPE_CCCCVC:
        case FORMATION_STEP_TYPE_CCDCVD:
        case FORMATION_STEP_TYPE_GOTO:
        case FORMATION_STEP_TYPE_REST:
        case FORMATION_STEP_TYPE_CYCLE:
             return 0;
        break;

        case FORMATION_STEP_TYPE_CPC:
             logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;
             logic_voltage = combine_4paired_register_region(logic_voltage_region);

             //printf("logic_voltage %"PRIu64"\n", logic_voltage);

             vu_bias_region = &_register->physical_channel[logical_channel_id].vu_bias_region;
             vu_bias = combine_4paired_register_region(vu_bias_region);

             //printf("vu_bias %"PRIu64"\n", vu_bias);

             // get user defined power
             common_power_region = &_register->logical_channel[logical_channel_id].step_param.info.common.power_region;
             common_power = combine_4paired_register_region(common_power_region);

             //printf("common_power %"PRIu64"\n", common_power);

             temp1 = logic_voltage*UNIT_FACTOR_BETWEEN_10NV_AND_10PV;
             temp2 = common_power;
             temp3 = (temp2/temp1);

             logic_current = temp3 * UNIT_FACTOR_BETWEEN_A_AND_01UA;

             //printf("temp1: %.15f, temp2:%.15f, temp3: %.15f\n", temp1, temp2, temp3);

             //printf("logic_current %"PRIu64"\n", logic_current);

             ret = update_bound_physical_channel_current_source_and_vi_registers(manager, logical_channel_id, logic_current);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_CURRENT_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_CURRENT_EXCEEDED;
                return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CPD:
             logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;
             logic_voltage = combine_4paired_register_region(logic_voltage_region);

             //printf("logic_voltage %"PRIu64"\n", logic_voltage);

             vu_bias_region = &_register->physical_channel[logical_channel_id].vu_bias_region;
             vu_bias = combine_4paired_register_region(vu_bias_region);

             //printf("vu_bias %"PRIu64"\n", vu_bias);

             // get user defined power
             common_power_region = &_register->logical_channel[logical_channel_id].step_param.info.common.power_region;
             common_power = combine_4paired_register_region(common_power_region);

             //printf("common_power %"PRIu64"\n", common_power);

             temp1 = logic_voltage*UNIT_FACTOR_BETWEEN_10NV_AND_10PV;
             temp2 = common_power;
             temp3 = (temp2/temp1);

             logic_current = (uint64_t)(temp3 * UNIT_FACTOR_BETWEEN_A_AND_01UA) | 0x8000000000000000;

             //printf("temp1: %.15f, temp2:%.15f, temp3: %.15f\n", temp1, temp2, temp3);

             //printf("logic_current %"PRIu64"\n", logic_current);

             ret = update_bound_physical_channel_current_source_and_vi_registers(manager, logical_channel_id, logic_current);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_CURRENT_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_CURRENT_EXCEEDED;
                return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CRD:

             // get present logic channel voltage
             logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;
             logic_voltage = combine_4paired_register_region(logic_voltage_region);

             common_resistance_region = &_register->logical_channel[logical_channel_id].step_param.info.common.resistance_region;
             common_resistance = combine_4paired_register_region(common_resistance_region);

             temp1 = logic_voltage;
             temp2 = common_resistance;
             temp3 = (temp1) / (temp2);

             logic_current = (uint64_t)(temp3 * UNIT_FACTOR_BETWEEN_A_AND_01UA) | 0x8000000000000000;

             ret = update_bound_physical_channel_current_source_and_vi_registers(manager, logical_channel_id, logic_current);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_CURRENT_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_CURRENT_EXCEEDED;
                return -1;
             }

             return 0;
        break;

        default:
             // errors occur
             return ERROR_UNKNOWN_STEP_TYPE;
        break;
    }

    return ret;
}

int init_physical_channel(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    int ret = 0, mask, logical_channel_id = channel_id;
    Register *_register = NULL;
    uint16_t *step_type_reg, *step_pointer_reg, *exp_message_reg; 

    _4paired_register_region *vu_bias_region, *jump_voltage_region;
    _4paired_register_region *common_power_region, *logic_voltage_region;
    _4paired_register_region *common_voltage_region, *common_current_region, *common_resistance_region;
    uint64_t logic_voltage, logic_current, vu_bias, common_voltage, common_current, jump_voltage, common_power, common_resistance; 

    _4paired_register_region *calibration_voltage_region, *calibration_current_region;
    uint64_t calibration_voltage, calibration_current;

    double temp1, temp2, temp3;

    _register = &manager->modbus_ctx._register;

    step_type_reg = _register->logical_channel[logical_channel_id].step_param.step_type;
    step_pointer_reg = _register->logical_channel[logical_channel_id].step_pointer;
    exp_message_reg = _register->logical_channel[logical_channel_id].exception_message;

    printf("init_physical_channel\n");

    switch(*step_type_reg) {

        case FORMATION_STEP_TYPE_REST:

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_ENABLE);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCC: 
             vu_bias_region = &_register->physical_channel[logical_channel_id].vu_bias_region;
             vu_bias = combine_4paired_register_region(vu_bias_region);

             jump_voltage_region = &_register->logical_channel[logical_channel_id].step_param.jump.voltage_region;
             jump_voltage = combine_4paired_register_region(jump_voltage_region);

             common_current_region = &_register->logical_channel[logical_channel_id].step_param.info.common.current_region;
             common_current = combine_4paired_register_region(common_current_region);

             logic_current = common_current;
             logic_voltage = jump_voltage + vu_bias;

             ret = update_bound_physical_channel_current_source_and_vi_registers(manager, logical_channel_id, logic_current);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_CURRENT_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_CURRENT_EXCEEDED;
                return -1;
             }

             ret = update_bound_physical_channel_vu_registers(manager, logical_channel_id, logic_voltage);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_VOLTAGE_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_VOLTAGE_EXCEEDED;
                return -1;
             }

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_DISABLE);
             
             update_bound_physical_channel_info_io_registers(_manager, logical_channel_id, *step_type_reg);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCD:
             vu_bias_region = &_register->physical_channel[logical_channel_id].vu_bias_region;
             vu_bias = combine_4paired_register_region(vu_bias_region);

             jump_voltage_region = &_register->logical_channel[logical_channel_id].step_param.jump.voltage_region;
             jump_voltage = combine_4paired_register_region(jump_voltage_region);

             common_current_region = &_register->logical_channel[logical_channel_id].step_param.info.common.current_region;
             common_current = combine_4paired_register_region(common_current_region);             

             logic_current = common_current | 0x8000000000000000;
             logic_voltage = jump_voltage - vu_bias;

             printf("common current %"PRIu64" logic_current %"PRIu64"\n", common_current, logic_current);

             ret = update_bound_physical_channel_current_source_and_vi_registers(manager, logical_channel_id, logic_current);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_CURRENT_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_CURRENT_EXCEEDED;
                return -1;
             }

             ret = update_bound_physical_channel_vu_registers(manager, logical_channel_id, logic_voltage);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_VOLTAGE_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_VOLTAGE_EXCEEDED;
                return -1;
             }

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_DISABLE);
             
             update_bound_physical_channel_info_io_registers(_manager, logical_channel_id, *step_type_reg);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCCCVC:
             vu_bias_region = &_register->physical_channel[logical_channel_id].vu_bias_region;
             vu_bias = combine_4paired_register_region(vu_bias_region);

             common_voltage_region = &_register->logical_channel[logical_channel_id].step_param.info.common.voltage_region;
             common_voltage = combine_4paired_register_region(common_voltage_region);

             common_current_region = &_register->logical_channel[logical_channel_id].step_param.info.common.current_region;
             common_current = combine_4paired_register_region(common_current_region);

             logic_current = common_current;
             logic_voltage = common_voltage;

             printf("common current %"PRIu64" logic_current %"PRIu64"\n", common_current, logic_current);

             ret = update_bound_physical_channel_current_source_and_vi_registers(manager, logical_channel_id, logic_current);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_CURRENT_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_CURRENT_EXCEEDED;
                return -1;
             }

             ret = update_bound_physical_channel_vu_registers(manager, logical_channel_id, logic_voltage);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_VOLTAGE_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_VOLTAGE_EXCEEDED;
                return -1;
             }

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_DISABLE);

             update_bound_physical_channel_info_io_registers(_manager, logical_channel_id, *step_type_reg);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCDCVD:
             vu_bias_region = &_register->physical_channel[logical_channel_id].vu_bias_region;
             vu_bias = combine_4paired_register_region(vu_bias_region);

             common_voltage_region = &_register->logical_channel[logical_channel_id].step_param.info.common.voltage_region;
             common_voltage = combine_4paired_register_region(common_voltage_region);

             common_current_region = &_register->logical_channel[logical_channel_id].step_param.info.common.current_region;
             common_current = combine_4paired_register_region(common_current_region);

             logic_current = common_current | 0x8000000000000000;
             logic_voltage = common_voltage;

             printf("common current %"PRIu64" logic_current %"PRIu64"\n", common_current, logic_current);

             ret = update_bound_physical_channel_current_source_and_vi_registers(manager, logical_channel_id, logic_current);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_CURRENT_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_CURRENT_EXCEEDED;
                return -1;
             }

             ret = update_bound_physical_channel_vu_registers(manager, logical_channel_id, logic_voltage);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_VOLTAGE_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_VOLTAGE_EXCEEDED;
                return -1;
             }

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_DISABLE);

             update_bound_physical_channel_info_io_registers(_manager, logical_channel_id, *step_type_reg);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CPC:
             
             // get present logic channel voltage
             logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;
             logic_voltage = combine_4paired_register_region(logic_voltage_region);
    
             printf("logic_voltage %"PRIu64"\n", logic_voltage);

             vu_bias_region = &_register->physical_channel[logical_channel_id].vu_bias_region;
             vu_bias = combine_4paired_register_region(vu_bias_region);

             jump_voltage_region = &_register->logical_channel[logical_channel_id].step_param.jump.voltage_region;
             jump_voltage = combine_4paired_register_region(jump_voltage_region);
             
             printf("vu_bias %"PRIu64"\n", vu_bias);
             printf("jump_voltage %"PRIu64"\n", jump_voltage);

             // get user defined power
             common_power_region = &_register->logical_channel[logical_channel_id].step_param.info.common.power_region;
             common_power = combine_4paired_register_region(common_power_region);

             printf("common_power %"PRIu64"\n", common_power);

             temp1 = logic_voltage*UNIT_FACTOR_BETWEEN_10NV_AND_10PV;
             temp2 = common_power;
             temp3 = (temp2/temp1);

             logic_current = temp3 * UNIT_FACTOR_BETWEEN_A_AND_01UA;
             logic_voltage = jump_voltage + vu_bias;

             //printf("temp1: %.15f, temp2:%.15f, temp3: %.15f\n", temp1, temp2, temp3);

             printf("logic_voltage %"PRIu64"  logic_current %"PRIu64"\n", logic_voltage, logic_current);

             ret = update_bound_physical_channel_current_source_and_vi_registers(manager, logical_channel_id, logic_current);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_CURRENT_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_CURRENT_EXCEEDED;
                return -1;
             }

             ret = update_bound_physical_channel_vu_registers(manager, logical_channel_id, logic_voltage);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_VOLTAGE_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_VOLTAGE_EXCEEDED;
                return -1;
             }

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_DISABLE);

             update_bound_physical_channel_info_io_registers(_manager, logical_channel_id, *step_type_reg);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CPD:

             // get present logic channel voltage
             logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;
             logic_voltage = combine_4paired_register_region(logic_voltage_region);
    
             printf("logic_voltage %"PRIu64"\n", logic_voltage);

             vu_bias_region = &_register->physical_channel[logical_channel_id].vu_bias_region;
             vu_bias = combine_4paired_register_region(vu_bias_region);

             jump_voltage_region = &_register->logical_channel[logical_channel_id].step_param.jump.voltage_region;
             jump_voltage = combine_4paired_register_region(jump_voltage_region);
             
             printf("vu_bias %"PRIu64"\n", vu_bias);
             printf("jump_voltage %"PRIu64"\n", jump_voltage);

             // get user defined power
             common_power_region = &_register->logical_channel[logical_channel_id].step_param.info.common.power_region;
             common_power = combine_4paired_register_region(common_power_region);

             printf("common_power %"PRIu64"\n", common_power);

             temp1 = logic_voltage*UNIT_FACTOR_BETWEEN_10NV_AND_10PV;
             temp2 = common_power;
             temp3 = (temp2/temp1);

             logic_current = (uint64_t)(temp3 * UNIT_FACTOR_BETWEEN_A_AND_01UA) | 0x8000000000000000;
             logic_voltage = jump_voltage - vu_bias;

             //printf("temp1: %.15f, temp2:%.15f, temp3: %.15f\n", temp1, temp2, temp3);

             printf("logic_voltage %"PRIu64"  logic_current %"PRIu64"\n", logic_voltage, logic_current);

             ret = update_bound_physical_channel_current_source_and_vi_registers(manager, logical_channel_id, logic_current);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_CURRENT_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_CURRENT_EXCEEDED;
                return -1;
             }

             ret = update_bound_physical_channel_vu_registers(manager, logical_channel_id, logic_voltage);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_VOLTAGE_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_VOLTAGE_EXCEEDED;
                return -1;
             }

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_DISABLE);

             update_bound_physical_channel_info_io_registers(_manager, logical_channel_id, *step_type_reg);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CRD:

             // get present logic channel voltage
             logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;
             logic_voltage = combine_4paired_register_region(logic_voltage_region);

             vu_bias_region = &_register->physical_channel[logical_channel_id].vu_bias_region;
             vu_bias = combine_4paired_register_region(vu_bias_region);

             jump_voltage_region = &_register->logical_channel[logical_channel_id].step_param.jump.voltage_region;
             jump_voltage = combine_4paired_register_region(jump_voltage_region);

             common_resistance_region = &_register->logical_channel[logical_channel_id].step_param.info.common.resistance_region;
             common_resistance = combine_4paired_register_region(common_resistance_region);

             temp1 = logic_voltage;
             temp2 = common_resistance;
             temp3 = (temp1) / (temp2);

             logic_current = (uint64_t)(temp3 * UNIT_FACTOR_BETWEEN_A_AND_01UA) | 0x8000000000000000;
             logic_voltage = jump_voltage - vu_bias;

             ret = update_bound_physical_channel_current_source_and_vi_registers(manager, logical_channel_id, logic_current);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_CURRENT_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_CURRENT_EXCEEDED;
                return -1;
             }

             ret = update_bound_physical_channel_vu_registers(manager, logical_channel_id, logic_voltage);
             if(ret < 0){
                printf("init_physical_channel PHYSICAL_VOLTAGE_EXCEEDED\n");
                *exp_message_reg = ERROR_MSG_PHYSICAL_VOLTAGE_EXCEEDED;
                return -1;
             }

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_DISABLE);

             update_bound_physical_channel_info_io_registers(_manager, logical_channel_id, *step_type_reg);

             return 0;
        break;

        case CALIBRATION_STEP_TYPE_CCC:

             calibration_voltage_region = &_register->logical_channel[logical_channel_id].calibration_param.calibration_voltage_region;
             calibration_voltage = combine_4paired_register_region(calibration_voltage_region);

             calibration_current_region = &_register->logical_channel[logical_channel_id].calibration_param.calibration_current_region;
             calibration_current = combine_4paired_register_region(calibration_current_region);

             ret = update_bound_physical_channel_vi_registers_by_calibration_current(manager, logical_channel_id, calibration_current);
             if(ret < 0) {
                 printf("update_bound_physical_channel_vi_registers_by_calibration_current fail\n");
                 return -1;
             }

             ret = update_bound_physical_channel_vu_registers_by_calibration_voltage(manager, logical_channel_id, calibration_voltage);
             if(ret < 0) {
                 printf("update_bound_physical_channel_vu_registers_by_calibration_voltage fail\n");
                 return ret;
             }

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_DISABLE);

             update_bound_physical_channel_info_io_registers(_manager, logical_channel_id, *step_type_reg);

             return 0;
        break;

        case CALIBRATION_STEP_TYPE_CCD:

             calibration_voltage_region = &_register->logical_channel[logical_channel_id].calibration_param.calibration_voltage_region;
             calibration_voltage = combine_4paired_register_region(calibration_voltage_region);

             calibration_current_region = &_register->logical_channel[logical_channel_id].calibration_param.calibration_current_region;
             calibration_current = combine_4paired_register_region(calibration_current_region);

             calibration_current = calibration_current | 0x8000000000000000;

             ret = update_bound_physical_channel_vi_registers_by_calibration_current(manager, logical_channel_id, calibration_current);
             if(ret < 0) {
                 printf("update_bound_physical_channel_vi_registers_by_calibration_current fail\n");
                 return -1;
             }

             ret = update_bound_physical_channel_vu_registers_by_calibration_voltage(manager, logical_channel_id, calibration_voltage);
             if(ret < 0) {
                 printf("update_bound_physical_channel_vu_registers_by_calibration_voltage fail\n");
                 return ret;
             }

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_DISABLE);

             update_bound_physical_channel_info_io_registers(_manager, logical_channel_id, *step_type_reg);

             return 0;
        break;

        case CALIBRATION_STEP_TYPE_CVAD:
    
             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_ENABLE);

             return 0;
        break;

        case CALIBRATION_STEP_TYPE_CVDA:

             calibration_voltage_region = &_register->logical_channel[logical_channel_id].calibration_param.calibration_voltage_region;
             calibration_voltage = combine_4paired_register_region(calibration_voltage_region);

             calibration_current_region = &_register->logical_channel[logical_channel_id].calibration_param.calibration_current_region;
             calibration_current = combine_4paired_register_region(calibration_current_region);

             ret = update_bound_physical_channel_vi_registers_by_calibration_current(manager, logical_channel_id, calibration_current);
             if(ret < 0) {
                 printf("update_bound_physical_channel_vi_registers_by_calibration_current fail\n");
                 return -1;
             }

             ret = update_bound_physical_channel_vu_registers_by_calibration_voltage(manager, logical_channel_id, calibration_voltage);
             if(ret < 0) {
                 printf("update_bound_physical_channel_vu_registers_by_calibration_voltage fail\n");
                 return ret;
             }

             update_bound_physical_channel_jc_registers(manager, logical_channel_id, JC_DISABLE);

             update_bound_physical_channel_info_io_registers(_manager, logical_channel_id, *step_type_reg);

             return 0;
        break;

        default:
             return ERROR_UNKNOWN_STEP_TYPE;
        break;
    }

    return 0;
}

void update_bv_and_cv(Manager *_manager)
{
    int cmd, ret, nb_samples, i, channel_index;
    double voltage, voltage_amplitude;
    Manager *manager = _manager;
    Register *_register = NULL;
    adc_controller_context *_adc_controller_ctx = NULL;
    adc_controller_ioctl_parameters *_adc_controller_ioctl_param = NULL;
    _4paired_register_region *bv_region, *cv_region;
    _4paired_register_region *last_unit_converted_bv_region, *last_unit_converted_cv_region, *unit_converted_bv_region, *unit_converted_cv_region;
    uint64_t bv, cv, bv_amplitude, cv_amplitude, unit_converted_bv, unit_converted_bv_amplitude, unit_converted_cv, unit_converted_cv_amplitude, sign;
    int flag_bc; // b = 1; c = 2; = 0, do nothing 

    _adc_controller_ctx = &manager->resources.adc_controller_ctx;
    _adc_controller_ioctl_param = &manager->resources.adc_controller_ioctl_param_U1;

    _register = &manager->modbus_ctx._register;

    cmd = ADC_CONTROLLER_IOCTL_CMD_STM32_CONVERSION;
    ret = _adc_controller_ctx->ops.ioctl(_adc_controller_ctx, cmd, _adc_controller_ioctl_param);
    if(ret < 0) {
        printf("adc_controller_ioctl fail\n");
        return ;
    }

    nb_samples = _adc_controller_ioctl_param->stm32_conversion_param.out.nb_samples;

    for(i=0; i<nb_samples; i++) {

        flag_bc = 0;

        channel_index = _adc_controller_ioctl_param->stm32_conversion_param.out.samples[i].index;
        voltage = _adc_controller_ioctl_param->stm32_conversion_param.out.samples[i].voltage;

        if(voltage >= 0 ){
            sign = 0x0000000000000000;
            voltage_amplitude = voltage;
        }
        else{
            sign = 0x8000000000000000;
            voltage_amplitude = voltage * (-1.0);
        }

        if(channel_index == BV1_CHANNEL_INDEX) {
            flag_bc = 1;
            bv_region = &_register->physical_channel[0].bv_region;
            last_unit_converted_bv_region = &_register->physical_channel[0].temp.last_unit_converted_bv_region;
            unit_converted_bv_region = &_register->physical_channel[0].temp.unit_converted_bv_region;
        } else if(channel_index == CV1_CHANNEL_INDEX) {
            flag_bc = 2;
            cv_region = &_register->physical_channel[0].cv_region;
            last_unit_converted_cv_region = &_register->physical_channel[0].temp.last_unit_converted_cv_region;
            unit_converted_cv_region = &_register->physical_channel[0].temp.unit_converted_cv_region;
        } else if(channel_index == BV2_CHANNEL_INDEX) {
            flag_bc = 1;
            bv_region = &_register->physical_channel[1].bv_region;
            last_unit_converted_bv_region = &_register->physical_channel[1].temp.last_unit_converted_bv_region;
            unit_converted_bv_region = &_register->physical_channel[1].temp.unit_converted_bv_region;
        } else if(channel_index == CV2_CHANNEL_INDEX) {
            flag_bc = 2;
            cv_region = &_register->physical_channel[1].cv_region;
            last_unit_converted_cv_region = &_register->physical_channel[1].temp.last_unit_converted_cv_region;
            unit_converted_cv_region = &_register->physical_channel[1].temp.unit_converted_cv_region;
        } else if(channel_index == BV3_CHANNEL_INDEX) {
            flag_bc = 1;
            bv_region = &_register->physical_channel[2].bv_region;
            last_unit_converted_bv_region = &_register->physical_channel[2].temp.last_unit_converted_bv_region;
            unit_converted_bv_region = &_register->physical_channel[2].temp.unit_converted_bv_region;
        } else if(channel_index == CV3_CHANNEL_INDEX) {
            flag_bc = 2;
            cv_region = &_register->physical_channel[2].cv_region;
            last_unit_converted_cv_region = &_register->physical_channel[2].temp.last_unit_converted_cv_region;
            unit_converted_cv_region = &_register->physical_channel[2].temp.unit_converted_cv_region;
        } else if(channel_index == BV4_CHANNEL_INDEX) {
            flag_bc = 1;
            bv_region = &_register->physical_channel[3].bv_region;
            last_unit_converted_bv_region = &_register->physical_channel[3].temp.last_unit_converted_bv_region;
            unit_converted_bv_region = &_register->physical_channel[3].temp.unit_converted_bv_region;
        } else if(channel_index == CV4_CHANNEL_INDEX) {
            flag_bc = 2;
            cv_region = &_register->physical_channel[3].cv_region;
            last_unit_converted_cv_region = &_register->physical_channel[3].temp.last_unit_converted_cv_region;
            unit_converted_cv_region = &_register->physical_channel[3].temp.unit_converted_cv_region;
        } else if(channel_index == BV5_CHANNEL_INDEX) {
            flag_bc = 1;
            bv_region = &_register->physical_channel[4].bv_region;
            last_unit_converted_bv_region = &_register->physical_channel[4].temp.last_unit_converted_bv_region;
            unit_converted_bv_region = &_register->physical_channel[4].temp.unit_converted_bv_region;
        } else if(channel_index == CV5_CHANNEL_INDEX) {
            flag_bc = 2;
            cv_region = &_register->physical_channel[4].cv_region;
            last_unit_converted_cv_region = &_register->physical_channel[4].temp.last_unit_converted_cv_region;
            unit_converted_cv_region = &_register->physical_channel[4].temp.unit_converted_cv_region;
        } else if(channel_index == BV6_CHANNEL_INDEX) {
            flag_bc = 1;
            bv_region = &_register->physical_channel[5].bv_region;
            last_unit_converted_bv_region = &_register->physical_channel[5].temp.last_unit_converted_bv_region;
            unit_converted_bv_region = &_register->physical_channel[5].temp.unit_converted_bv_region;
        } else if(channel_index == CV6_CHANNEL_INDEX) {
            flag_bc = 2;
            cv_region = &_register->physical_channel[5].cv_region;
            last_unit_converted_cv_region = &_register->physical_channel[5].temp.last_unit_converted_cv_region;
            unit_converted_cv_region = &_register->physical_channel[5].temp.unit_converted_cv_region;
        } else if(channel_index == BV7_CHANNEL_INDEX) {
            flag_bc = 1;
            bv_region = &_register->physical_channel[6].bv_region;
            last_unit_converted_bv_region = &_register->physical_channel[6].temp.last_unit_converted_bv_region;
            unit_converted_bv_region = &_register->physical_channel[6].temp.unit_converted_bv_region;
        } else if(channel_index == CV7_CHANNEL_INDEX) {
            flag_bc = 2;
            cv_region = &_register->physical_channel[6].cv_region;
            last_unit_converted_cv_region = &_register->physical_channel[6].temp.last_unit_converted_cv_region;
            unit_converted_cv_region = &_register->physical_channel[6].temp.unit_converted_cv_region;
        } else if(channel_index == BV8_CHANNEL_INDEX) {
            flag_bc = 1;
            bv_region = &_register->physical_channel[7].bv_region;
            last_unit_converted_bv_region = &_register->physical_channel[7].temp.last_unit_converted_bv_region;
            unit_converted_bv_region = &_register->physical_channel[7].temp.unit_converted_bv_region;
        } else if(channel_index == CV8_CHANNEL_INDEX) {
            flag_bc = 2;
            cv_region = &_register->physical_channel[7].cv_region;
            last_unit_converted_cv_region = &_register->physical_channel[7].temp.last_unit_converted_cv_region;
            unit_converted_cv_region = &_register->physical_channel[7].temp.unit_converted_cv_region;
        }

        if(flag_bc == 1) { /* bv */

            bv_amplitude = voltage_amplitude * 1000000000000000;
            bv = bv_amplitude | sign;
            unit_converted_bv_amplitude = voltage_amplitude * UNIT_FACTOR_BETWEEN_V_AND_10NV;
            unit_converted_bv = unit_converted_bv_amplitude | sign;
            update_4paired_register_region(bv_region, bv);
            update_4paired_register_region(unit_converted_bv_region, unit_converted_bv);
            update_4paired_register_region(last_unit_converted_bv_region, unit_converted_bv);
            
#if 0
            if(channel_index == BV1_CHANNEL_INDEX) {
                printf("voltage %f.15\n", voltage);
                printf("bv %llu, calc_bv %llu\n", bv, unit_converted_bv);
            }
#endif

        } else if(flag_bc == 2) { /* cv */
            cv_amplitude = voltage_amplitude * 1000000000000000;
            cv = cv_amplitude | sign;
            unit_converted_cv_amplitude = voltage_amplitude * UNIT_FACTOR_BETWEEN_V_AND_10NV;
            unit_converted_cv = unit_converted_cv_amplitude | sign;
            update_4paired_register_region(cv_region, cv);
            update_4paired_register_region(unit_converted_cv_region, unit_converted_cv);
            update_4paired_register_region(last_unit_converted_cv_region, unit_converted_cv);
        } else {
            printf("unknown flag_bc\n");
            exit(-1);
        }

        //printf("channel_index %x voltage %f\n", channel_index, voltage);
    }
}

int update_sample_voltage_and_current(Manager *_manager)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    int physical_channel_id;
    uint16_t *current_source_level_reg;
    uint64_t current_capacity;
    _4paired_register_region *unit_converted_bv_region, *unit_converted_cv_region, *sample_voltage_region, *sample_current_region;    
    _4paired_register_region *level_1_region, *level_2_region, *level_3_region;
    uint64_t unit_converted_bv, unit_converted_cv, sample_current, sample_voltage, unit_converted_cv_amplitude;
    uint64_t base_voltage, sign, temp;
    double diff, temp1, temp2;
    double unit_scale; /*  0.1uA/0.1mV  */
    int cv_polarity;

    _register = &manager->modbus_ctx._register;

    for(physical_channel_id=PHYSICAL_CHANNEL_1_ID; physical_channel_id<NB_PHYSICAL_CHANNEL; physical_channel_id++) {
        unit_converted_bv_region = &_register->physical_channel[physical_channel_id].temp.unit_converted_bv_region;
        unit_converted_cv_region = &_register->physical_channel[physical_channel_id].temp.unit_converted_cv_region;

        sample_voltage_region = &_register->physical_channel[physical_channel_id].sample.voltage_region;
        sample_current_region = &_register->physical_channel[physical_channel_id].sample.current_region;

        current_source_level_reg = _register->physical_channel[physical_channel_id].current_source_level;

        level_1_region = &_register->physical_channel[physical_channel_id].source_current_level_1_region;
        level_2_region = &_register->physical_channel[physical_channel_id].source_current_level_2_region;
        level_3_region = &_register->physical_channel[physical_channel_id].source_current_level_3_region;

        switch (*current_source_level_reg){
            case PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1:
                current_capacity = combine_4paired_register_region(level_1_region);
            break;
            
            case PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2:
                current_capacity = combine_4paired_register_region(level_2_region);
            break;
            
            case PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3:
                current_capacity = combine_4paired_register_region(level_3_region);
            break;
            
            default:
                printf("update_sample_voltage_and_current: loss source_current_level\n");
                return ERROR_INTERNEL_ERRORS;
            break;
        }

        unit_converted_cv = combine_4paired_register_region(unit_converted_cv_region);
        temp1 = current_capacity;
        temp2 = PHYSICAL_CHANNEL_CV_HALFSCALE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
        unit_scale = temp1/temp2; 
        base_voltage = PHYSICAL_CHANNEL_BASE_VOLTAGE_MV * UNIT_FACTOR_BETWEEN_MV_AND_10NV;

        if(unit_converted_cv & 0x8000000000000000){
            cv_polarity = 0;
            unit_converted_cv_amplitude = unit_converted_cv & 0x7fffffffffffffff;
        } else {
            cv_polarity = 1;
            unit_converted_cv_amplitude = unit_converted_cv;
        }
        //printf("temp1 %.15f, temp2 %.15f\n", temp1, temp2);
        //printf("calc_cv %"PRIu64", calc_bv %"PRIu64", base_voltage %"PRIu64", unit_scale %.15f\n", unit_converted_cv, unit_converted_bv, base_voltage, unit_scale);

        if(cv_polarity){
            if(unit_converted_cv_amplitude > base_voltage){
                
                //*sample_current_reg = (uint16_t)(((double)*cv_reg - PHYSICAL_CHANNEL_BASE_VOLTAGE)/(PHYSICAL_CHANNEL_CV_FULLSCALE/2)*current_capacity) | sign;

                sign=0x0000000000000000;
                diff = (unit_converted_cv_amplitude - base_voltage);
                temp = diff * unit_scale;
                sample_current = temp | sign;

                //printf("diff 2 %.15f, sample_current %"PRIu64"\n", diff, sample_current);

                update_4paired_register_region(sample_current_region, sample_current);

            } else {
                
                //*sample_current_reg = (uint16_t)(((double)PHYSICAL_CHANNEL_BASE_VOLTAGE - *cv_reg)/(PHYSICAL_CHANNEL_CV_FULLSCALE/2)*current_capacity) | sign;

                sign = 0x8000000000000000;
                diff = (base_voltage - unit_converted_cv_amplitude);
                temp = diff * unit_scale;
                sample_current = temp | sign;

                //printf("diff %.15f, sample_current %"PRIu64"\n", diff, sample_current);

                update_4paired_register_region(sample_current_region, sample_current);
            }
        } else {
            //*sample_current_reg = (uint16_t)(((double)PHYSICAL_CHANNEL_BASE_VOLTAGE - *cv_reg)/(PHYSICAL_CHANNEL_CV_FULLSCALE/2)*current_capacity) | sign;
            
            sign = 0x8000000000000000;
            diff = (base_voltage + unit_converted_cv_amplitude);
            temp = diff * unit_scale;
            sample_current = temp | sign;
            
            //printf("diff %.15f, sample_current %"PRIu64"\n", diff, sample_current);
            
            update_4paired_register_region(sample_current_region, sample_current);
        }

        // update sample voltage
        unit_converted_bv = combine_4paired_register_region(unit_converted_bv_region);
        update_4paired_register_region(sample_voltage_region, unit_converted_bv);
    }

    return 0;
}


void reset_physical_channel_to_prepare_state(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    Data_Model *data_model = NULL;
    int physical_channel_id = channel_id;
    uint64_t sample_current, sample_voltage;
    uint64_t level_1, level_2, level_3, vi, vu;
    _4paired_register_region *vi_region, *vu_region;
    _4paired_register_region *sample_current_region, *sample_voltage_region;
    _4paired_register_region *level_1_region, *level_2_region, *level_3_region;

    _register = &manager->modbus_ctx._register;

    *_register->physical_channel[physical_channel_id].ready = IDEL_READY_SETTING;
    *_register->physical_channel[physical_channel_id].status = PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;
    
    sample_current = 0;
    sample_current_region = &_register->physical_channel[physical_channel_id].sample.current_region;
    update_4paired_register_region(sample_current_region, sample_current);

    sample_voltage = 0;
    sample_voltage_region = &_register->physical_channel[physical_channel_id].sample.voltage_region;
    update_4paired_register_region(sample_voltage_region, sample_voltage);

    level_1 = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL1_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
    level_1_region = &_register->physical_channel[physical_channel_id].source_current_level_1_region;
    update_4paired_register_region(level_1_region, level_1);

    level_2 = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL2_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
    level_2_region = &_register->physical_channel[physical_channel_id].source_current_level_2_region;
    update_4paired_register_region(level_2_region, level_2);

    level_3 = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL3_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
    level_3_region = &_register->physical_channel[physical_channel_id].source_current_level_3_region;
    update_4paired_register_region(level_3_region, level_3);

    vi = IDEL_VI_SETTING * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
    vi_region = &_register->physical_channel[physical_channel_id].vi_region;
    update_4paired_register_region(vi_region, vi);

    vu = IDEL_VU_SETTING * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
    vu_region = &_register->physical_channel[physical_channel_id].vu_region;
    update_4paired_register_region(vu_region, vu);

    *_register->physical_channel[physical_channel_id].kz_h = IDEL_KZ_H_SETTING;
    *_register->physical_channel[physical_channel_id].kz_m = IDEL_KZ_M_SETTING;
    *_register->physical_channel[physical_channel_id].kz_l = IDEL_KZ_L_SETTING;
    *_register->physical_channel[physical_channel_id].jc = IDEL_JC_SETTING;
    *_register->physical_channel[physical_channel_id].cf = IDEL_CF_SETTING;
    *_register->physical_channel[physical_channel_id].ck = IDEL_CK_SETTING;
    *_register->physical_channel[physical_channel_id].cled = IDEL_CLED_SETTING;
    *_register->physical_channel[physical_channel_id].fled = IDEL_FLED_SETTING;

}

int check_if_bound_physical_channels_in_prepare_state(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint16_t *nb_binding, *binding, i;
    uint16_t *phy_ready = NULL, *bound_relation = NULL;
    int physical_channel_id, count, retries;
    uint16_t *kz_h, *kz_m, *kz_l, *jc, *cf, *ck, *cled, *fled;
    uint64_t vi, vu;

    _4paired_register_region *vi_region, *vu_region; 

    _register = &manager->modbus_ctx._register;

    *_register->logical_channel[logical_channel_id].exception_message = 0;

    binding = _register->logical_channel[logical_channel_id].binding;

    nb_binding = _register->logical_channel[logical_channel_id].nb_binding;

    count = 0;
    retries = 0;

    while((count != *nb_binding) && retries <= MAX_PREPARE_STATE_CHECK) {

        for(i=0; i<NB_LOGIC_CHANNEL; i++) {
            
            //sleep(1);

            if((((*binding )>>i) & 0x0001) > 0) {
                
                physical_channel_id = i;
        
                vi_region = &_register->physical_channel[physical_channel_id].vi_region;
                vi = combine_4paired_register_region(vi_region);

                vu_region = &_register->physical_channel[physical_channel_id].vu_region;
                vu = combine_4paired_register_region(vu_region);

                kz_h = _register->physical_channel[physical_channel_id].kz_h;
                kz_m = _register->physical_channel[physical_channel_id].kz_m;
                kz_l = _register->physical_channel[physical_channel_id].kz_l;
                jc = _register->physical_channel[physical_channel_id].jc;
                cf = _register->physical_channel[physical_channel_id].cf;
                ck = _register->physical_channel[physical_channel_id].ck;
                cled = _register->physical_channel[physical_channel_id].cled;
                fled = _register->physical_channel[physical_channel_id].fled;
                
                // check vi
                if(vi != (IDEL_VI_SETTING*UNIT_FACTOR_BETWEEN_MV_AND_10NV)) {
                    printf("VI\n");
                    continue;
                }
                
                // check vu
                if(vu != (IDEL_VU_SETTING*UNIT_FACTOR_BETWEEN_MV_AND_10NV)) {
                    printf("VU\n");
                    continue;
                }

                // check kz_h
                if(*kz_h != IDEL_KZ_H_SETTING) {
                    printf("KZ_H\n");
                    continue;
                }

                // check kz_m
                if(*kz_m != IDEL_KZ_M_SETTING) {
                    printf("KZ_M\n");
                    continue;
                }

                // check kz_l
                if(*kz_l != IDEL_KZ_L_SETTING) {
                    printf("KZ_L\n");
                    continue;
                }

                // check jc
                if(*jc != IDEL_JC_SETTING) {
                    printf("JC\n");
                    continue;
                }

                // check cf
                if(*cf != IDEL_CF_SETTING) {
                    printf("CF\n");
                    continue;
                }

                // check ck
                if(*ck != IDEL_CK_SETTING) {
                    printf("CK\n");
                    continue;
                }

                // check cled
                if(*cled != IDEL_CLED_SETTING) {
                    printf("cled\n");
                    continue;
                }

                // check fled
                if(*fled != IDEL_FLED_SETTING) {
                    printf("fled\n");
                    continue;
                }

                count++;
                
            }
        }    

        retries++;
    }

    return 0;
}

int reset_all_bound_physical_channels_to_prepare_state(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint16_t *nb_binding, *binding, i;
    uint16_t *phy_ready = NULL, *bound_relation = NULL;

    _register = &manager->modbus_ctx._register;

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;
    binding = _register->logical_channel[logical_channel_id].binding;
    nb_binding = _register->logical_channel[logical_channel_id].nb_binding;

    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        
        if((((*binding )>>i) & 0x0001) > 0) {
     
            //printf("reset_all_bound_physical_channels_to_prepare_state %d\n", i);

            phy_ready = _register->physical_channel[i].ready;
            bound_relation = _register->physical_channel[i].bound_relation;

            if(*bound_relation == logical_channel_id) {
                *phy_ready = 0;
                reset_physical_channel_to_prepare_state(manager, i);
            } else {
                printf("reset_all_bound_physical_channels_to_defualt_state fail\n");
                return ERROR_INTERNEL_ERRORS;
            }
        }
    }

    return 0;
}

void reset_all_physical_channel_to_default_state(Manager *_manager)
{
    Manager *manager = _manager;
    int physical_channel_id, ret;
    
    for(physical_channel_id=PHYSICAL_CHANNEL_1_ID; physical_channel_id < NB_PHYSICAL_CHANNEL; physical_channel_id++) {
        reset_physical_channel_to_default_state(manager, physical_channel_id);
    }
}

void reset_physical_channel_to_default_state(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    Data_Model *data_model = NULL;
    int physical_channel_id = channel_id, ret;

    uint64_t temp ;
    _4paired_register_region *temp_region;

    _register = &manager->modbus_ctx._register;

    *_register->physical_channel[physical_channel_id].ready = DEFAULT_READY_SETTING;
    *_register->physical_channel[physical_channel_id].status = PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_IDEL;
    
    temp_region = &_register->physical_channel[physical_channel_id].sample.current_region;
    update_4paired_register_region(temp_region, 0);

    temp_region = &_register->physical_channel[physical_channel_id].sample.voltage_region;
    update_4paired_register_region(temp_region, 0);
    
    *_register->physical_channel[physical_channel_id].current_source_level = DEFAULT_CURRENT_SOURCE_LEVEL_SETTING;

    temp = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL1_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
    temp_region = &_register->physical_channel[physical_channel_id].source_current_level_1_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL2_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
    temp_region = &_register->physical_channel[physical_channel_id].source_current_level_2_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL3_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
    temp_region = &_register->physical_channel[physical_channel_id].source_current_level_3_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_VI_SETTING_IN_MV_UNIT * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
    temp_region = &_register->physical_channel[physical_channel_id].vi_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_VU_SETTING_IN_MV_UNIT * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
    temp_region = &_register->physical_channel[physical_channel_id].vu_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_VU_IN_MV_UNIT * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
    temp_region = &_register->physical_channel[physical_channel_id].kb.vu_b_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_BV_IN_MV_UNIT * UNIT_FACTOR_BETWEEN_MV_AND_10NV;
    temp_region = &_register->physical_channel[physical_channel_id].kb.bv_b_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_C_VI_B_FOR_L1_IN_01UA_UNIT;
    temp_region = &_register->physical_channel[physical_channel_id].kb.c_vi_b_level1_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_C_VI_B_FOR_L2_IN_01UA_UNIT;
    temp_region = &_register->physical_channel[physical_channel_id].kb.c_vi_b_level2_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_C_VI_B_FOR_L3_IN_01UA_UNIT;
    temp_region = &_register->physical_channel[physical_channel_id].kb.c_vi_b_level3_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_D_VI_B_FOR_L1_IN_01UA_UNIT;
    temp_region = &_register->physical_channel[physical_channel_id].kb.d_vi_b_level1_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_D_VI_B_FOR_L2_IN_01UA_UNIT;
    temp_region = &_register->physical_channel[physical_channel_id].kb.d_vi_b_level2_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_D_VI_B_FOR_L3_IN_01UA_UNIT;
    temp_region = &_register->physical_channel[physical_channel_id].kb.d_vi_b_level3_region;
    update_4paired_register_region(temp_region, temp);


    temp = DEFAULT_PHYSICAL_CHANNEL_C_CV_B_FOR_L1_IN_01UA_UNIT;
    temp_region = &_register->physical_channel[physical_channel_id].kb.c_cv_b_level1_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_C_CV_B_FOR_L2_IN_01UA_UNIT;
    temp_region = &_register->physical_channel[physical_channel_id].kb.c_cv_b_level2_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_C_CV_B_FOR_L3_IN_01UA_UNIT;
    temp_region = &_register->physical_channel[physical_channel_id].kb.c_cv_b_level3_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_D_CV_B_FOR_L1_IN_01UA_UNIT;
    temp_region = &_register->physical_channel[physical_channel_id].kb.d_cv_b_level1_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_D_CV_B_FOR_L2_IN_01UA_UNIT;
    temp_region = &_register->physical_channel[physical_channel_id].kb.d_cv_b_level2_region;
    update_4paired_register_region(temp_region, temp);

    temp = DEFAULT_PHYSICAL_CHANNEL_D_CV_B_FOR_L3_IN_01UA_UNIT;
    temp_region = &_register->physical_channel[physical_channel_id].kb.d_cv_b_level3_region;
    update_4paired_register_region(temp_region, temp);


    *_register->physical_channel[physical_channel_id].kb.bv_k = DEFAULT_PHYSICAL_CHANNEL_BV_K_IN_00001_UNIT;
    *_register->physical_channel[physical_channel_id].kb.vu_k = DEFAULT_PHYSICAL_CHANNEL_VU_K_IN_00001_UNIT;


    *_register->physical_channel[physical_channel_id].kb.c_vi_k_level_1 = DEFAULT_PHYSICAL_CHANNEL_C_VI_K_FOR_L1_IN_00001_UNIT;
    *_register->physical_channel[physical_channel_id].kb.c_vi_k_level_2 = DEFAULT_PHYSICAL_CHANNEL_C_VI_K_FOR_L2_IN_00001_UNIT;
    *_register->physical_channel[physical_channel_id].kb.c_vi_k_level_3 = DEFAULT_PHYSICAL_CHANNEL_C_VI_K_FOR_L3_IN_00001_UNIT;
    *_register->physical_channel[physical_channel_id].kb.d_vi_k_level_1 = DEFAULT_PHYSICAL_CHANNEL_D_VI_K_FOR_L1_IN_00001_UNIT;
    *_register->physical_channel[physical_channel_id].kb.d_vi_k_level_2 = DEFAULT_PHYSICAL_CHANNEL_D_VI_K_FOR_L2_IN_00001_UNIT;
    *_register->physical_channel[physical_channel_id].kb.d_vi_k_level_3 = DEFAULT_PHYSICAL_CHANNEL_D_VI_K_FOR_L3_IN_00001_UNIT;


    *_register->physical_channel[physical_channel_id].kb.c_cv_k_level_1 = DEFAULT_PHYSICAL_CHANNEL_C_CV_K_FOR_L1_IN_00001_UNIT;
    *_register->physical_channel[physical_channel_id].kb.c_cv_k_level_2 = DEFAULT_PHYSICAL_CHANNEL_C_CV_K_FOR_L2_IN_00001_UNIT;
    *_register->physical_channel[physical_channel_id].kb.c_cv_k_level_3 = DEFAULT_PHYSICAL_CHANNEL_C_CV_K_FOR_L3_IN_00001_UNIT;

    *_register->physical_channel[physical_channel_id].kb.d_cv_k_level_1 = DEFAULT_PHYSICAL_CHANNEL_D_CV_K_FOR_L1_IN_00001_UNIT;
    *_register->physical_channel[physical_channel_id].kb.d_cv_k_level_2 = DEFAULT_PHYSICAL_CHANNEL_D_CV_K_FOR_L2_IN_00001_UNIT;
    *_register->physical_channel[physical_channel_id].kb.d_cv_k_level_3 = DEFAULT_PHYSICAL_CHANNEL_D_CV_K_FOR_L3_IN_00001_UNIT;

    *_register->physical_channel[physical_channel_id].kz_h = DEFAULT_KZ_H_SETTING;
    *_register->physical_channel[physical_channel_id].kz_m = DEFAULT_KZ_M_SETTING;
    *_register->physical_channel[physical_channel_id].kz_l = DEFAULT_KZ_L_SETTING;
    *_register->physical_channel[physical_channel_id].jc = DEFAULT_JC_SETTING;
    *_register->physical_channel[physical_channel_id].cf = DEFAULT_CF_SETTING;
    *_register->physical_channel[physical_channel_id].ck = DEFAULT_CK_SETTING;
    *_register->physical_channel[physical_channel_id].cled = DEFAULT_CLED_SETTING;
    *_register->physical_channel[physical_channel_id].fled = DEFAULT_FLED_SETTING;
    
    //temp = DEFAULT_LAST_CURRENT_SETTING;
    //temp_region = &_register->physical_channel[physical_channel_id].temp.last_current_region;
    //update_4paired_register_region(temp_region, temp);

    //temp = DEFAULT_LAST_VOLTAGE_SETTING;
    //temp_region = &_register->physical_channel[physical_channel_id].temp.last_voltage_region;
    //update_4paired_register_region(temp_region, temp);

    //temp = DEFAULT_LAST_VI_SETTING;
    //temp_region = &_register->physical_channel[physical_channel_id].temp.last_vi_region;
    //update_4paired_register_region(temp_region, temp);

    //temp = DEFAULT_LAST_VU_SETTING;
    //temp_region = &_register->physical_channel[physical_channel_id].temp.last_vu_region;
    //update_4paired_register_region(temp_region, temp);
    
    //*_register->physical_channel[physical_channel_id].temp.last_kz_h = DEFAULT_LAST_KZ_H_SETTING;
    //*_register->physical_channel[physical_channel_id].temp.last_kz_m = DEFAULT_LAST_KZ_M_SETTING;
    //*_register->physical_channel[physical_channel_id].temp.last_kz_l = DEFAULT_LAST_KZ_L_SETTING;
    //*_register->physical_channel[physical_channel_id].temp.last_jc = DEFAULT_LAST_JC_SETTING;
    //*_register->physical_channel[physical_channel_id].temp.last_cf = DEFAULT_LAST_CF_SETTING;
    //*_register->physical_channel[physical_channel_id].temp.last_ck = DEFAULT_LAST_CK_SETTING;
    //*_register->physical_channel[physical_channel_id].temp.last_cled = DEFAULT_LAST_CLED_SETTING;
    //*_register->physical_channel[physical_channel_id].temp.last_fled = DEFAULT_LAST_FLED_SETTING;
    
    //*_register->physical_channel[physical_channel_id].temp.last_current_source_level = DEFAULT_LAST_CURRENT_SOURCE_LEVEL_SETTING;

    ret = reload_one_physical_channel_KB_value(manager, physical_channel_id);
    if(ret != 0){
        printf("reset_physical_channel_to_default_state->reload_one_physical_channel_KB_value %d physical_channel_id reload fail\n", physical_channel_id);
        exit(-1);
    }


}

int reset_one_physical_channel(Manager *manager, int channel_id)
{
    Data_Model *data_model = NULL;
    int ret = 0, logical_channel_id = channel_id, mask, i;
    Register *_register = NULL;

    if(!manager || logical_channel_id < LOGICAL_CHANNEL_1_ID || logical_channel_id >= NB_LOGIC_CHANNEL) {
        printf("do_formation_logic fail, incorrect parameters, channel %d\n", logical_channel_id);
        return -1;
    }

    if(logical_channel_id == LOGICAL_CHANNEL_1_ID) {
        mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH1_MASK_BIT;
    } else if(logical_channel_id == LOGICAL_CHANNEL_2_ID) {
        mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH2_MASK_BIT;
    } else if(logical_channel_id == LOGICAL_CHANNEL_3_ID) {
        mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH3_MASK_BIT;
    } else if(logical_channel_id == LOGICAL_CHANNEL_4_ID) {
        mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH4_MASK_BIT;
    } else if(logical_channel_id == LOGICAL_CHANNEL_5_ID) {
        mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH5_MASK_BIT;
    } else if(logical_channel_id == LOGICAL_CHANNEL_6_ID) {
        mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH6_MASK_BIT;
    } else if(logical_channel_id == LOGICAL_CHANNEL_7_ID) {
        mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH7_MASK_BIT;
    } else if(logical_channel_id == LOGICAL_CHANNEL_8_ID) {
        mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH8_MASK_BIT;
    } else {
        return ERROR_UNKNOWN_LOGICAL_CHANNEL_ID;
    }

    if((*_register->logical_channel[logical_channel_id].binding) & mask) {
        reset_physical_channel_to_default_state(manager, logical_channel_id);
    }

    return 0;
}

int reset_bound_physical_channels(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Data_Model *data_model = NULL;
    int ret = 0, logical_channel_id = channel_id, mask, i;
    Register *_register = NULL;

    if(!manager || logical_channel_id < LOGICAL_CHANNEL_1_ID || logical_channel_id >= NB_LOGIC_CHANNEL) {
        printf("do_formation_logic fail, incorrect parameters, channel %d\n", logical_channel_id);
        return -1;
    }

    _register = &manager->modbus_ctx._register;

    //free all physical channels that we've bound for this logical channel
    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH1_MASK_BIT;
    if((*_register->logical_channel[logical_channel_id].binding) & mask) {
        reset_physical_channel_to_default_state(manager, PHYSICAL_CHANNEL_1_ID);
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH2_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        reset_physical_channel_to_default_state(manager, PHYSICAL_CHANNEL_2_ID);
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH3_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        reset_physical_channel_to_default_state(manager, PHYSICAL_CHANNEL_3_ID);
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH4_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        reset_physical_channel_to_default_state(manager, PHYSICAL_CHANNEL_4_ID);
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH5_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        reset_physical_channel_to_default_state(manager, PHYSICAL_CHANNEL_5_ID);
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH6_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        reset_physical_channel_to_default_state(manager, PHYSICAL_CHANNEL_6_ID);
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH7_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        reset_physical_channel_to_default_state(manager, PHYSICAL_CHANNEL_7_ID);
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH8_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        reset_physical_channel_to_default_state(manager, PHYSICAL_CHANNEL_8_ID);
    }

    return 0;
}

void init_physical_channel_register_mapping(Manager * manager, int _channel_id)
{
    int channel_id = _channel_id;
    Register *_register = NULL;
    Data_Model *data_model = NULL;
    _4paired_register_region *sample_current_region, *sample_voltage_region;
    _4paired_register_region *level_1_region, *level_2_region, *level_3_region;

    _register =  &manager->modbus_ctx._register;
    data_model = manager->modbus_ctx._register.data_model;

    // mode
    _register->physical_channel[channel_id].ready = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_READY_REGISTER_OFFSET];
    _register->physical_channel[channel_id].status = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_STATUS_REGISTER_OFFSET];

    _register->physical_channel[channel_id].cv_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CV_WORD1_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].cv_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CV_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].cv_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CV_WORD3_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].cv_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CV_WORD4_REGISTER_OFFSET];

    _register->physical_channel[channel_id].bv_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_BV_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].bv_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_BV_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].bv_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_BV_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].bv_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_BV_WORD4_REGISTER_OFFSET];
    
    _register->physical_channel[channel_id].vu_bias_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_BIAS_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].vu_bias_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_BIAS_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].vu_bias_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_BIAS_WORD3_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].vu_bias_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_BIAS_WORD4_REGISTER_OFFSET];

    _register->physical_channel[channel_id].vi_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VI_WORD1_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].vi_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VI_WORD2_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].vi_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VI_WORD3_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].vi_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VI_WORD4_REGISTER_OFFSET];

    _register->physical_channel[channel_id].vu_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].vu_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_WORD2_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].vu_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_WORD3_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].vu_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_WORD4_REGISTER_OFFSET];

    _register->physical_channel[channel_id].kz_h = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_KZ_H_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kz_m = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_KZ_M_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kz_l = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_KZ_L_REGISTER_OFFSET];
    _register->physical_channel[channel_id].jc = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_JC_REGISTER_OFFSET];
    _register->physical_channel[channel_id].cf = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CF_REGISTER_OFFSET];
    _register->physical_channel[channel_id].ck = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CK_REGISTER_OFFSET];
    _register->physical_channel[channel_id].cled = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CLED_REGISTER_OFFSET];
    _register->physical_channel[channel_id].fled = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_FLED_REGISTER_OFFSET];
    _register->physical_channel[channel_id].bell = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_BELL_REGISTER_OFFSET];
    _register->physical_channel[channel_id].rdry = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_RDRY_REGISTER_OFFSET];

    _register->physical_channel[channel_id].temp.last_vu_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_VU_WORD1_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].temp.last_vu_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_VU_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_vu_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_VU_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_vu_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_VU_WORD4_REGISTER_OFFSET];
    
    _register->physical_channel[channel_id].temp.last_vi_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_VI_WORD1_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].temp.last_vi_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_VI_WORD2_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].temp.last_vi_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_VI_WORD3_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].temp.last_vi_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_VI_WORD4_REGISTER_OFFSET];

    _register->physical_channel[channel_id].temp.last_kz_h = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_KZ_H_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_kz_m = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_KZ_M_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_kz_l = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_KZ_L_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_jc = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_JC_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_cf = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_CF_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_ck = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_CK_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_cled = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_CLED_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_fled = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_FLED_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_bell = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_BELL_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_rdry = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_RDRY_REGISTER_OFFSET];
    
    _register->physical_channel[channel_id].temp.last_current_source_level = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_CURRENT_SOURCE_LEVEL_REGISTER_OFFSET];

    
    _register->physical_channel[channel_id].current_source_level = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_REGISTER_OFFSET];
    
    _register->physical_channel[channel_id].bound_relation = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_BOUND_RELATION_REGISTER_OFFSET];
    
    _register->physical_channel[channel_id].kb.c_vi_k_level_1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_K_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_k_level_2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_K_FOR_L2_REGISTER_OFFSET]; 
    _register->physical_channel[channel_id].kb.c_vi_k_level_3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_K_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_k_level_1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_K_FOR_L1_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].kb.d_vi_k_level_2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_K_FOR_L2_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].kb.d_vi_k_level_3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_K_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_b_level1_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_B_WORD1_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_b_level1_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_B_WORD2_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_b_level1_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_B_WORD3_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_b_level1_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_B_WORD4_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_b_level2_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_B_WORD1_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_b_level2_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_B_WORD2_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_b_level2_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_B_WORD3_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_b_level2_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_B_WORD4_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_b_level3_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_B_WORD1_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_b_level3_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_B_WORD2_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_b_level3_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_B_WORD3_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_vi_b_level3_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_VI_B_WORD4_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_b_level1_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_B_WORD1_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_b_level1_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_B_WORD2_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_b_level1_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_B_WORD3_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_b_level1_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_B_WORD4_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_b_level2_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_B_WORD1_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_b_level2_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_B_WORD2_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_b_level2_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_B_WORD3_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_b_level2_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_B_WORD4_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_b_level3_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_B_WORD1_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_b_level3_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_B_WORD2_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_b_level3_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_B_WORD3_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_vi_b_level3_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_VI_B_WORD4_FOR_L3_REGISTER_OFFSET];

    _register->physical_channel[channel_id].kb.c_cv_k_level_1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_K_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_cv_k_level_2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_K_FOR_L2_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].kb.c_cv_k_level_3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_K_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_k_level_1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_K_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_k_level_2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_K_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_k_level_3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_K_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_cv_b_level1_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_B_WORD1_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_cv_b_level1_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_B_WORD2_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_cv_b_level1_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_B_WORD3_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_cv_b_level1_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_B_WORD4_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_cv_b_level2_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_B_WORD1_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_cv_b_level2_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_B_WORD2_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_cv_b_level2_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_B_WORD3_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_cv_b_level2_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_B_WORD4_FOR_L2_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].kb.c_cv_b_level3_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_B_WORD1_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_cv_b_level3_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_B_WORD2_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_cv_b_level3_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_B_WORD3_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.c_cv_b_level3_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_C_CV_B_WORD4_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_b_level1_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_B_WORD1_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_b_level1_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_B_WORD2_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_b_level1_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_B_WORD3_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_b_level1_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_B_WORD4_FOR_L1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_b_level2_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_B_WORD1_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_b_level2_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_B_WORD2_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_b_level2_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_B_WORD3_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_b_level2_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_B_WORD4_FOR_L2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_b_level3_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_B_WORD1_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_b_level3_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_B_WORD2_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_b_level3_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_B_WORD3_FOR_L3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.d_cv_b_level3_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_D_CV_B_WORD4_FOR_L3_REGISTER_OFFSET];

    _register->physical_channel[channel_id].kb.bv_k = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_BV_K_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.bv_b_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_BV_B_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.bv_b_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_BV_B_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.bv_b_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_BV_B_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.bv_b_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_BV_B_WORD4_REGISTER_OFFSET];

    _register->physical_channel[channel_id].kb.vu_k = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_K_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.vu_b_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_B_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.vu_b_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_B_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.vu_b_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_B_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].kb.vu_b_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VU_B_WORD4_REGISTER_OFFSET];

    _register->physical_channel[channel_id].temp.last_unit_converted_bv_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_UNIT_CONVERTED_BV_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_unit_converted_bv_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_UNIT_CONVERTED_BV_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_unit_converted_bv_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_UNIT_CONVERTED_BV_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_unit_converted_bv_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_UNIT_CONVERTED_BV_WORD4_REGISTER_OFFSET];

    _register->physical_channel[channel_id].temp.last_unit_converted_cv_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_UNIT_CONVERTED_CV_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_unit_converted_cv_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_UNIT_CONVERTED_CV_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_unit_converted_cv_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_UNIT_CONVERTED_CV_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_unit_converted_cv_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_UNIT_CONVERTED_CV_WORD4_REGISTER_OFFSET];

 
    _register->physical_channel[channel_id].temp.unit_converted_bv_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_UNIT_CONVERTED_BV_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.unit_converted_bv_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_UNIT_CONVERTED_BV_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.unit_converted_bv_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_UNIT_CONVERTED_BV_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.unit_converted_bv_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_UNIT_CONVERTED_BV_WORD4_REGISTER_OFFSET];

    _register->physical_channel[channel_id].temp.unit_converted_cv_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_UNIT_CONVERTED_CV_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.unit_converted_cv_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_UNIT_CONVERTED_CV_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.unit_converted_cv_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_UNIT_CONVERTED_CV_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.unit_converted_cv_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_UNIT_CONVERTED_CV_WORD4_REGISTER_OFFSET];

    // voltage
    _register->physical_channel[channel_id].voltage_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VOLTAGE_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].voltage_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VOLTAGE_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].voltage_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VOLTAGE_WORD3_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].voltage_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VOLTAGE_WORD4_REGISTER_OFFSET];

    // current
    _register->physical_channel[channel_id].current_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CURRENT_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].current_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CURRENT_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].current_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CURRENT_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].current_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CURRENT_WORD4_REGISTER_OFFSET];
    
    // last voltage
    _register->physical_channel[channel_id].temp.last_voltage_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_VOLTAGE_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_voltage_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_VOLTAGE_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_voltage_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_VOLTAGE_WORD3_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].temp.last_voltage_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_VOLTAGE_WORD4_REGISTER_OFFSET];
        
    // last current
    _register->physical_channel[channel_id].temp.last_current_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_CURRENT_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_current_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_CURRENT_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_current_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_CURRENT_WORD3_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].temp.last_current_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_CURRENT_WORD4_REGISTER_OFFSET];
    
    // sample.voltage
    _register->physical_channel[channel_id].sample.voltage_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VOLTAGE_SAMPLE_WORD1_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].sample.voltage_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VOLTAGE_SAMPLE_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].sample.voltage_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VOLTAGE_SAMPLE_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].sample.voltage_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_VOLTAGE_SAMPLE_WORD4_REGISTER_OFFSET];

    // sample current
    _register->physical_channel[channel_id].sample.current_region.word1 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CURRENT_SAMPLE_WORD1_REGISTER_OFFSET]; 
    _register->physical_channel[channel_id].sample.current_region.word2 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CURRENT_SAMPLE_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].sample.current_region.word3 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CURRENT_SAMPLE_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].sample.current_region.word4 = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_CURRENT_SAMPLE_WORD4_REGISTER_OFFSET];

    // source current level 1
    _register->physical_channel[channel_id].source_current_level_1_region.word1 = &data_model[channel_id].holding_registers[HOLDING_SOURCE_CURRENT_CAPACITY_LEVEL1_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].source_current_level_1_region.word2 = &data_model[channel_id].holding_registers[HOLDING_SOURCE_CURRENT_CAPACITY_LEVEL1_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].source_current_level_1_region.word3 = &data_model[channel_id].holding_registers[HOLDING_SOURCE_CURRENT_CAPACITY_LEVEL1_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].source_current_level_1_region.word4 = &data_model[channel_id].holding_registers[HOLDING_SOURCE_CURRENT_CAPACITY_LEVEL1_WORD4_REGISTER_OFFSET];
    
    // source current level 2
    _register->physical_channel[channel_id].source_current_level_2_region.word1 = &data_model[channel_id].holding_registers[HOLDING_SOURCE_CURRENT_CAPACITY_LEVEL2_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].source_current_level_2_region.word2 = &data_model[channel_id].holding_registers[HOLDING_SOURCE_CURRENT_CAPACITY_LEVEL2_WORD2_REGISTER_OFFSET];    
    _register->physical_channel[channel_id].source_current_level_2_region.word3 = &data_model[channel_id].holding_registers[HOLDING_SOURCE_CURRENT_CAPACITY_LEVEL2_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].source_current_level_2_region.word4 = &data_model[channel_id].holding_registers[HOLDING_SOURCE_CURRENT_CAPACITY_LEVEL2_WORD4_REGISTER_OFFSET];

    // source current level 3
    _register->physical_channel[channel_id].source_current_level_3_region.word1 = &data_model[channel_id].holding_registers[HOLDING_SOURCE_CURRENT_CAPACITY_LEVEL3_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].source_current_level_3_region.word2 = &data_model[channel_id].holding_registers[HOLDING_SOURCE_CURRENT_CAPACITY_LEVEL3_WORD2_REGISTER_OFFSET];
    _register->physical_channel[channel_id].source_current_level_3_region.word3 = &data_model[channel_id].holding_registers[HOLDING_SOURCE_CURRENT_CAPACITY_LEVEL3_WORD3_REGISTER_OFFSET];
    _register->physical_channel[channel_id].source_current_level_3_region.word4 = &data_model[channel_id].holding_registers[HOLDING_SOURCE_CURRENT_CAPACITY_LEVEL3_WORD4_REGISTER_OFFSET];

    // device host
    _register->physical_channel[channel_id].device_host.word1 = &data_model[channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_DEVICE_HOST_WORD1_REGISTER_OFFSET];
    _register->physical_channel[channel_id].device_host.word2 = &data_model[channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_DEVICE_HOST_WORD2_REGISTER_OFFSET];

    // device host save enable
    _register->physical_channel[channel_id].device_host_save_enable = &data_model[channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_DEVICE_HOST_UPDATE_ENABLE_REGISTER_OFFSET];

    // device tower light
    _register->physical_channel[channel_id].led_ready = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LED_READY_REGISTER_OFFSET];
    _register->physical_channel[channel_id].led_run = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LED_RUN_REGISTER_OFFSET];
    _register->physical_channel[channel_id].led_error = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LED_ERROR_REGISTER_OFFSET];
    _register->physical_channel[channel_id].ac_loss = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_AC_LOSS_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_led_ready = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_LED_READY_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_led_run = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_LED_RUN_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_led_error = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_LED_ERROR_REGISTER_OFFSET];
    _register->physical_channel[channel_id].temp.last_ac_loss = &data_model[channel_id].holding_registers[HOLDING_PHYSICAL_CHANNEL_LAST_AC_LOSS_REGISTER_OFFSET];

}



