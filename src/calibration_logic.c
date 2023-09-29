/**
* @file     calibration_logic.c
* @brief    calibration_logic.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     02/13/2019
*/

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <inttypes.h>  

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

#include "formation_logic.h"
#include "configuration_logic.h"
#include "step_timming_parameters.h"
#include "logical_channel.h"
#include "physical_channel.h"
#include "debug_logic.h"
#include "misc.h"

static int check_general_parameters(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;

    _register = &manager->modbus_ctx._register;

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

    if(*_register->logical_channel[logical_channel_id].step_quantity > 1) {
        return ERROR_STEP_QUANTITY_MORE_THAN_ONE;
    }

    return 0;
}

int calculate_and_update_calibration_voltage_and_current(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    int logical_channel_id = channel_id, i;
    uint16_t *binding;
    Register *_register = NULL;
    _4paired_register_region *sample_voltage_region, *sample_current_region;
    uint64_t unsign_voltage_amplitude, unsigned_calc_current_amplitude;
    uint64_t sample_voltage, sample_current;    
    _4paired_register_region *calibration_sample_current_region, *calibration_sample_voltage_region;
    _4paired_register_region *physical_current_region, *physical_voltage_region;
    uint64_t calibration_current, calibration_voltage, channel_current, channel_voltage, current_sign, voltage_sign;
    uint64_t amplitude_mask = 0x7fffffffffffffff, current_amplitude;
    int operation;
    uint16_t *current_source_level;
    _4paired_register_region *c_cv_b_region, *d_cv_b_region, *bv_b_region;
    uint16_t *c_cv_k_reg, *d_cv_k_reg, *bv_k_reg;
    int64_t c_cv_b_sign, d_cv_b_sign, bv_b_sign;
    uint64_t c_cv_b_unsign, d_cv_b_unsign, bv_b_unsign;
    int64_t temp_channel_current, calc_voltage_amplitude, calc_current_amplitude;
    double c_cv_k, d_cv_k, bv_k;

    _register = &manager->modbus_ctx._register;

    binding = _register->logical_channel[logical_channel_id].binding;

    if(*binding == 0) {
        printf("calculate_and_update_logical_voltage_and_current fail, binding = 0\n");
        return -1;
    }

    current_source_level = _register->logical_channel[logical_channel_id].calibration_param.current_source_level;

    for(i=0; i<NB_PHYSICAL_CHANNEL; i++) {

        if(((( *binding )>>i) & 0x0001) > 0) {

            calibration_sample_current_region = &_register->logical_channel[i].calibration_param.calibration_sample_current_region;
            calibration_sample_voltage_region = &_register->logical_channel[i].calibration_param.calibration_sample_voltage_region;

            sample_voltage_region = &_register->physical_channel[i].sample.voltage_region;
            sample_current_region = &_register->physical_channel[i].sample.current_region;
    
            sample_voltage = combine_4paired_register_region(sample_voltage_region);
            sample_current = combine_4paired_register_region(sample_current_region);

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
                operation = -1; 
            } else {//positive 
                operation = 1; 
            }

            current_amplitude = channel_current & amplitude_mask;

            temp_channel_current = current_amplitude * operation;

            if(operation > 0) { /* plus */
                temp_channel_current = (double)(temp_channel_current) * c_cv_k + c_cv_b_sign;
            } else { /* minus */
                temp_channel_current = (double)(temp_channel_current) * d_cv_k + d_cv_b_sign;
            }
           
            calc_current_amplitude = temp_channel_current;

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

            calibration_current = (unsigned_calc_current_amplitude & 0x7fffffffffffffff) | current_sign;
            update_4paired_register_region(calibration_sample_current_region, calibration_current);

            calibration_voltage = (unsign_voltage_amplitude & 0x7fffffffffffffff) | voltage_sign;
            update_4paired_register_region(calibration_sample_voltage_region, calibration_voltage);
        }
    }

    return 0;
}


int calibration_reset(Manager *_manager, int channel_id)
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
    
    return 0;
}

static int init_ccc_registers(Manager *_manager, int channel_id)
{
    int i;
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint16_t *current_source_level_reg, *binding, *kz_h, *kz_m, *kz_l;
    uint16_t temp_kz_h, temp_kz_m, temp_kz_l;

    _register = &manager->modbus_ctx._register;

    binding = _register->logical_channel[logical_channel_id].binding;
    current_source_level_reg = _register->logical_channel[logical_channel_id].calibration_param.current_source_level;

    if(*current_source_level_reg == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1) {
        temp_kz_h = 0;
        temp_kz_m = 0;
        temp_kz_l = 1;
    } else if(*current_source_level_reg == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2) {
        temp_kz_h = 0;
        temp_kz_m = 1;
        temp_kz_l = 0;
    } else if(*current_source_level_reg == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3) {
        temp_kz_h = 0;
        temp_kz_m = 0;
        temp_kz_l = 0;
    }

    for(i=0; i<NB_LOGIC_CHANNEL; i++) {   
        if((((*binding )>>i) & 0x0001) > 0) {

            *_register->physical_channel[i].current_source_level = *current_source_level_reg;

            kz_h = _register->physical_channel[i].kz_h;
            kz_m = _register->physical_channel[i].kz_m;
            kz_l = _register->physical_channel[i].kz_l;

            *kz_h = temp_kz_h;
            *kz_m = temp_kz_m;
            *kz_l = temp_kz_l;
        }
    }

    return 0;
}

static int init_ccd_registers(Manager *_manager, int channel_id)
{   
    int i;
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint16_t *current_source_level_reg, *binding, *kz_h, *kz_m, *kz_l;
    uint16_t temp_kz_h, temp_kz_m, temp_kz_l;

    _register = &manager->modbus_ctx._register;

    binding = _register->logical_channel[logical_channel_id].binding;
    current_source_level_reg = _register->logical_channel[logical_channel_id].calibration_param.current_source_level;

    if(*current_source_level_reg == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1) {
        temp_kz_h = 0;
        temp_kz_m = 0;
        temp_kz_l = 1;
    } else if(*current_source_level_reg == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2) {
        temp_kz_h = 0;
        temp_kz_m = 1;
        temp_kz_l = 0;
    } else if(*current_source_level_reg == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3) {
        temp_kz_h = 0;
        temp_kz_m = 0;
        temp_kz_l = 0;
    }

    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        if((((*binding )>>i) & 0x0001) > 0) {

            *_register->physical_channel[i].current_source_level = *current_source_level_reg;

            kz_h = _register->physical_channel[i].kz_h;
            kz_m = _register->physical_channel[i].kz_m;
            kz_l = _register->physical_channel[i].kz_l;

            *kz_h = temp_kz_h;
            *kz_m = temp_kz_m;
            *kz_l = temp_kz_l;
        }
    }

    return 0;
}

static int init_cvad_registers(Manager *_manager, int channel_id)
{
    int i;
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint16_t *current_source_level_reg, *binding, *kz_h, *kz_m, *kz_l;
    uint16_t temp_kz_h, temp_kz_m, temp_kz_l;

    _register = &manager->modbus_ctx._register;

    binding = _register->logical_channel[logical_channel_id].binding;
    current_source_level_reg = _register->logical_channel[logical_channel_id].calibration_param.current_source_level;

    if(*current_source_level_reg == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1) {
        temp_kz_h = 0;
        temp_kz_m = 0;
        temp_kz_l = 1;
    } else if(*current_source_level_reg == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2) {
        temp_kz_h = 0;
        temp_kz_m = 1;
        temp_kz_l = 0;
    } else if(*current_source_level_reg == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3) {
        temp_kz_h = 0;
        temp_kz_m = 0;
        temp_kz_l = 0;
    }

    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        if((((*binding )>>i) & 0x0001) > 0) {

            *_register->physical_channel[i].current_source_level = *current_source_level_reg;

            kz_h = _register->physical_channel[i].kz_h;
            kz_m = _register->physical_channel[i].kz_m;
            kz_l = _register->physical_channel[i].kz_l;

            *kz_h = temp_kz_h;
            *kz_m = temp_kz_m;
            *kz_l = temp_kz_l;
        }
    }

    return 0;
}

static int init_cvda_registers(Manager *_manager, int channel_id)
{
    int i;
    Manager *manager = _manager;
    Register *_register = NULL;
    int logical_channel_id = channel_id;
    uint16_t *current_source_level_reg, *binding, *kz_h, *kz_m, *kz_l;
    uint16_t temp_kz_h, temp_kz_m, temp_kz_l;

    _register = &manager->modbus_ctx._register;

    binding = _register->logical_channel[logical_channel_id].binding;
    current_source_level_reg = _register->logical_channel[logical_channel_id].calibration_param.current_source_level;

    if(*current_source_level_reg == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1) {
        temp_kz_h = 0;
        temp_kz_m = 0;
        temp_kz_l = 1;
    } else if(*current_source_level_reg == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2) {
        temp_kz_h = 0;
        temp_kz_m = 1;
        temp_kz_l = 0;
    } else if(*current_source_level_reg == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3) {
        temp_kz_h = 0;
        temp_kz_m = 0;
        temp_kz_l = 0;
    }

    for(i=0; i<NB_LOGIC_CHANNEL; i++) {   
        if((((*binding )>>i) & 0x0001) > 0) {

            *_register->physical_channel[i].current_source_level = *current_source_level_reg;

            kz_h = _register->physical_channel[i].kz_h;
            kz_m = _register->physical_channel[i].kz_m;
            kz_l = _register->physical_channel[i].kz_l;

            *kz_h = temp_kz_h;
            *kz_m = temp_kz_m;
            *kz_l = temp_kz_l;
        }
    }

    return 0;
}

static int check_ccc_parameters(Manager *_manager, int channel_id)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = channel_id;
    uint16_t *current_source_level, *exp_message_reg;
    uint64_t calibration_current, calibration_voltage, maximum_channel_current;
    _4paired_register_region *calibration_current_region;
    _4paired_register_region *calibration_voltage_region;

    _register = &manager->modbus_ctx._register;

    calibration_voltage_region = &_register->logical_channel[logical_channel_id].calibration_param.calibration_voltage_region;
    calibration_current_region = &_register->logical_channel[logical_channel_id].calibration_param.calibration_current_region;

    exp_message_reg = _register->logical_channel[logical_channel_id].exception_message;
    current_source_level = _register->logical_channel[logical_channel_id].calibration_param.current_source_level;

    calibration_current = combine_4paired_register_region(calibration_current_region);
    calibration_voltage = combine_4paired_register_region(calibration_voltage_region);

    if(calibration_current == 0) {
        printf("check_ccc_parameters fail, calibration_current is zero 0\n");
        return ERROR_CCC_CALIBRATION_CURRENT_ZERO;
    }

    if(calibration_voltage == 0) {
        printf("check_ccc_parameters fail, calibration_voltage is zero 0\n");
        return ERROR_CCC_CALIBRATION_VOLTAGE_ZERO;
    }

    if((*current_source_level!=PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1) \
        &&(*current_source_level!=PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2) \
        &&(*current_source_level!=PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3)) {
        printf("check_ccc_parameters fail, incorrect current_source_level %d\n", *current_source_level);
    }

    if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1) {
        maximum_channel_current = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL1_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
        if(calibration_current > (maximum_channel_current)) {
            printf("check_ccc_parameters fail, incorrect calibration_current");
            return ERROR_CALIBRATION_CURRENT_LIMIT_EXCEEDED;
        }
    } else if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2) {
        maximum_channel_current = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL2_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
        if(calibration_current > (maximum_channel_current)) {
            printf("check_ccc_parameters fail, incorrect calibration_current");
            return ERROR_CALIBRATION_CURRENT_LIMIT_EXCEEDED;
        }
    } else if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3) {
        maximum_channel_current = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL3_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
        if(calibration_current > (maximum_channel_current)) {
            printf("check_ccc_parameters fail, incorrect calibration_current");
            return ERROR_CALIBRATION_CURRENT_LIMIT_EXCEEDED;
        }
    }

#if 0
    printf("current_source_level %d\n", *current_source_level);
    printf("calibration_current %"PRIu64"\n", calibration_current);
    printf("calibration_voltage %"PRIu64"\n", calibration_voltage);
    printf("for joe debug purposes\n");
#endif

    return 0;
}

static int check_ccd_parameters(Manager *_manager, int channel_id)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = channel_id;
    uint16_t *current_source_level, *exp_message_reg;
    uint64_t calibration_current, calibration_voltage, maximum_channel_current;
    _4paired_register_region *calibration_current_region;
    _4paired_register_region *calibration_voltage_region;

    _register = &manager->modbus_ctx._register;

    calibration_voltage_region = &_register->logical_channel[logical_channel_id].calibration_param.calibration_voltage_region;
    calibration_current_region = &_register->logical_channel[logical_channel_id].calibration_param.calibration_current_region;

    exp_message_reg = _register->logical_channel[logical_channel_id].exception_message;
    current_source_level = _register->logical_channel[logical_channel_id].calibration_param.current_source_level;

    calibration_current = combine_4paired_register_region(calibration_current_region);
    calibration_voltage = combine_4paired_register_region(calibration_voltage_region);

    if(calibration_current == 0) {
        printf("check_ccd_parameters fail, calibration_current is zero 0\n");
        return ERROR_CCC_CALIBRATION_CURRENT_ZERO;
    }

    if(calibration_voltage == 0) {
        printf("check_ccd_parameters fail, calibration_voltage is zero 0\n");
        return ERROR_CCC_CALIBRATION_VOLTAGE_ZERO;
    }

    if((*current_source_level!=PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1) \
        &&(*current_source_level!=PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2) \
        &&(*current_source_level!=PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3)) {
        printf("check_ccd_parameters fail, incorrect current_source_level %d\n", *current_source_level);
    }

    if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1) {
        maximum_channel_current = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL1_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
        if(calibration_current > (maximum_channel_current)) {
            printf("check_ccd_parameters fail, incorrect calibration_current");
            return ERROR_CALIBRATION_CURRENT_LIMIT_EXCEEDED;
        }
    } else if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2) {
        maximum_channel_current = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL2_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
        if(calibration_current > (maximum_channel_current)) {
            printf("check_ccd_parameters fail, incorrect calibration_current");
            return ERROR_CALIBRATION_CURRENT_LIMIT_EXCEEDED;
        }
    } else if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3) {
        maximum_channel_current = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL3_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
        if(calibration_current > (maximum_channel_current)) {
            printf("check_ccd_parameters fail, incorrect calibration_current");
            return ERROR_CALIBRATION_CURRENT_LIMIT_EXCEEDED;
        }
    }

#if 0
    printf("current_source_level %d\n", *current_source_level);
    printf("calibration_current %"PRIu64"\n", calibration_current);
    printf("calibration_voltage %"PRIu64"\n", calibration_voltage);
    printf("for joe debug purposes\n");
#endif

    return 0;
}


static int check_cvad_parameters(Manager *_manager, int channel_id)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = channel_id;
    uint16_t *current_source_level, *exp_message_reg;
    uint64_t calibration_current, calibration_voltage, maximum_channel_current;
    _4paired_register_region *calibration_current_region;
    _4paired_register_region *calibration_voltage_region;

    //_register = &manager->modbus_ctx._register;


    return 0;
}

static int check_cvda_parameters(Manager *_manager, int channel_id)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = channel_id;
    uint16_t *current_source_level, *exp_message_reg;
    uint64_t calibration_current, calibration_voltage, maximum_channel_current;
    _4paired_register_region *calibration_current_region;
    _4paired_register_region *calibration_voltage_region;

    _register = &manager->modbus_ctx._register;

    calibration_voltage_region = &_register->logical_channel[logical_channel_id].calibration_param.calibration_voltage_region;
    calibration_current_region = &_register->logical_channel[logical_channel_id].calibration_param.calibration_current_region;

    exp_message_reg = _register->logical_channel[logical_channel_id].exception_message;
    current_source_level = _register->logical_channel[logical_channel_id].calibration_param.current_source_level;

    calibration_current = combine_4paired_register_region(calibration_current_region);
    calibration_voltage = combine_4paired_register_region(calibration_voltage_region);

    if(calibration_current == 0) {
        printf("check_cvda_parameters fail, calibration_current is zero 0\n");
        return ERROR_CVDA_CALIBRATION_CURRENT_ZERO;
    }

    if(calibration_voltage == 0) {
        printf("check_cvda_parameters fail, calibration_voltage is zero 0\n");
        return ERROR_CVDA_CALIBRATION_VOLTAGE_ZERO;
    }

    if((*current_source_level!=PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1) \
        &&(*current_source_level!=PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2) \
        &&(*current_source_level!=PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3)) {
        printf("check_cvda_parameters fail, incorrect current_source_level %d\n", *current_source_level);
    }

    if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_1) {
        maximum_channel_current = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL1_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
        if(calibration_current > (maximum_channel_current)) {
            printf("check_cvda_parameters fail, incorrect calibration_current");
            return ERROR_CALIBRATION_CURRENT_LIMIT_EXCEEDED;
        }
    } else if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_2) {
        maximum_channel_current = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL2_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
        if(calibration_current > (maximum_channel_current)) {
            printf("check_cvda_parameters fail, incorrect calibration_current");
            return ERROR_CALIBRATION_CURRENT_LIMIT_EXCEEDED;
        }
    } else if(*current_source_level == PHYSICAL_CHANNEL_CURRENT_SOURCE_LEVEL_3) {
        maximum_channel_current = DEFAULT_SOURCE_CURRENT_CAPACITY_LEVEL3_SETTING_IN_MA_UNIT * UNIT_FACTOR_BETWEEN_MA_AND_01UA;
        if(calibration_current > (maximum_channel_current)) {
            printf("check_cvda_parameters fail, incorrect calibration_current");
            return ERROR_CALIBRATION_CURRENT_LIMIT_EXCEEDED;
        }
    }

    return 0;
}

int calibration_start(Manager *_manager, int channel_id)
{
    int ret;
    uint16_t *step_type_reg;
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = channel_id;
    uint64_t level_1, level_2, level_3, temp, maximum_source_current;

    if(!manager || logical_channel_id < LOGICAL_CHANNEL_1_ID || logical_channel_id >= NB_LOGIC_CHANNEL) {
        printf("calibration_start fail, incorrect parameters, channel %d\n", logical_channel_id);
        return -1;
    }

    _register = &manager->modbus_ctx._register;

    step_type_reg = _register->logical_channel[logical_channel_id].step_param.step_type;

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

    printf("calibration_start step_type: %d\n", *step_type_reg);

    switch(*step_type_reg) {

        case CALIBRATION_STEP_TYPE_CCC:
             printf("CALIBRATION_STEP_TYPE_CCC\n");

             ret = check_ccc_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("calibration_start->check_ccc_parameters fail\n");
                 return ret;
             }

             init_ccc_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("calibration_start->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             return 0;
        break;

        case CALIBRATION_STEP_TYPE_CCD:
             printf("CALIBRATION_STEP_TYPE_CCD\n");

             ret = check_ccd_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("calibration_start->check_ccc_parameters fail\n");
                 return ret;
             }

             init_ccd_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("calibration_start->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             return 0;
        break;

        case CALIBRATION_STEP_TYPE_CVAD:
             printf("CALIBRATION_STEP_TYPE_CVAD\n");

             ret = check_cvad_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("calibration_start->check_cvad_parameters fail\n");
                 return ret;
             }

             init_cvad_registers(manager, logical_channel_id);
             
             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("calibration_start->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             return 0;
        break;

        case CALIBRATION_STEP_TYPE_CVDA:
             printf("CALIBRATION_STEP_TYPE_CVDA\n");
             
             ret = check_cvda_parameters(manager, logical_channel_id); 
             if(ret < 0) {
                 printf("calibration_start->check_cvda_parameters fail\n");
                 return ret;
             }

             init_cvda_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("calibration_start->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             return 0;
        break;

        default:
             // do nothing
             return ERROR_UNKNOWN_CALIBRATION_STEP_TYPE;
        break;

    }

    return 0;
}

int do_ccc_calibration_logic(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    int logical_channel_id = channel_id, i;
    
    calculate_and_update_calibration_voltage_and_current(manager, logical_channel_id);

    check_and_update_KB(manager, logical_channel_id);

    return 0;
}

int do_ccd_calibration_logic(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    int logical_channel_id = channel_id, i;
    
    calculate_and_update_calibration_voltage_and_current(manager, logical_channel_id);

    check_and_update_KB(manager, logical_channel_id);

    return 0;
}

int do_cvad_calibration_logic(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    int logical_channel_id = channel_id, i;
    
    calculate_and_update_calibration_voltage_and_current(manager, logical_channel_id);

    check_and_update_KB(manager, logical_channel_id);

    return 0;
}

int do_cvda_calibration_logic(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    int logical_channel_id = channel_id, i;
    
    calculate_and_update_calibration_voltage_and_current(manager, logical_channel_id);

    check_and_update_KB(manager, logical_channel_id);

    return 0;
}

int calibration_run(Manager *_manager, int channel_id)
{
    int ret, logical_channel_id = channel_id;
    Register *_register = NULL;
    Manager *manager = _manager;
    uint16_t *calibration_enable, *step_type_reg;
    
    _register = &manager->modbus_ctx._register;

    calibration_enable = _register->logical_channel[logical_channel_id].calibration_param.enable;
    step_type_reg = _register->logical_channel[logical_channel_id].step_param.step_type;

    if(*calibration_enable) {

        //printf("step_type %d\n", *step_type_reg);

        switch(*step_type_reg) {

            case CALIBRATION_STEP_TYPE_CCC:

                 ret = do_ccc_calibration_logic(manager, logical_channel_id);
                 if(ret < 0) {
                     printf("do_calibration_logic->do_ccc_calibration_logic fail\n");
                     return ret;
                 }

                 return 0;
            break;

            case CALIBRATION_STEP_TYPE_CCD:

                 ret = do_ccd_calibration_logic(manager, logical_channel_id);
                 if(ret < 0) {
                     printf("do_calibration_logic->do_ccd_calibration_logic fail\n");
                     return ret;
                 }

                 return 0;
            break;

            case CALIBRATION_STEP_TYPE_CVAD:

                 ret = do_cvad_calibration_logic(manager, logical_channel_id);
                 if(ret < 0) {
                     printf("do_calibration_logic->do_ccd_calibration_logic fail\n");
                     return ret;
                 }

                 return 0;
            break;

            case CALIBRATION_STEP_TYPE_CVDA:

                 ret = do_cvda_calibration_logic(manager, logical_channel_id);
                 if(ret < 0) {
                     printf("do_calibration_logic->do_ccd_calibration_logic fail\n");
                     return ret;
                 }

                 return 0;
            break;

            default:
                // do nothing
                return ERROR_UNKNOWN_CALIBRATION_STEP_TYPE;
            break;

        }
    }

    return ret;
} 

