/**
* @file     recovery_logic.c
* @brief    recovery_logic
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     02/21/2018
*/

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "channel.h"
#include "log_controller.h"
#include "tcp.h"
#include "registers.h"
#include "modbus_tcp.h"
#include "spi_controller.h"
#include "gpio_controller.h"
#include "dac_controller.h"
#include "io_expander_controller.h"
#include "adc_controller.h"
#include "ssd1306_oled_controller.h"
#include "i2c_controller.h"

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
#include "calibration_logic.h"

#include "recovery_logic.h"

int check_if_all_bound_physical_channels_in_idel_state(Manager *manager, int channel_id)
{   
    uint16_t logical_channel_id = channel_id;
    int ret = 0, mask, nb_binding = 0;
    Register *_register = NULL;

    _register = &manager->modbus_ctx._register;

    // clear exception message
    *_register->logical_channel[logical_channel_id].exception_message = 0;

    // check if physical channel 1 is in count
    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH1_MASK_BIT;
    if((*_register->logical_channel[logical_channel_id].binding) & mask) {
        if(*_register->physical_channel[PHYSICAL_CHANNEL_1_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %d\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        }
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH2_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_2_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %d\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        }
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH3_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_3_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %d\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        }
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH4_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_4_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %d\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        }
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH5_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_5_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %d\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        }
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH6_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_6_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %d\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        }
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH7_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_7_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %hu\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        }
    }

    mask = LOGICAL_CHANNEL_BINDING_REGISTER_PHYCH8_MASK_BIT;
    if(*_register->logical_channel[logical_channel_id].binding & mask) {
        if(*_register->physical_channel[LOGICAL_CHANNEL_8_ID].status == PHYSICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {
            printf("Pysichal Channel 1 is busy, exception occurs in logical channel %hu\n", logical_channel_id);
            *_register->logical_channel[logical_channel_id].exception_message |= mask;
            ret = ERROR_PHYSICAL_CHANNEL_BUSY;
        }
    }

    return ret;
}

int recovery_one_logical_channel_holding_registers_from_file(Manager *_manager, char *_file_name, int channel_id)
{
    int ret, i, j, logical_channel_id = channel_id, physical_channel_id = channel_id, offset, channel_index;    
    FILE *holding_fd;
    Register *_register = NULL;
    Manager *manager = _manager;
    char *file_name = _file_name;
    uint16_t temp_logical_channel_registers[NB_HOLDING_REGISTERS], ori_mode, *binding_reg, *exp_message_reg;
    uint16_t temp[NB_HOLDING_REGISTERS];
    uint64_t temp_64;
    _4paired_register_region *temp_region_p, temp_region;

    if(!manager || !file_name) {
        printf("reload_all_holding_registers_from_file fail, incorrect parameters\n");
        return ERROR_INTERNEL_ERRORS;
    }

    _register = &manager->modbus_ctx._register;

    exp_message_reg = _register->logical_channel[logical_channel_id].exception_message;

    // check if file exsists
    ret = access(file_name, F_OK);
    if(ret < 0) {
        printf("file %s doesn't exsist\n", file_name);
        return ERROR_NO_SUCH_FILE_OR_DIRECTORY;
    }

    // open file and read content
    holding_fd = fopen(file_name,"r");
    if(!holding_fd) {
        printf("fopen %s fail\n", file_name);
        return ERROR_NO_SUCH_FILE_OR_DIRECTORY;
    }

    // recovery logical_channel first
    offset = logical_channel_id * NB_HOLDING_REGISTERS;
    fseek(holding_fd, offset, SEEK_SET);
    ret = fread(temp_logical_channel_registers, 2, NB_HOLDING_REGISTERS, holding_fd);
    if(ret != NB_HOLDING_REGISTERS) {
        fclose(holding_fd);
        return ERROR_READ_RECOVERY_FILE_FAIL;
    }

    ori_mode = temp_logical_channel_registers[HOLDING_MODE_REGISTER_OFFSET];
    printf("channel_id %d ori_mode %d\n", logical_channel_id, ori_mode);

    _register->data_model[logical_channel_id].holding_registers[HOLDING_BINDING_REGISTER_OFFSET] = temp_logical_channel_registers[HOLDING_BINDING_REGISTER_OFFSET];

    // recovery physical channels
    // check if all physical channels are in idel status
    ret = check_if_all_bound_physical_channels_in_idel_state(manager, logical_channel_id);
    if(ret < 0) {
        printf("recovery_one_logical_channel_holding_registers_from_file->check_physical_channels_and_update_info fail\n");
        fclose(holding_fd);
        return ret;
    }
    
    for(i=0; i<NB_HOLDING_REGISTERS; i++) {
        _register->data_model[logical_channel_id].holding_registers[i] = temp_logical_channel_registers[i];
    }

    // load registers of all physical channels from file
    binding_reg = &temp_logical_channel_registers[HOLDING_BINDING_REGISTER_OFFSET];

    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        
        if((((*binding_reg )>>i) & 0x0001) > 0) {
            
            temp_region_p = &_register->physical_channel[i].temp.last_vi_region;
            update_4paired_register_region(temp_region_p, 0);

            temp_region_p = &_register->physical_channel[i].temp.last_vu_region;
            update_4paired_register_region(temp_region_p, 0);
            
            *_register->physical_channel[i].temp.last_kz_h = DEFAULT_LAST_KZ_H_SETTING;
            *_register->physical_channel[i].temp.last_kz_m = DEFAULT_LAST_KZ_M_SETTING;
            *_register->physical_channel[i].temp.last_kz_l = DEFAULT_LAST_KZ_L_SETTING;
            *_register->physical_channel[i].temp.last_jc   = DEFAULT_LAST_JC_SETTING;
            *_register->physical_channel[i].temp.last_cf   = DEFAULT_LAST_CF_SETTING;
            *_register->physical_channel[i].temp.last_ck   = DEFAULT_LAST_CK_SETTING;
            *_register->physical_channel[i].temp.last_cled = DEFAULT_LAST_CLED_SETTING;
            *_register->physical_channel[i].temp.last_fled = DEFAULT_LAST_FLED_SETTING;
            
            // read physical channel registers
            offset = (i* NB_HOLDING_REGISTERS)*2;
            fseek(holding_fd, offset, SEEK_SET);
            ret = fread(temp, 2, NB_HOLDING_REGISTERS, holding_fd);
            if(ret != NB_HOLDING_REGISTERS) {
                fclose(holding_fd);
                return ERROR_READ_RECOVERY_FILE_FAIL;
            }

            //sleep(1);

            *_register->physical_channel[i].kz_h = temp[HOLDING_PHYSICAL_CHANNEL_KZ_H_REGISTER_OFFSET];
            *_register->physical_channel[i].kz_m = temp[HOLDING_PHYSICAL_CHANNEL_KZ_M_REGISTER_OFFSET];
            *_register->physical_channel[i].kz_l = temp[HOLDING_PHYSICAL_CHANNEL_KZ_L_REGISTER_OFFSET];
            *_register->physical_channel[i].jc = temp[HOLDING_PHYSICAL_CHANNEL_JC_REGISTER_OFFSET];
            *_register->physical_channel[i].cf = temp[HOLDING_PHYSICAL_CHANNEL_CF_REGISTER_OFFSET];
            *_register->physical_channel[i].ck = temp[HOLDING_PHYSICAL_CHANNEL_CK_REGISTER_OFFSET];
            *_register->physical_channel[i].cled = temp[HOLDING_PHYSICAL_CHANNEL_CLED_REGISTER_OFFSET];
            *_register->physical_channel[i].fled = temp[HOLDING_PHYSICAL_CHANNEL_FLED_REGISTER_OFFSET];

            // vi
            temp_region.word1 =  &temp[HOLDING_PHYSICAL_CHANNEL_VI_WORD1_REGISTER_OFFSET];
            temp_region.word2 =  &temp[HOLDING_PHYSICAL_CHANNEL_VI_WORD2_REGISTER_OFFSET];
            temp_region.word3 =  &temp[HOLDING_PHYSICAL_CHANNEL_VI_WORD3_REGISTER_OFFSET];
            temp_region.word4 =  &temp[HOLDING_PHYSICAL_CHANNEL_VI_WORD4_REGISTER_OFFSET];
            temp_64 = combine_4paired_register_region(&temp_region);
            temp_region_p = &_register->physical_channel[i].vi_region;
            update_4paired_register_region(temp_region_p, temp_64);
            
            // vu
            temp_region.word1 =  &temp[HOLDING_PHYSICAL_CHANNEL_VU_WORD1_REGISTER_OFFSET];
            temp_region.word2 =  &temp[HOLDING_PHYSICAL_CHANNEL_VU_WORD2_REGISTER_OFFSET];
            temp_region.word3 =  &temp[HOLDING_PHYSICAL_CHANNEL_VU_WORD3_REGISTER_OFFSET];
            temp_region.word4 =  &temp[HOLDING_PHYSICAL_CHANNEL_VU_WORD4_REGISTER_OFFSET];
            temp_64 = combine_4paired_register_region(&temp_region);
            temp_region_p = &_register->physical_channel[i].vu_region;
            update_4paired_register_region(temp_region_p, temp_64);

        }
    }

    // load step parameter registers
    for(i=0; i<NB_PARAMETER_CHANNELS_FOR_EACH_LOGICAL_CHANNEL; i++) {

        channel_index = i + NB_PHYSICAL_CHANNEL + (logical_channel_id*NB_PARAMETER_CHANNELS_FOR_EACH_LOGICAL_CHANNEL);

        offset = (channel_index * NB_HOLDING_REGISTERS)*2;

        printf("load step parameters, channel_index %d, offset %d\n", channel_index, offset);

        fseek(holding_fd, offset, SEEK_SET);

        ret = fread(temp, 2, NB_HOLDING_REGISTERS, holding_fd);
        if(ret != NB_HOLDING_REGISTERS) {
            fclose(holding_fd);
            return ERROR_READ_RECOVERY_FILE_FAIL;
        }

        //dump_data16(temp, 60);
        
        for(j=0; j<NB_HOLDING_REGISTERS; j++) {
            _register->data_model[channel_index].holding_registers[j] = temp[j];
        }
        
    }

    return ret;
}

int do_save_all_logical_channel_holding_registers(Manager *_manager, char *_file_name)
{
    int ret, i;    
    FILE *holding_fd;
    Register *_register = NULL;
    Manager *manager = _manager;
    char *file_name = _file_name;

    if(!manager || !file_name) {
        printf("reload_all_holding_registers_from_file fail, incorrect parameters\n");
        return ERROR_INTERNEL_ERRORS;
    }

    _register = &manager->modbus_ctx._register;

    holding_fd = fopen(file_name,"w+");
    if(!holding_fd) {
        printf("fopen %s fail\n", file_name);
        return ERROR_NO_SUCH_FILE_OR_DIRECTORY;
    }    

    for(i=0; i<NB_LOGIC_CHANNEL + NB_STEP_PARAMETER_CHANNELS; i++){
        ret = fwrite(_register->data_model[i].holding_registers, 2, NB_HOLDING_REGISTERS, holding_fd);
        if(ret != NB_HOLDING_REGISTERS){
            printf("fwrite %s fail\n", file_name);
            exit(0);
        }
    }
    
    fflush(holding_fd);

    fsync(fileno(holding_fd));

    fclose(holding_fd);

    return 0;

}

int do_one_logical_channel_recovery(Manager *_manager, int channel_id)
{
    int ret, i, logical_channel_id = channel_id;    
    FILE *holding_fd;
    Register *_register = NULL;
    Manager *manager = _manager;

    printf("do_one_logical_channel_recovery\n");

    ret = recovery_one_logical_channel_holding_registers_from_file(manager, RECOVERY_FILE_NAME, logical_channel_id);
    if(ret < 0) {
        printf("do_one_logical_channel_recovery->reload_one_logical_channel_holding_registers_from_file fail\n");
        return ret;
    }

    return 0;
}

int recovery_logic_reset(Manager *_manager, int channel_id)
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

    reset_one_logical_channel(manager, logical_channel_id);
    reset_step_parameter_register(manager, logical_channel_id);
    
    return 0;
}

