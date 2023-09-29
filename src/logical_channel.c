/**
* @file     logical_channel.c
* @brief    logical_channel.c
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

#include "logical_channel.h"

#include "registers.h"

#include "error_no.h"
#include "error_msg.h"

void init_step_parameter_pointer_register_mapping(Manager * manager, int channel_id, int first_step_param_channel_id)
{
    int logical_channel_id = channel_id, step_channel_id = first_step_param_channel_id;
    Register *_register = NULL;
    Data_Model *data_model = NULL;

    _register =  &manager->modbus_ctx._register;
    data_model = manager->modbus_ctx._register.data_model;

    //printf("init_step_parameter_pointer_register_mapping channel_id %d first_step_param_channel_id %d\n", logical_channel_id, step_channel_id);

    // 1th channel
    _register->logical_channel[logical_channel_id].step_param_pointers.nb_1_655 = &data_model[step_channel_id].holding_registers[0];

    // 2th channel
    step_channel_id++;
    _register->logical_channel[logical_channel_id].step_param_pointers.nb_656_1310 = &data_model[step_channel_id].holding_registers[0];

    // 3th channel
    step_channel_id++;
    _register->logical_channel[logical_channel_id].step_param_pointers.nb_1311_1965 = &data_model[step_channel_id].holding_registers[0];

    // 4th channel
    step_channel_id++;
    _register->logical_channel[logical_channel_id].step_param_pointers.nb_1966_2620 = &data_model[step_channel_id].holding_registers[0];

    // 5th channel
    step_channel_id++;
    _register->logical_channel[logical_channel_id].step_param_pointers.nb_2621_3275 = &data_model[step_channel_id].holding_registers[0];

    // 6th channel
    step_channel_id++;
    _register->logical_channel[logical_channel_id].step_param_pointers.nb_3276_3930 = &data_model[step_channel_id].holding_registers[0];

    // 7th channel
    step_channel_id++;
    _register->logical_channel[logical_channel_id].step_param_pointers.nb_3931_4585 = &data_model[step_channel_id].holding_registers[0];

    // 8th channel
    step_channel_id++;
    _register->logical_channel[logical_channel_id].step_param_pointers.nb_4586_5240 = &data_model[step_channel_id].holding_registers[0];

    // 9th channel
    step_channel_id++;
    _register->logical_channel[logical_channel_id].step_param_pointers.nb_5241_5895 = &data_model[step_channel_id].holding_registers[0];

    // 10th channel
    step_channel_id++;
    _register->logical_channel[logical_channel_id].step_param_pointers.nb_5896_6550 = &data_model[step_channel_id].holding_registers[0];

    // 11th channel
    step_channel_id++;
    _register->logical_channel[logical_channel_id].step_param_pointers.nb_6551_7205 = &data_model[step_channel_id].holding_registers[0];

    // 12th channel
    step_channel_id++;
    _register->logical_channel[logical_channel_id].step_param_pointers.nb_7206_7860 = &data_model[step_channel_id].holding_registers[0];
}

void init_step_parameter_register_mapping(Manager * manager, int channel_id)
{
    int logical_channel_id = channel_id;
    Register *_register = NULL;
    Data_Model *data_model = NULL;
    uint16_t *step_definition;

    _register =  &manager->modbus_ctx._register;
    data_model = manager->modbus_ctx._register.data_model;

    // by default, the step_definiton register is a pointer that points to the base address of the first step parameter channel
    step_definition = _register->logical_channel[logical_channel_id].step_param_pointers.nb_1_655;

    // step definition register
    _register->logical_channel[logical_channel_id].step_param.step_definition = step_definition;

    // step type
    _register->logical_channel[logical_channel_id].step_param.step_type = step_definition + HOLDING_STEP_TYPE_REGISTER_SUBOFFSET;

    _register->logical_channel[logical_channel_id].step_param.timing.established_time_region.word1 = step_definition + HOLDING_ESTABLISHED_TIME_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.timing.established_time_region.word2 = step_definition + HOLDING_ESTABLISHED_TIME_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.timing.established_time_region.word3 = step_definition + HOLDING_ESTABLISHED_TIME_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.timing.established_time_region.word4 = step_definition + HOLDING_ESTABLISHED_TIME_WORD4_REGISTER_SUBOFFSET;

    _register->logical_channel[logical_channel_id].step_param.timing.abolished_time_region.word1 = step_definition + HOLDING_ABOLISHED_TIME_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.timing.abolished_time_region.word2 = step_definition + HOLDING_ABOLISHED_TIME_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.timing.abolished_time_region.word3 = step_definition + HOLDING_ABOLISHED_TIME_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.timing.abolished_time_region.word4 = step_definition + HOLDING_ABOLISHED_TIME_WORD4_REGISTER_SUBOFFSET;

    // jump condiction parameters
    _register->logical_channel[logical_channel_id].step_param.jump.enable = step_definition + HOLDING_JUMP_CONDICTION_ENABLE_REGISTER_SUBOFFSET;

    // jump voltage
    _register->logical_channel[logical_channel_id].step_param.jump.voltage_region.word1 = step_definition + HOLDING_JUMP_CONDICTION_VOLTAGE_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.voltage_region.word2 = step_definition + HOLDING_JUMP_CONDICTION_VOLTAGE_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.voltage_region.word3 = step_definition + HOLDING_JUMP_CONDICTION_VOLTAGE_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.voltage_region.word4 = step_definition + HOLDING_JUMP_CONDICTION_VOLTAGE_WORD4_REGISTER_SUBOFFSET;

    // jump current
    _register->logical_channel[logical_channel_id].step_param.jump.current_region.word1 = step_definition + HOLDING_JUMP_CONDICTION_CURRENT_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.current_region.word2 = step_definition + HOLDING_JUMP_CONDICTION_CURRENT_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.current_region.word3 = step_definition + HOLDING_JUMP_CONDICTION_CURRENT_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.current_region.word4 = step_definition + HOLDING_JUMP_CONDICTION_CURRENT_WORD4_REGISTER_SUBOFFSET;

    // jump capacity
    _register->logical_channel[logical_channel_id].step_param.jump.capacity_region.word1 = step_definition + HOLDING_JUMP_CONDICTION_CAPACITY_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.capacity_region.word2 = step_definition + HOLDING_JUMP_CONDICTION_CAPACITY_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.capacity_region.word3 = step_definition + HOLDING_JUMP_CONDICTION_CAPACITY_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.capacity_region.word4 = step_definition + HOLDING_JUMP_CONDICTION_CAPACITY_WORD4_REGISTER_SUBOFFSET;

    // jump energy
    _register->logical_channel[logical_channel_id].step_param.jump.energy_region.word1 = step_definition + HOLDING_JUMP_CONDICTION_ENERGY_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.energy_region.word2 = step_definition + HOLDING_JUMP_CONDICTION_ENERGY_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.energy_region.word3 = step_definition + HOLDING_JUMP_CONDICTION_ENERGY_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.energy_region.word4 = step_definition + HOLDING_JUMP_CONDICTION_ENERGY_WORD4_REGISTER_SUBOFFSET;

    // jump time
    _register->logical_channel[logical_channel_id].step_param.jump.time_region.word1 = step_definition + HOLDING_JUMP_CONDICTION_TIME_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.time_region.word2 = step_definition + HOLDING_JUMP_CONDICTION_TIME_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.time_region.word3 = step_definition + HOLDING_JUMP_CONDICTION_TIME_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.time_region.word4 = step_definition + HOLDING_JUMP_CONDICTION_TIME_WORD4_REGISTER_SUBOFFSET;

    // step info
    _register->logical_channel[logical_channel_id].step_param.info.cycle.begin = step_definition + HOLDING_STEP_INFO_CYCLE_BEGINE_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.cycle.number = step_definition + HOLDING_STEP_INFO_CYCLE_NUMBER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.cycle.count = step_definition + HOLDING_STEP_INFO_CYCLE_COUNT_SUBOFFSET;
 
    // last logic holding time region
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_holding_time_region.word1 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_HOLDING_TIME_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_holding_time_region.word2 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_HOLDING_TIME_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_holding_time_region.word3 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_HOLDING_TIME_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_holding_time_region.word4 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_HOLDING_TIME_WORD4_REGISTER_SUBOFFSET;

    // last logic voltage region
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_voltage_region.word1 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_VOLTAGE_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_voltage_region.word2 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_VOLTAGE_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_voltage_region.word3 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_VOLTAGE_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_voltage_region.word4 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_VOLTAGE_WORD4_REGISTER_SUBOFFSET;

    // last logic current region
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_current_region.word1 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CURRENT_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_current_region.word2 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CURRENT_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_current_region.word3 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CURRENT_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_current_region.word4 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CURRENT_WORD4_REGISTER_SUBOFFSET;

    // last logic capacity region
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_capacity_region.word1 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CAPACITY_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_capacity_region.word2 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CAPACITY_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_capacity_region.word3 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CAPACITY_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_capacity_region.word4 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CAPACITY_WORD4_REGISTER_SUBOFFSET;

    // last logic energy region
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_energy_region.word1 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_ENERGY_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_energy_region.word2 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_ENERGY_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_energy_region.word3 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_ENERGY_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_energy_region.word4 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_ENERGY_WORD4_REGISTER_SUBOFFSET;
    
    // last logic temperature region
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_temperature_region.word1 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_TEMPERATURE_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_temperature_region.word2 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_TEMPERATURE_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_temperature_region.word3 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_TEMPERATURE_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_temperature_region.word4 = step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_TEMPERATURE_WORD4_REGISTER_SUBOFFSET;

    // last goto operand2 region
    _register->logical_channel[logical_channel_id].step_param.info.goingto.operand2_region.word1 = step_definition + HOLDING_STEP_INFO_GOTO_OPERAND2_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.operand2_region.word2 = step_definition + HOLDING_STEP_INFO_GOTO_OPERAND2_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.operand2_region.word3 = step_definition + HOLDING_STEP_INFO_GOTO_OPERAND2_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.operand2_region.word4 = step_definition + HOLDING_STEP_INFO_GOTO_OPERAND2_WORD4_REGISTER_SUBOFFSET;

    _register->logical_channel[logical_channel_id].step_param.info.goingto.operator = step_definition + HOLDING_STEP_INFO_GOTO_OPERATOR_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.step_index = step_definition + HOLDING_STEP_INFO_GOTO_STEP_INDEX_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.behavior = step_definition + HOLDING_STEP_INFO_GOTO_BEHAVIOR_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.next = step_definition + HOLDING_STEP_INFO_GOTO_NEXT_STEP_INDEX_REGISTER_SUBOFFSET;
    
    // common current
    _register->logical_channel[logical_channel_id].step_param.info.common.current_region.word1 = step_definition + HOLDING_STEP_INFO_COMMON_CURRENT_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.current_region.word2 = step_definition + HOLDING_STEP_INFO_COMMON_CURRENT_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.current_region.word3 = step_definition + HOLDING_STEP_INFO_COMMON_CURRENT_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.current_region.word4 = step_definition + HOLDING_STEP_INFO_COMMON_CURRENT_WORD4_REGISTER_SUBOFFSET;

    // common voltage
    _register->logical_channel[logical_channel_id].step_param.info.common.voltage_region.word1 = step_definition + HOLDING_STEP_INFO_COMMON_VOLTAGE_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.voltage_region.word2 = step_definition + HOLDING_STEP_INFO_COMMON_VOLTAGE_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.voltage_region.word3 = step_definition + HOLDING_STEP_INFO_COMMON_VOLTAGE_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.voltage_region.word4 = step_definition + HOLDING_STEP_INFO_COMMON_VOLTAGE_WORD4_REGISTER_SUBOFFSET;

    // common power
    _register->logical_channel[logical_channel_id].step_param.info.common.power_region.word1 = step_definition + HOLDING_STEP_INFO_COMMON_POWER_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.power_region.word2 = step_definition + HOLDING_STEP_INFO_COMMON_POWER_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.power_region.word3 = step_definition + HOLDING_STEP_INFO_COMMON_POWER_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.power_region.word4 = step_definition + HOLDING_STEP_INFO_COMMON_POWER_WORD4_REGISTER_SUBOFFSET;

    // common resistance
    _register->logical_channel[logical_channel_id].step_param.info.common.resistance_region.word1 = step_definition + HOLDING_STEP_INFO_COMMON_RESISTANCE_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.resistance_region.word2 = step_definition + HOLDING_STEP_INFO_COMMON_RESISTANCE_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.resistance_region.word3 = step_definition + HOLDING_STEP_INFO_COMMON_RESISTANCE_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.resistance_region.word4 = step_definition + HOLDING_STEP_INFO_COMMON_RESISTANCE_WORD4_REGISTER_SUBOFFSET;

    // samping interval
    _register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region.word1 = step_definition + HOLDING_STEP_INFO_SAMPLING_INTERVAL_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region.word2 = step_definition + HOLDING_STEP_INFO_SAMPLING_INTERVAL_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region.word3 = step_definition + HOLDING_STEP_INFO_SAMPLING_INTERVAL_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region.word4 = step_definition + HOLDING_STEP_INFO_SAMPLING_INTERVAL_WORD4_REGISTER_SUBOFFSET;

    // common status
    _register->logical_channel[logical_channel_id].step_param.info.common.status = step_definition + HOLDING_STEP_INFO_COMMON_STATUS_REGISTER_SUBOFFSET;

    // accumulated_logic_capacity
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_logic_capacity_region.word1 = step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_LOGIC_CAPACITY_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_logic_capacity_region.word2 = step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_LOGIC_CAPACITY_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_logic_capacity_region.word3 = step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_LOGIC_CAPACITY_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_logic_capacity_region.word4 = step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_LOGIC_CAPACITY_WORD4_REGISTER_SUBOFFSET;

    // accumulated_step_logic_capacity
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_step_logic_capacity_region.word1 = step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_STEP_LOGIC_CAPACITY_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_step_logic_capacity_region.word2 = step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_STEP_LOGIC_CAPACITY_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_step_logic_capacity_region.word3 = step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_STEP_LOGIC_CAPACITY_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_step_logic_capacity_region.word4 = step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_STEP_LOGIC_CAPACITY_WORD4_REGISTER_SUBOFFSET;

}

void update_step_parameter_register_mapping(Manager * manager, int channel_id, uint16_t *step_definition)
{
    int logical_channel_id = channel_id;
    Register *_register = NULL;
    Data_Model *data_model = NULL;
    uint16_t *_step_definition = step_definition;

    _register =  &manager->modbus_ctx._register;
    data_model = manager->modbus_ctx._register.data_model;

    // step definition register
    _register->logical_channel[logical_channel_id].step_param.step_definition = _step_definition;

    // step type
    _register->logical_channel[logical_channel_id].step_param.step_type = _step_definition + HOLDING_STEP_TYPE_REGISTER_SUBOFFSET;

    _register->logical_channel[logical_channel_id].step_param.timing.established_time_region.word1 = _step_definition + HOLDING_ESTABLISHED_TIME_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.timing.established_time_region.word2 = _step_definition + HOLDING_ESTABLISHED_TIME_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.timing.established_time_region.word3 = _step_definition + HOLDING_ESTABLISHED_TIME_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.timing.established_time_region.word4 = _step_definition + HOLDING_ESTABLISHED_TIME_WORD4_REGISTER_SUBOFFSET;

    _register->logical_channel[logical_channel_id].step_param.timing.abolished_time_region.word1 = _step_definition + HOLDING_ABOLISHED_TIME_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.timing.abolished_time_region.word2 = _step_definition + HOLDING_ABOLISHED_TIME_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.timing.abolished_time_region.word3 = _step_definition + HOLDING_ABOLISHED_TIME_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.timing.abolished_time_region.word4 = _step_definition + HOLDING_ABOLISHED_TIME_WORD4_REGISTER_SUBOFFSET;

    // jump condiction parameters
    _register->logical_channel[logical_channel_id].step_param.jump.enable = _step_definition + HOLDING_JUMP_CONDICTION_ENABLE_REGISTER_SUBOFFSET;

    _register->logical_channel[logical_channel_id].step_param.jump.voltage_region.word1 = _step_definition + HOLDING_JUMP_CONDICTION_VOLTAGE_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.voltage_region.word2 = _step_definition + HOLDING_JUMP_CONDICTION_VOLTAGE_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.voltage_region.word3 = _step_definition + HOLDING_JUMP_CONDICTION_VOLTAGE_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.voltage_region.word4 = _step_definition + HOLDING_JUMP_CONDICTION_VOLTAGE_WORD4_REGISTER_SUBOFFSET;
    
    _register->logical_channel[logical_channel_id].step_param.jump.current_region.word1 = _step_definition + HOLDING_JUMP_CONDICTION_CURRENT_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.current_region.word2 = _step_definition + HOLDING_JUMP_CONDICTION_CURRENT_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.current_region.word3 = _step_definition + HOLDING_JUMP_CONDICTION_CURRENT_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.current_region.word4 = _step_definition + HOLDING_JUMP_CONDICTION_CURRENT_WORD4_REGISTER_SUBOFFSET;
    
    _register->logical_channel[logical_channel_id].step_param.jump.capacity_region.word1 = _step_definition + HOLDING_JUMP_CONDICTION_CAPACITY_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.capacity_region.word2 = _step_definition + HOLDING_JUMP_CONDICTION_CAPACITY_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.capacity_region.word3 = _step_definition + HOLDING_JUMP_CONDICTION_CAPACITY_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.capacity_region.word4 = _step_definition + HOLDING_JUMP_CONDICTION_CAPACITY_WORD4_REGISTER_SUBOFFSET;
    
    _register->logical_channel[logical_channel_id].step_param.jump.energy_region.word1 = _step_definition + HOLDING_JUMP_CONDICTION_ENERGY_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.energy_region.word2 = _step_definition + HOLDING_JUMP_CONDICTION_ENERGY_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.energy_region.word3 = _step_definition + HOLDING_JUMP_CONDICTION_ENERGY_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.energy_region.word4 = _step_definition + HOLDING_JUMP_CONDICTION_ENERGY_WORD4_REGISTER_SUBOFFSET;
    
    _register->logical_channel[logical_channel_id].step_param.jump.time_region.word1 = _step_definition + HOLDING_JUMP_CONDICTION_TIME_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.time_region.word2 = _step_definition + HOLDING_JUMP_CONDICTION_TIME_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.time_region.word3 = _step_definition + HOLDING_JUMP_CONDICTION_TIME_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.jump.time_region.word4 = _step_definition + HOLDING_JUMP_CONDICTION_TIME_WORD4_REGISTER_SUBOFFSET;

    // step info
    _register->logical_channel[logical_channel_id].step_param.info.cycle.begin = _step_definition + HOLDING_STEP_INFO_CYCLE_BEGINE_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.cycle.number = _step_definition + HOLDING_STEP_INFO_CYCLE_NUMBER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.cycle.count = _step_definition + HOLDING_STEP_INFO_CYCLE_COUNT_SUBOFFSET;

    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_holding_time_region.word1 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_HOLDING_TIME_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_holding_time_region.word2 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_HOLDING_TIME_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_holding_time_region.word3 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_HOLDING_TIME_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_holding_time_region.word4 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_HOLDING_TIME_WORD4_REGISTER_SUBOFFSET;    

    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_voltage_region.word1 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_VOLTAGE_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_voltage_region.word2 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_VOLTAGE_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_voltage_region.word3 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_VOLTAGE_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_voltage_region.word4 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_VOLTAGE_WORD4_REGISTER_SUBOFFSET;
    
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_current_region.word1 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CURRENT_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_current_region.word2 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CURRENT_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_current_region.word3 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CURRENT_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_current_region.word4 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CURRENT_WORD4_REGISTER_SUBOFFSET;
    
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_capacity_region.word1 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CAPACITY_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_capacity_region.word2 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CAPACITY_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_capacity_region.word3 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CAPACITY_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_capacity_region.word4 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_CAPACITY_WORD4_REGISTER_SUBOFFSET;
    
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_energy_region.word1 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_ENERGY_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_energy_region.word2 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_ENERGY_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_energy_region.word3 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_ENERGY_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_energy_region.word4 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_ENERGY_WORD4_REGISTER_SUBOFFSET;
    
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_temperature_region.word1 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_TEMPERATURE_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_temperature_region.word2 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_TEMPERATURE_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_temperature_region.word3 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_TEMPERATURE_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_temperature_region.word4 = _step_definition + HOLDING_STEP_INFO_GOTO_LAST_LOGIC_TEMPERATURE_WORD4_REGISTER_SUBOFFSET;

    _register->logical_channel[logical_channel_id].step_param.info.goingto.operand2_region.word1 = _step_definition + HOLDING_STEP_INFO_GOTO_OPERAND2_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.operand2_region.word2 = _step_definition + HOLDING_STEP_INFO_GOTO_OPERAND2_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.operand2_region.word3 = _step_definition + HOLDING_STEP_INFO_GOTO_OPERAND2_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.operand2_region.word4 = _step_definition + HOLDING_STEP_INFO_GOTO_OPERAND2_WORD4_REGISTER_SUBOFFSET;

    _register->logical_channel[logical_channel_id].step_param.info.goingto.operator = _step_definition + HOLDING_STEP_INFO_GOTO_OPERATOR_REGISTER_SUBOFFSET;    
    _register->logical_channel[logical_channel_id].step_param.info.goingto.step_index = _step_definition + HOLDING_STEP_INFO_GOTO_STEP_INDEX_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.behavior = _step_definition + HOLDING_STEP_INFO_GOTO_BEHAVIOR_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.goingto.next = _step_definition + HOLDING_STEP_INFO_GOTO_NEXT_STEP_INDEX_REGISTER_SUBOFFSET;
    
    _register->logical_channel[logical_channel_id].step_param.info.common.current_region.word1 = _step_definition + HOLDING_STEP_INFO_COMMON_CURRENT_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.current_region.word2 = _step_definition + HOLDING_STEP_INFO_COMMON_CURRENT_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.current_region.word3 = _step_definition + HOLDING_STEP_INFO_COMMON_CURRENT_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.current_region.word4 = _step_definition + HOLDING_STEP_INFO_COMMON_CURRENT_WORD4_REGISTER_SUBOFFSET;
    
    _register->logical_channel[logical_channel_id].step_param.info.common.voltage_region.word1 = _step_definition + HOLDING_STEP_INFO_COMMON_VOLTAGE_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.voltage_region.word2 = _step_definition + HOLDING_STEP_INFO_COMMON_VOLTAGE_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.voltage_region.word3 = _step_definition + HOLDING_STEP_INFO_COMMON_VOLTAGE_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.voltage_region.word4 = _step_definition + HOLDING_STEP_INFO_COMMON_VOLTAGE_WORD4_REGISTER_SUBOFFSET;
    
    _register->logical_channel[logical_channel_id].step_param.info.common.power_region.word1 = _step_definition + HOLDING_STEP_INFO_COMMON_POWER_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.power_region.word2 = _step_definition + HOLDING_STEP_INFO_COMMON_POWER_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.power_region.word3 = _step_definition + HOLDING_STEP_INFO_COMMON_POWER_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.power_region.word4 = _step_definition + HOLDING_STEP_INFO_COMMON_POWER_WORD4_REGISTER_SUBOFFSET;

    _register->logical_channel[logical_channel_id].step_param.info.common.resistance_region.word1 = _step_definition + HOLDING_STEP_INFO_COMMON_RESISTANCE_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.resistance_region.word2 = _step_definition + HOLDING_STEP_INFO_COMMON_RESISTANCE_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.resistance_region.word3 = _step_definition + HOLDING_STEP_INFO_COMMON_RESISTANCE_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.resistance_region.word4 = _step_definition + HOLDING_STEP_INFO_COMMON_RESISTANCE_WORD4_REGISTER_SUBOFFSET;

    _register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region.word1 = _step_definition + HOLDING_STEP_INFO_SAMPLING_INTERVAL_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region.word2 = _step_definition + HOLDING_STEP_INFO_SAMPLING_INTERVAL_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region.word3 = _step_definition + HOLDING_STEP_INFO_SAMPLING_INTERVAL_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.info.common.sampling_interval_region.word4 = _step_definition + HOLDING_STEP_INFO_SAMPLING_INTERVAL_WORD4_REGISTER_SUBOFFSET;

    _register->logical_channel[logical_channel_id].step_param.info.common.status = _step_definition + HOLDING_STEP_INFO_COMMON_STATUS_REGISTER_SUBOFFSET;

    // accumulated_logic_capacity
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_logic_capacity_region.word1 = _step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_LOGIC_CAPACITY_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_logic_capacity_region.word2 = _step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_LOGIC_CAPACITY_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_logic_capacity_region.word3 = _step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_LOGIC_CAPACITY_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_logic_capacity_region.word4 = _step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_LOGIC_CAPACITY_WORD4_REGISTER_SUBOFFSET;

    // accumulated_step_logic_capacity
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_step_logic_capacity_region.word1 = _step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_STEP_LOGIC_CAPACITY_WORD1_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_step_logic_capacity_region.word2 = _step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_STEP_LOGIC_CAPACITY_WORD2_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_step_logic_capacity_region.word3 = _step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_STEP_LOGIC_CAPACITY_WORD3_REGISTER_SUBOFFSET;
    _register->logical_channel[logical_channel_id].step_param.temp.accumulated_step_logic_capacity_region.word4 = _step_definition + HOLDING_STEP_PARAM_TEMP_ACCUMULATED_STEP_LOGIC_CAPACITY_WORD4_REGISTER_SUBOFFSET;
    
}


void init_logical_channel_register_mapping(Manager * _manager, int channel_id)
{
    Manager *manager = _manager;
    int logical_channel_id = channel_id;
    Register *_register = NULL;
    Data_Model *data_model = NULL;

    _register =  &manager->modbus_ctx._register;
    data_model = manager->modbus_ctx._register.data_model;

    // mode
    _register->logical_channel[logical_channel_id].mode = &data_model[logical_channel_id].holding_registers[HOLDING_MODE_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].status = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_STATUS_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].action = &data_model[logical_channel_id].holding_registers[HOLDING_ACTION_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].binding = &data_model[logical_channel_id].holding_registers[HOLDING_BINDING_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].exception_code = &data_model[logical_channel_id].holding_registers[HOLDING_EXCEPTION_CODE_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].exception_message = &data_model[logical_channel_id].holding_registers[HOLDING_EXCEPTION_MESSAGE_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].step_pointer = &data_model[logical_channel_id].holding_registers[HOLDING_STEP_POINTER_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].step_quantity = &data_model[logical_channel_id].holding_registers[HOLDING_STEP_QUANTITY_REGISTER_OFFSET];
    
    _register->logical_channel[logical_channel_id].temp.holding_time_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_HOLDING_TIME_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.holding_time_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_HOLDING_TIME_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.holding_time_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_HOLDING_TIME_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.holding_time_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_HOLDING_TIME_WORD4_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.holding_time_reference_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_HOLDING_TIME_REFERENCE_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.holding_time_reference_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_HOLDING_TIME_REFERENCE_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.holding_time_reference_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_HOLDING_TIME_REFERENCE_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.holding_time_reference_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_HOLDING_TIME_REFERENCE_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].temp.left_boundary_time_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_LEFT_BOUNDARY_TIME_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.left_boundary_time_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_LEFT_BOUNDARY_TIME_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.left_boundary_time_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_LEFT_BOUNDARY_TIME_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.left_boundary_time_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_LEFT_BOUNDARY_TIME_WORD4_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.right_boundary_time_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_RIGHT_BOUNDARY_TIME_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.right_boundary_time_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_RIGHT_BOUNDARY_TIME_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.right_boundary_time_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_RIGHT_BOUNDARY_TIME_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.right_boundary_time_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_RIGHT_BOUNDARY_TIME_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].temp.last_logic_holding_time_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_HOLDING_TIME_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_holding_time_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_HOLDING_TIME_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_holding_time_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_HOLDING_TIME_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_holding_time_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_HOLDING_TIME_WORD4_REGISTER_OFFSET];  

    _register->logical_channel[logical_channel_id].temp.last_logic_voltage_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_VOLTAGE_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_voltage_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_VOLTAGE_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_voltage_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_VOLTAGE_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_voltage_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_VOLTAGE_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].temp.last_logic_current_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_CURRENT_WORD1_REGISTER_OFFSET];    
    _register->logical_channel[logical_channel_id].temp.last_logic_current_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_CURRENT_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_current_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_CURRENT_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_current_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_CURRENT_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].temp.last_accumulated_step_logic_capacity_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_ACCUMULATED_LOGIC_CAPACITY_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_accumulated_step_logic_capacity_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_ACCUMULATED_LOGIC_CAPACITY_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_accumulated_step_logic_capacity_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_ACCUMULATED_LOGIC_CAPACITY_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_accumulated_step_logic_capacity_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_ACCUMULATED_LOGIC_CAPACITY_WORD4_REGISTER_OFFSET];
    
    _register->logical_channel[logical_channel_id].temp.last_logic_energy_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_ENERGY_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_energy_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_ENERGY_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_energy_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_ENERGY_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_energy_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_ENERGY_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].temp.last_logic_temperature_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_TEMPERATURE_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_temperature_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_TEMPERATURE_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_temperature_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_TEMPERATURE_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_logic_temperature_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_LOGIC_TEMPERATURE_WORD4_REGISTER_OFFSET];
    
    _register->logical_channel[logical_channel_id].temp.last_sampling_record_time_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_SAMPLING_RECORD_TIME_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_sampling_record_time_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_SAMPLING_RECORD_TIME_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_sampling_record_time_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_SAMPLING_RECORD_TIME_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_sampling_record_time_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_LAST_SAMPLING_RECORD_TIME_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].temp.jump_time_reference_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_JUMP_TIME_REFERENCE_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.jump_time_reference_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_JUMP_TIME_REFERENCE_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.jump_time_reference_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_JUMP_TIME_REFERENCE_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.jump_time_reference_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_JUMP_TIME_REFERENCE_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].temp.ready_left_boundary_time_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_READY_LEFT_BOUNDARY_TIME_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.ready_left_boundary_time_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_READY_LEFT_BOUNDARY_TIME_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.ready_left_boundary_time_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_READY_LEFT_BOUNDARY_TIME_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.ready_left_boundary_time_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_READY_LEFT_BOUNDARY_TIME_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].temp.ready_right_boundary_time_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_READY_RIGHT_BOUNDARY_TIME_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.ready_right_boundary_time_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_READY_RIGHT_BOUNDARY_TIME_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.ready_right_boundary_time_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_READY_RIGHT_BOUNDARY_TIME_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.ready_right_boundary_time_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_TEMP_READY_RIGHT_BOUNDARY_TIME_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].nb_binding = &data_model[logical_channel_id].holding_registers[HOLDING_NB_BINDING_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].tolerance.common_node_voltage_tolerance_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_COMMON_NODE_VOLTAGE_TOLERANCE_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].tolerance.common_node_voltage_tolerance_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_COMMON_NODE_VOLTAGE_TOLERANCE_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].tolerance.common_node_voltage_tolerance_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_COMMON_NODE_VOLTAGE_TOLERANCE_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].tolerance.common_node_voltage_tolerance_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_COMMON_NODE_VOLTAGE_TOLERANCE_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].tolerance.constant_voltage_stage_confirmation_tolerance_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CONSTANT_VOLTAGE_STAGE_CONFIRMATION_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].tolerance.constant_voltage_stage_confirmation_tolerance_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CONSTANT_VOLTAGE_STAGE_CONFIRMATION_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].tolerance.constant_voltage_stage_confirmation_tolerance_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CONSTANT_VOLTAGE_STAGE_CONFIRMATION_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].tolerance.constant_voltage_stage_confirmation_tolerance_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CONSTANT_VOLTAGE_STAGE_CONFIRMATION_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].tolerance.constant_voltage_stage_current_drop_confirmation_tolerance_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CONSTANT_VOLTAGE_STAGE_CURRENT_DROP_CONFIRMATION_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].tolerance.constant_voltage_stage_current_drop_confirmation_tolerance_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CONSTANT_VOLTAGE_STAGE_CURRENT_DROP_CONFIRMATION_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].tolerance.constant_voltage_stage_current_drop_confirmation_tolerance_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CONSTANT_VOLTAGE_STAGE_CURRENT_DROP_CONFIRMATION_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].tolerance.constant_voltage_stage_current_drop_confirmation_tolerance_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CONSTANT_VOLTAGE_STAGE_CURRENT_DROP_CONFIRMATION_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].logic_voltage_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_PRESENT_LOGIC_VOLTAGE_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].logic_voltage_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_PRESENT_LOGIC_VOLTAGE_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].logic_voltage_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_PRESENT_LOGIC_VOLTAGE_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].logic_voltage_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_PRESENT_LOGIC_VOLTAGE_WORD4_REGISTER_OFFSET];
    
    _register->logical_channel[logical_channel_id].logic_current_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_PRESENT_LOGIC_CURRENT_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].logic_current_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_PRESENT_LOGIC_CURRENT_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].logic_current_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_PRESENT_LOGIC_CURRENT_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].logic_current_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_PRESENT_LOGIC_CURRENT_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].logic_energy_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_LOGIC_ENERGY_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].logic_energy_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_LOGIC_ENERGY_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].logic_energy_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_LOGIC_ENERGY_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].logic_energy_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_LOGIC_ENERGY_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].accumulated_logic_capacity_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_ACCUMULATED_LOGIC_CAPACITY_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].accumulated_logic_capacity_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_ACCUMULATED_LOGIC_CAPACITY_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].accumulated_logic_capacity_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_ACCUMULATED_LOGIC_CAPACITY_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].accumulated_logic_capacity_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_ACCUMULATED_LOGIC_CAPACITY_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].timing.step_turnaround_delay_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_STEP_TURNAROUND_DELAY_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].timing.step_turnaround_delay_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_STEP_TURNAROUND_DELAY_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].timing.step_turnaround_delay_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_STEP_TURNAROUND_DELAY_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].timing.step_turnaround_delay_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_STEP_TURNAROUND_DELAY_WORD4_REGISTER_OFFSET];

    _register->logical_channel[logical_channel_id].maximum_source_current_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_MAXIMUM_SOURCE_CURRENT_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].maximum_source_current_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_MAXIMUM_SOURCE_CURRENT_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].maximum_source_current_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_MAXIMUM_SOURCE_CURRENT_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].maximum_source_current_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_MAXIMUM_SOURCE_CURRENT_WORD4_REGISTER_OFFSET];
    

    // Calibration Parameters
    // calibration enable
    _register->logical_channel[logical_channel_id].calibration_param.enable = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_ENABLE_REGISTER_OFFSET];

    // current_source_level
    _register->logical_channel[logical_channel_id].calibration_param.current_source_level = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_CURRENT_SOURCE_LEVEL_REGISTER_OFFSET];

    // kb save enable
    _register->logical_channel[logical_channel_id].calibration_param.kb_save_enable = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_KB_SAVE_ENABLE_REGISTER_OFFSET];

    // calibration voltage
    _register->logical_channel[logical_channel_id].calibration_param.calibration_voltage_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_VOLTAGE_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].calibration_param.calibration_voltage_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_VOLTAGE_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].calibration_param.calibration_voltage_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_VOLTAGE_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].calibration_param.calibration_voltage_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_VOLTAGE_WORD4_REGISTER_OFFSET];
    
    // calibration current
    _register->logical_channel[logical_channel_id].calibration_param.calibration_current_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_CURRENT_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].calibration_param.calibration_current_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_CURRENT_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].calibration_param.calibration_current_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_CURRENT_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].calibration_param.calibration_current_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_CURRENT_WORD4_REGISTER_OFFSET];

    // calibration sample voltage
    _register->logical_channel[logical_channel_id].calibration_param.calibration_sample_voltage_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_SAMPLE_VOLTAGE_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].calibration_param.calibration_sample_voltage_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_SAMPLE_VOLTAGE_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].calibration_param.calibration_sample_voltage_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_SAMPLE_VOLTAGE_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].calibration_param.calibration_sample_voltage_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_SAMPLE_VOLTAGE_WORD4_REGISTER_OFFSET];

    // calibration sample current
    _register->logical_channel[logical_channel_id].calibration_param.calibration_sample_current_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_SAMPLE_CURRENT_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].calibration_param.calibration_sample_current_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_SAMPLE_CURRENT_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].calibration_param.calibration_sample_current_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_SAMPLE_CURRENT_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].calibration_param.calibration_sample_current_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALIBRATION_SAMPLE_CURRENT_WORD4_REGISTER_OFFSET];

    // last_sampling_time_region
    _register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_LAST_ACCUMULATED_LOGIC_CAPACITY_SAMPLING_TIME_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_LAST_ACCUMULATED_LOGIC_CAPACITY_SAMPLING_TIME_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_LAST_ACCUMULATED_LOGIC_CAPACITY_SAMPLING_TIME_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_LAST_ACCUMULATED_LOGIC_CAPACITY_SAMPLING_TIME_WORD4_REGISTER_OFFSET];

    // accumulated_step_logic_capacity
    _register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region.word1 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_ACCUMULATED_STEP_LOGIC_CAPACITY_WORD1_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region.word2 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_ACCUMULATED_STEP_LOGIC_CAPACITY_WORD2_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region.word3 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_ACCUMULATED_STEP_LOGIC_CAPACITY_WORD3_REGISTER_OFFSET];
    _register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region.word4 = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_ACCUMULATED_STEP_LOGIC_CAPACITY_WORD4_REGISTER_OFFSET];

    // calc_accumulated_capacity_time_interval
    _register->logical_channel[logical_channel_id].timing.calc_accumulated_capacity_time_interval = &data_model[logical_channel_id].holding_registers[HOLDING_LOGICAL_CHANNEL_CALC_ACCUMULATED_CAPACITY_TIME_INTERVAL_REGISTER_OFFSET];
}

int reset_one_logical_channel_to_prepare_state(Manager * _manager, int channel_id)
{
    Manager *manager = _manager;
    int logical_channel_id = channel_id;
    _4paired_register_region *jump_time_reference_region, *ready_left_boundary_time_region, *ready_right_boundary_time_region;
    
    Register *_register = NULL;

    _register = &manager->modbus_ctx._register;

    jump_time_reference_region = &_register->logical_channel[logical_channel_id].temp.jump_time_reference_region;

    ready_left_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.ready_left_boundary_time_region;

    ready_right_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.ready_right_boundary_time_region;

    update_4paired_register_region(jump_time_reference_region, 0);

    update_4paired_register_region(ready_left_boundary_time_region, 0);

    update_4paired_register_region(ready_right_boundary_time_region, 0);

    //*_register->logical_channel[logical_channel_id].present_logical_voltage = 0;
    //*_register->logical_channel[logical_channel_id].present_logical_current = 0;

    return 0;
}

int reset_all_logical_channels(Manager * _manager)
{
    int logical_channel_id, ret = 0;
    Manager *manager = _manager;

    for(logical_channel_id=LOGICAL_CHANNEL_1_ID; logical_channel_id<NB_LOGIC_CHANNEL; logical_channel_id++) {

        ret = reset_one_logical_channel(manager, logical_channel_id);
        if(ret < 0){
            printf("reset_all_logical_channels -> reset_one_logical_channel fail\n");
            return ret;
        }
    }
}

int reset_one_logical_channel(Manager * _manager, int channel_id)
{
    Manager *manager = _manager;
    Data_Model *data_model = NULL;
    int ret = 0, logical_channel_id = channel_id, mask, i;
    Register *_register = NULL;
    _4paired_register_region *temp_region;
    uint64_t temp, common_node_voltage_tolerance, constant_voltage_stage_confirmation_tolerance;
    uint64_t constant_voltage_stage_current_drop_confirmation_tolerance;
    uint64_t logic_voltage, logic_current, maximum_source_current;
    uint64_t last_voltage, last_current, last_capacity, last_energy;
    _4paired_register_region *last_voltage_region, *last_current_region, *last_accumulated_logic_capacity_region, *accumulated_logic_capacity_region, *last_energy_region;
    _4paired_register_region *common_node_voltage_tolerance_region;
    _4paired_register_region *constant_voltage_stage_confirmation_tolerance_region;
    _4paired_register_region *constant_voltage_stage_current_drop_confirmation_tolerance_region;
    _4paired_register_region *logic_voltage_region, *logic_current_region;
    _4paired_register_region *maximum_source_current_region;
    _4paired_register_region *last_logic_holding_time_region;
    _4paired_register_region *last_logic_temperature_region;
    _4paired_register_region *step_turnaround_delay_region, *jump_time_reference_region, *ready_left_boundary_time_region, *ready_right_boundary_time_region;
    _4paired_register_region *holding_time_region, *holding_time_reference_region, *left_boundary_time_region, *right_boundary_time_region;
    _4paired_register_region *last_accumulated_logic_capacity_sampling_time_region;

    if(!manager || logical_channel_id < LOGICAL_CHANNEL_1_ID || logical_channel_id >= NB_LOGIC_CHANNEL) {
        printf("do_formation_logic fail, incorrect parameters, channel %d\n", logical_channel_id);
        return -1;
    }

    _register = &manager->modbus_ctx._register;

    data_model = manager->modbus_ctx._register.data_model;

    // mode
    *_register->logical_channel[logical_channel_id].mode = LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_CONFIGURATION;
    *_register->logical_channel[logical_channel_id].status = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_IDEL;
    *_register->logical_channel[logical_channel_id].action = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;
    *_register->logical_channel[logical_channel_id].binding = 0;
    *_register->logical_channel[logical_channel_id].exception_code = 0;
    *_register->logical_channel[logical_channel_id].exception_message = 0;
    *_register->logical_channel[logical_channel_id].step_pointer = 0;
    *_register->logical_channel[logical_channel_id].step_quantity = 0;
    
    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    holding_time_reference_region = &_register->logical_channel[logical_channel_id].temp.holding_time_reference_region;
    left_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.left_boundary_time_region;
    right_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.right_boundary_time_region;
    update_4paired_register_region(holding_time_region, 0);
    update_4paired_register_region(holding_time_reference_region, 0);
    update_4paired_register_region(left_boundary_time_region, 0);
    update_4paired_register_region(right_boundary_time_region, 0);

    last_logic_holding_time_region = &_register->logical_channel[logical_channel_id].temp.last_logic_holding_time_region;
    update_4paired_register_region(last_logic_holding_time_region, 0);    

    last_logic_temperature_region = &_register->logical_channel[logical_channel_id].temp.last_logic_temperature_region;
    update_4paired_register_region(last_logic_temperature_region, 0);

    *_register->logical_channel[logical_channel_id].nb_binding = 0;

    last_voltage_region = &_register->logical_channel[logical_channel_id].temp.last_logic_voltage_region;
    update_4paired_register_region(last_voltage_region, 0);

    last_current_region = &_register->logical_channel[logical_channel_id].temp.last_logic_current_region;
    update_4paired_register_region(last_current_region, 0);

    last_accumulated_logic_capacity_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_step_logic_capacity_region;
    update_4paired_register_region(last_accumulated_logic_capacity_region, 0);

    last_energy_region = &_register->logical_channel[logical_channel_id].temp.last_logic_energy_region;
    update_4paired_register_region(last_energy_region, 0);

    common_node_voltage_tolerance_region = &_register->logical_channel[logical_channel_id].tolerance.common_node_voltage_tolerance_region;
    update_4paired_register_region(common_node_voltage_tolerance_region, 0);
    
    constant_voltage_stage_confirmation_tolerance_region = &_register->logical_channel[logical_channel_id].tolerance.constant_voltage_stage_confirmation_tolerance_region;
    update_4paired_register_region(constant_voltage_stage_confirmation_tolerance_region, 0);

    constant_voltage_stage_current_drop_confirmation_tolerance_region = &_register->logical_channel[logical_channel_id].tolerance.constant_voltage_stage_current_drop_confirmation_tolerance_region;
    update_4paired_register_region(constant_voltage_stage_current_drop_confirmation_tolerance_region, 0);

    logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;
    update_4paired_register_region(logic_voltage_region, 0);

    logic_current_region = &_register->logical_channel[logical_channel_id].logic_current_region;
    update_4paired_register_region(logic_current_region, 0);

    accumulated_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_logic_capacity_region;
    update_4paired_register_region(accumulated_logic_capacity_region, 0);

    maximum_source_current_region = &_register->logical_channel[logical_channel_id].maximum_source_current_region;
    update_4paired_register_region(maximum_source_current_region, 0);

    step_turnaround_delay_region = &_register->logical_channel[logical_channel_id].timing.step_turnaround_delay_region;
    update_4paired_register_region(step_turnaround_delay_region, DEFAULT_STEP_TURNAROUND_DELAY_SETTING_IN_MS);

    jump_time_reference_region = &_register->logical_channel[logical_channel_id].temp.jump_time_reference_region;
    update_4paired_register_region(jump_time_reference_region, 0);

    ready_left_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.ready_left_boundary_time_region;
    update_4paired_register_region(ready_left_boundary_time_region, 0);

    ready_right_boundary_time_region = &_register->logical_channel[logical_channel_id].temp.ready_right_boundary_time_region;
    update_4paired_register_region(ready_right_boundary_time_region, 0);

    last_accumulated_logic_capacity_sampling_time_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_logic_capacity_sampling_time_region;
    update_4paired_register_region(last_accumulated_logic_capacity_sampling_time_region, 0);

    return ret;
}

