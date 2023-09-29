/**
* @file     channel.h
* @brief    channel
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     10/10/2018
*/

#ifndef __LOGICAL_CHANNEL_H__
#define __LOGICAL_CHANNEL_H__

#define PHYSICAL_CHANNEL_01_UNIT_IDENTIFIER 0
#define PHYSICAL_CHANNEL_02_UNIT_IDENTIFIER 1
#define PHYSICAL_CHANNEL_03_UNIT_IDENTIFIER 2
#define PHYSICAL_CHANNEL_04_UNIT_IDENTIFIER 3
#define PHYSICAL_CHANNEL_05_UNIT_IDENTIFIER 4
#define PHYSICAL_CHANNEL_06_UNIT_IDENTIFIER 5
#define PHYSICAL_CHANNEL_07_UNIT_IDENTIFIER 6
#define PHYSICAL_CHANNEL_08_UNIT_IDENTIFIER 7

int reset_one_logical_channel(Manager * _manager, int channel_id);
void reset_step_parameter_register(Manager * _manager, int channel_id);

int check_if_exception(Manager *manager, int logical_channel_id);
int check_if_meet_jump_condiction(Manager *manager, int logical_channel_id);
int check_if_do_channel_regulation(Manager *manager, int logical_channel_id);

int calculate_and_update_logical_voltage_and_current(Manager *manager, int channel_id);

void init_step_parameter_pointer_register_mapping(Manager * manager, int channel_id, int first_step_param_channel_id);
void init_step_parameter_register_mapping(Manager * manager, int channel_id);
void init_logical_channel_register_mapping(Manager * _manager, int channel_id);
void update_step_parameter_register_mapping(Manager * manager, int channel_id, uint16_t *step_definition);

int reset_all_logical_channels(Manager * _manager);
int reset_one_logical_channel_to_prepare_state(Manager * _manager, int channel_id);


#endif  //__LOGICAL_CHANNEL_H__

