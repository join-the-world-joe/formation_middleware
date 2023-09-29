/**
* @file     physical_channel.h
* @brief    physical_channel.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     21/12/2018
*/

#ifndef __PHYSICAL_CHANNEL_H__
#define __PHYSICAL_CHANNEL_H__

#define MAX_PREPARE_STATE_CHECK 100

#define PHYSICAL_CHANNEL_BASE_VOLTAGE_MV            2500
#define PHYSICAL_CHANNEL_VI_FULLSCALE               4000
#define PHYSICAL_CHANNEL_CV_FULLSCALE               4000
#define PHYSICAL_CHANNEL_CV_HALFSCALE_MV            2000
#define PHYSICAL_CHANNEL_VI_HALFSCALE_MV            2000

#define PHYSICAL_CHANNEL_VI_MAX_IN_MV_UNIT                  5000
#define PHYSICAL_CHANNEL_VU_MAX_IN_MV_UNIT                  5000

int init_physical_channel(Manager *manager, int channel_id);
void init_physical_channel_register_mapping(Manager * manager, int channel_id);
void update_bv_and_cv(Manager *manager);

int check_physical_channels_and_update_info(Manager *manager, int logical_channel_id);

int check_voltage_of_all_physical_channels(Manager *manager, int channel_id);
int check_if_voltage_difference_of_all_physical_channels_in_tolerance(Manager *manager, int channel_id);
int check_if_all_physical_channels_ready(Manager *manager, int channel_id);
int reset_bound_physical_channels(Manager *manager, int channel_id);

void update_bound_physical_channel_status_to_running(Manager *_manager, int channel_id);

int reset_all_bound_physical_channels_to_prepare_state(Manager *_manager, int channel_id);
int check_if_bound_physical_channels_in_prepare_state(Manager *_manager, int channel_id);

int check_if_do_channel_regulation(Manager *_manager, int channel_id);

void reset_physical_channel_to_default_state(Manager *_manager, int channel_id);
void reset_all_physical_channel_to_default_state(Manager *_manager);
int update_sample_voltage_and_current(Manager *_manager);
int update_bound_physical_channel_vu_registers(Manager *_manager, int _channel_id, uint64_t _logic_voltage);
int update_bound_physical_channel_current_source_and_vi_registers(Manager *_manager, int _channel_id, uint64_t _logic_current);

int update_bound_physical_channel_jc_registers(Manager *_manager, int _channel_id, int _operation);

#endif // __PHYSICAL_CHANNEL_H__

