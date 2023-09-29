/**
* @file     misc.h
* @brief    misc.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     01/15/2018
*/

#ifndef __MISC_H__
#define __MISC_H__


#define PHYSICAL_CHANNEL_KB_BASE_OFFSET_BYTE                   56
#define PHYSICAL_CHANNEL_KB_VU_K_SUBOFFSET_BYTE                0
#define PHYSICAL_CHANNEL_KB_VU_B_SUBOFFSET_BYTE                2
#define PHYSICAL_CHANNEL_KB_BV_K_SUBOFFSET_BYTE                4
#define PHYSICAL_CHANNEL_KB_BV_B_SUBOFFSET_BYTE                6

#define PHYSICAL_CHANNEL_KB_C_VI_K_LEVEL_1_SUBOFFSET_BYTE       8
#define PHYSICAL_CHANNEL_KB_C_VI_B_LEVEL_1_SUBOFFSET_BYTE       10
#define PHYSICAL_CHANNEL_KB_C_VI_K_LEVEL_2_SUBOFFSET_BYTE       12
#define PHYSICAL_CHANNEL_KB_C_VI_B_LEVEL_2_SUBOFFSET_BYTE       14
#define PHYSICAL_CHANNEL_KB_C_VI_K_LEVEL_3_SUBOFFSET_BYTE       16
#define PHYSICAL_CHANNEL_KB_C_VI_B_LEVEL_3_SUBOFFSET_BYTE       18

#define PHYSICAL_CHANNEL_KB_C_CV_K_LEVEL_1_SUBOFFSET_BYTE       20
#define PHYSICAL_CHANNEL_KB_C_CV_B_LEVEL_1_SUBOFFSET_BYTE       22
#define PHYSICAL_CHANNEL_KB_C_CV_K_LEVEL_2_SUBOFFSET_BYTE       24
#define PHYSICAL_CHANNEL_KB_C_CV_B_LEVEL_2_SUBOFFSET_BYTE       26
#define PHYSICAL_CHANNEL_KB_C_CV_K_LEVEL_3_SUBOFFSET_BYTE       28
#define PHYSICAL_CHANNEL_KB_C_CV_B_LEVEL_3_SUBOFFSET_BYTE       30

#define PHYSICAL_CHANNEL_KB_D_VI_K_LEVEL_1_SUBOFFSET_BYTE       32
#define PHYSICAL_CHANNEL_KB_D_VI_B_LEVEL_1_SUBOFFSET_BYTE       34
#define PHYSICAL_CHANNEL_KB_D_VI_K_LEVEL_2_SUBOFFSET_BYTE       36
#define PHYSICAL_CHANNEL_KB_D_VI_B_LEVEL_2_SUBOFFSET_BYTE       38
#define PHYSICAL_CHANNEL_KB_D_VI_K_LEVEL_3_SUBOFFSET_BYTE       40
#define PHYSICAL_CHANNEL_KB_D_VI_B_LEVEL_3_SUBOFFSET_BYTE       42

#define PHYSICAL_CHANNEL_KB_D_CV_K_LEVEL_1_SUBOFFSET_BYTE       44
#define PHYSICAL_CHANNEL_KB_D_CV_B_LEVEL_1_SUBOFFSET_BYTE       46
#define PHYSICAL_CHANNEL_KB_D_CV_K_LEVEL_2_SUBOFFSET_BYTE       48
#define PHYSICAL_CHANNEL_KB_D_CV_B_LEVEL_2_SUBOFFSET_BYTE       50
#define PHYSICAL_CHANNEL_KB_D_CV_K_LEVEL_3_SUBOFFSET_BYTE       52
#define PHYSICAL_CHANNEL_KB_D_CV_B_LEVEL_3_SUBOFFSET_BYTE       54

#define PHYSICAL_CHANNEL8_EXTRA_DEVICE_HOST_SUBOFFSET_BYTE      56

#define INFO_TYPE_VU_K                                           0
#define INFO_TYPE_VU_B                                           1
#define INFO_TYPE_BV_K                                           2
#define INFO_TYPE_BV_B                                           3

#define INFO_TYPE_C_VI_K_LEVEL_1                                 4
#define INFO_TYPE_C_VI_B_LEVEL_1                                 5
#define INFO_TYPE_C_VI_K_LEVEL_2                                 6
#define INFO_TYPE_C_VI_B_LEVEL_2                                 7
#define INFO_TYPE_C_VI_K_LEVEL_3                                 8
#define INFO_TYPE_C_VI_B_LEVEL_3                                 9

#define INFO_TYPE_D_VI_K_LEVEL_1                                 10
#define INFO_TYPE_D_VI_B_LEVEL_1                                 11
#define INFO_TYPE_D_VI_K_LEVEL_2                                 12
#define INFO_TYPE_D_VI_B_LEVEL_2                                 13
#define INFO_TYPE_D_VI_K_LEVEL_3                                 14
#define INFO_TYPE_D_VI_B_LEVEL_3                                 15

#define INFO_TYPE_C_CV_K_LEVEL_1                                 16
#define INFO_TYPE_C_CV_B_LEVEL_1                                 17
#define INFO_TYPE_C_CV_K_LEVEL_2                                 18
#define INFO_TYPE_C_CV_B_LEVEL_2                                 19
#define INFO_TYPE_C_CV_K_LEVEL_3                                 20
#define INFO_TYPE_C_CV_B_LEVEL_3                                 21

#define INFO_TYPE_D_CV_K_LEVEL_1                                 22
#define INFO_TYPE_D_CV_B_LEVEL_1                                 23
#define INFO_TYPE_D_CV_K_LEVEL_2                                 24
#define INFO_TYPE_D_CV_B_LEVEL_2                                 25
#define INFO_TYPE_D_CV_K_LEVEL_3                                 26
#define INFO_TYPE_D_CV_B_LEVEL_3                                 27


#define INFO_TYPE_DEVICE_HOST                                    28

#define INFO_TYPE_KB_COUNT                                       28

#define HOST_IN_OLED_CHAR_ROW_IDX                                1



int jc_register_enable(Manager *_manager, int channel_id);
int jc_register_disable(Manager *_manager, int channel_id);

int bell_enable(Manager *_manager);
int bell_disable(Manager *_manager);

int led_ready_enable(Manager *_manager);
int led_ready_disable(Manager *_manager);

int led_run_enable(Manager *_manager);
int led_run_disable(Manager *_manager);

int led_error_enable(Manager *_manager);
int led_error_disable(Manager *_manager);

int clear_one_logical_channel_log(Manager * _manager, int channel_id);
int record_one_logical_channel_sampling(Manager * _manager, int channel_id);
int check_record_one_logical_channel_currve_log(Manager * _manager, int channel_id);

void init_system_log(Manager *manager);
void init_sampling_log(Manager *_manager);


int bind_physical_channel_and_update_info(Manager *_manager, int _logical_channel_id, int _physical_channel_id);

void clear_holding_registers(uint16_t *start_address, uint16_t _quantity);

void uint16_t_to_uint8_t(uint16_t conver_value, uint8_t *byte_array);
void uint32_t_to_uint8_t(uint32_t conver_value, uint8_t *byte_array);

int write_info_to_file(uint8_t *byte_arry, uint16_t byte_count, uint16_t byte_offset_count);
int read_info_from_file(uint8_t *byte_arry, uint16_t byte_count, uint16_t byte_offset_count);

int save_info_to_ROM(Manager *_manager, int channel_id, int _KB_type);
int reload_info_from_ROM(Manager *_manager, int channel_id, int _KB_type);

int creat_default_info_file();
int save_one_physical_channel_KB_value(Manager *_manager, int channel_id, uint16_t calibration_step_type);
int reload_one_physical_channel_KB_value(Manager *_manager, int channel_id);
int check_and_update_KB(Manager *_manager, int channel_id);

int save_and_update_IP(Manager *_manager);
int reload_and_update_IP(Manager *_manager);
int get_IP_and_update_oled(Manager *_manager);
int check_and_update_IP(Manager *_manager);



void check_and_update_tower_light(Manager *_manager);



#endif // __MISC_H__

