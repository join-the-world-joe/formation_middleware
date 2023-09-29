/**
* @file     recovery_logic.h
* @brief    recovery_logic.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     02/21/2018
*/

#ifndef __RECOVERY_LOGIC_H__
#define __RECOVERY_LOGIC_H__

#define RECOVERY_FILE_NAME "/home/pi/holding_registers.dat"

int recovery_logic_reset(Manager *_manager, int channel_id);
int do_save_all_logical_channel_holding_registers(Manager *_manager, char *_file_name);

int do_one_logical_channel_recovery(Manager *_manager, int channel_id);

#endif // __RECOVERY_LOGIC_H__
