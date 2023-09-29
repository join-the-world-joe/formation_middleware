/**
* @file     calibration_logic.h
* @brief    calibration_logic.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     02/13/2019
*/

#ifndef __CALIBRATION_LOGIC_H__
#define __CALIBRATION_LOGIC_H__

int calibration_run(Manager *_manager, int channel_id);
int calibration_start(Manager *_manager, int channel_id);
int calibration_reset(Manager *_manager, int channel_id);
int check_calibration_parameters(Manager *_manager, int channel_id);
int do_ccc_calibration_logic(Manager *_manager, int channel_id);
int do_ccd_calibration_logic(Manager *_manager, int channel_id);
int do_cvad_calibration_logic(Manager *_manager, int channel_id);
int do_cvda_calibration_logic(Manager *_manager, int channel_id);


#endif // __CALIBRATION_LOGIC_H__

