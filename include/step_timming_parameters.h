
/**
* @file     date.h
* @brief    date.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     21/12/2018
*/


#ifndef __STEP_TIMMING_PARAMETERS_H__
#define __STEP_TIMMING_PARAMETERS_H__

void update_step_established_time(Manager *manager, int channel_id);
void update_step_abolished_time(Manager *manager, int channel_id);
void update_holding_time_reference(Manager *manager, int channel_id);
void update_holding_time_left_boundary_now(Manager *manager, int channel_id);
void update_right_boundary_and_holding_time(Manager *manager, int channel_id);

#endif // __STEP_TIMMING_PARAMETERS_H__

