/**
* @file     register_logic.h
* @brief    register_logic.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     12/12/2018
*/

#ifndef __FORMATION_LOGIC_H__
#define __FORMATION_LOGIC_H__

int formation_start(Manager*, int);
int formation_pause(Manager*, int);
int formation_reset(Manager *_manager, int channel_id);
int formation_run(Manager*, int);
int formation_resume(Manager*, int);

// for rest
void init_REST_registers(Manager *_manager, int channel_id);
int check_REST_parameters(Manager *_manager, int channel_id);

// for CCC
void init_CCC_registers(Manager *_manager, int channel_id);
int check_CCC_parameters(Manager *_manager, int channel_id);

// for CCD
void init_CCD_registers(Manager *_manager, int channel_id);
int check_CCD_parameters(Manager *_manager, int channel_id);

// for CCCCVC
void init_CCCCVC_registers(Manager *_manager, int channel_id);
int check_CCCCVC_parameters(Manager *_manager, int channel_id);

// for CCDCVD
void init_CCDCVD_registers(Manager *_manager, int channel_id);
int check_CCDCVD_parameters(Manager *_manager, int channel_id);

// for CPC
void init_CPC_registers(Manager *_manager, int channel_id);
int check_CPC_parameters(Manager *_manager, int channel_id);

// for CPD
void init_CPD_registers(Manager *_manager, int channel_id);
int check_CPD_parameters(Manager *_manager, int channel_id);

// for CRD
void init_CRD_registers(Manager *_manager, int channel_id);
int check_CRD_parameters(Manager *_manager, int channel_id);

#endif // __FORMATION_LOGIC_H__

