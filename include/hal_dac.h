/**
* @file     hal_da.h
* @brief    hal_da.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     13/11/2018
*/

#ifndef __HAL_DA_H__
#define __HAL_DA_H__

int hal_dac_init(register_dac_parameters *);
int hal_dac_ioctl(dac_ioctl_parameters *);
int hal_dac_deinit(unregister_dac_parameters *);

#endif // __HAL_DA_H__