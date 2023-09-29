/**
* @file     hal_adc.h
* @brief    hal_adc.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     15/11/2018
*/

#ifndef __HAL_ADC_H__
#define __HAL_ADC_H__

int hal_adc_init(register_adc_parameters *);
int hal_adc_ioctl(adc_ioctl_parameters *);
int hal_adc_deinit(unregister_adc_parameters *);


#endif // __HAL_ADC_H__


