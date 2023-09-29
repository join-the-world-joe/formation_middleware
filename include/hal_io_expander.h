/**
* @file     hal_io_expander.h
* @brief    hal_io_expander.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     12/11/2018
*/

#ifndef __HAL_IO_EXPANDER_H__
#define  __HAL_IO_EXPANDER_H__

int hal_io_expander_init(register_io_expander_parameters *);
int hal_io_expander_ioctl(io_expander_ioctl_parameters *);
int hal_io_expander_deinit(unregister_io_expander_parameters *);

#endif // __HAL_IO_EXPANDER_H__
