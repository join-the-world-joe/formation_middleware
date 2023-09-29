
#ifndef __HAL_LOG_H__
#define __HAL_LOG_H__

int hal_log_init(register_log_parameters*);
int hal_log_ioctl(log_ioctl_parameters *);
int hal_log_deinit(unregister_log_parameters *);

#endif // __HAL_LOG_H__
