
#ifndef __LOG_POSIX_H__
#define __LOG_POSIX_H__

#define DEBUG_LOG1_FD 1

int log_posix_init(register_log_parameters *);
int log_posix_ioctl(log_ioctl_parameters *);
int log_posix_deinit(unregister_log_parameters *);

#endif // __LOG_POSIX_H__
