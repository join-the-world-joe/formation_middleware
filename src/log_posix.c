
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <pthread.h>


#include "log_controller.h"

#include "log_posix.h"

#include "debug.h"

int log_posix_init(register_log_parameters *log_param)
{
    int ret = 0, log_fd = -1, existd_flag;
    char *_URI = NULL;
    register_log_parameters *_log_param = log_param;

#ifdef DATA_FLOW_DEBUG

    _log_param->out.log_fd = DEBUG_LOG1_FD;

    printf("DATA_FOW_DEBUG(LOG Begin): log_posix_init\n");
    printf("Signature: %s\n", _log_param->in.signature);
    printf("Path: %s\n", _log_param->in.path);
    printf("URI: %s\n", _log_param->out.URI);
    printf("FD = %d\n", _log_param->out.log_fd);
    printf("DATA_FOW_DEBUG(LOG End): log_posix_init\n");

    return ret;

#endif

#if 1
    _URI = _log_param->out.URI;

    ret = access(_URI, F_OK);
    if(ret < 0) {
        existd_flag = 0;
    } else {
        existd_flag = 1;
    }

    if(existd_flag == 0) {

        printf("Creating log file %s\n", _URI);

        log_fd = open(_URI, O_RDWR|O_CREAT, 0777);
        if(log_fd < 0) {
            printf("log_posix_init fail\n");
            return -1;
        }

        _log_param->out.log_fd = log_fd;
    }

    if(existd_flag == 1) {

        printf("URI: %s exists\n", _URI);

        log_fd = open(_URI, O_RDWR | O_APPEND);
        if(log_fd < 0) {
            printf("log_posix_init open %s fail\n", _URI);
            return -1;
        }

        _log_param->out.log_fd = log_fd;

    }
#endif
    
    printf("ret %d\n", ret);

    return 0;
}

int log_posix_ioctl(log_ioctl_parameters *ioctl_param)
{
    char *_msg = NULL;
    int ret = 0, _cmd, _length, _log_fd;
    pthread_mutex_t *_log_mutex = NULL;
    log_ioctl_parameters *_ioctl_param = ioctl_param;

    _log_mutex = _ioctl_param->log_message_param.temp._log_mutex;
    
    _cmd = _ioctl_param->cmd;
    
    switch(_cmd) {

        case LOG_IOCTL_CMD_LOG_MESSAGE:
             
#ifdef DATA_FLOW_DEBUG
             pthread_mutex_lock(_log_mutex);
             printf("DATA_FOW_DEBUG(LOG Begin): log_posix_ioctl, log_message\n");
             printf("URI: %s\n", _ioctl_param->log_message_param.temp._uri);
             printf("Level: %d\n", *_ioctl_param->log_message_param.in._level);
             printf("Message: %s\n", _ioctl_param->log_message_param.in._message);
             printf("DATA_FOW_DEBUG(LOG END): log_posix_ioctl, log_message\n");

             pthread_mutex_unlock(_log_mutex);
             return 0;
#endif
      
             //printf("Message: %s\n", _ioctl_param->log_message_param.in._message);

             _msg = _ioctl_param->log_message_param.in._message;
             _length = _ioctl_param->log_message_param.temp.length;
             _log_fd = _ioctl_param->log_message_param.temp.log_fd;

             pthread_mutex_lock(_log_mutex);
             
             write(_log_fd, _msg, _length);

             pthread_mutex_unlock(_log_mutex);

             return 0;
        break;

        case LOG_IOCTL_CMD_REMOVE_LOG_CONTENT:

             _log_fd = _ioctl_param->log_message_param.temp.log_fd;

             pthread_mutex_lock(_log_mutex);

             ftruncate(_log_fd, 0);

             pthread_mutex_unlock(_log_mutex);

             return 0;
        break;

        default:
             printf("log_posix_ioctl fail, unknown cmd %d\n", _cmd);
             return -1;
        break;
    }

    return ret;
}

int log_posix_deinit(unregister_log_parameters *log_param)
{
    int ret = 0, _log_fd;
    unregister_log_parameters *_log_param = log_param;

#ifdef DATA_FLOW_DEBUG
    
        printf("DATA_FOW_DEBUG(LOG Begin): log_posix_deinit\n");
        printf("Path: %s\n", _log_param->temp._path);
        printf("Signature: %s\n", _log_param->in.signature);
        printf("URI: %s\n", _log_param->temp._uri);
        printf("FD = %d\n", _log_param->temp.log_fd);
        printf("DATA_FOW_DEBUG(LOG End): log_posix_deinit\n");

        return ret;
    
#endif

    _log_fd = _log_param->temp.log_fd;

    if(_log_fd > 0) {
        printf("close log_fd %d\n", _log_fd);
        close(_log_fd);
        _log_param->temp.log_fd = -1;
    }

    return 0;
}


