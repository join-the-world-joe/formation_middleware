
#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include "log_controller.h"

#include "hal_log.h"

#include "log_posix.h"

int hal_log_init(register_log_parameters *log_param)
{
    char *_log_signature = NULL, *_path = NULL, *_uri = NULL;
    int ret = 0, length_signature, length_path, length_uri;
    register_log_parameters *_log_param = log_param;

    if(!_log_param) {
        printf("hal_log_init fail, incorrect parameters\n");
        return -1;
    }

    _uri = _log_param->out.URI;
    _path = _log_param->in.path;
    _log_signature = _log_param->in.signature;

    if(!_log_signature) {
        printf("hal_log_init fail, incorrect signature!\n");
        return -1;
    }

    length_signature = strlen(_log_signature);
    if(((length_signature==0) || (length_signature>MAX_LENGTH_OF_LOG_SIGNATURE-1))) {
        printf("hal_log_init fail, incorrect length %d\n", length_signature);
        return -1;
    }
    
    length_path = strlen(_path);
    if(((length_path==0) || (length_path>=MAX_LENGTH_OF_LOG_PATH_SIGNATURE))) {
        printf("hal_log_init fail, incorrect length %d\n", length_path);
        return -1;
    }

    memset(_uri, 0, MAX_LENGTH_OF_LOG_URI_SIGNATURE);
    if(_path[length_path-1] == '/') {
        strcpy(_uri, _path);
        strcat(_uri, _log_signature);
    } else {
        strcpy(_uri, _path);
        strcat(_uri, "/");
        strcat(_uri, _log_signature);
    }

    ret = log_posix_init(_log_param);
    if(ret < 0) {
        printf("hal_log_init->log_posix_init fail\n");
        return -1;
    }

    return ret;
}

int hal_log_ioctl(log_ioctl_parameters *log_ioctl_param)
{
    int ret = 0, _cmd, _length;
    log_ioctl_parameters *_ioctl_param = log_ioctl_param;

    if(!_ioctl_param) {
        printf("hal_log_ioctl fail, incorrect parameters\n");
        return -1;
    }

    _cmd = _ioctl_param->cmd;

    switch(_cmd) {

        case LOG_IOCTL_CMD_LOG_MESSAGE:
             if(!_ioctl_param->log_message_param.in._level) {
                 printf("hal_log_ioctl fail, incorrect parameter\n");
                 return -1;
             }

             if(!_ioctl_param->log_message_param.in._message) {
                 printf("hal_log_ioctl fail, incorrect parameter\n");
                 return -1;
             }

             _length = strlen(_ioctl_param->log_message_param.in._message);
             if(_length > MAX_LENGTH_OF_LOG_MESSAGE) {
                 printf("hal_log_ioctl fail, incorrect length of message\n");
                 return -1;
             }

             _ioctl_param->log_message_param.temp.length = _length;

             ret = log_posix_ioctl(_ioctl_param);
             if(ret < 0) {
                 printf("log_posix_ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        case LOG_IOCTL_CMD_REMOVE_LOG_CONTENT:

             ret = log_posix_ioctl(_ioctl_param);
             if(ret < 0) {
                 printf("log_posix_ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        default:
             printf("hal_log_ioctl fail, unknown cmd %d\n", _cmd);
             return -1;
        break;
    }

    return ret;
}


int hal_log_deinit(unregister_log_parameters *log_param)
{
    int ret = 0;
    char *_uri = NULL, *_path = NULL;
    int _log_fd = 0;
    unregister_log_parameters *_log_param = log_param;

    if(!_log_param) {
        printf("hal_log_deinit fail, incorrect parameters\n");
        return -1;
    }

    _log_fd = _log_param->temp.log_fd;
    if(_log_fd < 0) {
        printf("hal_log_deinit fail, incorrect log_fd\n");
        return -1;
    }

    _path = _log_param->temp._path;
    if(!_path) {
        printf("hal_log_deinit fail, incorrect Path\n");
        return -1;
    }

    _uri = _log_param->temp._uri;
    if(!_uri) {
        printf("hal_log_deinit fail, incorrect URI\n");
        return -1;
    }

    ret = log_posix_deinit(_log_param);
    if(ret < 0) {
        printf("hal_log_deinit->log_posix_deinit fail\n");
        return -1;
    }

    return ret;
}


