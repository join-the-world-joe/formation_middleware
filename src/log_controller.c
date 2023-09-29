
#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include "log_controller.h"
#include "hal_log.h"

/*
Resource Manager: log_controller.c
Hal Level: hal_log.c
Platform Level: log_posix.c
*/

void log_controller_list_resources(log_controller_context *log_controller_ctx)
{
    int index;
    int _nb_logs;
    log__context *_log_ctx = NULL;
    log_controller_context *_controller_ctx = log_controller_ctx;

    if(!_controller_ctx) {
        printf("gpio_controller_list_resources fail, incorrect parameters\n");
        return ;
    }

    _log_ctx = _controller_ctx->log_ctx;
    _nb_logs = _controller_ctx->nb_logs;

    printf("***********************************LOG Resources***********************************\n");
    for(index=0; index<_nb_logs; index++) {
        if(_log_ctx[index].in_use) {
            printf("Index: %d\n", index);
            printf("Path: %s\n", _log_ctx[index].path);
            printf("Signature: %s\n", _log_ctx[index].signature);
            printf("URI: %s\n", _log_ctx[index].URI);
            printf("FD: %d\n", _log_ctx[index].log_fd);
        }
    }
    printf("************************************************************************************\n");
}

int log_controller_init(log_controller_context *log_controller_ctx)
{
    int ret = 0, _nb_logs = NB_LOGS, i, j;
    log__context *_log_ctx = NULL;
    log_controller_context *_controller_ctx = log_controller_ctx;

    if(!_controller_ctx) {
        printf("log_controller_init fail, incorrect parameters\n");
        return -1;
    }

    _controller_ctx->count = 0;
    _controller_ctx->nb_logs = _nb_logs;

    _log_ctx = _controller_ctx->log_ctx;
    
    // we'd better initialize all the important variables here    
    for(i=0; i<_nb_logs; i++) {
        _log_ctx[i].index = i;
        _log_ctx[i].in_use = 0;
        _log_ctx[i].log_fd = -1;
        pthread_mutex_init(&_log_ctx[i].log_mutex , NULL);
        memset(_log_ctx[i].URI, 0, MAX_LENGTH_OF_LOG_URI_SIGNATURE);
        memset(_log_ctx[i].path, 0, MAX_LENGTH_OF_LOG_PATH_SIGNATURE);
        memset(_log_ctx[i].signature, 0, MAX_LENGTH_OF_LOG_SIGNATURE);
    }

    _controller_ctx->ops.init = log_controller_init;
    _controller_ctx->ops.register_context = log_controller_register_context;
    _controller_ctx->ops.unregister_context = log_controller_unregister_context;
    _controller_ctx->ops.list_resources = log_controller_list_resources;
    _controller_ctx->ops.ioctl = log_controller_ioctl;
    _controller_ctx->ops.deinit = log_controller_deinit;

    _controller_ctx->log_ops.init = hal_log_init;
    _controller_ctx->log_ops.ioctl = hal_log_ioctl;
    _controller_ctx->log_ops.deinit = hal_log_deinit;

    return ret;
}

void log_controller_deinit(log_controller_context *log_controller_ctx)
{
    int ret = 0, index;
    int _nb_logs;
    log__context *_log_ctx = NULL;
    log__ops *_log_ops = NULL;
    log_controller_context *_controller_ctx = log_controller_ctx;
    unregister_log_parameters log_param;

    if(!_controller_ctx) {
        printf("gpio_controller_deinit fail, incorrect parameters\n");
        return ;
    }

    _log_ops = &_controller_ctx->log_ops;
    _log_ctx = _controller_ctx->log_ctx; 
    _nb_logs = _controller_ctx->nb_logs;

    for(index=0; index<_nb_logs; index++) {
        if(_log_ctx[index].in_use) {
            memset(log_param.in.signature, 0, MAX_LENGTH_OF_LOG_SIGNATURE);
            strcpy(log_param.in.signature, _log_ctx[index].signature);
            ret = _controller_ctx->ops.unregister_context(_controller_ctx, &log_param);
            if(ret < 0) {
                printf("log_controller_deinit->unregister_log fail, port %s\n", _log_ctx[index].signature);
                return ;
            }
        }
    }

    return ;
}

int log_controller_register_context(log_controller_context *log_controller_ctx, register_log_parameters *log_param)
{
    int ret = 0, i, done;
    int _nb_logs;
    char *_log_signatrue = NULL, *_log_path = NULL;
    log__context *_log_ctx = NULL;
    log__ops *_log_ops = NULL;
    register_log_parameters *_log_param = log_param;
    log_controller_context *_controller_ctx = log_controller_ctx;

    if(!_controller_ctx || !_log_param) {
        printf("log_controller_register_log fail, incorrect parameters\n");
        return -1;
    }

    _nb_logs = _controller_ctx->nb_logs;
    _log_ctx = _controller_ctx->log_ctx;
    _log_ops = &_controller_ctx->log_ops;
    _log_path = _log_param->in.path;
    _log_signatrue = _log_param->in.signature;

    if(_controller_ctx->count + 1 > _nb_logs) {
        printf("log context in controller is full, there might be something wrong\n");
        return -1;
    }

    ret = _log_ops->init(_log_param);
    if(ret < 0) {
        printf("log_controller_register_log->io_ops->init fail\n");
        return -1;
    }

    // first, find a free context 
    for(i=0, done=0; i<_nb_logs; i++) {

        if(_log_ctx[i].in_use == 0) {

            // record input parameters
            strcpy(_log_ctx[i].path, _log_path);
            strcpy(_log_ctx[i].signature, _log_signatrue);

            // record output parameters
            _log_ctx[i].log_fd = _log_param->out.log_fd;
            strcpy(_log_ctx[i].URI, _log_param->out.URI);

            // change the status of this log_context
            _log_ctx[i].in_use = 1;

            done = 1; // all the related contents are recoreded in log_context

            break;
        }
    }

    if(!done) {
        printf("log_controller_register fail, can't record log %s\n", _log_signatrue);
        return -1;
    }

    _controller_ctx->count++; /* if done, increase the object counter */

    return ret;
}

int log_controller_unregister_context(log_controller_context *log_controller_ctx, unregister_log_parameters *log_param)
{
    int ret = 0, index, done, i, same, find;
    int _nb_logs;
    char *_log_signatrue = NULL;
    log__context *_log_ctx = NULL;
    log__ops *_log_ops = NULL;
    unregister_log_parameters *_log_param = log_param;
    log_controller_context *_controller_ctx = log_controller_ctx;

    if(!_controller_ctx || !_log_param) {
        printf("log_controller_unregister_log fail, incorrect parameters\n");
        return -1;
    }

    _nb_logs = _controller_ctx->nb_logs;
    _log_ctx = _controller_ctx->log_ctx;
    _log_ops = &_controller_ctx->log_ops;
    _log_signatrue = _log_param->in.signature;

    if((_controller_ctx->count - 1) < 0) {
        printf("log context in controller is empty, there might be something wrong\n");
        return -1;
    }

    // check if we can find this signature in log_context
    for(index=0, find=0; index<_nb_logs; index++) {

        same = !strcmp(_log_ctx[index].signature, _log_signatrue);
        if(same) {

            _log_param->temp._path = _log_ctx[index].path;
            _log_param->temp.log_fd = _log_ctx[index].log_fd;
            _log_param->temp._uri = _log_ctx[index].URI;

            find = 1;

            break;
        }
    }

    if(!find) {
        printf("signature %s doesn't match any recorded signature in log_context of log_controller\n", _log_signatrue);
        return -1;
    }

    ret = _log_ops->deinit(_log_param);
    if(ret < 0) {
        printf("gpio_controller_unregister_gpio_port->io_ops->deinit fail\n");
        return -1;
    }

    _log_ctx[index].in_use = 0;

    return ret;
}

int log_controller_ioctl(log_controller_context *log_controller_ctx, int cmd, log_controller_ioctl_parameters *log_controller_ioctl_param)
{
    int ret = 0, same, done, index;
    char *_log_signature = NULL;
    int _cmd = cmd;    
    int _nb_logs;
    log__ops *_log_ops = NULL;
    log__context *_log_ctx = NULL;
    log_ioctl_parameters log_ioctl_param;
    log_controller_context *_controller_ctx = log_controller_ctx;
    log_controller_ioctl_parameters *_controller_ioctl_param = log_controller_ioctl_param;
    
    if(!_controller_ctx || !_controller_ioctl_param || _cmd < LOG_CONTROLLER_IOCTL_CMD_NB_BEGIN || _cmd > LOG_CONTROLLER_IOCTL_CMD_NB_END) {
        printf("log_controller_ioctl incorrect parameters\n");
        return -1;
    }

    _nb_logs = _controller_ctx->nb_logs;
    _log_ops = &_controller_ctx->log_ops;
    
    switch(_cmd) {

        case LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX:
             _log_ctx = _controller_ctx->log_ctx;
             _log_signature = _controller_ioctl_param->acquire_log_ctx_param.in.signature;         
             for(index=0, done=0; index<_nb_logs; index++) {
                 same = !strcmp(_log_signature, _log_ctx[index].signature);
                 if(same) {
    
                    _controller_ioctl_param->temp.index = index;
    
                    _controller_ioctl_param->temp.log_ctx = &_log_ctx[index];   
    
                    done = 1;
    
                    break;
                 }
             }
    
             if(done == 0) {
                 printf("log_controller_ioctl->acquire_log_context fail\n");
                 return -1;
             }

             return 0; // acquire gpio context done!!!
        break;

        case LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE:
             index = _controller_ioctl_param->temp.index;
             if(index < 0 || index >= _nb_logs) {
                 printf("log_controller_ioctl log message fail, incorrect index = %d\n", index);
                 return -1;
             }

             _log_ctx = _controller_ioctl_param->temp.log_ctx; 
             if(_log_ctx != &_controller_ctx->log_ctx[index]) {
                 printf("log_controller_ioctl, incorrect ioctl parameters\n");
                 return -1;
             }
             
             log_ioctl_param.cmd = LOG_IOCTL_CMD_LOG_MESSAGE;
             log_ioctl_param.log_message_param.temp._path = _log_ctx->path;
             log_ioctl_param.log_message_param.temp._signature = _log_ctx->signature;
             log_ioctl_param.log_message_param.temp._uri = _log_ctx->URI;
             log_ioctl_param.log_message_param.temp.log_fd = _log_ctx->log_fd;
             log_ioctl_param.log_message_param.temp._log_mutex = &_log_ctx->log_mutex;
             log_ioctl_param.log_message_param.in._level = &_controller_ioctl_param->log_message_param.in.level;
             log_ioctl_param.log_message_param.in._message = _controller_ioctl_param->log_message_param.in.message;
             ret = _log_ops->ioctl(&log_ioctl_param);
             if(ret < 0) {
                 printf("log_controller_ioctl->log_ops->ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        case LOG_CONTROLLER_IOCTL_CMD_REMOVE_LOG_CONTENT:
             index = _controller_ioctl_param->temp.index;
             if(index < 0 || index >= _nb_logs) {
                 printf("log_controller_ioctl log message fail, incorrect index = %d\n", index);
                 return -1;
             }

             _log_ctx = _controller_ioctl_param->temp.log_ctx; 
             if(_log_ctx != &_controller_ctx->log_ctx[index]) {
                 printf("log_controller_ioctl, incorrect ioctl parameters\n");
                 return -1;
             }

             log_ioctl_param.cmd = LOG_IOCTL_CMD_REMOVE_LOG_CONTENT;
             log_ioctl_param.log_message_param.temp._path = _log_ctx->path;
             log_ioctl_param.log_message_param.temp._signature = _log_ctx->signature;
             log_ioctl_param.log_message_param.temp._uri = _log_ctx->URI;
             log_ioctl_param.log_message_param.temp.log_fd = _log_ctx->log_fd;
             log_ioctl_param.log_message_param.temp._log_mutex = &_log_ctx->log_mutex;
             ret = _log_ops->ioctl(&log_ioctl_param);
             if(ret < 0) {
                 printf("log_controller_ioctl->log_ops->ioctl fail\n");
                 return -1;
             }

             return 0;
        break;

        default:
             printf("log_controller_ioctl, unknown cmd %d\n", _cmd);
             return -1;
        break;
    }

    return ret;
}

#if 1
int log_controller_testApp(void)
{
    int ret = 0, cmd;
    log_controller_context log_controller_ctx;
    register_log_parameters register_log_param;
    unregister_log_parameters unregister_log_param;
    log_controller_ioctl_parameters log_controller_ioctl_param;

    ret = log_controller_init(&log_controller_ctx);
    if(ret < 0) {
        printf("log_controller_init fail\n");
        return -1;
    }

    strcpy(register_log_param.in.path, "/mnt/hgfs/Projects/appLibC");
    strcpy(register_log_param.in.signature, "system.log");
    ret = log_controller_ctx.ops.register_context(&log_controller_ctx, &register_log_param);
    if(ret < 0) {
        printf("register_log fail\n");
        return -1;
    }

    log_controller_ctx.ops.list_resources(&log_controller_ctx);

    cmd = LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX;
    strcpy(log_controller_ioctl_param.acquire_log_ctx_param.in.signature, "system.log");
    ret = log_controller_ctx.ops.ioctl(&log_controller_ctx, cmd, &log_controller_ioctl_param);
    if(ret < 0) {
        printf("acquire log context fail\n");
        return -1;
    }

#if 0
    cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
    strcpy(log_controller_ioctl_param.log_message_param.in.message, "Hello, World!!!");
    log_controller_ioctl_param.log_message_param.in.level = LOG_LEVEL_DEBUG;
    ret = log_controller_ctx.ops.ioctl(&log_controller_ctx, cmd, &log_controller_ioctl_param);
    if(ret < 0) {
        printf("log message fail\n");
        return -1;
    }
#endif

#if 1
    cmd = LOG_CONTROLLER_IOCTL_CMD_REMOVE_LOG_CONTENT;
    ret = log_controller_ctx.ops.ioctl(&log_controller_ctx, cmd, &log_controller_ioctl_param);
    if(ret < 0) {
        printf("remove log content fail\n");
        return -1;
    }
#endif

    log_controller_ctx.ops.deinit(&log_controller_ctx);

    log_controller_ctx.ops.list_resources(&log_controller_ctx);

    return 0;
}
#endif

#if 0
// for test purposes
int log_controller_testApp(void)
{
    int ret = 0, cmd;
    log_controller_context log_controller_ctx;
    register_log_parameters register_log_param;
    unregister_log_parameters unregister_log_param;
    log_controller_ioctl_parameters log_controller_ioctl_param;

    ret = log_controller_init(&log_controller_ctx);
    if(ret < 0) {
        printf("log_controller_init fail\n");
        return -1;
    }

    strcpy(register_log_param.in.path, "/mnt/hgfs/Projects/appLibC/log");
    strcpy(register_log_param.in.signature, "system.log");
    ret = log_controller_ctx.ops.register_context(&log_controller_ctx, &register_log_param);
    if(ret < 0) {
        printf("register_log fail\n");
        return -1;
    }

    strcpy(register_log_param.in.path, "/var/log");
    strcpy(register_log_param.in.signature, "mysql.log");
    ret = log_controller_ctx.ops.register_context(&log_controller_ctx, &register_log_param);
    if(ret < 0) {
        printf("register_log fail\n");
        return -1;
    }

    log_controller_ctx.ops.list_resources(&log_controller_ctx);

    cmd = LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX;
    strcpy(log_controller_ioctl_param.acquire_log_ctx_param.in.signature, "mysql.log");
    ret = log_controller_ctx.ops.ioctl(&log_controller_ctx, cmd, &log_controller_ioctl_param);
    if(ret < 0) {
        printf("acquire log context fail\n");
        return -1;
    }

    //printf("index %d\n", log_controller_ioctl_param.temp.index);

    cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
    strcpy(log_controller_ioctl_param.log_message_param.in.message, "Hello, World!!!");
    log_controller_ioctl_param.log_message_param.in.level = LOG_LEVEL_DEBUG;
    ret = log_controller_ctx.ops.ioctl(&log_controller_ctx, cmd, &log_controller_ioctl_param);
    if(ret < 0) {
        printf("log message fail\n");
        return -1;
    }

    cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
    strcpy(log_controller_ioctl_param.log_message_param.in.message, "This is a text message from user!!");
    log_controller_ioctl_param.log_message_param.in.level = LOG_LEVEL_INFO;
    ret = log_controller_ctx.ops.ioctl(&log_controller_ctx, cmd, &log_controller_ioctl_param);
    if(ret < 0) {
        printf("log message fail\n");
        return -1;
    }

    cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
    strcpy(log_controller_ioctl_param.log_message_param.in.message, "Unknow file name!!");
    log_controller_ioctl_param.log_message_param.in.level = LOG_LEVEL_ERROR;
    ret = log_controller_ctx.ops.ioctl(&log_controller_ctx, cmd, &log_controller_ioctl_param);
    if(ret < 0) {
        printf("log message fail\n");
        return -1;
    }

    log_controller_ctx.ops.deinit(&log_controller_ctx);

    log_controller_ctx.ops.list_resources(&log_controller_ctx);

    return ret;
}
#endif

