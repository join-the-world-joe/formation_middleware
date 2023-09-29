
#ifndef __LOG_CONTROLLER_H__
#define __LOG_CONTROLLER_H__

#define NB_LOGS 10

#define MAX_LENGTH_OF_LOG_SIGNATURE 50

#define MAX_LENGTH_OF_LOG_PATH_SIGNATURE 125

#define MAX_LENGTH_OF_LOG_URI_SIGNATURE 200

#define MAX_LENGTH_OF_LOG_MESSAGE 1024

#define LOG_CONTROLLER_IOCTL_CMD_NB_BEGIN 1
#define LOG_CONTROLLER_IOCTL_CMD_ACQUIRE_LOG_CONTEX 1
#define LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE 2
#define LOG_CONTROLLER_IOCTL_CMD_REMOVE_LOG_CONTENT 3
#define LOG_CONTROLLER_IOCTL_CMD_NB_END 3

#define LOG_IOCTL_CMD_NB_BEGIN 1
#define LOG_IOCTL_CMD_LOG_MESSAGE 1
#define LOG_IOCTL_CMD_REMOVE_LOG_CONTENT 2
#define LOG_IOCTL_CMD_NB_END 2

#define LOG_LEVEL_BEGIN 0
#define LOG_LEVEL_DEBUG       0
#define LOG_LEVEL_INFO        1
#define LOG_LEVEL_WARNING     2 
#define LOG_LEVEL_EXCEPTION   3
#define LOG_LEVEL_ERROR       4
#define LOG_LEVEL_END   4

typedef struct _register_log_parameters {
    struct { 
        char path[MAX_LENGTH_OF_LOG_PATH_SIGNATURE];
        char signature[MAX_LENGTH_OF_LOG_SIGNATURE];
    } in;
    struct {
        int log_fd;
        char URI[MAX_LENGTH_OF_LOG_URI_SIGNATURE];
    } out;
} register_log_parameters;

typedef struct _unregister_log_parameters {
    struct {
        char signature[MAX_LENGTH_OF_LOG_SIGNATURE];
    } in;
    struct {
        char *_path;
        char *_uri;
        int log_fd;
    } temp;
} unregister_log_parameters;

typedef struct _log_ioctl_log_message_cmd_parameters {
    struct {
        int *_level;
        char *_message;
    } in;
    struct {
        char *_path;
        char *_signature;
        char *_uri;
        int log_fd;
        int length;
        pthread_mutex_t *_log_mutex;
    } temp;
} log_ioctl_log_message_cmd_parameters;

typedef struct _log_ioctl_remove_log_content_cmd_parameters {
    struct {
        char *_path;
        char *_signature;
        char *_uri;
        int _log_fd;
        pthread_mutex_t *_log_mutex;
    } temp;
} log_ioctl_remove_log_content_cmd_parameters;

typedef struct _log_ioctl_parameters {
    int cmd;    
    log_ioctl_log_message_cmd_parameters log_message_param;
    log_ioctl_remove_log_content_cmd_parameters remove_log_content;
} log_ioctl_parameters;

typedef struct _log__context {
    int log_fd;
    int in_use; /* 0-free, 1-not free*/
    int index;
    char URI[MAX_LENGTH_OF_LOG_URI_SIGNATURE]; 
    char path[MAX_LENGTH_OF_LOG_PATH_SIGNATURE];
    char signature[MAX_LENGTH_OF_LOG_SIGNATURE];
    pthread_mutex_t log_mutex;
} log__context;

typedef struct _log_controller_ioctl_acquire_log_context_cmd_parameters {
    struct {
        char signature[MAX_LENGTH_OF_LOG_SIGNATURE];
    } in;
} log_controller_ioctl_acquire_log_context_cmd_parameters; 

typedef struct _log_controller_ioctl_log_message_cmd_parameters {
    struct {
        int level;
        char message[MAX_LENGTH_OF_LOG_MESSAGE];
    } in;
} log_controller_ioctl_log_message_cmd_parameters;

typedef struct _log_controller_ioctl_remove_log_content_cmd_parameters {
    struct {
        int level;
        char message[MAX_LENGTH_OF_LOG_MESSAGE];
    } in;
} log_controller_ioctl_remove_log_content_cmd_parameters;


typedef struct _log_controller_ioctl_parameters {
    struct {
        int index;
        log__context *log_ctx;
    } temp;
    
    log_controller_ioctl_acquire_log_context_cmd_parameters acquire_log_ctx_param;
    log_controller_ioctl_log_message_cmd_parameters log_message_param;
    log_controller_ioctl_remove_log_content_cmd_parameters remove_log_content_param;
} log_controller_ioctl_parameters;

typedef struct _log__ops {

    int (*init)(register_log_parameters*);
    int (*ioctl)(log_ioctl_parameters *);
    int (*deinit)(unregister_log_parameters*);

} log__ops;

struct _log_controller_context ;

typedef struct _log_controller_ops {
    int (*init)(struct _log_controller_context *);
    int (*register_context)(struct _log_controller_context *, register_log_parameters*);
    int (*unregister_context)(struct _log_controller_context *, unregister_log_parameters*);
    void (*list_resources)(struct _log_controller_context *);
    int (*ioctl)(struct _log_controller_context *, int, log_controller_ioctl_parameters *);
    void (*deinit)(struct _log_controller_context *);
} log_controller_ops;

typedef struct _log_controller_context {

    int count;

    log__ops log_ops;

    int nb_logs;

    log__context log_ctx[NB_LOGS];

    log_controller_ops ops;

} log_controller_context;

int log_controller_init(log_controller_context *);
int log_controller_register_context(log_controller_context *, register_log_parameters *);
int log_controller_unregister_context(log_controller_context *, unregister_log_parameters *);
int log_controller_ioctl(log_controller_context *, int, log_controller_ioctl_parameters *);
void log_controller_list_resources(log_controller_context *);
void log_controller_deinit(log_controller_context *);
int log_controller_testApp(void);

#endif // __LOG_CONTROLLER_H__
