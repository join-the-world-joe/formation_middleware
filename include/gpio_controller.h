/**
* @file     gpio_controller.h
* @brief    gpio_controller.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     09/11/2018
*/

#ifndef __GPIO_CONTROLLER_H__
#define __GPIO_CONTROLLER_H__

#define NB_GPIO_PORTS 7

#define MAX_LENGTH_OF_GPIO_PORT_SIGNATURE 15

#define GPIO_DIRECTION_IN   0
#define GPIO_DIRECTION_OUT  1
#define GPIO_DIRECTION_UNKNOWN 2

#define GPIO_LEVEL_HIGH 1
#define GPIO_LEVEL_LOW 0
#define GPIO_LEVEL_UNKNOWN 2

#define GPIO_CONTROLLER_IOCTL_CMD_NB_BEGIN 1
#define GPIO_CONTROLLER_IOCTL_CMD_ACQUIRE_GPIO_CONTEX   1
#define GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL 2
#define GPIO_CONTROLLER_IOCTL_CMD_GET_LEVEL 3
#define GPIO_CONTROLLER_IOCTL_CMD_NB_END 3

#define GPIO_IOCTL_CMD_NB_BEGIN 1
#define GPIO_IOCTL_CMD_SET_LEVEL 1
#define GPIO_IOCTL_CMD_GET_LEVEL 2
#define GPIO_IOCTL_CMD_NB_END 2

typedef struct _gpio_ioctl_get_level_cmd_parameters {
    struct {
        int *level;
    } out;
    struct {
        int32_t dev_fd;
        uint8_t direction;
    } temp;
} gpio_ioctl_get_level_cmd_parameters;

typedef struct _gpio_ioctl_set_level_cmd_parameters {
    struct {
        int level;
        int dev_fd;
        int direction;
    } in;
} gpio_ioctl_set_level_cmd_parameters;

typedef struct _gpio_ioctl_parameters {
    int cmd;    
    gpio_ioctl_set_level_cmd_parameters set_level_param;
    gpio_ioctl_get_level_cmd_parameters get_level_param;
} gpio_ioctl_parameters;

typedef struct _register_gpio_port_parameters {
    struct {
        int direction;    
        char signature[MAX_LENGTH_OF_GPIO_PORT_SIGNATURE];
    } in;
    struct {
        int dev_fd;
    } out;
} register_gpio_port_parameters;

typedef struct _unregister_gpio_port_parameters {
    struct {
        char signature[MAX_LENGTH_OF_GPIO_PORT_SIGNATURE];
    } in;
    struct {
        int dev_fd;
    } temp;
} unregister_gpio_port_parameters;

typedef struct _gpio_context {
    int dev_fd;
    int in_use; /* 0-free, 1-not free*/
    int direction;
    int index; 
    char signature[MAX_LENGTH_OF_GPIO_PORT_SIGNATURE];
} gpio_context;

typedef struct _gpio_controller_ioctl_acquire_gpio_context_cmd_parameters {
    struct {
        char signature[MAX_LENGTH_OF_GPIO_PORT_SIGNATURE];
    } in;
} gpio_controller_ioctl_acquire_gpio_context_cmd_parameters;   

typedef struct _gpio_controller_ioctl_set_level_cmd_parameters {
    struct {
        int level;
    } in;
} gpio_controller_ioctl_set_level_cmd_parameters;

typedef struct _gpio_controller_ioctl_get_level_cmd_parameters {
    struct {
        int level;
    } out;
} gpio_controller_ioctl_get_level_cmd_parameters;

typedef struct _gpio_controller_ioctl_parameters {
    struct {
        int index;
        gpio_context *gpio_ctx;
    } temp;
    gpio_controller_ioctl_set_level_cmd_parameters set_level_param;
    gpio_controller_ioctl_get_level_cmd_parameters get_level_param;
    gpio_controller_ioctl_acquire_gpio_context_cmd_parameters acquire_gpio_ctx_param;
} gpio_controller_ioctl_parameters;

typedef struct _gpio_ops {

    int (*init)(register_gpio_port_parameters*);
    int (*ioctl)(gpio_ioctl_parameters *);
    int (*deinit)(unregister_gpio_port_parameters*);
} gpio_ops;


struct _gpio_controller_context;

typedef struct _gpio_controller_ops {

    int (*init)(struct _gpio_controller_context *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
    int (*register_context)(struct _gpio_controller_context *, register_gpio_port_parameters*);
    int (*unregister_context)(struct _gpio_controller_context *, unregister_gpio_port_parameters*);
    void (*list_resources)(struct _gpio_controller_context *);
    int (*ioctl)(struct _gpio_controller_context *, uint16_t, gpio_controller_ioctl_parameters *);
    void (*deinit)(struct _gpio_controller_context *);

} gpio_controller_ops;

typedef struct _gpio_controller_context {

    int count;

    gpio_ops io_ops;

    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param; 

    int nb_gpio_ports;

    gpio_context gpio_ctx[NB_GPIO_PORTS];
    
    gpio_controller_ops ops;

} gpio_controller_context;

int gpio_controller_init(gpio_controller_context *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
void gpio_controller_list_resources(gpio_controller_context *);
int gpio_controller_register_gpio_port(gpio_controller_context *, register_gpio_port_parameters *);
int gpio_controller_unregister_gpio_port(gpio_controller_context *, unregister_gpio_port_parameters *);
int gpio_controller_ioctl(gpio_controller_context *, uint16_t, gpio_controller_ioctl_parameters *);
void gpio_controller_deinit(gpio_controller_context *);

#endif // __GPIO_CONTROLLER_H__

