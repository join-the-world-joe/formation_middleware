/**
* @file     io_expander_controller.h
* @brief    io_expander_controller.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     12/11/2018
*/

#ifndef __IO_EXPANDER_CONTROLLER_H__
#define __IO_EXPANDER_CONTROLLER_H__

#define NB_IO_EXPANDER_DEVICES 4

#define MAX_NB_OF_IO_EXPANDER_PORTS 16

#define MAX_LENGTH_OF_IO_EXPANDER_SIGNATURE 5

#define MAX_LENGTH_OF_IO_EXPANDER_TERMINAL_SIGNATURE 10

#define IO_EXPANDER_PORT_LEVEL_HIGH 1
#define IO_EXPANDER_PORT_LEVEL_LOW 0

#define IO_EXPANDER_CONTROLLER_IOCTL_CMD_NB_BEGIN 1
#define IO_EXPANDER_CONTROLLER_IOCTL_CMD_ACQUIRE_IO_EXPANDER_CONTEX 1
#define IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL 2
#define IO_EXPANDER_CONTROLLER_IOCTL_CMD_GET_LEVEL 3
#define IO_EXPANDER_CONTROLLER_IOCTL_CMD_NB_END 3

#define IO_EXPANDER_IOCTL_CMD_NB_BEGIN 1
#define IO_EXPANDER_IOCTL_CMD_SET_LEVEL 1
#define IO_EXPANDER_IOCTL_CMD_NB_END 1

#define IO_EXPANDER_BUS_TYPE_UNKNOWN 0
#define IO_EXPANDER_BUS_TYPE_SPI 1
#define IO_EXPANDER_BUS_TYPE_IIC 2

#define IO_EXPANDER_DIRECTION_IN   0
#define IO_EXPANDER_DIRECTION_OUT  1
#define IO_EXPANDER_DIRECTION_UNKNOWN 2

typedef struct {
    int direction;  /* fill manually */
    char terminal[MAX_LENGTH_OF_IO_EXPANDER_TERMINAL_SIGNATURE]; /* fill manually */
    uint8_t config_data1; /* Port Number(bit number) if exists */
    uint8_t config_data2; /* Direction Register Address if exists */
    uint8_t config_data3; /* GPIO Port Register Address if exists */
} io_port;

typedef struct _register_io_expander_parameters {
    struct {
        int bus_type;
        int nb_io_ports;
        void *spi_controller_ctx;
        void *spi_controller_ioctl_param;
        char signature[MAX_LENGTH_OF_IO_EXPANDER_SIGNATURE];
    } in;
    struct {
        io_port io_ports[MAX_NB_OF_IO_EXPANDER_PORTS];
    } in_out;
} register_io_expander_parameters;

typedef struct _unregister_io_expander_parameters {
    struct {
        char signature[MAX_LENGTH_OF_IO_EXPANDER_SIGNATURE];
    } in;
} unregister_io_expander_parameters;

typedef struct _io_expander_ioctl_set_level_cmd_parameters {
    struct {
        int *_bus_type;
        uint8_t *_level;
        int _nb_io_ports;
        io_port *_io_ports;
        void *_spi_controller_ctx;
        void *_spi_controller_ioctl_param;
        char *_terminal;
        uint8_t *_gpio_a;
        uint8_t *_gpio_b;
    } in;
    struct {
        io_port *_io_port;
    } temp;
} io_expander_ioctl_set_level_cmd_parameters;

typedef struct _io_expander_ioctl_parameters {
    int cmd;    
    io_expander_ioctl_set_level_cmd_parameters io_expander_set_level_param;
} io_expander_ioctl_parameters;

typedef struct _io_expander_ops {
    int (*init)(register_io_expander_parameters*);
    int (*ioctl)(io_expander_ioctl_parameters *);
    int (*deinit)(unregister_io_expander_parameters*);
} io_expander_ops;

typedef struct _io_expander_context {
    int index;
    uint8_t in_use;
    int bus_type;
    int nb_io_ports;
    void *spi_controller_ctx;
    void *spi_controller_ioctl_param;
    io_port io_ports[MAX_NB_OF_IO_EXPANDER_PORTS];
    char signature[MAX_LENGTH_OF_IO_EXPANDER_SIGNATURE];
    uint8_t gpio_a;
    uint8_t gpio_b;
} io_expander_context;

typedef struct _io_expander_controller_ioctl_acquire_io_expander_context_cmd_parameters {
    struct {
        char signature[MAX_LENGTH_OF_IO_EXPANDER_SIGNATURE];
    } in;
} io_expander_controller_ioctl_acquire_io_expander_context_cmd_parameters;   

typedef struct _io_expander_controller_ioctl_set_level_cmd_parameters {
    struct {
        uint8_t level;
        char terminal[MAX_LENGTH_OF_IO_EXPANDER_TERMINAL_SIGNATURE];
    } in;
} io_expander_controller_ioctl_set_level_cmd_parameters;

typedef struct _io_expander_controller_ioctl_parameters {
    struct {
        int index;
        io_expander_context *io_expander_ctx;
    } temp;
    
    io_expander_controller_ioctl_acquire_io_expander_context_cmd_parameters acquire_io_expander_ctx_param;
    io_expander_controller_ioctl_set_level_cmd_parameters set_level_param;
} io_expander_controller_ioctl_parameters;

struct _io_expander_controller_context;

typedef struct _io_expander_controller_ops {
    int (*init)(struct _io_expander_controller_context *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
    int (*register_context)(struct _io_expander_controller_context *, register_io_expander_parameters*);
    int (*unregister_context)(struct _io_expander_controller_context *, unregister_io_expander_parameters*);
    void (*list_resources)(struct _io_expander_controller_context *);
    int (*ioctl)(struct _io_expander_controller_context *, uint16_t, io_expander_controller_ioctl_parameters *);
    void (*deinit)(struct _io_expander_controller_context *);
} io_expander_controller_ops;

typedef struct _io_expander_controller_context {

    int count;

    io_expander_ops io_expander_ops;
    
    int nb_io_expander_devices;

    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param; 

    io_expander_context io_expander_ctx[NB_IO_EXPANDER_DEVICES];

    io_expander_controller_ops ops;

} io_expander_controller_context;

int io_expander_controller_init(io_expander_controller_context *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
int io_expander_controller_register_io_expander(io_expander_controller_context*, register_io_expander_parameters*);
int io_expander_controller_unregister_io_expander(io_expander_controller_context*, unregister_io_expander_parameters*);
void io_expander_controller_list_resources(io_expander_controller_context*);
int io_expander_controller_ioctl(io_expander_controller_context*, uint16_t, io_expander_controller_ioctl_parameters*);
void io_expander_controller_deinit(io_expander_controller_context*);


#endif // __IO_EXPANDER_CONTROLLER_H__

