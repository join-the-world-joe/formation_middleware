/**
* @file     i2c_controller.h
* @brief    i2c_controller.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     01/12/2018
*/

#ifndef __I2C_CONTROLLER_H__
#define __I2C_CONTROLLER_H__

#define NB_I2C_BUS 1

#define MAX_LENGTH_OF_I2C_BUS_SIGNATURE 10

#define MAX_LENGTH_OF_I2C_DEV_PATH  255

#define MAX_LENGTH_OF_I2C_DATA      1025

#define I2C_CONTROLLER_IOCTL_CMD_NB_BEGIN 1
#define I2C_CONTROLLER_IOCTL_CMD_ACQUIRE_I2C_CONTEX 2
#define I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES 3
#define I2C_CONTROLLER_IOCTL_CMD_READ_MULTI_BYTES 4
#define I2C_CONTROLLER_IOCTL_CMD_NB_END 4

#define I2C_IOCTL_CMD_NB_BEGIN 1
#define I2C_IOCTL_CMD_WRITE_MULTI_BYTES 2
#define I2C_IOCTL_CMD_READ_MULTI_BYTES 3
#define I2C_IOCTL_CMD_NB_END 3

typedef struct _i2c_ioctl_read_multi_bytes_cmd_parameters {
    struct {
        int _dev_fd;
        uint8_t *_i2c_dev_addr;
        uint8_t *_wr_data;
        int *_wr_length;
        int *_rd_length;
    } in;
    struct {
        uint8_t *_rd_data;
    } out;
} i2c_ioctl_read_multi_bytes_cmd_parameters;

typedef struct _i2c_ioctl_write_multi_bytes_cmd_parameters {
    struct {
        int _dev_fd;
        uint8_t *_i2c_dev_addr;
        uint8_t *_data;
        int *_length;
    } in;
} i2c_ioctl_write_multi_bytes_cmd_parameters;

typedef struct _i2c_ioctl_parameters {
    int cmd;    
    i2c_ioctl_write_multi_bytes_cmd_parameters  write_multi_bytes_param;
    i2c_ioctl_read_multi_bytes_cmd_parameters   read_multi_bytes_param;
} i2c_ioctl_parameters;

typedef struct _register_i2c_bus_parameters {
    struct {
        char signature[MAX_LENGTH_OF_I2C_BUS_SIGNATURE];
    } in;
    struct {
        int dev_fd;
    } out;
    struct {
        char path[MAX_LENGTH_OF_I2C_DEV_PATH];
    } temp;
} register_i2c_bus_parameters;

typedef struct _unregister_i2c_bus_parameters {
    struct {
        char signature[MAX_LENGTH_OF_I2C_BUS_SIGNATURE];
    } in;
    struct {
        int dev_fd;
    } temp;
} unregister_i2c_bus_parameters;

struct _i2c_controller_context;

typedef struct _i2c_context {
    int index;
    int dev_fd;
    uint8_t in_use;
    char path[MAX_LENGTH_OF_I2C_DEV_PATH];
    char signature[MAX_LENGTH_OF_I2C_BUS_SIGNATURE];
} i2c_context;

typedef struct _i2c_bus_ops {
    int (*init)(register_i2c_bus_parameters*);
    int (*ioctl)(i2c_ioctl_parameters *);
    int (*deinit)(unregister_i2c_bus_parameters*);
} i2c_bus_ops;

typedef struct _i2c_controller_ioctl_read_multi_bytes_cmd_parameters {
    struct {
        uint8_t i2c_dev_addr; /* 7 bit */
        uint8_t wr_data[MAX_LENGTH_OF_I2C_DATA]; 
        int wr_length;
        int rd_length;
    } in;
    struct {
        uint8_t rd_data[MAX_LENGTH_OF_I2C_DATA];
    } out;
} i2c_controller_ioctl_read_multi_bytes_cmd_parameters;

typedef struct _i2c_controller_ioctl_write_multi_bytes_cmd_parameters {
    struct {
        uint8_t i2c_dev_addr; /* 7 bit */
        uint8_t data[MAX_LENGTH_OF_I2C_DATA]; 
        int length;
    } in;
} i2c_controller_ioctl_write_multi_bytes_cmd_parameters;

typedef struct _i2c_controller_ioctl_acquire_i2c_context_cmd_parameters {
    struct {
        char signature[MAX_LENGTH_OF_I2C_BUS_SIGNATURE];
    } in;
} i2c_controller_ioctl_acquire_i2c_context_cmd_parameters;  

typedef struct _i2c_controller_ioctl_parameters {
    struct {
        int index;
        i2c_context *i2c_ctx;
    } temp;
    
    i2c_controller_ioctl_acquire_i2c_context_cmd_parameters acquire_i2c_ctx_param;
    i2c_controller_ioctl_write_multi_bytes_cmd_parameters write_multi_bytes_param;
    i2c_controller_ioctl_read_multi_bytes_cmd_parameters read_multi_bytes_param;
} i2c_controller_ioctl_parameters;

typedef struct _i2c_controller_ops {

    int (*init)(struct _i2c_controller_context *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
    int (*register_context)(struct _i2c_controller_context *, register_i2c_bus_parameters*);
    int (*unregister_context)(struct _i2c_controller_context *, unregister_i2c_bus_parameters*);
    void (*list_resources)(struct _i2c_controller_context *);
    int (*ioctl)(struct _i2c_controller_context *, uint16_t, i2c_controller_ioctl_parameters *);
    void (*deinit)(struct _i2c_controller_context *);

} i2c_controller_ops;

typedef struct _i2c_controller_context {

    int count;

    i2c_bus_ops i2c_ops;
    
    int nb_i2c_bus;

    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param;

    i2c_context i2c_ctx[NB_I2C_BUS];

    i2c_controller_ops ops;

} i2c_controller_context;

int i2c_controller_init(i2c_controller_context *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
void i2c_controller_deinit(i2c_controller_context *);
int i2c_controller_register_i2c_bus(i2c_controller_context *, register_i2c_bus_parameters *);
int i2c_controller_unregister_i2c_bus(i2c_controller_context *, unregister_i2c_bus_parameters *);
int i2c_controller_ioctl(i2c_controller_context *, uint16_t, i2c_controller_ioctl_parameters *);
void i2c_controller_list_resources(i2c_controller_context *);

#endif // __I2C_CONTROLLER_H__

