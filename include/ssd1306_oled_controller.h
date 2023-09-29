/*
* @file     ssd1306_oled_controller.h
* @brief    ssd1306_oled_controller.h
* @author   Jerry.Chen <mikuyear@sina.cn>
* @date     31/1/2019
*/

#ifndef __SSD1306_OLED_CONTROLLER_H__
#define __SSD1306_OLED_CONTROLLER_H__

#define NB_SSD1306_OLED_DEVICES 1
#define MAX_LENGTH_OF_SSD1306_OLED_SIGNATURE 20

#define SSD1306_OLED_CONTROLLER_IOCTL_CMD_NB_BEGIN 1
#define SSD1306_OLED_CONTROLLER_IOCTL_CMD_ACQUIRE_SSD1306_OLED_CONTEXT 1
#define SSD1306_OLED_CONTROLLER_IOCTL_CMD_START_SCOLL   2
#define SSD1306_OLED_CONTROLLER_IOCTL_CMD_STOP_SCOLL    3
#define SSD1306_OLED_CONTROLLER_IOCTL_CMD_WRITE_BYTE    4
#define SSD1306_OLED_CONTROLLER_IOCTL_CMD_WRITE_CHAR    5
#define SSD1306_OLED_CONTROLLER_IOCTL_CMD_NB_END 5


#define SSD1306_OLED_IOCTL_CMD_NB_BEGIN 1
#define SSD1306_OLED_IOCTL_CMD_START_SCOLL  1
#define SSD1306_OLED_IOCTL_CMD_STOP_SCOLL   2
#define SSD1306_OLED_IOCTL_CMD_WRITE_BYTE   3
#define SSD1306_OLED_IOCTL_CMD_WRITE_CHAR   4
#define SSD1306_OLED_IOCTL_CMD_NB_END 4



//hal param
typedef struct _register_ssd1306_oled_parameters {
    struct {
        void *i2c_controller_ctx;
        void *i2c_controller_ioctl_param;
        char signature[MAX_LENGTH_OF_SSD1306_OLED_SIGNATURE];
    } in;
    struct {
        uint8_t dev_addr;
    } out;
} register_ssd1306_oled_parameters;


typedef struct _unregister_ssd1306_oled_parameters {
    struct {
        char signature[MAX_LENGTH_OF_SSD1306_OLED_SIGNATURE];
    } in;
} unregister_ssd1306_oled_parameters;


typedef struct _ssd1306_oled_ioctl_write_byte_parameters {
    struct {

        uint8_t *byte;
        uint8_t *row_idx;
        uint8_t *column_idx;
    } in;
    struct {
        void *i2c_controller_ctx;
        void *i2c_controller_ioctl_param;
    } temp;
} ssd1306_oled_ioctl_write_byte_parameters;


typedef struct _ssd1306_oled_ioctl_write_char_parameters {
    struct {
        char *_char;
        uint8_t *char_row_idx;
        uint8_t *char_column_idx;
    } in;
    struct {
        void *i2c_controller_ctx;
        void *i2c_controller_ioctl_param;
    } temp;
} ssd1306_oled_ioctl_write_char_parameters;


typedef struct _ssd1306_oled_ioctl_start_scoll_parameters {
    struct {
        void *i2c_controller_ctx;
        void *i2c_controller_ioctl_param;
    } temp;
} ssd1306_oled_ioctl_start_scoll_parameters;


typedef struct _ssd1306_oled_ioctl_stop_scoll_parameters {
    struct {
        void *i2c_controller_ctx;
        void *i2c_controller_ioctl_param;
    } temp;
} ssd1306_oled_ioctl_stop_scoll_parameters;


typedef struct _ssd1306_oled_ioctl_parameters {
    int cmd;
    ssd1306_oled_ioctl_write_byte_parameters write_byte_param;
    ssd1306_oled_ioctl_write_char_parameters write_char_param;
    ssd1306_oled_ioctl_start_scoll_parameters start_scoll_param;
    ssd1306_oled_ioctl_stop_scoll_parameters stop_scoll_param;
} ssd1306_oled_ioctl_parameters;


//hal ops
typedef struct _ssd1306_oled_hal_ops {
    int (*init)(register_ssd1306_oled_parameters *);
    int (*ioctl)(ssd1306_oled_ioctl_parameters *);
    int (*deinit)(unregister_ssd1306_oled_parameters *);
} ssd1306_oled_hal_ops;

//controller param
typedef struct _ssd1306_oled_controller_acquire_ssd1306_oled_context_cmd_parameters {
    struct {
        char signature[MAX_LENGTH_OF_SSD1306_OLED_SIGNATURE];
    } in;
} ssd1306_oled_controller_acquire_ssd1306_oled_context_cmd_parameters;

typedef struct _ssd1306_oled_controller_ioctl_write_byte_parameters {
    struct {
        uint8_t byte;
        uint8_t row_idx;
        uint8_t column_idx;
    } in;
} ssd1306_oled_controller_ioctl_write_byte_parameters;

typedef struct _ssd1306_oled_controller_ioctl_write_char_parameters {
    struct {
        char _char;
        uint8_t char_row_idx;
        uint8_t char_column_idx;
    } in;
} ssd1306_oled_controller_ioctl_write_char_parameters;

typedef struct _ssd1306_oled_context ssd1306_oled_context;

typedef struct _ssd1306_oled_controller_ioctl_parameters {
    struct {
        int index;
        ssd1306_oled_context *ssd1306_oled_ctx;
    } temp;

    ssd1306_oled_controller_acquire_ssd1306_oled_context_cmd_parameters acquire_ssd1306_oled_ctx_param;
    ssd1306_oled_controller_ioctl_write_byte_parameters write_byte_param;
    ssd1306_oled_controller_ioctl_write_char_parameters write_char_param;

} ssd1306_oled_controller_ioctl_parameters;


typedef struct _ssd1306_oled_controller_context ssd1306_oled_controller_context;

//controller ops
typedef struct _ssd1306_oled_controller_ops {
    int (*init)(ssd1306_oled_controller_context*, log_controller_context*, log_controller_ioctl_parameters *);
    void (*deinit)(ssd1306_oled_controller_context *);
    int (*register_context)(ssd1306_oled_controller_context*, register_ssd1306_oled_parameters*);
    int (*unregister_context)(ssd1306_oled_controller_context *, unregister_ssd1306_oled_parameters *);
    void (*list_resources)(ssd1306_oled_controller_context *);
    int (*ioctl)(ssd1306_oled_controller_context *, uint16_t, ssd1306_oled_controller_ioctl_parameters *);
} ssd1306_oled_controller_ops;


//example context
typedef struct _ssd1306_oled_context {
    int index;
    uint8_t in_use;
    uint8_t dev_addr;
    void *i2c_controller_ctx;
    void *i2c_controller_ioctl_param;
    char signature[MAX_LENGTH_OF_SSD1306_OLED_SIGNATURE];
} ssd1306_oled_context;


//content total info about example
typedef struct _ssd1306_oled_controller_context {
    //controller context property
    int count;
    int nb_ssd1306_oled_devices;

    //log info
    log_controller_context *log_ctx;
    log_controller_ioctl_parameters *log_ioctl_param;

    //ssd1306 ops
    ssd1306_oled_hal_ops ssd1306_oled_ops;
    //ssd1306 context ops
    ssd1306_oled_controller_ops ops;

    //example context
    //context
    ssd1306_oled_context ssd1306_oled_ctx[NB_SSD1306_OLED_DEVICES];
} ssd1306_oled_controller_context;


//creat example context
int ssd1306_oled_controller_init(ssd1306_oled_controller_context *ssd1306_oled_controller_ctx, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
void ssd1306_oled_controller_deinit(ssd1306_oled_controller_context *);
//register/unregister example
int ssd1306_oled_controller_register_ssd1306_oled(ssd1306_oled_controller_context*, register_ssd1306_oled_parameters*);
int ssd1306_oled_controller_unregister_ssd1306_oled(ssd1306_oled_controller_context *, unregister_ssd1306_oled_parameters *);
void ssd1306_oled_controller_list_resources(ssd1306_oled_controller_context *);
//control example
int ssd1306_oled_controller_ioctl(ssd1306_oled_controller_context *, uint16_t, ssd1306_oled_controller_ioctl_parameters *);

#endif // __SSD1306_OLED_CONTROLLER_H__

