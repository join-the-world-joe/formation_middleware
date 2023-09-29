/**
* @file     dac_controller.h
* @brief    dac_controller.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     13/11/2018
*/

#ifndef __DA_CONTROLLER_H__
#define __DA_CONTROLLER_H__

#define NB_DAC_DEVICES 2

#define MAX_NB_OF_DAC_ANALOG_OUTPUT_TERMINALS 8

#define MAX_LENGTH_OF_DAC_SIGNATURE 5

#define MAX_LENGTH_OF_DAC_TERMINAL_SIGNATURE 10

#define DAC_BUS_TYPE_UNKNOWN 0
#define DAC_BUS_TYPE_SPI 1
#define DAC_BUS_TYPE_IIC 2

#define DAC_CONTROLLER_IOCTL_CMD_NB_BEGIN 1
#define DAC_CONTROLLER_IOCTL_CMD_ACQUIRE_DAC_CONTEX 1
#define DAC_CONTROLLER_IOCTL_CMD_WRITE_AND_UPDATE_CHANNEL 2
#define DAC_CONTROLLER_IOCTL_CMD_NB_END 2

#define DAC_IOCTL_CMD_NB_BEGIN 1
#define DAC_IOCTL_CMD_WRITE_AND_UPDATE_CHANNEL 1
#define DAC_IOCTL_CMD_NB_END 1

typedef struct {
    char terminal[MAX_LENGTH_OF_DAC_TERMINAL_SIGNATURE];
    uint8_t config_data1; /* channel address */
    uint8_t config_data2; /* reserved */
    uint8_t config_data3; /* reserved */
} analog_output;

typedef struct _register_dac_parameters {
    struct {
        int bus_type;
        int nb_terminals;
        void *spi_controller_ctx;
        void *spi_controller_ioctl_param;
        char signature[MAX_LENGTH_OF_DAC_SIGNATURE];
    } in;
    struct {
        analog_output analog_outputs[MAX_NB_OF_DAC_ANALOG_OUTPUT_TERMINALS];
    } in_out;
} register_dac_parameters;

typedef struct _unregister_dac_parameters {
    struct {
        char signature[MAX_LENGTH_OF_DAC_SIGNATURE];
    } in;
} unregister_dac_parameters;

typedef struct _dac_ioctl_write_and_update_channel_cmd_parameters {
    struct {
        char *_terminal;
        double voltage;
    } in;

    struct {
        uint16_t pcm_code;
    } out;

    struct {
        int bus_type;
        char *_signature;
        int _nb_terminals;
        uint16_t pcm_code;
        uint8_t _address_bits;
        void *spi_controller_ctx;
        analog_output *_analog_outputs;
        void *spi_controller_ioctl_param;
    } temp;
} dac_ioctl_write_and_update_channel_cmd_parameters;

typedef struct _dac_ioctl_parameters {
    int cmd;    
    dac_ioctl_write_and_update_channel_cmd_parameters write_and_update_channel_param;
} dac_ioctl_parameters;

typedef struct _dac_context {
    int index;
    int gain;
    uint8_t in_use;
    int bus_type;
    int nb_terminals;
    void *spi_controller_ctx;
    void *spi_controller_ioctl_param;
    analog_output analog_outputs[MAX_NB_OF_DAC_ANALOG_OUTPUT_TERMINALS];
    char signature[MAX_LENGTH_OF_DAC_SIGNATURE];
} dac_context;

typedef struct _dac_ops {
    int (*init)(register_dac_parameters*);
    int (*ioctl)(dac_ioctl_parameters *);
    int (*deinit)(unregister_dac_parameters*);
} dac_ops;

typedef struct _dac_controller_ioctl_acquire_dac_context_cmd_parameters {
    struct {
        char signature[MAX_LENGTH_OF_DAC_SIGNATURE];
    } in;
} dac_controller_ioctl_acquire_dac_context_cmd_parameters; 

typedef struct _dac_controller_ioctl_write_and_update_channel_cmd_parameters {
    struct {
        char terminal[MAX_LENGTH_OF_DAC_TERMINAL_SIGNATURE];
        double voltage;
    } in;

    struct {
        uint16_t pcm_code;
    } out;
} dac_controller_ioctl_write_and_update_channel_cmd_parameters;

typedef struct _dac_controller_ioctl_parameters {
    struct {
        int index;
        dac_context *dac_ctx;
    } temp;
    
    dac_controller_ioctl_acquire_dac_context_cmd_parameters acquire_dac_ctx_param;
    dac_controller_ioctl_write_and_update_channel_cmd_parameters write_and_update_channel_param;
} dac_controller_ioctl_parameters;

struct _dac_controller_context;

typedef struct _dac_controller_ops {

    int (*init)(struct _dac_controller_context *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
    int (*register_context)(struct _dac_controller_context*, register_dac_parameters*);
    int (*unregister_context)(struct _dac_controller_context *, unregister_dac_parameters*);
    void (*list_resources)(struct _dac_controller_context *);
    int (*ioctl)(struct _dac_controller_context *, uint16_t, dac_controller_ioctl_parameters *);
    void (*deinit)(struct _dac_controller_context *);

} dac_controller_ops;

typedef struct _dac_controller_context {

    uint32_t count;

    dac_ops da_ops;
    
    int nb_dac_devices;

    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param;

    dac_context dac_ctx[NB_DAC_DEVICES];

    dac_controller_ops ops;

} dac_controller_context;

int dac_controller_init(dac_controller_context *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
int dac_controller_register_dac(dac_controller_context *, register_dac_parameters* );
int dac_controller_unregister_dac(dac_controller_context *, unregister_dac_parameters*);
void dac_controller_list_resources(dac_controller_context *);
int dac_controller_ioctl(dac_controller_context *, uint16_t, dac_controller_ioctl_parameters *);
void dac_controller_deinit(dac_controller_context *);
#endif // __DA_CONTROLLER_H__
