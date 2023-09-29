/**
* @file     adc_controller.h
* @brief    adc_controller.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     14/11/2018
*/

#ifndef __ADC_CONTROLLER_H__
#define __ADC_CONTROLLER_H__

#define NB_ADC_DEVICES  1

#define MAX_LENGTH_OF_ADC_SIGNATURE 5

#define MAX_NB_OF_ADC_ANALOG_INPUT_TERMINALS 16

#define MAX_LENGTH_OF_ADC_TERMINAL_SIGNATURE 10

#define ADC_CONTROLLER_IOCTL_CMD_NB_BEGIN 1
#define ADC_CONTROLLER_IOCTL_CMD_ACQUIRE_ADC_CONTEX 1
#define ADC_CONTROLLER_IOCTL_CMD_PULSE_CONVERSION   2
#define ADC_CONTROLLER_IOCTL_CMD_STM32_CONVERSION   3
#define ADC_CONTROLLER_IOCTL_CMD_NB_END 3

#define ADC_IOCTL_CMD_NB_BEGIN 1
#define ADC_IOCTL_CMD_PULSE_CONVERSION 1
#define ADC_IOCTL_CMD_STM32_CONVERSION 2
#define ADC_IOCTL_CMD_NB_END 2

#define ADC_BUS_TYPE_UNKNOWN 0
#define ADC_BUS_TYPE_SPI 1
#define ADC_BUS_TYPE_IIC 2

typedef struct _analog_input{
    char terminal[MAX_LENGTH_OF_ADC_TERMINAL_SIGNATURE];
    uint8_t config_data1; /* Channel Register */
    uint8_t config_data2; /* Channel bit mask */
    uint8_t config_data3;
} analog_input;

typedef struct _register_adc_parameters {
    struct {
        int bus_type;
        int nb_terminals;
        void *spi_controller_ctx;
        void *spi_controller_ioctl_param;
        void *gpio_controller_ctx;
        void *gpio_controller_ioctl_param_drdy;
        char signature[MAX_LENGTH_OF_ADC_SIGNATURE];
    } in;
    struct {
        analog_input analog_inputs[MAX_NB_OF_ADC_ANALOG_INPUT_TERMINALS];
    } in_out;
} register_adc_parameters;

typedef struct _unregister_adc_parameters {
    struct {
        char signature[MAX_LENGTH_OF_ADC_SIGNATURE];
    } in;
} unregister_adc_parameters;

typedef struct _adc_ioctl_pulse_conversion_cmd_parameters {
    struct {
        char *_terminal;
    } in;
    struct {
        double *_voltage;
    } out;
    struct {
        int _bus_type;
        char *_signature;
        int _nb_terminals;
        void *_spi_controller_ctx;
        analog_input *_analog_inputs;
        analog_input *_analog_input;
        void *_spi_controller_ioctl_param;
    } temp;
} adc_ioctl_pulse_conversion_cmd_parameters;

typedef struct _adc_sample {
    int index;
    double voltage;
} adc_sample;

typedef struct _adc_ioctl_stm32_conversion_cmd_parameters {
    struct {
        int *_nb_samples;
        adc_sample *_samples;
    } out;
    struct {
        int _bus_type;
        char *_signature;
        int _nb_terminals;
        void *_spi_controller_ctx;
        //analog_input *_analog_inputs;
        //analog_input *_analog_input;
        void *_gpio_controller_ctx;
        void *_spi_controller_ioctl_param;
        void *_gpio_controller_ioctl_param_drdy;
    } temp;
} adc_ioctl_stm32_conversion_cmd_parameters;

typedef struct _adc_ioctl_parameters {
    int cmd;    
    adc_ioctl_pulse_conversion_cmd_parameters pulse_conversion_param;
    adc_ioctl_stm32_conversion_cmd_parameters stm32_conversion_param;
} adc_ioctl_parameters;

typedef struct _adc_context {
    int index;
    uint8_t in_use;
    int bus_type;
    int nb_terminals;
    void *spi_controller_ctx;
    void *spi_controller_ioctl_param;
    void *gpio_controller_ioctl_param_drdy;
    void *gpio_controller_ctx;
    analog_input analog_inputs[MAX_NB_OF_ADC_ANALOG_INPUT_TERMINALS];
    char signature[MAX_LENGTH_OF_ADC_SIGNATURE];
} adc_context;

typedef struct _adc_ops {
    int (*init)(register_adc_parameters*);
    int (*ioctl)(adc_ioctl_parameters *);
    int (*deinit)(unregister_adc_parameters*);
} adc_ops;

typedef struct _adc_controller_ioctl_acquire_adc_context_cmd_parameters {
    struct {
        char signature[MAX_LENGTH_OF_ADC_SIGNATURE];
    } in;
} adc_controller_ioctl_acquire_adc_context_cmd_parameters; 

typedef struct _adc_controller_ioctl_pulse_conversion_cmd_parameters {
    struct {
        char terminal[MAX_LENGTH_OF_ADC_TERMINAL_SIGNATURE];
    } in;
    struct {
        double voltage;
    } out;
} adc_controller_ioctl_pulse_conversion_cmd_parameters;

typedef struct _adc_controller_ioctl_stm32_conversion_cmd_parameters {
    struct {
        int nb_samples;
        adc_sample samples[MAX_NB_OF_ADC_ANALOG_INPUT_TERMINALS];
    } out;
} adc_controller_ioctl_stm32_conversion_cmd_parameters;

typedef struct _adc_controller_ioctl_parameters {
    struct {
        int index;
        adc_context *adc_ctx;
    } temp;
    
    adc_controller_ioctl_acquire_adc_context_cmd_parameters acquire_adc_ctx_param;
    adc_controller_ioctl_pulse_conversion_cmd_parameters pulse_conversion_param;
    adc_controller_ioctl_stm32_conversion_cmd_parameters stm32_conversion_param;
} adc_controller_ioctl_parameters;

struct _adc_controller_context;

typedef struct _adc_controller_ops {

    int (*init)(struct _adc_controller_context *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
    int (*register_context)(struct _adc_controller_context*, register_adc_parameters*);
    int (*unregister_context)(struct _adc_controller_context *, unregister_adc_parameters*);
    void (*list_resources)(struct _adc_controller_context *);
    int (*ioctl)(struct _adc_controller_context *, uint16_t, adc_controller_ioctl_parameters *);
    void (*deinit)(struct _adc_controller_context *);

} adc_controller_ops;

typedef struct _adc_controller_context {

    uint32_t count;

    adc_ops ad_ops;
    
    uint32_t nb_adc_devices;

    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param;

    adc_context adc_ctx[NB_ADC_DEVICES];

    adc_controller_ops ops;

} adc_controller_context;

int adc_controller_init(adc_controller_context *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
void adc_controller_deinit(adc_controller_context *);
int adc_controller_register_adc(adc_controller_context *, register_adc_parameters* );
int adc_controller_unregister_adc(adc_controller_context *, unregister_adc_parameters*);
int adc_controller_ioctl(adc_controller_context *, uint16_t, adc_controller_ioctl_parameters *);
#endif // __ADC_CONTROLLER_H__
