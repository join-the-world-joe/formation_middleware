/**
* @file     spi_controller.h
* @brief    spi_controller.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     09/11/2018
*/

#ifndef __SPI_CONTROLLER_H__
#define __SPI_CONTROLLER_H__

#define NB_SPI_BUS 8

#define MAX_SPI_TRANSFER_BYTES 100

#define MAX_LENGTH_OF_SPI_BUS_SIGNATURE 30

#define SPI_CONTROLLER_IOCTL_CMD_NB_BEGIN 1
#define SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX 1
#define SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE 2
#define SPI_CONTROLLER_IOCTL_CMD_NB_END 2

#define SPI_IOCTL_CMD_NB_BEGIN 1
#define SPI_IOCTL_CMD_TRANSFER_MESSAGE 1
#define SPI_IOCTL_CMD_NB_END 1

#define SPI_MODE_00 0
#define SPI_MODE_01 1
#define SPI_MODE_10 2
#define SPI_MODE_11 3

#define SPI_BPW_08BITS  8
#define SPI_BPW_16BITS  16

#define SPI_CS_POLARITY_HIGH 1
#define SPI_CS_POLARITY_LOW  0

#define SPI_SPEED_1MHZ (1*1000*1000)
#define SPI_SPEED_2MHZ (2*1000*1000)
#define SPI_SPEED_3MHZ (3*1000*1000)
#define SPI_SPEED_4MHZ (4*1000*1000)
#define SPI_SPEED_10MHZ (10*1000*1000)
#define SPI_SPEED_16MHZ (16*1000*1000)

typedef struct _spi_ioctl_transfer_message_cmd_parameters {
    struct {
        int dev_fd;
        uint8_t mode;
        uint32_t speed;
        uint32_t cs_polarity;
        uint8_t bits_per_word;
        uint8_t *_tx_buf;
        uint32_t *_tx_len;
    } in;
    struct {
        uint8_t *_rx_buf;
        uint32_t *_rx_len;
    } in_out;
} spi_ioctl_transfer_message_cmd_parameters;

typedef struct _spi_ioctl_parameters {
    int cmd;    
    spi_ioctl_transfer_message_cmd_parameters transfer_message_param;
} spi_ioctl_parameters;

typedef struct _register_spi_bus_parameters {
    struct {
        uint8_t mode;
        uint32_t speed;
        uint32_t cs_polarity;
        uint8_t bits_per_word;
        char signature[MAX_LENGTH_OF_SPI_BUS_SIGNATURE];
    } in;
    struct {
        int dev_fd;
    } out;
} register_spi_bus_parameters;

typedef struct _unregister_spi_bus_parameters {
    struct {
        char signature[MAX_LENGTH_OF_SPI_BUS_SIGNATURE];
    } in;
    struct {
        int32_t dev_fd;
    } temp;
} unregister_spi_bus_parameters;

typedef struct _spi_context {
    int index;
    int dev_fd;
    uint8_t mode;
    uint8_t in_use;
    uint32_t speed;
    uint32_t cs_polarity;
    uint8_t bits_per_word;
    char signature[MAX_LENGTH_OF_SPI_BUS_SIGNATURE];
} spi_context;

typedef struct _spi_bus_ops {
    int (*init)(register_spi_bus_parameters*);
    int (*ioctl)(spi_ioctl_parameters *);
    int (*deinit)(unregister_spi_bus_parameters*);
} spi_bus_ops;

typedef struct _spi_controller_ioctl_acquire_spi_context_cmd_parameters {
    struct {
        char signature[MAX_LENGTH_OF_SPI_BUS_SIGNATURE];
    } in;
} spi_controller_ioctl_acquire_spi_context_cmd_parameters;   

typedef struct _spi_controller_ioctl_transfer_message_cmd_parameters {
    struct {
        uint8_t tx_buf[MAX_SPI_TRANSFER_BYTES];
        uint32_t tx_len;   
    } in;
    struct {
        uint8_t rx_buf[MAX_SPI_TRANSFER_BYTES]; 
        uint32_t rx_len; 
    } out;
} spi_controller_ioctl_transfer_message_cmd_parameters;

typedef struct _spi_controller_ioctl_parameters {
    struct {
        int index;
        spi_context *spi_ctx;
    } temp;
    
    spi_controller_ioctl_acquire_spi_context_cmd_parameters acquire_spi_ctx_param;
    spi_controller_ioctl_transfer_message_cmd_parameters transfer_message_param;
} spi_controller_ioctl_parameters;


struct _spi_controller_context;

typedef struct _spi_controller_ops {

    int (*init)(struct _spi_controller_context *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
    int (*register_context)(struct _spi_controller_context *, register_spi_bus_parameters*);
    int (*unregister_context)(struct _spi_controller_context *, unregister_spi_bus_parameters*);
    void (*list_resources)(struct _spi_controller_context *);
    int (*ioctl)(struct _spi_controller_context *, uint16_t, spi_controller_ioctl_parameters *);
    void (*deinit)(struct _spi_controller_context *);

} spi_controller_ops;

typedef struct _spi_controller_context {

    uint32_t count;

    spi_bus_ops spi_ops;
    
    uint32_t nb_spi_bus;

    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param;

    spi_context spi_ctx[NB_SPI_BUS];

    spi_controller_ops ops;

} spi_controller_context;

int spi_controller_init(spi_controller_context*, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
void spi_controller_list_resources(spi_controller_context *);
int spi_controller_register_spi_bus(spi_controller_context*, register_spi_bus_parameters*);
int spi_controller_unregister_spi_bus(spi_controller_context *, unregister_spi_bus_parameters *);
int spi_controller_ioctl(spi_controller_context *, uint16_t, spi_controller_ioctl_parameters *);
void spi_controller_deinit(spi_controller_context *);
#endif // __SPI_CONTROLLER_H__

