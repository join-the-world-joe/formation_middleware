
#ifndef __EEPROM_CONTROLLER_H__
#define __EEPROM_CONTROLLER_H__

#define NB_EEPROM_DEVICES 1

#define MAX_LENGTH_OF_EEPROM_SIGNATURE 10

#define EEPROM_BUS_TYPE_UNKNOWN 0
#define EEPROM_BUS_TYPE_SPI 1
#define EEPROM_BUS_TYPE_IIC 2

#define MAX_MEMORY_SIZE_IN_BYTES_OF_EEPROM 256

#define MEMORY_SIZE_IN_BYTES_OF_24C02   256
#define MEMORY_SIZE_IN_BYTES_OF_24C04   512

#define EEPROM_CONTROLLER_IOCTL_CMD_NB_BEGIN 1
#define EEPROM_CONTROLLER_IOCTL_CMD_ACQUIRE_EEPROM_CONTEX 1
#define EEPROM_CONTROLLER_IOCTL_CMD_PROGRAM_SINGLE_BYTE 2
#define EEPROM_CONTROLLER_IOCTL_CMD_READ_MULTI_BYTES 3
#define EEPROM_CONTROLLER_IOCTL_CMD_NB_END 3

#define EEPROM_IOCTL_CMD_NB_BEGIN 1
#define EEPROM_IOCTL_CMD_PROGRAM_SINGLE_BYTE 2
#define EEPROM_IOCTL_CMD_READ_MULTI_BYTES 3
#define EEPROM_IOCTL_CMD_NB_END 4

typedef struct _register_eeprom_parameters {
    struct {
        int bus_type;
        int nb_bytes;
        void *i2c_controller_ctx;
        void *i2c_controller_ioctl_param;
        char signature[MAX_LENGTH_OF_EEPROM_SIGNATURE];
    } in;
    struct {
        uint8_t dev_addr;
    } out;
} register_eeprom_parameters;

typedef struct _unregister_eeprom_parameters {
    struct {
        char signature[MAX_LENGTH_OF_EEPROM_SIGNATURE];
    } in;
} unregister_eeprom_parameters;

typedef struct _eeprom_ioctl_program_single_byte_cmd_parameters {
    struct {
        int *_offset;
        uint8_t *_data;
    } in;
    struct {
        
    } temp;
} eeprom_ioctl_program_single_byte_cmd_parameters; 

typedef struct _eeprom_ioctl_parameters {
    int cmd;    
    eeprom_ioctl_program_single_byte_cmd_parameters program_single_byte;
} eeprom_ioctl_parameters;

typedef struct _eeprom_ops {
    int (*init)(register_eeprom_parameters*);
    int (*ioctl)(eeprom_ioctl_parameters *);
    int (*deinit)(unregister_eeprom_parameters*);
} eeprom_ops;

typedef struct _eeprom_context {
    int index;
    int bus_type;
    int nb_bytes;
    uint8_t in_use;
    uint8_t dev_addr;
    void *i2c_controller_ctx;
    void *i2c_controller_ioctl_param;
    char signature[MAX_LENGTH_OF_EEPROM_SIGNATURE];
} eeprom_context;

struct _eeprom_controller_context;

typedef struct _eeprom_controller_ioctl_acquire_eeprom_context_cmd_parameters {
    struct {
        char signature[MAX_LENGTH_OF_EEPROM_SIGNATURE];
    } in;
} eeprom_controller_ioctl_acquire_eeprom_context_cmd_parameters;   

typedef struct _eeprom_controller_ioctl_program_single_byte_cmd_parameters {
    struct {
        int offset;
        uint8_t data;
    } in;
} eeprom_controller_ioctl_program_single_byte_cmd_parameters; 

typedef struct _eeprom_controller_ioctl_parameters {
    struct {
        int index;
        eeprom_context *eeprom_ctx;
    } temp;
    
    eeprom_controller_ioctl_acquire_eeprom_context_cmd_parameters acquire_eeprom_ctx_param;
    eeprom_controller_ioctl_program_single_byte_cmd_parameters   program_single_byte;
} eeprom_controller_ioctl_parameters;

typedef struct _eeprom_controller_ops {
    int (*init)(struct _eeprom_controller_context *, log_controller_context *);
    int (*register_context)(struct _eeprom_controller_context *, register_eeprom_parameters*);
    //int (*unregister_context)(struct _io_expander_controller_context *, unregister_io_expander_parameters*);
    void (*list_resources)(struct _eeprom_controller_context *);
    int (*ioctl)(struct _eeprom_controller_context *, uint16_t, eeprom_controller_ioctl_parameters *);
    //void (*deinit)(struct _io_expander_controller_context *);
} eeprom_controller_ops;

typedef struct _eeprom_controller_context {

    int count;

    eeprom_ops eeprom_ops;
    
    int nb_eeprom_devices;

    log_controller_context *log_ctx;

    eeprom_context eeprom_ctx[NB_EEPROM_DEVICES];

    eeprom_controller_ops ops;

} eeprom_controller_context;

void eeprom_controller_list_resources(eeprom_controller_context *eeprom_controller_ctx);
int eeprom_controller_init(eeprom_controller_context *eeprom_controller_ctx, log_controller_context *log_ctx);
int eeprom_controller_register_eeprom(eeprom_controller_context *eeprom_controller_ctx, register_eeprom_parameters *eeprom_param);
int eeprom_controller_ioctl(eeprom_controller_context *eeprom_controller_ctx, uint16_t cmd, eeprom_controller_ioctl_parameters *eeprom_controller_ioctl_param);



#endif // __EEPROM_CONTROLLER_H__

