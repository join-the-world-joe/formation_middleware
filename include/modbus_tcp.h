/**
* @file     modbus_tcp.h
* @brief    This file is the header file of the modbus tcp protocol
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     20/09/2018
*/

#ifndef __MODBUS_TCP_H__
#define __MODBUS_TCP_H__

//#define MODBUS_TCP_PORT 502
#define MODBUS_TCP_PORT 5020

#define SELECT_TIMEOUT_IN_SEC 5

#define MODBUS_MBAP_HEADER_LENGTH 7
#define MODBUS_MAX_TCP_ADU_LENGTH 260

#define MODBUS_STATUS_INIT
#define MODBUS_STATUS_RUNNING
#define MODBUS_STATUS_STOP
#define MODBUS_STATUS_SHUTDOWN

#define NB_FUNCTION_CODE 127

#define MAX_NB_OF_MULTIPLE_REGISTERS 123 

#define MODBUS_TCP_WR_MUL_REG_BYTE_COUNT_POS 13
#define MODBUS_TCP_UNIT_IDENTIFIER_POS 7
#define MODBUS_TCP_FUNCTION_CODE_POS 8
#define MODBUS_TCP_PROTOCOL_ID_POS 3
#define MODBUS_TCP_PLAYLOAD_LENGTH_POS 5

#define MODBUS_FC_READ_COILS                1
#define MODBUS_FC_READ_DISCRETE_INPUTS      2
#define MODBUS_FC_READ_HOLDING_REGISTERS    3
#define MODBUS_FC_READ_INPUT_REGISTERS      4
#define MODBUS_FC_WRITE_SINGLE_COIL         5
#define MODBUS_FC_WRITE_SINGLE_REGISTER     6
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  16

#define MODBUS_EXCEPTION_CODE00     0x00
#define MODBUS_EXCEPTION_CODE01     0x01
#define MODBUS_EXCEPTION_CODE02     0x02
#define MODBUS_EXCEPTION_CODE03     0x03
#define MODBUS_EXCEPTION_CODE04     0x04

#define MODBUS_TCP_FUNCTION_CODE1_ADU_LENGTH 12
#define MODBUS_TCP_FUNCTION_CODE2_ADU_LENGTH 12
#define MODBUS_TCP_FUNCTION_CODE3_ADU_LENGTH 12
#define MODBUS_TCP_FUNCTION_CODE4_ADU_LENGTH 12
#define MODBUS_TCP_FUNCTION_CODE5_ADU_LENGTH 12
#define MODBUS_TCP_FUNCTION_CODE6_ADU_LENGTH 12

struct _modbus_tcp_context;

typedef struct {
    uint8_t function_code;
    int32_t (*handler)(struct _modbus_tcp_context *, uint8_t *, uint16_t);
    int32_t (*responser)(struct _modbus_tcp_context *, int, uint8_t *, uint16_t, int (*tcp_write)(tcp_context *, int, uint8_t *, int));
}function_code_handler;

typedef struct _modbus_tcp_context {
    uint8_t platform_endian;
    int cur_status; /* the status of modbus service */
    int *manager_status; /* the status of manager */
    tcp_context tcp_ctx;  /* tcp context */    
    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param;
    struct timeval timeout;
    Register _register;
    fd_set client_set; /*store all socket descriptors of clients*/
    fd_set temp_set;  /* store socket descriptors temporarily*/
    int cur_client_sockfd;
    int cur_max_fd; // it has to be the maximum fd plus one
    function_code_handler func_handler_map[NB_FUNCTION_CODE]; 
} modbus_tcp_context;

int modbus_tcp_init(modbus_tcp_context *modbus_ctx, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
int modbus_tcp_send_msg(modbus_tcp_context *modbus_ctx, int client_sockfd, uint8_t *msg, uint16_t length);
int modbus_tcp_recv_msg(modbus_tcp_context *modbus_ctx, int client_sockfd, uint8_t *msg, uint16_t *length);
int modbus_tcp_request_treatment(modbus_tcp_context *modbus_ctx, int client_sockfd, uint8_t *ADU, uint16_t length);
int modbus_tcp_wait(modbus_tcp_context *modbus_ctx);
int modbus_tcp_run(modbus_tcp_context *modbus_ctx);
void modbus_tcp_deinit(modbus_tcp_context *modbus_ctx);

#endif  //__MODBUS_TCP_H__


