/**
* @file     modbus_tcp.c
* @brief    This file implements the modbus tcp functionalities
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     21/09/2018
*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/time.h>

#include "channel.h"
#include "log_controller.h"
#include "tcp.h"
#include "registers.h"
#include "modbus_tcp.h"

#include "function_code_handler.h"
#include "macros.h"
#include "debug.h"

#include "register_offset.h"
#include "register_values.h"

/**
* @fn               set_data_model_to_default
* @param [in]       reg
*                   a pointer points to Register instance
* @return           =0, success; <0, fail;
* @brief            This function sets data model to default value
*/
static int set_data_model_to_default(Register *reg)
{
    int i, j, holding_offset, temp;
    Register *_register = reg;
    Data_Model *_data_model = NULL;

    if(!_register) {
        printf("set_data_model_to_default fail, incorrect parameters\n");
        return -1;
    }

    _data_model = _register->data_model;
#if 0
    // set all values to zero
    for(i=0; i<NB_LOGIC_CHANNEL; i++) {

        // default setting for discretes input
        for(j=0; j<NB_DISCRETES_INPUT_REGISTERS; j++) 
            _data_model[i].discretes_input[j] = 0;       

        // default setting for coils
        for(j=0; j<NB_COILS_REGISTERS; j++)
            _data_model[i].coils[j] = 0;

        // default setting for input_registers
        for(j=0; j<NB_INPUT_REGISTERS; j++) 
            _data_model[i].input_registers[j] = 0;

        // default setting for holding_registers
        for(j=0; j<NB_HOLDING_REGISTERS; j++) 
            _data_model[i].holding_registers[j] = 0;
    }
#endif

    // set channel mode to be configuration mode
    holding_offset = HOLDING_MODE_REGISTER_OFFSET;
    temp = LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_CONFIGURATION;
    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        _data_model[i].holding_registers[holding_offset] = temp;
    }

    // set channel status to be idel
    holding_offset = HOLDING_LOGICAL_STATUS_REGISTER_OFFSET;
    temp = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_IDEL;
    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        _data_model[i].holding_registers[holding_offset] = temp;
    }

    // set channel action to be none
    holding_offset = HOLDING_ACTION_REGISTER_OFFSET;
    temp = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;
    for(i=0; i<NB_LOGIC_CHANNEL; i++) {
        _data_model[i].holding_registers[holding_offset] = temp;
    }

}

/**
* @fn               register_function_code_handler
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       function_code
*                   modbus function code
* @param [in]       handler
*                   modbus handler
* @param [in]       responser
*                   modbus responser
* @param [in]       tcp_write
*                   tcp write
* @return           =0, success; <0, fail   
* @brief            to register a function code's handler and responser
*/
int register_function_code_handler(modbus_tcp_context *modbus_ctx, uint8_t function_code, int (*handler)(modbus_tcp_context *, uint8_t *, uint16_t), int32_t (*responser)(modbus_tcp_context *, int, uint8_t *, uint16_t, int (*tcp_write)(tcp_context *, int, uint8_t *, int)))
{
    int ret = 0;

    if(function_code>127 || !modbus_ctx || !handler || !responser) {
        printf("register_function_code_handler, incorrect parameters\n");
        return -1;
    }

    if(modbus_ctx->func_handler_map[function_code-1].handler) {
        // function code has already got a handler, errors occur
        printf("function code has already got a handler, errors occur\n");
        return -1;
    } else {
        modbus_ctx->func_handler_map[function_code-1].handler = handler;
        modbus_ctx->func_handler_map[function_code-1].responser = responser;
    } 

    return ret;
}


/**
* @fn               modbus_tcp_init
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       log_ctx
*                   logging context
* @return           =0, success; <0, fail   
* @brief            to initialize a modbus tcp service
*/
int modbus_tcp_init(modbus_tcp_context *modbus_ctx, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param)
{

    int i, ret = 0, j;
    Register *_register = NULL;
    modbus_tcp_context *_modbus_ctx = modbus_ctx;
    log_controller_context *_log_controller_ctx = log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param = log_controller_ioctl_param; 


    if(!_modbus_ctx || !_log_controller_ctx || !_log_controller_ioctl_param) {
        printf("modbus_tcp_init, incorrect parameters\n");
        return -1;
    }
    
    _register = &_modbus_ctx->_register;

    for(i=0; i<NB_FUNCTION_CODE; i++) {
        _modbus_ctx->func_handler_map[i].handler = NULL;
        _modbus_ctx->func_handler_map[i].responser = NULL;
    }

    ret = register_function_code_handler(_modbus_ctx, MODBUS_FC_READ_COILS, func_code01_handler, func_code01_responser);
    if(ret < 0) {
        printf("fail to register handler and responser for function code 0x01\n");
        return -1;
    }

    ret = register_function_code_handler(_modbus_ctx, MODBUS_FC_READ_DISCRETE_INPUTS, func_code02_handler, func_code02_responser);
    if(ret < 0) {
        printf("fail to register handler and responser for function code 0x02\n");
        return -1;
    }

    ret = register_function_code_handler(_modbus_ctx, MODBUS_FC_READ_HOLDING_REGISTERS, func_code03_handler, func_code03_responser);
    if(ret < 0) {
        printf("fail to register handler and responser for function code 0x03\n");
        return -1;
    }

    ret = register_function_code_handler(_modbus_ctx, MODBUS_FC_READ_INPUT_REGISTERS, func_code04_handler, func_code04_responser);
    if(ret < 0) {
        printf("fail to register handler and responser for function code 0x04\n");
        return -1;
    }

    ret = register_function_code_handler(_modbus_ctx, MODBUS_FC_WRITE_SINGLE_COIL, func_code05_handler, func_code05_responser);
    if(ret < 0) {
        printf("fail to register handler and responser for function code 0x05\n");
        return -1;
    }

    ret = register_function_code_handler(_modbus_ctx, MODBUS_FC_WRITE_SINGLE_REGISTER, func_code06_handler, func_code06_responser);
    if(ret < 0) {
        printf("fail to register handler and responser for function code 0x06\n");
        return -1;
    }

    ret = register_function_code_handler(_modbus_ctx, MODBUS_FC_WRITE_MULTIPLE_REGISTERS, func_code16_handler, func_code16_responser);
    if(ret < 0) {
        printf("fail to register handler and responser for function code 0x16\n");
        return -1;
    }

    ret = tcp_init(&_modbus_ctx->tcp_ctx, MODBUS_TCP_PORT, _log_controller_ctx, _log_controller_ioctl_param);
    if(ret < 0) {
        printf("modbus_tcp_init, incorrect parameters\n");
        return -1;
    }

    ret = modbus_ctx->tcp_ctx.tcp_listen(&_modbus_ctx->tcp_ctx);
    if(ret < 0) {
        printf("modbus_tcp_init->tcp_listen, fail\n");
        modbus_ctx->tcp_ctx.tcp_close(&_modbus_ctx->tcp_ctx, _modbus_ctx->tcp_ctx.tcp_listen_sockfd);
        return -1;
    }

    ret = register_init(&_modbus_ctx->_register, _log_controller_ctx, _log_controller_ioctl_param);
    if(ret < 0) {
        printf("modbus_tcp_init->register_init, fail\n");
        return -1;
    }

    // load registers' values for default
    set_data_model_to_default(_register);

    FD_ZERO(&_modbus_ctx->client_set);
    FD_SET(_modbus_ctx->tcp_ctx.tcp_listen_sockfd, &_modbus_ctx->client_set); // add listen socket to client set
    _modbus_ctx->cur_max_fd = _modbus_ctx->tcp_ctx.tcp_listen_sockfd;

    return ret;
}


/**
* @fn               modbus_tcp_send_msg
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       client_sockfd
*                   client socket file descriptor
* @param [in]       msg
*                   message
* @param [in]       length
*                   length of message
* @return           =0, success; <0, fail   
* @brief            to send modbus tcp message
*/
int modbus_tcp_send_msg(modbus_tcp_context *modbus_ctx, int client_sockfd, uint8_t *msg, uint16_t length)
{
    int ret = 0;
    
    if(!modbus_ctx || !msg || length < 0 || client_sockfd < 0) {
        printf("modbus_tcp_send_msg, incorrect parameters\n");
        return -1;
    }

    ret = modbus_ctx->tcp_ctx.tcp_write(&modbus_ctx->tcp_ctx, client_sockfd, msg, length);
    if(ret < 0) {
        printf("modbus_tcp_send_msg, fail\n");
        return -1;
    }

    return ret;
}


/**
* @fn               modbus_tcp_recv_msg
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       client_sockfd
*                   client socket file descriptor
* @param [in]       msg
*                   message
* @param [out]       length
*                   length of message
* @return           =0, success; <0, fail   
* @brief            to receive modbus tcp message
*/
int modbus_tcp_recv_msg(modbus_tcp_context *modbus_ctx, int client_sockfd, uint8_t *msg, uint16_t *length)
{
    int ret = 0;
    fd_set read_set;
    int bytes_to_read = -1, byte_count = -1;
    uint8_t ADU[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    int ADU_length = 0;
    int read_count = 0;
    uint16_t function_code = 0, protocol_identifier = -1;
    int flag = 0;
    uint16_t adu_length;
    uint8_t response_adu[MODBUS_MBAP_HEADER_LENGTH + 2 + 1] = {0}; /* 7 bytes(MBAP) + 1 byte(Function code + 0x80) + 1 byte(Exception Code) */

    if(!modbus_ctx || !msg || !length || client_sockfd < 0) {
        printf("modbus_tcp_recv_msg, incorrect parameters\n");
        return -1;
    }

    FD_ZERO(&read_set);
    FD_SET(client_sockfd, &read_set);

    // read MBAP header if possible
    bytes_to_read = MODBUS_MBAP_HEADER_LENGTH + 1; // read MBAP header and function code

    while(bytes_to_read != 0) {
        ret = select(client_sockfd+1, &read_set, NULL, NULL, NULL);
        if(ret < 0) {
            printf("modbus_tcp_read_msg, select fail\n");
            return -1;
        }

        read_count = modbus_ctx->tcp_ctx.tcp_read(&modbus_ctx->tcp_ctx, client_sockfd, &ADU[ADU_length], bytes_to_read);
        if(read_count < 0) {
            printf("modbus_tcp_read_msg, tcp_read fail\n");
            return -1;
        }

        ADU_length += read_count;
        bytes_to_read -= read_count;

        if(bytes_to_read == 0) {

            if(flag == 0) {
            
                flag = 1;

                // check if the protocol identifier is zero
                protocol_identifier = ADU[MODBUS_TCP_PROTOCOL_ID_POS - 1];
                if(protocol_identifier != 0) {
                    printf("modbus_tcp_read_msg error, protocol_identifier is %d\n", protocol_identifier);
                    // flush cache buffer and report error message
                    modbus_ctx->tcp_ctx.tcp_flush(&modbus_ctx->tcp_ctx, client_sockfd);
                    return -1;
                }

                // parse function code and determine the remain bytes 
                function_code = ADU[MODBUS_TCP_FUNCTION_CODE_POS - 1];
                 
                printf("function code %d\n", function_code);

                switch(function_code) {
                    case MODBUS_FC_READ_COILS:
                    case MODBUS_FC_READ_DISCRETE_INPUTS:
                    case MODBUS_FC_READ_HOLDING_REGISTERS:
                    case MODBUS_FC_READ_INPUT_REGISTERS:
                    case MODBUS_FC_WRITE_SINGLE_COIL:
                    case MODBUS_FC_WRITE_SINGLE_REGISTER:
                         bytes_to_read = 4;
                    break;

                    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
                         if(ADU_length < MODBUS_MBAP_HEADER_LENGTH + 1) {
                             bytes_to_read = (MODBUS_MBAP_HEADER_LENGTH + 1) - ADU_length;
                             flag = 0; //read again
                         } else if(ADU_length == 8) {
                             bytes_to_read = 5; /* starting_address + quantity_of_registers + byte_count */
                             flag = 0; //read again
                         } else if(ADU_length > 8) {
                             //check if we have got a complete APU
                             byte_count = ADU[12];
                             if(ADU_length < (MODBUS_MBAP_HEADER_LENGTH + 6 + byte_count)) {
                                 bytes_to_read = (MODBUS_MBAP_HEADER_LENGTH + 6 + byte_count) - ADU_length;
                                 flag = 0; //read again
                             } else if(ADU_length == (MODBUS_MBAP_HEADER_LENGTH + 6 + byte_count)){

                                 //a completed APU is achieved

                                 *length = ADU_length;

                                 memcpy(msg, ADU, ADU_length);

                                 return 0;

                             } else {
                                 //printf("unknow error!!!\n");
                                 return -1;
                             }
                         }
                         
                    break;

                    default:
                         printf("unsupported function code %d\n", function_code);
                         // exception occurs 
                         memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); /* MBAP */
                         response_adu[MODBUS_MBAP_HEADER_LENGTH] = function_code + 0x80; /* function code + 0x80 */
                         response_adu[MODBUS_MBAP_HEADER_LENGTH + 1] = (MODBUS_EXCEPTION_CODE01&0x00FF); /* exception code */
                         adu_length = MODBUS_MBAP_HEADER_LENGTH + 2;
                         //dump_data8(response_adu, adu_length);
                         modbus_ctx->tcp_ctx.tcp_write(&modbus_ctx->tcp_ctx, client_sockfd, response_adu, adu_length);
                         return -1;
                    break;
                }
            }else if(flag == 1) {

                //a completed APU is achieved

                *length = ADU_length;

                memcpy(msg, ADU, ADU_length);
                
                return 0;
            }
        }
    }
    return ret;
}


/**
* @fn               modbus_tcp_request_treatment
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       client_sockfd
*                   client socket file descriptor
* @param [in]       ADU
*                   modbus application data unit
* @param [out]       length
*                   length of ADU
* @return           =0, success; <0, fail   
* @brief            to handle the received ADU
*/
int modbus_tcp_request_treatment(modbus_tcp_context *modbus_ctx, int client_sockfd, uint8_t *ADU, uint16_t length)
{
    int ret = 0, function_code = -1, exp_code = -1;
    uint16_t adu_length;
    uint8_t response_adu[MODBUS_MBAP_HEADER_LENGTH + 2 + 1] = {0}; /* 7 bytes(MBAP) + 1 byte(Function code + 0x80) + 1 byte(Exception Code) */

    if(!modbus_ctx || !ADU || length < 0 || client_sockfd < 0) {
        printf("modbus_tcp_send_msg, incorrect parameters\n");
        return -1;
    }

    function_code = ADU[MODBUS_TCP_FUNCTION_CODE_POS - 1];
    if(function_code < 0 || function_code > NB_FUNCTION_CODE) {
        printf("function code %d is out of range!!!\n", function_code);
        return -1;
    }
    
    switch(function_code) {
        case MODBUS_FC_READ_COILS:
             printf("read coils\n");
             exp_code =  modbus_ctx->func_handler_map[MODBUS_FC_READ_COILS-1].handler(modbus_ctx, ADU, length);
             modbus_ctx->func_handler_map[MODBUS_FC_READ_COILS-1].responser(modbus_ctx, exp_code, ADU, length, modbus_ctx->tcp_ctx.tcp_write);
        break;

        case MODBUS_FC_READ_DISCRETE_INPUTS:
             printf("read_discrete_input\n");
             exp_code =  modbus_ctx->func_handler_map[MODBUS_FC_READ_DISCRETE_INPUTS-1].handler(modbus_ctx, ADU, length);
             modbus_ctx->func_handler_map[MODBUS_FC_READ_DISCRETE_INPUTS-1].responser(modbus_ctx, exp_code, ADU, length, modbus_ctx->tcp_ctx.tcp_write);
        break;

        case MODBUS_FC_READ_HOLDING_REGISTERS:
             printf("read holding registers\n");
             exp_code =  modbus_ctx->func_handler_map[MODBUS_FC_READ_HOLDING_REGISTERS-1].handler(modbus_ctx, ADU, length);
             modbus_ctx->func_handler_map[MODBUS_FC_READ_HOLDING_REGISTERS-1].responser(modbus_ctx, exp_code, ADU, length, modbus_ctx->tcp_ctx.tcp_write);
        break;

        case MODBUS_FC_READ_INPUT_REGISTERS:
             printf("read input registers\n");
             exp_code =  modbus_ctx->func_handler_map[MODBUS_FC_READ_INPUT_REGISTERS-1].handler(modbus_ctx, ADU, length);
             modbus_ctx->func_handler_map[MODBUS_FC_READ_INPUT_REGISTERS-1].responser(modbus_ctx, exp_code, ADU, length, modbus_ctx->tcp_ctx.tcp_write);
        break;

        case MODBUS_FC_WRITE_SINGLE_COIL:
             printf("write single coil\n");
             exp_code =  modbus_ctx->func_handler_map[MODBUS_FC_WRITE_SINGLE_COIL-1].handler(modbus_ctx, ADU, length);
             modbus_ctx->func_handler_map[MODBUS_FC_WRITE_SINGLE_COIL-1].responser(modbus_ctx, exp_code, ADU, length, modbus_ctx->tcp_ctx.tcp_write);
        break;

        case MODBUS_FC_WRITE_SINGLE_REGISTER:
             printf("write single register\n");
             exp_code =  modbus_ctx->func_handler_map[MODBUS_FC_WRITE_SINGLE_REGISTER-1].handler(modbus_ctx, ADU, length);
             modbus_ctx->func_handler_map[MODBUS_FC_WRITE_SINGLE_REGISTER-1].responser(modbus_ctx, exp_code, ADU, length, modbus_ctx->tcp_ctx.tcp_write);
        break;

        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
             printf("write multiple registers\n");
             exp_code =  modbus_ctx->func_handler_map[MODBUS_FC_WRITE_MULTIPLE_REGISTERS - 1].handler(modbus_ctx, ADU, length);
             modbus_ctx->func_handler_map[MODBUS_FC_WRITE_MULTIPLE_REGISTERS - 1].responser(modbus_ctx, exp_code, ADU, length, modbus_ctx->tcp_ctx.tcp_write);
        break;

        default:
             printf("Function code %d unsupported\n", function_code);
             return -1;
        break;
    }

    return ret;
}


/**
* @fn               modbus_tcp_wait
* @param [in]       modbus_ctx
*                   modbus context
* @return           =0, success; <0, fail   
* @brief            to wait for a new client connection
*/
int modbus_tcp_wait(modbus_tcp_context *modbus_ctx)
{
    int ret = 0;
    fd_set client_set;
    int nb_incoming = -1;
    int listen_sockfd, client_sockfd;

    if(!modbus_ctx) {
        printf("modbus_tcp_wait, incorrect parameters\n");
        return -1;
    }

    client_set = modbus_ctx->client_set;
    listen_sockfd = modbus_ctx->tcp_ctx.tcp_listen_sockfd;

    if(SELECT_TIMEOUT_IN_SEC > 0) {
        modbus_ctx->timeout.tv_sec = SELECT_TIMEOUT_IN_SEC;
        modbus_ctx->timeout.tv_usec = 0;
    }

    FD_ZERO(&modbus_ctx->temp_set);

    nb_incoming = select(modbus_ctx->cur_max_fd+1, &client_set, NULL, NULL, &modbus_ctx->timeout);
    if(nb_incoming < 0) {
        printf("modbus_tcp_wait->select, fail\n");
        return -1;
    }
 
    if(nb_incoming > 0) {

        //printf("modbus_tcp_wait->select, nb_incoming = %d\n", nb_incoming);

        //check if we have a new commer
        if(FD_ISSET(listen_sockfd, &client_set)) {

            client_sockfd = modbus_ctx->tcp_ctx.tcp_accept(&modbus_ctx->tcp_ctx);
            if(client_sockfd < 0) {
                printf("modbus_tcp_wait->tcp_accept, unknown error\n");
                return -1;
            }

            // add current client to client_set
            FD_SET(client_sockfd, &modbus_ctx->client_set);

            if(client_sockfd > modbus_ctx->cur_max_fd)
                modbus_ctx->cur_max_fd = client_sockfd;

            //set recv timeout
            //if(modbus_ctx->tcp_ctx.recv_timeout_in_usec > 0) {
            //    tv.tv_sec = 3;
            //    tv.tv_usec = modbus_ctx->tcp_ctx.recv_timeout_in_usec;
            //    setsockopt(client_sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
            //}
             
            nb_incoming--;

            if(nb_incoming > 0) 
                FD_CLR(listen_sockfd, &client_set);
              
        }

        if(nb_incoming > 0) {
            // since we've taken an action on listen sockfd, then we remove it from readable client set
            modbus_ctx->temp_set = client_set; //save the untreated client set to temp set
            return nb_incoming;
        }

        //modbus_ctx->temp_set = client_set;
    }

    return 0;
}


/**
* @fn               modbus_tcp_run
* @param [in]       modbus_ctx
*                   modbus context
* @return           =0, success; <0, fail   
* @brief            to enable a modbus tcp service
*/
int modbus_tcp_run(modbus_tcp_context *modbus_ctx)
{
    int ret = 0, nb_incoming = 0, client_sockfd;
    uint16_t length;
    uint8_t modbus_tcp_adu[MODBUS_MAX_TCP_ADU_LENGTH + 1];

    if(!modbus_ctx) {
        printf("modbus_tcp_run, incorrect parameters\n");
        return -1;
    }

    while(*modbus_ctx->manager_status == MANAGER_STATUS_RUNNING) {
        
        nb_incoming = modbus_tcp_wait(modbus_ctx);
        if(nb_incoming < 0) {
            printf("modbus_tcp_run->modbus_tcp_wait, fail\n");
            return -1;
        }

        printf("nb_incoming = %d\n", nb_incoming);

        if(nb_incoming == 0) {
            // no client, do nothing
            return 0;
        }

        if(nb_incoming > 0) {

            for(client_sockfd=0; client_sockfd<=modbus_ctx->cur_max_fd; client_sockfd++) {

                if(!FD_ISSET(client_sockfd, &modbus_ctx->temp_set)) 
                    continue; // current client_sockfd is not in the temp set, do nothing

                printf("modbus_tcp_run, handling socket %d\n", client_sockfd);

                ret = modbus_tcp_recv_msg(modbus_ctx, client_sockfd, modbus_tcp_adu, &length);
                if(ret < 0) {
                    printf("modbus_tcp_run->modbus_tcp_read_msg fail on socket %d\n", client_sockfd);
                    
                    modbus_ctx->tcp_ctx.tcp_close(&modbus_ctx->tcp_ctx, client_sockfd);

                    //remove this client from client_set in modbus_context
                    FD_CLR(client_sockfd, &modbus_ctx->client_set);

                    if(client_sockfd == modbus_ctx->cur_max_fd)
                         modbus_ctx->cur_max_fd = modbus_ctx->cur_max_fd - 1;

                    continue;
                }

                modbus_ctx->cur_client_sockfd = client_sockfd;

                //printf("******************************************************************\n");
                //dump_data8(modbus_tcp_adu, length);
                //printf("\n******************************************************************\n");

                ret = modbus_tcp_request_treatment(modbus_ctx, client_sockfd, modbus_tcp_adu, length);
                if(ret < 0) {
                    printf("modbus_tcp_request_treatment failure on socket %d\n", client_sockfd);
                    
                    modbus_ctx->tcp_ctx.tcp_close(&modbus_ctx->tcp_ctx, client_sockfd);

                    //remove this client from client_set in modbus_context
                    FD_CLR(client_sockfd, &modbus_ctx->client_set);

                    if(client_sockfd == modbus_ctx->cur_max_fd)
                         modbus_ctx->cur_max_fd = modbus_ctx->cur_max_fd - 1;
                }
            }
        }
    }

    return ret;
}


/**
* @fn               register_function_code_handler
* @param [in]       modbus_ctx
*                   modbus context
* @return           =0, success; <0, fail   
* @brief            to do something opposite to initialization
*/
void modbus_tcp_deinit(modbus_tcp_context *modbus_ctx)
{
    modbus_tcp_context *_modbus_ctx = modbus_ctx;

    if(!_modbus_ctx) {
        printf("modbus_tcp_run, incorrect parameters\n");
        return ;
    }

    _modbus_ctx->tcp_ctx.tcp_deinit(&_modbus_ctx->tcp_ctx, _modbus_ctx->_log_controller_ctx, _modbus_ctx->_log_controller_ioctl_param);
    _modbus_ctx->_register.ops.deinit(&_modbus_ctx->_register, _modbus_ctx->_log_controller_ctx, _modbus_ctx->_log_controller_ioctl_param);

}




