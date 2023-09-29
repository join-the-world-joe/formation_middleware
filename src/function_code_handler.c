/**
* @file     function_code_handler.c
* @brief    This file implements handlers of supported function codes
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     27/09/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "channel.h"
#include "tcp.h"
#include "registers.h"
#include "platform_endian.h"
#include "modbus_tcp.h"
#include "debug.h"


/**
* @fn               func_code01_handler
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @return           =0, success; >0, exception code   
* @brief            handler for modbus function code 1
*/
int func_code01_handler(modbus_tcp_context *modbus_ctx, uint8_t *ADU, uint16_t length)
{
    int ret = 0;
    uint8_t unit_identifier = 0;
    uint16_t starting_address_hi, starting_address_lo, quantity_of_output_hi, quantity_of_output_lo;
    uint16_t starting_address, quntity_of_outputs;

    if(!modbus_ctx || !ADU || length<0) {
        printf("func_code01_handler incorrect parameters\n");
        return -1;
    }

    if(length != MODBUS_TCP_FUNCTION_CODE1_ADU_LENGTH) {
        printf("func_code01_handler, incorrect ADU length\n");
        return MODBUS_EXCEPTION_CODE02;
    }

    unit_identifier = ADU[MODBUS_TCP_UNIT_IDENTIFIER_POS - 1]; 

    modbus_ctx->_register.cur_unit_identifier = unit_identifier;

    return ret;
}


/**
* @fn               func_code01_responser
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       exception_code
*                   exception code for modbus protocol
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @param [in]       tcp_write
*                   a pointer points to tcp write
* @return           =0, success; <0, fail   
* @brief            responser for modbus function code 1
*/
int func_code01_responser(modbus_tcp_context *modbus_ctx, int exception_code, uint8_t *ADU, uint16_t length, int (*tcp_write)(tcp_context *tcp_ctx, int, uint8_t *pData, int length))
{
    int ret = 0;
    uint16_t playload_length;
    uint8_t exp_code = 0, endian;
    int client_sockfd = 0;
    Register *_register = NULL;
    tcp_context *tcp_ctx = NULL;
    uint16_t pdu_length, adu_length;
    uint8_t response_pdu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    uint8_t response_adu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    
    if(!modbus_ctx || !ADU || !tcp_write || exception_code<0) {
        printf("func_code01_responser incorrect parameters\n");
        return -1;
    }

    endian = modbus_ctx->platform_endian;
    tcp_ctx = &modbus_ctx->tcp_ctx;
    client_sockfd = modbus_ctx->cur_client_sockfd;
    _register = &modbus_ctx->_register;

    if(exception_code == 0) {
        exp_code = _register->ops.read_coils(_register, &ADU[7], response_pdu, &pdu_length);
    } else {
        exp_code = exception_code;
    }

    switch(exp_code) {
        case MODBUS_EXCEPTION_CODE00:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + pdu_length;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             memcpy(&response_adu[MODBUS_MBAP_HEADER_LENGTH], response_pdu, pdu_length); // copy pdu
             // amend length field now
             playload_length = pdu_length + 1;
             printf("playload length = %d\n", playload_length);
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS -1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = playload_length; 
             }
             //dump_data(response_adu, adu_length);
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;

        case MODBUS_EXCEPTION_CODE01:
        case MODBUS_EXCEPTION_CODE02: 
        case MODBUS_EXCEPTION_CODE03:
        case MODBUS_EXCEPTION_CODE04:
             printf("func_code01_responser exception\n");
             adu_length = MODBUS_MBAP_HEADER_LENGTH + 2;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             // amend length field now
             playload_length = 3;
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = playload_length; 
             }
             response_adu[MODBUS_MBAP_HEADER_LENGTH] = MODBUS_FC_READ_COILS + 0x80; // pdu initialization
             response_adu[MODBUS_MBAP_HEADER_LENGTH + 1] = exp_code;
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;

        default:
            printf("func_code01_responser, unknown exception code %d\n", exp_code);
        break;
    }

    return ret;
}


/**
* @fn               func_code02_handler
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @return           =0, success; <0, fail   
* @brief            handler for modbus function code 2
*/
int func_code02_handler(modbus_tcp_context *modbus_ctx, uint8_t *ADU, uint16_t length)
{
    int ret = 0;
    uint8_t unit_identifier = 0;

    if(!modbus_ctx || !ADU || length<0) {
        printf("func_code02_handler incorrect parameters\n");
        return -1;
    }

    if(length != MODBUS_TCP_FUNCTION_CODE2_ADU_LENGTH) {
        printf("func_code02_handler, incorrect ADU length\n");
        return MODBUS_EXCEPTION_CODE02;
    }

    unit_identifier = ADU[MODBUS_TCP_UNIT_IDENTIFIER_POS - 1]; 

    modbus_ctx->_register.cur_unit_identifier = unit_identifier;

    return ret;
}


/**
* @fn               func_code02_responser
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       exception_code
*                   exception code for modbus protocol
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @param [in]       tcp_write
*                   a pointer points to tcp write
* @return           =0, success; <0, fail   
* @brief            responser for modbus function code 2
*/
int func_code02_responser(modbus_tcp_context *modbus_ctx, int exception_code, uint8_t *ADU, uint16_t length, int (*tcp_write)(tcp_context *tcp_ctx, int, uint8_t *pData, int length))
{
    int ret = 0;
    uint16_t playload_length;
    uint8_t exp_code = 0, endian;
    int client_sockfd = 0;
    Register *_register = NULL;
    tcp_context *tcp_ctx = NULL;
    uint16_t pdu_length, adu_length;
    uint8_t response_pdu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    uint8_t response_adu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    
    if(!modbus_ctx || !ADU || !tcp_write || exception_code<0) {
        printf("func_code02_responser incorrect parameters\n");
        return -1;
    }

    endian = modbus_ctx->platform_endian;
    tcp_ctx = &modbus_ctx->tcp_ctx;
    client_sockfd = modbus_ctx->cur_client_sockfd;
    _register = &modbus_ctx->_register;

    if(exception_code == 0) {
        exp_code = _register->ops.read_discrete_inputs(_register, &ADU[7], response_pdu, &pdu_length);
    } else {
        exp_code = exception_code;
    }

    switch(exp_code) {

        case MODBUS_EXCEPTION_CODE00:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + pdu_length;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             memcpy(&response_adu[MODBUS_MBAP_HEADER_LENGTH], response_pdu, pdu_length); // copy pdu
             // amend length field now
             playload_length = pdu_length + 1;
             printf("playload length = %d\n", playload_length);
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS -1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = playload_length; 
             }
             //dump_data(response_adu, adu_length);
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;

        case MODBUS_EXCEPTION_CODE01:
        case MODBUS_EXCEPTION_CODE02:
        case MODBUS_EXCEPTION_CODE03:
        case MODBUS_EXCEPTION_CODE04:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + 2;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             // amend length field now
             playload_length = 3;
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = playload_length; 
             }
             response_adu[MODBUS_MBAP_HEADER_LENGTH] = MODBUS_FC_READ_COILS + 0x80; // pdu initialization
             response_adu[MODBUS_MBAP_HEADER_LENGTH + 1] = exp_code;
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;

        default:
            printf("func_code02_responser, unknown exception code %d\n", exp_code);
        break;
    }

    return ret;
}


/**
* @fn               func_code03_handler
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @return           =0, success; <0, fail   
* @brief            handler for modbus function code 3
*/
int func_code03_handler(modbus_tcp_context *modbus_ctx, uint8_t *ADU, uint16_t length)
{
    int ret = 0;

    uint8_t unit_identifier = 0;

    if(!modbus_ctx || !ADU) {
        printf("func_code03_handler incorrect parameters\n");
        return -1;
    }

    if(length != MODBUS_TCP_FUNCTION_CODE3_ADU_LENGTH) {
        printf("func_code03_handler, incorrect ADU length\n");
        return MODBUS_EXCEPTION_CODE03;
    }

    unit_identifier = ADU[MODBUS_TCP_UNIT_IDENTIFIER_POS - 1]; 

    modbus_ctx->_register.cur_unit_identifier = unit_identifier;

    return ret;
}


/**
* @fn               func_code03_responser
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       exception_code
*                   exception code for modbus protocol
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @param [in]       tcp_write
*                   a pointer points to tcp write
* @return           =0, success; <0, fail   
* @brief            responser for modbus function code 3
*/
int func_code03_responser(modbus_tcp_context *modbus_ctx, int exception_code, uint8_t *ADU, uint16_t length, int (*tcp_write)(tcp_context *tcp_ctx, int, uint8_t *pData, int length))
{
    int ret = 0;
    uint8_t endian;
    uint16_t playload_length;
    uint8_t exp_code = 0;
    int client_sockfd = 0;
    Register *_register = NULL;
    tcp_context *tcp_ctx = NULL;
    uint16_t pdu_length, adu_length;
    uint8_t response_pdu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    uint8_t response_adu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    
    if(!modbus_ctx || !ADU || !tcp_write || exception_code<0) {
        printf("func_code03_responser incorrect parameters\n");
        return -1;
    }

    endian = modbus_ctx->platform_endian;
    tcp_ctx = &modbus_ctx->tcp_ctx;
    client_sockfd = modbus_ctx->cur_client_sockfd;
    _register = &modbus_ctx->_register;

    if(exception_code == 0) {
        exp_code = _register->ops.read_holding_registers(_register, &ADU[7], response_pdu, &pdu_length);
    } else {
        exp_code = exception_code;
    }

    switch(exp_code) {

        case MODBUS_EXCEPTION_CODE00:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + pdu_length;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             memcpy(&response_adu[MODBUS_MBAP_HEADER_LENGTH], response_pdu, pdu_length); // copy pdu
             // amend length field now
             playload_length = pdu_length + 1;
             printf("playload length = %d\n", playload_length);
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS -1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = playload_length; 
             }
             //dump_data8(response_adu, adu_length);
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;

        case MODBUS_EXCEPTION_CODE01:
        case MODBUS_EXCEPTION_CODE02:
        case MODBUS_EXCEPTION_CODE03:
        case MODBUS_EXCEPTION_CODE04:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + 2;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             // amend length field now
             playload_length = 3;
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = playload_length; 
             }
             response_adu[MODBUS_MBAP_HEADER_LENGTH] = MODBUS_FC_READ_HOLDING_REGISTERS + 0x80; // pdu initialization
             response_adu[MODBUS_MBAP_HEADER_LENGTH + 1] = exp_code;
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;

        default:
            printf("func_code03_responser, unknown exception code %d\n", exp_code);
        break;
    }

    return ret;
}

/**
* @fn               func_code04_handler
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @return           =0, success; <0, fail   
* @brief            handler for modbus function code 4
*/
int func_code04_handler(modbus_tcp_context *modbus_ctx, uint8_t *ADU, uint16_t length)
{
    int ret = 0;

    uint8_t unit_identifier = 0;

    if(!modbus_ctx || !ADU) {
        printf("func_code04_handler incorrect parameters\n");
        return -1;
    }

    if(length != MODBUS_TCP_FUNCTION_CODE4_ADU_LENGTH) {
        printf("func_code04_handler, incorrect ADU length\n");
        return MODBUS_EXCEPTION_CODE03;
    }

    unit_identifier = ADU[MODBUS_TCP_UNIT_IDENTIFIER_POS - 1]; 

    modbus_ctx->_register.cur_unit_identifier = unit_identifier;

    return ret;
}


/**
* @fn               func_code04_responser
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       exception_code
*                   exception code for modbus protocol
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @param [in]       tcp_write
*                   a pointer points to tcp write
* @return           =0, success; <0, fail   
* @brief            responser for modbus function code 4
*/
int func_code04_responser(modbus_tcp_context *modbus_ctx, int exception_code, uint8_t *ADU, uint16_t length, int (*tcp_write)(tcp_context *tcp_ctx, int, uint8_t *pData, int length))
{
    int ret = 0;
    uint16_t playload_length;
    uint8_t exp_code = 0, endian;
    int client_sockfd = 0;
    Register *_register = NULL;
    tcp_context *tcp_ctx = NULL;
    uint16_t pdu_length, adu_length;
    uint8_t response_pdu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    uint8_t response_adu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    
    if(!modbus_ctx || !ADU || !tcp_write || exception_code<0) {
        printf("func_code04_responser incorrect parameters\n");
        return -1;
    }

    endian = modbus_ctx->platform_endian;
    tcp_ctx = &modbus_ctx->tcp_ctx;
    client_sockfd = modbus_ctx->cur_client_sockfd;
    _register = &modbus_ctx->_register;

    if(exception_code == 0) {
        exp_code = _register->ops.read_input_registers(_register, &ADU[7], response_pdu, &pdu_length);
    } else {
        exp_code = exception_code;
    }

    switch(exp_code) {

        case MODBUS_EXCEPTION_CODE00:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + pdu_length;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             memcpy(&response_adu[MODBUS_MBAP_HEADER_LENGTH], response_pdu, pdu_length); // copy pdu
             // amend length field now
             playload_length = pdu_length + 1;
             printf("playload length = %d\n", playload_length);
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS -1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = playload_length; 
             }
             //dump_data8(response_adu, adu_length);
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;

        case MODBUS_EXCEPTION_CODE01:
        case MODBUS_EXCEPTION_CODE02:
        case MODBUS_EXCEPTION_CODE03:
        case MODBUS_EXCEPTION_CODE04:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + 2;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             // amend length field now
             playload_length = 3;
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = playload_length; 
             }
             response_adu[MODBUS_MBAP_HEADER_LENGTH] = MODBUS_FC_READ_INPUT_REGISTERS + 0x80; // pdu initialization
             response_adu[MODBUS_MBAP_HEADER_LENGTH + 1] = exp_code;
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;

        default:
            printf("func_code04_responser, unknown exception code %d\n", exp_code);
        break;
    }

    return ret;
}


/**
* @fn               func_code05_handler
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @return           =0, success; <0, fail   
* @brief            handler for modbus function code 5
*/
int func_code05_handler(modbus_tcp_context *modbus_ctx, uint8_t *ADU, uint16_t length)
{
    int ret = 0;

    uint8_t unit_identifier = 0;

    if(!modbus_ctx || !ADU) {
        printf("func_code05_handler incorrect parameters\n");
        return -1;
    }

    if(length != MODBUS_TCP_FUNCTION_CODE5_ADU_LENGTH) {
        printf("func_code05_handler, incorrect ADU length\n");
        return MODBUS_EXCEPTION_CODE03;
    }

    unit_identifier = ADU[MODBUS_TCP_UNIT_IDENTIFIER_POS - 1]; 

    modbus_ctx->_register.cur_unit_identifier = unit_identifier;

    return ret;
}


/**
* @fn               func_code05_responser
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       exception_code
*                   exception code for modbus protocol
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @param [in]       tcp_write
*                   a pointer points to tcp write
* @return           =0, success; <0, fail   
* @brief            responser for modbus function code 5
*/
int func_code05_responser(modbus_tcp_context *modbus_ctx, int exception_code, uint8_t *ADU, uint16_t length, int (*tcp_write)(tcp_context *tcp_ctx, int, uint8_t *pData, int length))
{
    int ret = 0;
    uint16_t playload_length;
    uint8_t exp_code = 0, endian;
    int client_sockfd = 0;
    Register *_register = NULL;
    tcp_context *tcp_ctx = NULL;
    uint16_t pdu_length, adu_length;
    uint8_t response_pdu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    uint8_t response_adu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    
    if(!modbus_ctx || !ADU || !tcp_write || exception_code<0) {
        printf("func_code05_responser incorrect parameters\n");
        return -1;
    }

    endian = modbus_ctx->platform_endian;
    tcp_ctx = &modbus_ctx->tcp_ctx;
    client_sockfd = modbus_ctx->cur_client_sockfd;
    _register = &modbus_ctx->_register;

    if(exception_code == 0) {
        exp_code = _register->ops.write_single_coil(_register, &ADU[7], response_pdu, &pdu_length);
    } else {
        exp_code = exception_code;
    }

    switch(exp_code) {

        case MODBUS_EXCEPTION_CODE00:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + pdu_length;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             memcpy(&response_adu[MODBUS_MBAP_HEADER_LENGTH], response_pdu, pdu_length); // copy pdu
             // amend length field now
             playload_length = pdu_length + 1;
             printf("playload length = %d\n", playload_length);
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS -1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = playload_length; 
             }
             //dump_data8(response_adu, adu_length);
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;

        case MODBUS_EXCEPTION_CODE01:
        case MODBUS_EXCEPTION_CODE02:
        case MODBUS_EXCEPTION_CODE03:
        case MODBUS_EXCEPTION_CODE04:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + 2;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             // amend length field now
             playload_length = 3;
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = playload_length; 
             }
             response_adu[MODBUS_MBAP_HEADER_LENGTH] = MODBUS_FC_READ_HOLDING_REGISTERS + 0x80; // pdu initialization
             response_adu[MODBUS_MBAP_HEADER_LENGTH + 1] = exp_code;
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;

        default:
            printf("func_code05_responser, unknown exception code %d\n", exp_code);
        break;
    }

    return ret;
}


/**
* @fn               func_code06_handler
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @return           =0, success; <0, fail   
* @brief            handler for modbus function code 6
*/
int func_code06_handler(modbus_tcp_context *modbus_ctx, uint8_t *ADU, uint16_t length)
{
    int ret = 0;

    uint8_t unit_identifier = 0;

    printf("func_code06_handler\n");

    if(!modbus_ctx || !ADU) {
        printf("func_code06_handler incorrect parameters\n");
        return -1;
    }

    //dump_data8(ADU, length);

    if(length != MODBUS_TCP_FUNCTION_CODE6_ADU_LENGTH) {
        printf("func_code06_handler, incorrect ADU length\n");
        return MODBUS_EXCEPTION_CODE03;
    }

    unit_identifier = ADU[MODBUS_TCP_UNIT_IDENTIFIER_POS - 1]; 

    printf("unit_identifier %d\n", unit_identifier);

    modbus_ctx->_register.cur_unit_identifier = unit_identifier;

    return ret;
}


/**
* @fn               func_code06_responser
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       exception_code
*                   exception code for modbus protocol
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @param [in]       tcp_write
*                   a pointer points to tcp write
* @return           =0, success; <0, fail   
* @brief            responser for modbus function code 6
*/
int func_code06_responser(modbus_tcp_context *modbus_ctx, int exception_code, uint8_t *ADU, uint16_t data_len, int (*tcp_write)(tcp_context *tcp_ctx, int, uint8_t *pData, int length))
{
    int ret = 0;
    uint16_t playload_length;
    uint8_t exp_code = 0, endian;
    int client_sockfd = 0;
    Register *_register = NULL;
    tcp_context *tcp_ctx = NULL;
    uint16_t pdu_length, adu_length;
    uint8_t response_pdu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    uint8_t response_adu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    
    printf("func_code06_responser\n");

    if(!modbus_ctx || !ADU || !tcp_write || exception_code<0) {
        printf("func_code06_responser incorrect parameters\n");
        return -1;
    }

    endian = modbus_ctx->platform_endian;
    tcp_ctx = &modbus_ctx->tcp_ctx;
    client_sockfd = modbus_ctx->cur_client_sockfd;
    _register = &modbus_ctx->_register;

    if(exception_code == 0) {
        exp_code = _register->ops.write_single_register(_register, &ADU[7], response_pdu, &pdu_length);
    } else {
        exp_code = exception_code;
    }

    switch(exp_code) {

        case MODBUS_EXCEPTION_CODE00:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + pdu_length;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             memcpy(&response_adu[MODBUS_MBAP_HEADER_LENGTH], response_pdu, pdu_length); // copy pdu
             // amend length field now
             playload_length = pdu_length + 1;
             printf("playload length = %d\n", playload_length);
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS -1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = playload_length; 
             }
             //dump_data8(response_adu, adu_length);
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;

        case MODBUS_EXCEPTION_CODE01:
        case MODBUS_EXCEPTION_CODE02:
        case MODBUS_EXCEPTION_CODE03:
        case MODBUS_EXCEPTION_CODE04:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + 2;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             // amend length field now
             playload_length = 3;
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = playload_length; 
             }
             response_adu[MODBUS_MBAP_HEADER_LENGTH] = MODBUS_FC_READ_HOLDING_REGISTERS + 0x80; // pdu initialization
             response_adu[MODBUS_MBAP_HEADER_LENGTH + 1] = exp_code;
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;

        default:
            printf("func_code05_responser, unknown exception code %d\n", exp_code);
        break;
    }

    return ret;
}

/**
* @fn               func_code16_handler
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @return           =0, success; <0, fail   
* @brief            handler for modbus function code 16
*/
int func_code16_handler(modbus_tcp_context *modbus_ctx, uint8_t *ADU, uint16_t length)
{
    int ret = 0;
    uint8_t unit_identifier = 0;
    int total_length = -1, data_length = -1;
    

    if(!modbus_ctx || !ADU) {
        printf("func_code16_handler incorrect parameters\n");
        return -1;
    }

    // compute total length first, check if it's a valid mobus tcp message
    if(length < MODBUS_MBAP_HEADER_LENGTH + 6) {
        return MODBUS_EXCEPTION_CODE03;
    }

    data_length = ADU[MODBUS_TCP_WR_MUL_REG_BYTE_COUNT_POS - 1];

    total_length = MODBUS_MBAP_HEADER_LENGTH + 6 + data_length;

    if(length < total_length) {
        printf("func_code16_handler, incorrect ADU length\n");
        return MODBUS_EXCEPTION_CODE03;
    }

    unit_identifier = ADU[MODBUS_TCP_UNIT_IDENTIFIER_POS - 1]; 

    modbus_ctx->_register.cur_unit_identifier = unit_identifier;

    return ret;

}


/**
* @fn               func_code16_responser
* @param [in]       modbus_ctx
*                   modbus context
* @param [in]       exception_code
*                   exception code for modbus protocol
* @param [in]       ADU
*                   modbus Application Data Unit
* @param [in]       length
*                   length of ADU
* @param [in]       tcp_write
*                   a pointer points to tcp write
* @return           =0, success; <0, fail   
* @brief            responser for modbus function code 16
*/
int func_code16_responser(modbus_tcp_context *modbus_ctx, int exception_code, uint8_t *ADU, uint16_t data_len, int (*tcp_write)(tcp_context *tcp_ctx, int, uint8_t *pData, int length))
{
    int ret = 0;
    uint16_t playload_length;
    uint8_t exp_code = 0, endian;
    int client_sockfd = 0;
    Register *_register = NULL;
    tcp_context *tcp_ctx = NULL;
    uint16_t pdu_length, adu_length;
    uint8_t response_pdu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
    uint8_t response_adu[MODBUS_MAX_TCP_ADU_LENGTH + 1];
     
    printf("func_code16_responser\n");
    
    if(!modbus_ctx || !ADU || !tcp_write || exception_code<0) {
        printf("func_code16_responser incorrect parameters\n");
        return -1;
    }
    
    endian = modbus_ctx->platform_endian;
    tcp_ctx = &modbus_ctx->tcp_ctx;
    client_sockfd = modbus_ctx->cur_client_sockfd;
    _register = &modbus_ctx->_register;
    
    if(exception_code == 0) {
        exp_code = _register->ops.write_multiple_registers(_register, &ADU[7], response_pdu, &pdu_length);
    } else {
        exp_code = exception_code;
    }
    
    switch(exp_code) {
    
        case MODBUS_EXCEPTION_CODE00:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + pdu_length;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             memcpy(&response_adu[MODBUS_MBAP_HEADER_LENGTH], response_pdu, pdu_length); // copy pdu
             // amend length field now
             playload_length = pdu_length + 1;
             printf("playload length = %d\n", playload_length);
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS -1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = playload_length; 
             }
             //dump_data8(response_adu, adu_length);
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;
    
        case MODBUS_EXCEPTION_CODE01:
        case MODBUS_EXCEPTION_CODE02:
        case MODBUS_EXCEPTION_CODE03:
        case MODBUS_EXCEPTION_CODE04:
             adu_length = MODBUS_MBAP_HEADER_LENGTH + 2;
             memcpy(response_adu, ADU, MODBUS_MBAP_HEADER_LENGTH); // copy MBAP header
             // amend length field now
             playload_length = 3;
             if(endian == __LITTLE_ENDIAN__) {
                 // Little-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS - 1] = (playload_length & 0xFF00)>>8;
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = (playload_length & 0xFF);
             } else {
                 // Big-Endian
                 response_adu[MODBUS_TCP_PLAYLOAD_LENGTH_POS] = playload_length; 
             }
             response_adu[MODBUS_MBAP_HEADER_LENGTH] = MODBUS_FC_READ_HOLDING_REGISTERS + 0x80; // pdu initialization
             response_adu[MODBUS_MBAP_HEADER_LENGTH + 1] = exp_code;
             tcp_write(tcp_ctx, client_sockfd, response_adu, adu_length);
        break;
    
        default:
             printf("func_code16_responser, unknown exception code %d\n", exp_code);
        break;
    }
    
    return ret;

}


