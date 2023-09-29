/**
* @file     function_code_handler.h
* @brief    This file implements handlers of supported function codes
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     27/09/2018
*/

#ifndef __FUNCTION_CODE_HANDLER_H__
#define __FUNCTION_CODE_HANDLER_H__

int func_code01_handler(modbus_tcp_context *, uint8_t *ADU, uint16_t length);
int func_code01_responser(modbus_tcp_context *, int exception_code, uint8_t *ADU, uint16_t data_len, int (*tcp_write)(tcp_context *, int, uint8_t *pData, int length));

int func_code02_handler(modbus_tcp_context *modbus_ctx, uint8_t *ADU, uint16_t length);
int func_code02_responser(modbus_tcp_context *modbus_ctx, int exception_code, uint8_t *ADU, uint16_t length, int (*tcp_write)(tcp_context *tcp_ctx, int, uint8_t *pData, int length));

int func_code03_handler(modbus_tcp_context *, uint8_t *ADU, uint16_t length);
int func_code03_responser(modbus_tcp_context *, int exception_code, uint8_t *ADU, uint16_t data_len, int (*tcp_write)(tcp_context *, int, uint8_t *pData, int length));

int func_code04_handler(modbus_tcp_context *modbus_ctx, uint8_t *ADU, uint16_t length);
int func_code04_responser(modbus_tcp_context *modbus_ctx, int exception_code, uint8_t *ADU, uint16_t length, int (*tcp_write)(tcp_context *tcp_ctx, int, uint8_t *pData, int length));

int func_code05_handler(modbus_tcp_context *, uint8_t *ADU, uint16_t length);
int func_code05_responser(modbus_tcp_context *, int exception_code, uint8_t *ADU, uint16_t data_len, int (*tcp_write)(tcp_context *, int, uint8_t *pData, int length));

int func_code06_handler(modbus_tcp_context *, uint8_t *ADU, uint16_t length);
int func_code06_responser(modbus_tcp_context *, int exception_code, uint8_t *ADU, uint16_t data_len, int (*tcp_write)(tcp_context *, int, uint8_t *pData, int length));

int func_code16_handler(modbus_tcp_context *modbus_ctx, uint8_t *ADU, uint16_t length);
int func_code16_responser(modbus_tcp_context *modbus_ctx, int exception_code, uint8_t *ADU, uint16_t data_len, int (*tcp_write)(tcp_context *tcp_ctx, int, uint8_t *pData, int length));

#endif

