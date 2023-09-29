#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "log_controller.h"
#include "i2c_controller.h"

#include "ssd1306_oled_controller.h"
#include "hal_ssd1306_oled.h"
#include "ssd1306_oled_raspberry_pi.h"
//#include "spi_controller.h"

#include "debug.h"




int hal_ssd1306_oled_init(register_ssd1306_oled_parameters *ssd1306_oled_param)
{
    int ret = 0, count, i;
    char *_ssd1306_oled_signature = NULL;
    void *i2c_controller_ctx = NULL;
    void *i2c_controller_ioctl_param = NULL;

    register_ssd1306_oled_parameters *_ssd1306_oled_param = ssd1306_oled_param;

    if(!_ssd1306_oled_param) {
        printf("hal_ssd1306_oled_init fail, incorrect parameters\n");
        return -1;
    }

    _ssd1306_oled_signature = _ssd1306_oled_param->in.signature;

    i2c_controller_ctx = _ssd1306_oled_param->in.i2c_controller_ctx;
    if(!i2c_controller_ctx) {
        printf("hal_ssd1306_oled_init fail, incorrect controller_ctx\n");
        return -1;
    }

    i2c_controller_ioctl_param = _ssd1306_oled_param->in.i2c_controller_ioctl_param;
    if(!i2c_controller_ioctl_param) {
        printf("hal_ssd1306_oled_init fail, incorrect controller_ioctl_param\n");
        return -1;
    }

    if(!strcmp(_ssd1306_oled_signature, SSD1306_OLED1_DEVICE_SIGNATURE)) {
        
    } else {
        printf("unknown signature %s\n", _ssd1306_oled_signature);
        return -1;
    }

#ifdef DATA_FLOW_DEBUG
    printf("come hal_ssd1306_oled_init->ssd1306_oled_init\n");
    return 0;
#endif


    ret = ssd1306_oled_init(ssd1306_oled_param);
    if(ret < 0) {
        printf("hal_ssd1306_oled_init->ssd1306_oled_init fail\n");
        return -1;
    }

    return ret;
}

int hal_ssd1306_oled_deinit(unregister_ssd1306_oled_parameters *ssd1306_oled_param)
{
    int ret = 0;

#ifdef DATA_FLOW_DEBUG
        printf("come hal_ssd1306_oled_deinit\n");
        return 0;
#endif

    ret = ssd1306_oled_deinit(ssd1306_oled_param);
    if(ret < 0) {
        printf("hal_ssd1306_oled_deinit->ssd1306_oled_deinit fail\n");
        return -1;
    }

    return 0;

}


int hal_ssd1306_oled_ioctl(ssd1306_oled_ioctl_parameters *ssd1306_oled_ioctl_param)
{
    int ret = 0, cmd, _cmd, i;
    void *i2c_controller_ctx = NULL;
    ssd1306_oled_ioctl_parameters *_ssd1306_oled_ioctl_param = ssd1306_oled_ioctl_param;
    void *controller_ioctl_param = NULL;
    uint8_t *byte, *row_idx, *column_idx;
    char *_char;
    uint8_t *char_row_idx, *char_column_idx;

    if(!ssd1306_oled_ioctl_param) {
        printf("hal_ssd1306_oled_ioctl fail, incorrect parameters\n");
        return -1;
    }

    _cmd = _ssd1306_oled_ioctl_param->cmd;

    switch(_cmd){

        case SSD1306_OLED_IOCTL_CMD_WRITE_BYTE:

            i2c_controller_ctx = _ssd1306_oled_ioctl_param->write_byte_param.temp.i2c_controller_ctx;
            if(!i2c_controller_ctx) {
                printf("hal_ssd1306_oled_ioctl->write_byte_param fail, incorrect parameters i2c_controller_ctx\n");
                return -1;
            }
            
            controller_ioctl_param = _ssd1306_oled_ioctl_param->write_byte_param.temp.i2c_controller_ioctl_param;
            if(!controller_ioctl_param) {
                printf("hal_ssd1306_oled_ioctl->write_byte_param fail, incorrect parameters controller_ioctl_param\n");
                return -1;
            }

#if 0
            byte = _ssd1306_oled_ioctl_param->write_byte_param.in.byte;
            if((*value != SSD1306_OLED_POINT_ENABLE) && (*value != SSD1306_OLED_POINT_DISABLE)) {
                printf("hal_ssd1306_oled_ioctl->write_byte_param fail, incorrect parameters value:%d\n", *value);
                return -1;
            }
#endif

            row_idx = _ssd1306_oled_ioctl_param->write_byte_param.in.row_idx;
            if((*row_idx < 0) || (*row_idx > SSD1306_OLED_BYTE_MAX_ROW_IDX)) {
                printf("hal_ssd1306_oled_ioctl->write_byte_param fail, incorrect parameters row_idx:%d\n", *row_idx);
                return -1;
            }

            column_idx = _ssd1306_oled_ioctl_param->write_byte_param.in.column_idx;
            if((*column_idx < 0) || (*column_idx > SSD1306_OLED_BYTE_MAX_COLUMN_IDX)) {
                printf("hal_ssd1306_oled_ioctl->write_byte_param fail, incorrect parameters column_idx:%d\n", *column_idx);
                return -1;
            }

#ifdef DATA_FLOW_DEBUG
                    printf("come hal_ssd1306_oled_ioctl SSD1306_OLED_IOCTL_CMD_WRITE_BYTE\n");
                    return 0;
#endif

            ret = ssd1306_oled_ioctl(_ssd1306_oled_ioctl_param);
            if(ret < 0) {
                printf("hal_ssd1306_oled_ioctl->ssd1306_oled_ioctl\n");
                return -1;
            }

            return 0;

        break;

        case SSD1306_OLED_IOCTL_CMD_WRITE_CHAR:
    
            i2c_controller_ctx = _ssd1306_oled_ioctl_param->write_char_param.temp.i2c_controller_ctx;
            if(!i2c_controller_ctx) {
                printf("hal_ssd1306_oled_ioctl->write_char_param fail, incorrect parameters i2c_controller_ctx\n");
                return -1;
            }
            
            controller_ioctl_param = _ssd1306_oled_ioctl_param->write_char_param.temp.i2c_controller_ioctl_param;
            if(!controller_ioctl_param) {
                printf("hal_ssd1306_oled_ioctl->write_char_param fail, incorrect parameters controller_ioctl_param\n");
                return -1;
            }

#if 0
            _char = _ssd1306_oled_ioctl_param->write_char_param.in._char;
            if(*_char != SSD1306_OLED_POINT_ENABLE && *_char != SSD1306_OLED_POINT_DISABLE) {
                printf("hal_ssd1306_oled_ioctl->write_char_param fail, incorrect parameters _char:%d\n", _char);
                return -1;
            }
#endif
            
            char_row_idx = _ssd1306_oled_ioctl_param->write_char_param.in.char_row_idx;
            if((*char_row_idx < 0) || (*char_row_idx > SSD1306_OLED_CHAR_MAX_ROW_IDX)) {
                printf("hal_ssd1306_oled_ioctl->write_char_param fail, incorrect parameters char_row_idx:%d\n", *char_row_idx);
                return -1;
            }
    
            char_column_idx = _ssd1306_oled_ioctl_param->write_char_param.in.char_column_idx;
            if((*char_column_idx < 0) || (*char_column_idx > SSD1306_OLED_CHAR_MAX_COLUMN_IDX)) {
                printf("hal_ssd1306_oled_ioctl->write_char_param fail, incorrect parameters column_idx:%d\n", *char_column_idx);
                return -1;
            }

#ifdef DATA_FLOW_DEBUG
            printf("come hal_ssd1306_oled_ioctl SSD1306_OLED_IOCTL_CMD_WRITE_CHAR\n");
            return 0;
#endif

            ret = ssd1306_oled_ioctl(_ssd1306_oled_ioctl_param);
            if(ret < 0) {
                printf("hal_ssd1306_oled_ioctl->ssd1306_oled_ioctl\n");
                return -1;
            }

            return 0;
    
        break;

        case SSD1306_OLED_IOCTL_CMD_START_SCOLL:
    
            i2c_controller_ctx = _ssd1306_oled_ioctl_param->start_scoll_param.temp.i2c_controller_ctx;
            if(!i2c_controller_ctx) {
                printf("hal_ssd1306_oled_ioctl->write_char_param fail, incorrect parameters i2c_controller_ctx\n");
                return -1;
            }
            
            controller_ioctl_param = _ssd1306_oled_ioctl_param->start_scoll_param.temp.i2c_controller_ioctl_param;
            if(!controller_ioctl_param) {
                printf("hal_ssd1306_oled_ioctl->write_char_param fail, incorrect parameters controller_ioctl_param\n");
                return -1;
            }

            ret = ssd1306_oled_ioctl(_ssd1306_oled_ioctl_param);
            if(ret < 0) {
                printf("hal_ssd1306_oled_ioctl->ssd1306_oled_ioctl\n");
                return -1;
            }

            return 0;
    
        break;

        case SSD1306_OLED_IOCTL_CMD_STOP_SCOLL:
    
            i2c_controller_ctx = _ssd1306_oled_ioctl_param->stop_scoll_param.temp.i2c_controller_ctx;
            if(!i2c_controller_ctx) {
                printf("hal_ssd1306_oled_ioctl->write_char_param fail, incorrect parameters i2c_controller_ctx\n");
                return -1;
            }
            
            controller_ioctl_param = _ssd1306_oled_ioctl_param->stop_scoll_param.temp.i2c_controller_ioctl_param;
            if(!controller_ioctl_param) {
                printf("hal_ssd1306_oled_ioctl->write_char_param fail, incorrect parameters controller_ioctl_param\n");
                return -1;
            }

            ret = ssd1306_oled_ioctl(_ssd1306_oled_ioctl_param);
            if(ret < 0) {
                printf("hal_ssd1306_oled_ioctl->ssd1306_oled_ioctl\n");
                return -1;
            }

            return 0;
    
        break;


        default:
            printf("hal_ssd1306_oled_ioctl fail, un-known cmd %d\n", _cmd);
            return -1;
        break;

    }

    return ret;
}



