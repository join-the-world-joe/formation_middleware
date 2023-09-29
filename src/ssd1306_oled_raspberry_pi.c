#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "log_controller.h"
#include "i2c_controller.h"

#include "ssd1306_oled_controller.h"
#include "ssd1306_oled_raspberry_pi.h"

#include "char_matrix.h"


#include "debug.h"

int ssd1306_oled_choose_GDDRAM_start_position(ssd1306_oled_ioctl_parameters *);
int ssd1306_oled_deint(i2c_controller_context *, i2c_controller_ioctl_parameters *, uint8_t *);



int ssd1306_oled_choose_GDDRAM_start_position(ssd1306_oled_ioctl_parameters *ssd1306_oled_param)
{
    int ret = 0, cmd, _cmd;
    i2c_controller_context *_controller_ctx = NULL;
    i2c_controller_ioctl_parameters *_controller_ioctl_param = NULL;
    ssd1306_oled_ioctl_parameters *_ssd1306_oled_ioctl_param = ssd1306_oled_param;
    uint8_t start_page_idx, end_page_idx, start_column_idx, end_column_idx;
    uint8_t row_idx, column_idx;
    uint8_t char_row_idx, char_column_idx;

    _cmd = _ssd1306_oled_ioctl_param->cmd;

    switch(_cmd) {
        
        case SSD1306_OLED_IOCTL_CMD_WRITE_BYTE:
            row_idx = *ssd1306_oled_param->write_byte_param.in.row_idx;
            column_idx = *ssd1306_oled_param->write_byte_param.in.column_idx;
            _controller_ctx = ssd1306_oled_param->write_byte_param.temp.i2c_controller_ctx;
            _controller_ioctl_param = ssd1306_oled_param->write_byte_param.temp.i2c_controller_ioctl_param;

            start_page_idx = row_idx * SSD1306_OLED_BYTE_ROW_TO_START_PAGE_UNIT_FACTOR;
            end_page_idx = start_page_idx + SSD1306_OLED_BYTE_START_PAGE_TO_END_PAGE_OFFSET;

            start_column_idx = column_idx * SSD1306_OLED_BYTE_COLUMN_TO_START_COLUMN_UNIT_FACTOR;
            end_column_idx = start_column_idx + SSD1306_OLED_BYTE_START_COLUMN_TO_END_COLUMN_OFFSET;

            if(start_page_idx > SSD1306_OLED_MAX_PAGE_IDX || end_page_idx > SSD1306_OLED_MAX_PAGE_IDX) {
                printf("ssd1306_oled_choose_GDDRAM_START_POSITION SSD1306_OLED_IOCTL_CMD_WRITE_BYTE page out of range\n");
                return -1;
            }

#if 0
            printf("start_page_idx %d\n", start_page_idx);
            printf("end_page_idx %d\n", end_page_idx);
            printf("start_column_idx %d\n", start_column_idx);
            printf("end_column_idx %d\n", end_column_idx);
#endif

            if(start_column_idx > SSD1306_OLED_MAX_COLUMN_IDX || end_column_idx > SSD1306_OLED_MAX_COLUMN_IDX) {
                printf("ssd1306_oled_choose_GDDRAM_START_POSITION SSD1306_OLED_IOCTL_CMD_WRITE_BYTE column out of range\n");
                return -1;
            }

            //choose page
            cmd = I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES;
            _controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
            _controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_PAGE_ADDRESS_HEAD;
            _controller_ioctl_param->write_multi_bytes_param.in.data[2] = start_page_idx;
            _controller_ioctl_param->write_multi_bytes_param.in.data[3] = end_page_idx;
            _controller_ioctl_param->write_multi_bytes_param.in.length = 4;
            
            ret = _controller_ctx->ops.ioctl(_controller_ctx, cmd, _controller_ioctl_param);
            if(ret < 0) {
                printf("ssd1306_oled_choose_GDDRAM_START_POSITION->_controller_ctx->ioctl fail choose column\n");
                return -1;
            }

            //choose column
            cmd = I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES;
            _controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
            _controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_COLUMN_ADDRESS_HEAD;
            _controller_ioctl_param->write_multi_bytes_param.in.data[2] = start_column_idx;
            _controller_ioctl_param->write_multi_bytes_param.in.data[3] = end_column_idx;
            _controller_ioctl_param->write_multi_bytes_param.in.length = 4;
            
            ret = _controller_ctx->ops.ioctl(_controller_ctx, cmd, _controller_ioctl_param);
            if(ret < 0) {
                printf("ssd1306_oled_choose_GDDRAM_START_POSITION->_controller_ctx->ioctl fail choose column\n");
                return -1;
            }

            return 0;

        break;
   
        case SSD1306_OLED_IOCTL_CMD_WRITE_CHAR:
            char_row_idx = *ssd1306_oled_param->write_char_param.in.char_row_idx;
            char_column_idx = *ssd1306_oled_param->write_char_param.in.char_column_idx;
            _controller_ctx = ssd1306_oled_param->write_char_param.temp.i2c_controller_ctx;
            _controller_ioctl_param = ssd1306_oled_param->write_char_param.temp.i2c_controller_ioctl_param;
        
            start_page_idx = char_row_idx * SSD1306_OLED_CHAR_ROW_TO_START_PAGE_UNIT_FACTOR;
            end_page_idx = start_page_idx + SSD1306_OLED_CHAR_START_PAGE_TO_END_PAGE_OFFSET;
        
            start_column_idx = char_column_idx * SSD1306_OLED_CHAR_COLUMN_TO_START_COLUMN_UNIT_FACTOR;
            end_column_idx = start_column_idx + SSD1306_OLED_CHAR_START_COLUMN_TO_END_COLUMN_OFFSET;
        
            if(start_page_idx > SSD1306_OLED_MAX_PAGE_IDX || end_page_idx > SSD1306_OLED_MAX_PAGE_IDX) {
                printf("ssd1306_oled_choose_GDDRAM_START_POSITION SSD1306_OLED_IOCTL_CMD_WRITE_BYTE page out of range\n");
                return -1;
            }
        
            if(start_column_idx > SSD1306_OLED_MAX_COLUMN_IDX || end_column_idx > SSD1306_OLED_MAX_COLUMN_IDX) {
                printf("ssd1306_oled_choose_GDDRAM_START_POSITION SSD1306_OLED_IOCTL_CMD_WRITE_BYTE column out of range\n");
                return -1;
            }

#if 0
            printf("start_page_idx %d\n", start_page_idx);
            printf("end_page_idx %d\n", end_page_idx);
            printf("start_column_idx %d\n", start_column_idx);
            printf("end_column_idx %d\n", end_column_idx);
#endif

            //choose page
            cmd = I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES;
            _controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
            _controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_PAGE_ADDRESS_HEAD;
            _controller_ioctl_param->write_multi_bytes_param.in.data[2] = start_page_idx;
            _controller_ioctl_param->write_multi_bytes_param.in.data[3] = end_page_idx;
            _controller_ioctl_param->write_multi_bytes_param.in.length = 4;
            
            ret = _controller_ctx->ops.ioctl(_controller_ctx, cmd, _controller_ioctl_param);
            if(ret < 0) {
                printf("ssd1306_oled_choose_GDDRAM_START_POSITION->_controller_ctx->ioctl fail choose column\n");
                return -1;
            }

            //choose column
            cmd = I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES;
            _controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
            _controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_COLUMN_ADDRESS_HEAD;
            _controller_ioctl_param->write_multi_bytes_param.in.data[2] = start_column_idx;
            _controller_ioctl_param->write_multi_bytes_param.in.data[3] = end_column_idx;
            _controller_ioctl_param->write_multi_bytes_param.in.length = 4;
            
            ret = _controller_ctx->ops.ioctl(_controller_ctx, cmd, _controller_ioctl_param);
            if(ret < 0) {
                printf("ssd1306_oled_choose_GDDRAM_START_POSITION->_controller_ctx->ioctl fail choose column\n");
                return -1;
            }
        
            return 0;
        
        break;

        default:
            printf("ssd1306_oled_ioctl fail, un-known cmd %d\n", _cmd);
            return -1;
        break;
    }

}

int ssd1306_oled_deint(i2c_controller_context *i2c_controller_ctx, i2c_controller_ioctl_parameters *i2c_controller_ioctl_param, uint8_t *dev_addr)
{
    int cmd, ret, i;
    i2c_controller_context *_i2c_controller_ctx = NULL;
    i2c_controller_ioctl_parameters *_i2c_controller_ioctl_param = NULL;

    _i2c_controller_ctx = i2c_controller_ctx;
    _i2c_controller_ioctl_param = i2c_controller_ioctl_param;
    _i2c_controller_ioctl_param->write_multi_bytes_param.in.i2c_dev_addr = *dev_addr;

    cmd = I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES;

    //close
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_ONY_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_SINGLE_BYTE_SET_DISPLAY_OFF_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 2;

        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail\n");
            return -1;
        }

    //close scoll
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_ONY_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_SINGLE_BYTE_DEACTIVATE_SCROLL_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 2;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail\n");
            return -1;
        }

    //config
    //continue cmd
        //memory mode page
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_MEMORY_ADDRESSING_MODE_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[2] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_MEMORY_ADDRESSING_MODE_SETTING_BYTE1_VERTICAL_ADDRESSING_MODE;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 3;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail memory mode page\n");
            return -1;
        }

        //Setup column start and end address
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_COLUMN_ADDRESS_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[2] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_COLUMN_ADDRESS_SETTING_BYTE1_COLUMN_START_ADDRESS;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[3] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_COLUMN_ADDRESS_SETTING_BYTE2_COLUMN_END_ADDRESS;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 4;

        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail Setup column start and end address\n");
            return -1;
        }

        //Setup page start and end address
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_PAGE_ADDRESS_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[2] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_PAGE_ADDRESS_SETTING_BYTE1_0PAGE_START_ADDRESS;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[3] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_PAGE_ADDRESS_SETTING_BYTE2_7PAGE_END_ADDRESS;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 4;

        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail Setup page start and end address\n");
            return -1;
        }

        //Display Start linue Address
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_ONY_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_SINGLE_BYTE_SET_DISPLAY_START_LINE_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 2;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail Display Start linue Address\n");
            return -1;
        }

        //light level
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_CONTRAST_CONTROL_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[2] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_CONTRAST_CONTROL_SETTING_BYTE1_CONTRAST_LEVEL;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 3;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail light level\n");
            return -1;
        }

        //Segment Re-map
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_ONY_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_SINGLE_BYTE_SET_SEGMENT_REMAP_COLUMN_127_SEG0_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 2;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail Segment Re-map\n");
            return -1;
        }

        //GDDRAM 0 mean off
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_ONY_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_SINGLE_BYTE_SET_NORMAL_DISPLAY_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 2;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail GDDRAM 0 mean off\n");
            return -1;
        }

        //Multiplex Ratio
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_MULTIPLEX_RATIO_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[2] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_MULTIPLEX_RATIO_SETTING_BYTE1_MUX_RATIO;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 3;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail Multiplex Ratio\n");
            return -1;
        }

        //Display Offset
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_DISPLAY_OFFSET_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[2] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_DISPLAY_OFFSET_SETTING_BYTE1_SHIFT_BY_COM;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 3;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail Display Offset\n");
            return -1;
        }

        //Set Osc Frequency
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_DISPLAY_CLOCK_DIVIDE_OSCILLATOR_FREQUENCY_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[2] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_DISPLAY_CLOCK_DIVIDE_OSCILLATOR_FREQUENCY_SETTING_BYTE1_DIVIDE_RATIO;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 3;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail Set Osc Frequency\n");
            return -1;
        }

        //Pre-charge Period
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_PRE_CHARGE_PERIOD_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[2] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_PRE_CHARGE_PERIOD_SETTING_BYTE1_DIVIDE_RATIO;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 3;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail Pre-charge Period\n");
            return -1;
        }

        //Set COM Pins Hardware Configuration
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_COM_PINS_HARDWARE_CONFIGURATION_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[2] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_COM_PINS_HARDWARE_CONFIGURATION_SETTING_BYTE1_ALTERNATIVE_COM_AND_DISABLE_COM_LEFT_RIGHT_REMAP;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 3;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail Charge Pump Setting\n");
            return -1;
        }


        //VCOMH Deselect Level
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_VCOMH_DESELECT_LEVEL_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[2] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_SET_VCOMH_DESELECT_LEVEL_SETTING_BYTE1_VCOMH_DESELECT_LEVEL2;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 3;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail VCOMH Deselect Level\n");
            return -1;
        }

        //Charge Pump Setting
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_CHARGE_PUMP_SETTING_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[2] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_CHARGE_PUMP_SETTING_SETTING_BYTE1_ENABLE_CHARGE_PUMP;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 3;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail Charge Pump Setting\n");
            return -1;
        }

        //COM Output Scan Direction
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_ONY_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_SINGLE_BYTE_SET_COM_OUTPUT_SCAN_DIRECTION_COMN_TO_COM0_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 2;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail COM Output Scan Direction\n");
            return -1;
        }

        //GDDRAM OUTPUT FOLLOWING
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_ONY_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_SINGLE_BYTE_ENTIRE_DISPLAY_ON_FOLLOWS_RAM_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 2;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail GDDRAM OUTPUT FOLLOWING\n");
            return -1;
        }

        //Continuous Horizontal Scroll Setup
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_CONTINUOUS_HORIZONTAL_SCROLL_SETUP_RIGHT_HORIZONTAL_SCROLL_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[2] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_CONTINUOUS_HORIZONTAL_SCROLL_SETUP_RIGHT_HORIZONTAL_SCROLL_SETTING_BYTE1_DUMMY_BYTE;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[3] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_CONTINUOUS_HORIZONTAL_SCROLL_SETUP_RIGHT_HORIZONTAL_SCROLL_SETTING_BYTE2_START_PAGE_ADDRESS;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[4] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_CONTINUOUS_HORIZONTAL_SCROLL_SETUP_RIGHT_HORIZONTAL_SCROLL_SETTING_BYTE3_TIME_INTERVAL;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[5] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_CONTINUOUS_HORIZONTAL_SCROLL_SETUP_RIGHT_HORIZONTAL_SCROLL_SETTING_BYTE4_END_PAGE_ADDRESS;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[6] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_CONTINUOUS_HORIZONTAL_SCROLL_SETUP_RIGHT_HORIZONTAL_SCROLL_SETTING_BYTE5_DUMMY_BYTE;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[7] = SSD1306_OLED_DATA_BYTE_CMD_MULTI_BYTE_CONTINUOUS_HORIZONTAL_SCROLL_SETUP_RIGHT_HORIZONTAL_SCROLL_SETTING_BYTE6_DUMMY_BYTE;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 8;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail Charge Pump Setting\n");
            return -1;
        }

#if 0
        //Activate scroll
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_ONY_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_SINGLE_BYTE_ACTIVATE_SCROLL_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 2;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail Charge Pump Setting\n");
            return -1;
        }
#endif

    //wake up
    //wake up
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_ONY_BYTE_CMD_HEAR;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_SINGLE_BYTE_SET_DISPLAY_ON_HEAD;
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 2;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail wake up\n");
            return -1;
        }

    //set all menory to 1
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_DATA_HEAR;
        for(i = 0;i < SSD1306_OLED_PIXEL_IN_BYTE;i++) {
            _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[i+1] = 0xff;
        }
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 1 + SSD1306_OLED_PIXEL_IN_BYTE;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail clean all menory\n");
            return -1;
        }

        sleep(1);

    //clean all menory
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_DATA_HEAR;
        for(i = 0;i < SSD1306_OLED_PIXEL_IN_BYTE;i++) {
            _i2c_controller_ioctl_param->write_multi_bytes_param.in.data[i+1] = 0x00;
        }
        _i2c_controller_ioctl_param->write_multi_bytes_param.in.length = 1 + SSD1306_OLED_PIXEL_IN_BYTE;
        
        ret = i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
        if(ret < 0) {
            printf("ssd1306_oled_deint->i2c_controller->ioctl fail clean all menory\n");
            return -1;
        }

    return ret;
}


int ssd1306_oled_init(register_ssd1306_oled_parameters *ssd1306_oled_param)
{
    int ret = 0, cmd;
    char *_signature;
    uint8_t *dev_addr;

    i2c_controller_context *_i2c_controller_ctx = NULL;
    register_ssd1306_oled_parameters *_ssd1306_oled_param = ssd1306_oled_param;
    i2c_controller_ioctl_parameters *_i2c_controller_ioctl_param = NULL;
    
    _i2c_controller_ctx = _ssd1306_oled_param->in.i2c_controller_ctx;
    _i2c_controller_ioctl_param = _ssd1306_oled_param->in.i2c_controller_ioctl_param;

    if(!_i2c_controller_ctx || !ssd1306_oled_param || !_i2c_controller_ioctl_param) {
        printf("ssd1306_oled fail, incorrect parameter\n");
        return -1;
    }

    _signature = ssd1306_oled_param->in.signature;
    dev_addr = &ssd1306_oled_param->out.dev_addr;

    if(strcmp(_signature, SSD1306_OLED1_DEVICE_SIGNATURE) == 0) {
        *dev_addr = SSD1306_OLED1_DEVICE_SIGNATURE_SSD1306_OLED_I2C_SLAVE_ADDR;
    } else {
        printf("unknown signature %s\n", _signature);
        return -1;
    }

    ret = ssd1306_oled_deint(_i2c_controller_ctx, _i2c_controller_ioctl_param, dev_addr);
    if(ret != 0){
        printf("ssd1306_oled_init->ssd1306_oled_deint fail\n");
        return ret;
    }

    return 0;

}

int ssd1306_oled_deinit(unregister_ssd1306_oled_parameters *ssd1306_oled_param)
{
    int ret = 0;

    return ret;
}


int ssd1306_oled_ioctl(ssd1306_oled_ioctl_parameters *ssd1306_oled_param)
{
    int ret = 0, cmd, _cmd, i;
    i2c_controller_context *_controller_ctx = NULL;
    i2c_controller_ioctl_parameters *_controller_ioctl_param = NULL;
    ssd1306_oled_ioctl_parameters *_ssd1306_oled_ioctl_param = ssd1306_oled_param;
    uint8_t byte, char_matrix_byte_array[CHAR_MATRIX_CHAR_MATRIX_SIZE_IN_BYTE];
    char _char;

    _cmd = _ssd1306_oled_ioctl_param->cmd;

    switch(_cmd) {

        case SSD1306_OLED_IOCTL_CMD_WRITE_BYTE:
            byte = *ssd1306_oled_param->write_byte_param.in.byte;
            _controller_ctx = ssd1306_oled_param->write_byte_param.temp.i2c_controller_ctx;
            _controller_ioctl_param = ssd1306_oled_param->write_byte_param.temp.i2c_controller_ioctl_param;

            ret = ssd1306_oled_choose_GDDRAM_start_position(ssd1306_oled_param);
            if(ret < 0) {
                printf("ssd1306_oled_ioctl->ssd1306_oled_choose_GDDRAM_START_POSITION fail\n");
                return -1;
            }

            cmd = I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES;
            _controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_ONY_BYTE_DATA_HEAR;
            _controller_ioctl_param->write_multi_bytes_param.in.data[1] = byte;
            _controller_ioctl_param->write_multi_bytes_param.in.length = 2;
            
            ret = _controller_ctx->ops.ioctl(_controller_ctx, cmd, _controller_ioctl_param);
            if(ret < 0) {
                printf("ssd1306_oled_choose_GDDRAM_START_POSITION->_controller_ctx->ioctl fail choose column\n");
                return -1;
            }
            
            return 0;
        break;

        case SSD1306_OLED_IOCTL_CMD_WRITE_CHAR:
            _char = *ssd1306_oled_param->write_char_param.in._char;
            _controller_ctx = ssd1306_oled_param->write_char_param.temp.i2c_controller_ctx;
            _controller_ioctl_param = ssd1306_oled_param->write_char_param.temp.i2c_controller_ioctl_param;

            ret = ssd1306_oled_choose_GDDRAM_start_position(ssd1306_oled_param);
            if(ret < 0) {
                printf("ssd1306_oled_ioctl->ssd1306_oled_choose_GDDRAM_START_POSITION fail\n");
                return -1;
            }

            ret = get_char_matrix(_char, char_matrix_byte_array);
            if(ret < 0) {
                printf("ssd1306_oled_ioctl->get_char_maxtrx fail unknow char %c\n", _char);
                return -1;
            }
            cmd = I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES;
            _controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_CONTINUE_BYTE_DATA_HEAR;
            for(i = 0;i < CHAR_MATRIX_CHAR_MATRIX_SIZE_IN_BYTE;i++) {
                _controller_ioctl_param->write_multi_bytes_param.in.data[i+1] = char_matrix_byte_array[i];
            }
            _controller_ioctl_param->write_multi_bytes_param.in.length = 1 + CHAR_MATRIX_CHAR_MATRIX_SIZE_IN_BYTE;
            
            ret = _controller_ctx->ops.ioctl(_controller_ctx, cmd, _controller_ioctl_param);
            if(ret < 0) {
                printf("ssd1306_oled_choose_GDDRAM_START_POSITION->_controller_ctx->ioctl fail choose column\n");
                return -1;
            }
            
            return 0;
        break;

        case SSD1306_OLED_IOCTL_CMD_START_SCOLL:
            _controller_ctx = ssd1306_oled_param->start_scoll_param.temp.i2c_controller_ctx;
            _controller_ioctl_param = ssd1306_oled_param->start_scoll_param.temp.i2c_controller_ioctl_param;

            cmd = I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES;
            _controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_ONY_BYTE_CMD_HEAR;
            _controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_SINGLE_BYTE_ACTIVATE_SCROLL_HEAD;
            _controller_ioctl_param->write_multi_bytes_param.in.length = 2;

            ret = _controller_ctx->ops.ioctl(_controller_ctx, cmd, _controller_ioctl_param);
            if(ret < 0) {
                printf("ssd1306_oled_choose_GDDRAM_START_POSITION->_controller_ctx->ioctl fail choose column\n");
                return -1;
            }
            return 0;
        break;

        case SSD1306_OLED_IOCTL_CMD_STOP_SCOLL:
            _controller_ctx = ssd1306_oled_param->stop_scoll_param.temp.i2c_controller_ctx;
            _controller_ioctl_param = ssd1306_oled_param->stop_scoll_param.temp.i2c_controller_ioctl_param;

            cmd = I2C_CONTROLLER_IOCTL_CMD_WRITE_MULTI_BYTES;
            _controller_ioctl_param->write_multi_bytes_param.in.data[0] = SSD1306_OLED_CONTROL_BYTE_ONY_BYTE_CMD_HEAR;
            _controller_ioctl_param->write_multi_bytes_param.in.data[1] = SSD1306_OLED_DATA_BYTE_CMD_SINGLE_BYTE_DEACTIVATE_SCROLL_HEAD;
            _controller_ioctl_param->write_multi_bytes_param.in.length = 2;

            ret = _controller_ctx->ops.ioctl(_controller_ctx, cmd, _controller_ioctl_param);
            if(ret < 0) {
                printf("ssd1306_oled_choose_GDDRAM_START_POSITION->_controller_ctx->ioctl fail choose column\n");
                return -1;
            }
            return 0;
        break;

        default:
            printf("ssd1306_oled_ioctl fail, un-known cmd %d\n", _cmd);
            return -1;
        break;

    }
}


