/**
* @file     adc_ti1258.c
* @brief    adc_ti1258.c
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     15/11/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "log_controller.h"
#include "adc_controller.h"
#include "adc_ti1258.h"

#include "debug.h"

#include "spi_controller.h"
#include "gpio_controller.h"

int extract_samples(uint8_t *_buf, int _length, adc_sample *_samples, int *_nb_samples)
{
    int i, j, length = _length, data_offset = -1, segment_length = -1;
    int check_sum, check_sum_calc;
    uint8_t *buf = _buf, byte1, byte2, byte3;
    uint16_t channel_mask;
    int playload_length = 0;
    int channel_id;
    int *nb_samples = _nb_samples;
    adc_sample *samples = _samples;
    uint32_t sample;
    double voltage;

    playload_length = 1 + 16*3 + 2;

    // check if we can find 0xaa, 0xbb, 0xcc
    for(i=0; i<length-playload_length; i++) {
        if(buf[i]==0xAA && buf[i+1]==0xBB && buf[i+2]==0xCC && buf[i+3]==0xDD) {
            
            data_offset = i + 4;
            break;
        }
    }

    //printf("length %d playload_length %d\n", length, playload_length);
    //printf("data_offset %d\n", data_offset);

    if((data_offset >  length-playload_length) || (data_offset < 0) ) {
        return -1;
    }

    segment_length = buf[data_offset];
    //printf("segment_length %d\n", segment_length);

    check_sum =  buf[data_offset + segment_length]<<8 | buf[data_offset + segment_length + 1];
    //printf("check_sum %d\n", check_sum);

    check_sum_calc = 0;
    for(i=0; i<segment_length; i++) {
        check_sum_calc += buf[data_offset + i];
    }
    //printf("check_sum_calc %d\n", check_sum_calc);
    
    if(check_sum != check_sum_calc) {
        printf("extract_samples fail, check_sum un-matched\n");
        return -1;
    } 

    channel_mask = buf[data_offset + segment_length - 2] << 8 | buf[data_offset + segment_length - 1];

    //printf("channel_mask 0x%x\n", channel_mask);

    *nb_samples = 0;
    
    data_offset = data_offset + 1;
    i = 0;
    channel_id = 1;
    while(channel_id <= channel_mask) {

        if(channel_id) {

            //printf("channel_id = 0x%x\n", channel_id);

            byte1 = buf[data_offset + i*3];
            byte2 = buf[data_offset + i*3 + 1];
            byte3 = buf[data_offset + i*3 + 2];

            //printf("i = %d byte1 0x%x byte2 0x%x byte3 0x%x\n", i, byte1, byte2, byte3);

            sample = byte1<<16 | byte2<<8 | byte3;

            samples[i].index = channel_id;

            if(sample & 0x800000) {
                sample = ~sample + 1;
                sample = sample & 0xFFFFFF;
                voltage = (5.0*sample)/7864320;
                voltage *= -1;
            } else {
                voltage = (5.0 * sample) / 7864320;
            }

            samples[i].voltage = voltage;
            
            //printf("channel_id %x voltage %f\n", channel_id, voltage);

            *nb_samples = *nb_samples + 1;
        }

        i++;
        channel_id = channel_id << 1;
    }

    //printf("nb_samples %d\n", *nb_samples);

    return 0;
}

int adc_ti1258_init(register_adc_parameters *adc_param)
{
    int ret = 0, cmd;
    uint8_t reg, reg_val;
    spi_controller_context *_spi_controller_ctx = NULL;
    register_adc_parameters *_adc_param = adc_param;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param = NULL;
    
    _spi_controller_ctx = _adc_param->in.spi_controller_ctx;
    _spi_controller_ioctl_param = _adc_param->in.spi_controller_ioctl_param;

    if(!_spi_controller_ctx || !_adc_param ) {
        printf("ads1258_init fail, incorrect parameter\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    reg = ADS1258_CONFIG0;
    reg_val = 0x01;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x60 | reg;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = reg_val;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 2;    
    ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("ads1258_init->spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    reg = ADS1258_CONFIG1;
    reg_val = 0x73;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x60 | reg;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = reg_val;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 2;    
    ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("ads1258_init->spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    
    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    reg = ADS1258_MUXDIF;
    reg_val = 0x00;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x60 | reg;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = reg_val;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 2;    
    ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("ads1258_init->spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    reg = ADS1258_MUXSG0;
    reg_val = 0x00;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x60 | reg;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = reg_val;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 2;    
    ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("ads1258_init->spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
    reg = ADS1258_MUXSG1;
    reg_val = 0x00;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x60 | reg;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = reg_val;
    _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 2;    
    ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("ads1258_init->spi_controller_ioctl->transfer_message fail\n");
        return -1;
    }

    return ret;
}

int adc_ti1258_ioctl(adc_ioctl_parameters *adc_ioctl_param)
{
    uint32_t sample, *_tx_len;
    int *_nb_samples = NULL;
    adc_sample *_samples = NULL;
    double voltage;
    int cmd, _cmd, ret, retries, done, new_sample, read_rdry_count;
    uint8_t reg, reg_val, *_rx_buf, *_tx_buf, status;
    analog_input *_analog_input = NULL;
    spi_controller_context *_spi_controller_ctx = NULL;
    adc_ioctl_parameters *_adc_ioctl_param = adc_ioctl_param;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param = NULL;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param_drdy = NULL;

    int head_end_idx,data_start_idx,chennal_data_start_idx,data_updata_start_idx,check_sum_start_idx,send_byte;
    uint8_t data_length;
    uint16_t data_update,check_sum,check_sum_calc;

    _cmd = _adc_ioctl_param->cmd;

    switch(_cmd) {

        case ADC_IOCTL_CMD_PULSE_CONVERSION:
             _analog_input = _adc_ioctl_param->pulse_conversion_param.temp._analog_input;
             printf("_analog_input %s config1 0x%x config2 0x%x\n", _analog_input->terminal, _analog_input->config_data1, _analog_input->config_data2);
             _spi_controller_ctx = _adc_ioctl_param->pulse_conversion_param.temp._spi_controller_ctx;
             _spi_controller_ioctl_param = _adc_ioctl_param->pulse_conversion_param.temp._spi_controller_ioctl_param;
             
             //channel selection
             cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
             reg = _analog_input->config_data1;
             reg_val = _analog_input->config_data2;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x60 | reg;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = reg_val;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 2;    
             ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
             if(ret < 0) {
                 printf("adc_ti1258_ioctl->spi_controller_ioctl->transfer_message fail\n");
                 return -1;
             }

             done = 0;
             retries = 5;
             _rx_buf = _spi_controller_ioctl_param->transfer_message_param.out.rx_buf;
             while(retries) {
                 // issue a pulse command
                 cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
                 _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x80;
                 _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 1;    
                 ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
                 if(ret < 0) {
                     printf("adc_ti1258_ioctl->spi_controller_ioctl->transfer_message fail\n");
                     return -1;
                 }

                 // check if new sample is ready
                 cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
                 _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x30;
                 _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = 0x00;
                 _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[2] = 0x00;
                 _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[3] = 0x00;
                 _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[4] = 0x00;
                 _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 5;    
                 ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
                 if(ret < 0) {
                     printf("adc_ti1258_ioctl->spi_controller_ioctl->transfer_message fail\n");
                     return -1;
                 }

                 status = _rx_buf[1];

                 if(status & 0x80) {

                     sample = _rx_buf[2] << 16 | _rx_buf[3] << 8 | _rx_buf[4];

                     if(sample & 0x800000) { // negative
            
                         sample = ~sample + 1;
                         sample = sample & 0xFFFFFF;
                         voltage = (5.0)*sample/0x780000;
                         voltage *= -1;
    
                         printf("status 0x%x sample 0x%x voltage: %f\n", status, sample, voltage);
            
                         done = 1;

                         break;
                     } else { // positive
                         voltage = (5.0 * sample) / 7864320;
    
                         printf("status 0x%x sample 0x%x voltage: %f\n", status, sample, voltage);
            
                         done = 1;

                         break;
                     }
                 }

                 //usleep(ADC_RETRY_DELAY_IN_MICROSECONDS);
                 
                 retries--;
             }

             cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
             reg = _analog_input->config_data1;
             //reg_val = _analog_input->config_data2;
             reg_val = 0;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[0] = 0x60 | reg;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_buf[1] = reg_val;
             _spi_controller_ioctl_param->transfer_message_param.in.tx_len = 2;    
             ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
             if(ret < 0) {
                 printf("adc_ti1258_ioctl->spi_controller_ioctl->transfer_message fail\n");
                 return -1;
             }

             if(done == 0) {
                 printf("adc_ti1258_ioctl->pulse_conversion fail, retries:%d\n", retries);
                 return -1;
             }

             //*(_adc_ioctl_param->pulse_conversion_param.out._voltage) = voltage;

             return 0;
        break;

        case ADC_IOCTL_CMD_STM32_CONVERSION:
             _spi_controller_ctx = _adc_ioctl_param->stm32_conversion_param.temp._spi_controller_ctx;
             _spi_controller_ioctl_param = _adc_ioctl_param->stm32_conversion_param.temp._spi_controller_ioctl_param;
             _gpio_controller_ctx = _adc_ioctl_param->stm32_conversion_param.temp._gpio_controller_ctx;
             _gpio_controller_ioctl_param_drdy = _adc_ioctl_param->stm32_conversion_param.temp._gpio_controller_ioctl_param_drdy;

             done = 0;
             retries = 10;
             _tx_buf = _spi_controller_ioctl_param->transfer_message_param.in.tx_buf;
             _rx_buf = _spi_controller_ioctl_param->transfer_message_param.out.rx_buf;
             _tx_len = &_spi_controller_ioctl_param->transfer_message_param.in.tx_len;

             while(retries) {

                 new_sample = 0;
                 read_rdry_count = 100;
                   
                 while(read_rdry_count) {

                     cmd = GPIO_CONTROLLER_IOCTL_CMD_GET_LEVEL;
                     ret = gpio_controller_ioctl(_gpio_controller_ctx, cmd, _gpio_controller_ioctl_param_drdy);
                     //printf("level %d\n", _gpio_controller_ioctl_param_drdy->get_level_param.out.level);
                     if(ret < 0) {
                         printf("gpio_controller_ioctl fail to get drdy\n");
                         //continue;
                         return -1;
                     }

                     if(_gpio_controller_ioctl_param_drdy->get_level_param.out.level == GPIO_LEVEL_LOW) {
                         new_sample = 1;
                         break;
                     }

                     read_rdry_count--;
                 }

                 if(!new_sample) {
                     retries--;
                     continue;
                 }

                 cmd = SPI_CONTROLLER_IOCTL_CMD_TRANSFER_MESSAGE;
                 _tx_buf[0] = 0x55;
                 *_tx_len = 81;
                 
                 memset(&_tx_buf[1], 0, 80);

                 ret = spi_controller_ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
                 if(ret < 0) {
                     printf("adc_ti1258_ioctl->spi_controller_ioctl->transfer_message fail\n");
                     return -1;
                 } 

                 //dump_data8(_rx_buf, 81);

                 _nb_samples = _adc_ioctl_param->stm32_conversion_param.out._nb_samples;
                 _samples = _adc_ioctl_param->stm32_conversion_param.out._samples;

                 ret = extract_samples(_rx_buf, 81, _samples, _nb_samples);
                 if(ret < 0) {
                     printf("adc_ti1258_ioctl->extract_samples fail\n");
                     return -1;
                 }
                 
                 done = 1;

             }

             if(done == 0) {
                 printf("adc_ti1258_ioctl->stm32_conversion fail, retries:%d\n", retries);
                 return -1;
             }

             return 0;
        break;

        default:

        break;
    }
}


int adc_ti1258_deinit(unregister_adc_parameters *adc_param)
{
    int ret = 0;

    return ret;
}


