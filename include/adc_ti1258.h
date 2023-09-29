/**
* @file     adc_ti1258.h
* @brief    adc_ti1258.h
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     15/11/2018
*/

#ifndef __ADC_TI1258_H__
#define __ADC_TI1258_H__

#define ADC1_DEVICE_SIGNATURE "U1"

//#define ADC_RETRY_DELAY_IN_MICROSECONDS 50

#define ADS1258_CONFIG0 0x00
#define ADS1258_CONFIG1 0x01
#define ADS1258_MUXSCH  0X02
#define ADS1258_MUXDIF  0X03
#define ADS1258_MUXSG0  0X04
#define ADS1258_MUXSG1  0X05

#define ADS1258_NB_ANALOG_INPUTS 16

#define MUXSG0_ADDR 4
#define MUXSG1_ADDR 5

#define TERMINAL_BV1_CONFIG_DATA1 MUXSG0_ADDR
#define TERMINAL_CV1_CONFIG_DATA1 MUXSG0_ADDR
#define TERMINAL_BV2_CONFIG_DATA1 MUXSG0_ADDR
#define TERMINAL_CV2_CONFIG_DATA1 MUXSG0_ADDR
#define TERMINAL_BV3_CONFIG_DATA1 MUXSG0_ADDR
#define TERMINAL_CV3_CONFIG_DATA1 MUXSG0_ADDR
#define TERMINAL_BV4_CONFIG_DATA1 MUXSG0_ADDR
#define TERMINAL_CV4_CONFIG_DATA1 MUXSG0_ADDR
#define TERMINAL_BV5_CONFIG_DATA1 MUXSG1_ADDR
#define TERMINAL_CV5_CONFIG_DATA1 MUXSG1_ADDR
#define TERMINAL_BV6_CONFIG_DATA1 MUXSG1_ADDR
#define TERMINAL_CV6_CONFIG_DATA1 MUXSG1_ADDR
#define TERMINAL_BV7_CONFIG_DATA1 MUXSG1_ADDR
#define TERMINAL_CV7_CONFIG_DATA1 MUXSG1_ADDR
#define TERMINAL_BV8_CONFIG_DATA1 MUXSG1_ADDR
#define TERMINAL_CV8_CONFIG_DATA1 MUXSG1_ADDR

#define TERMINAL_BV1_CONFIG_DATA2 (1)
#define TERMINAL_CV1_CONFIG_DATA2 (1<<1)
#define TERMINAL_BV2_CONFIG_DATA2 (1<<2)
#define TERMINAL_CV2_CONFIG_DATA2 (1<<3)
#define TERMINAL_BV3_CONFIG_DATA2 (1<<4)
#define TERMINAL_CV3_CONFIG_DATA2 (1<<5)
#define TERMINAL_BV4_CONFIG_DATA2 (1<<6)
#define TERMINAL_CV4_CONFIG_DATA2 (1<<7)
#define TERMINAL_BV5_CONFIG_DATA2 (1<<0)
#define TERMINAL_CV5_CONFIG_DATA2 (1<<1)
#define TERMINAL_BV6_CONFIG_DATA2 (1<<2)
#define TERMINAL_CV6_CONFIG_DATA2 (1<<3)
#define TERMINAL_BV7_CONFIG_DATA2 (1<<4)
#define TERMINAL_CV7_CONFIG_DATA2 (1<<5)
#define TERMINAL_BV8_CONFIG_DATA2 (1<<6)
#define TERMINAL_CV8_CONFIG_DATA2 (1<<7)

#define BV1_CHANNEL_INDEX   0x0001
#define CV1_CHANNEL_INDEX   0x0002
#define BV2_CHANNEL_INDEX   0x0004
#define CV2_CHANNEL_INDEX   0x0008
#define BV3_CHANNEL_INDEX   0x0010
#define CV3_CHANNEL_INDEX   0x0020
#define BV4_CHANNEL_INDEX   0x0040
#define CV4_CHANNEL_INDEX   0x0080
#define BV5_CHANNEL_INDEX   0x0100
#define CV5_CHANNEL_INDEX   0x0200
#define BV6_CHANNEL_INDEX   0x0400
#define CV6_CHANNEL_INDEX   0x0800
#define BV7_CHANNEL_INDEX   0x1000
#define CV7_CHANNEL_INDEX   0x2000
#define BV8_CHANNEL_INDEX   0x4000
#define CV8_CHANNEL_INDEX   0x8000

int adc_ti1258_init(register_adc_parameters *);
int adc_ti1258_ioctl(adc_ioctl_parameters *);
int adc_ti1258_deinit(unregister_adc_parameters *);

#endif // __ADC_TI1258_H__

