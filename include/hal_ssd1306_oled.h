/*
* @file     hal_ssd1306_oled.h
* @brief    hal_ssd1306_oled.h
* @author   Jerry.Chen <mikuyear@sina.cn>
* @date     31/1/2019
*/

#ifndef __HAL_SSD1306_OLED_H__
#define __HAL_SSD1306_OLED_H__

int hal_ssd1306_oled_init(register_ssd1306_oled_parameters *);
int hal_ssd1306_oled_ioctl(ssd1306_oled_ioctl_parameters *);
int hal_ssd1306_oled_deinit(unregister_ssd1306_oled_parameters *);


#endif // __HAL_SSD1306_OLED_H__