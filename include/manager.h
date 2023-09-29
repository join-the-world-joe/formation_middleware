/**
* @file     manager.h
* @brief    This file implements the management abstraction 
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     08/10/2018
*/

#ifndef __MANAGER_H__
#define __MANAGER_H__

#define MANAGER_LOG_FILE_NAME "manager.log"
#define LENGTH_OF_LOG_FILE_NAME 500

#define PRODUCT_TYPE_BLUE_KEY_XXX 1
#define BOARD_TYPE_BLUE_KEY_XXX 2

typedef struct _Resources {

    // GPIO
    gpio_controller_context gpio_controller_ctx;
    gpio_controller_ioctl_parameters gpio_controller_ioctl_param_bell;
    gpio_controller_ioctl_parameters gpio_controller_ioctl_param_rdry;
    gpio_controller_ioctl_parameters gpio_controller_ioctl_param_led_ready;
    gpio_controller_ioctl_parameters gpio_controller_ioctl_param_led_error;
    gpio_controller_ioctl_parameters gpio_controller_ioctl_param_led_run;
    gpio_controller_ioctl_parameters gpio_controller_ioctl_param_ac_loss;

    // SPI Bus
    spi_controller_context spi_controller_ctx;
    spi_controller_ioctl_parameters spi_controller_ioctl_param_node0;
    spi_controller_ioctl_parameters spi_controller_ioctl_param_node1;
    spi_controller_ioctl_parameters spi_controller_ioctl_param_node2;
    spi_controller_ioctl_parameters spi_controller_ioctl_param_node3;
    spi_controller_ioctl_parameters spi_controller_ioctl_param_node4;
    spi_controller_ioctl_parameters spi_controller_ioctl_param_node5;
    spi_controller_ioctl_parameters spi_controller_ioctl_param_node6;
    //spi_controller_ioctl_parameters spi_controller_ioctl_param_node7;

    // I2C Bus
    i2c_controller_context i2c_controller_ctx;
    i2c_controller_ioctl_parameters i2c_controller_ioctl_param_node1;

    // IO Expander
    io_expander_controller_context io_expander_controller_ctx;
    io_expander_controller_ioctl_parameters io_expander_controller_ioctl_param_U1;
    io_expander_controller_ioctl_parameters io_expander_controller_ioctl_param_U2;
    io_expander_controller_ioctl_parameters io_expander_controller_ioctl_param_U3;
    io_expander_controller_ioctl_parameters io_expander_controller_ioctl_param_U4;

    //DAC
    dac_controller_context dac_controller_ctx;
    dac_controller_ioctl_parameters dac_controller_ioctl_param_U2;
    dac_controller_ioctl_parameters dac_controller_ioctl_param_U3;

    //ADC
    adc_controller_context adc_controller_ctx;
    adc_controller_ioctl_parameters adc_controller_ioctl_param_U1;

    //SSD1306_OLED
    ssd1306_oled_controller_context ssd1306_oled_controller_ctx;
    ssd1306_oled_controller_ioctl_parameters ssd1306_oled_controller_ioctl_param_ssd1306_oled_1;

} Resources;

typedef struct _sampling_log {
    log_controller_ioctl_parameters log_controller_ioctl_param_channel0;
    log_controller_ioctl_parameters log_controller_ioctl_param_channel1;
    log_controller_ioctl_parameters log_controller_ioctl_param_channel2;
    log_controller_ioctl_parameters log_controller_ioctl_param_channel3;
    log_controller_ioctl_parameters log_controller_ioctl_param_channel4;
    log_controller_ioctl_parameters log_controller_ioctl_param_channel5;
    log_controller_ioctl_parameters log_controller_ioctl_param_channel6;
    log_controller_ioctl_parameters log_controller_ioctl_param_channel7;
} Sampling_Log;

typedef struct _manager {
    int status;
    int board_type;
    int product_type;
    
    log_controller_context log_controller_ctx;
    log_controller_ioctl_parameters log_controller_ioctl_param; 

    uint8_t platform_endian;

    pthread_mutex_t log_mutex;
    pthread_mutex_t data_model_mutex;

    pthread_t modbus_thread_id;
    pthread_t physical_channel_monitor_id;
    pthread_t logical_channel_manager_id;
    pthread_t ip_monitor_id;

    in_addr_t oled_host_ip;
    modbus_tcp_context modbus_ctx;
    char log_file_name[LENGTH_OF_LOG_FILE_NAME];
    
    Sampling_Log sampling_log;

    Resources resources;

}Manager;

int manager_init(Manager *manager);
int manager_load_resources(Manager *manager);
int manager_run(Manager *manager);
void manager_deinit(Manager *manager);


#endif


