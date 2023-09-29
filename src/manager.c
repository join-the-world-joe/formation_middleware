/**
* @file     manager.h
* @brief    This file implements the management abstraction 
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     08/10/2018
*/

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "channel.h"
#include "log_controller.h"
#include "tcp.h"
#include "registers.h"
#include "modbus_tcp.h"
#include "spi_controller.h"
#include "i2c_controller.h"
#include "gpio_controller.h"
#include "dac_controller.h"
#include "io_expander_controller.h"
#include "adc_controller.h"
#include "ssd1306_oled_controller.h"
#include "manager.h"
#include "modbus_tcp.h"
#include "macros.h"
#include "platform_endian.h"

#include "register_offset.h"
#include "register_values.h"
#include "error_no.h"

#include "formation_logic.h"
#include "configuration_logic.h"
#include "step_timming_parameters.h"
#include "logical_channel.h"
#include "physical_channel.h"
#include "debug_logic.h"
#include "misc.h"
#include "calibration_logic.h"
#include "recovery_logic.h"

void init_register_mapping(Manager * manager);
void *start_modbus_tcp_routine(void *parameter);
void *start_misc_monitor_routine(void *parameter);
void *start_physical_channel_monitor_routine(void *parameter);
void *start_logical_channel_manager_routine(void *parameter);
static int do_main_loop(Manager *manager);
static int update_goto_parameters(Manager *_manager, int channel_id);

int manager_init(Manager *manager)
{
    Manager *_manager = manager;
    int ret = 0;
    uint8_t endian;
    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param;
    modbus_tcp_context *modbus_ctx = NULL;
    pthread_mutex_t *log_mutex = NULL;

    if(!_manager) {
        printf("manager_init, fail\n");
        return -1;
    }

    _log_controller_ctx = &_manager->log_controller_ctx;
    _log_controller_ioctl_param = &_manager->log_controller_ioctl_param;

    modbus_ctx = &_manager->modbus_ctx;
    log_mutex = &_manager->log_mutex;

    endian = platform_endian();
    if(endian < 0) {
        printf("unknown endian\n");
        return -1;
    }

    printf("Endian: %s\n", endian == __LITTLE_ENDIAN__?"Little-Endian":"Big-Endian");

    init_system_log(_manager);
    init_sampling_log(_manager);

    ret = modbus_tcp_init(modbus_ctx, _log_controller_ctx, _log_controller_ioctl_param);
    if(ret < 0) {
            printf("manager_init->modbus_tcp_init fail\n");
            return -1;
    }

    manager->product_type = PRODUCT_TYPE_BLUE_KEY_XXX;
    manager->board_type = BOARD_TYPE_BLUE_KEY_XXX;
    manager->status = MANAGER_STATUS_POR;

    manager->platform_endian = endian;
    manager->modbus_ctx.platform_endian = endian;
    manager->modbus_ctx._register.platform_endian = endian;

    modbus_ctx->manager_status = &_manager->status;

    ret = manager_load_resources(_manager);
    if(ret < 0) {
        printf("manager_init->manager_load_resources fail\n");
        return -1;
    }

    init_register_mapping(_manager);

    reset_all_physical_channel_to_default_state(_manager);

    reload_and_update_IP(_manager);

    reset_all_logical_channels(_manager);

    return ret;
}

void init_gpio_resources(Manager *manager)
{
    int ret = 0, cmd;
    Manager *_manager = manager;
    register_gpio_port_parameters gpio_port_param;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param = NULL;
    gpio_controller_ioctl_parameters gpio_controller_ioctl_param_temp;

    _gpio_controller_ctx = &_manager->resources.gpio_controller_ctx;

    // Initialize GPIO Controller
    ret = gpio_controller_init(_gpio_controller_ctx, &_manager->log_controller_ctx, &_manager->log_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_init fail\n");
        exit(-1);
    }

    // Register GPIO Resources
    strcpy(gpio_port_param.in.signature, "RESET");
    gpio_port_param.in.direction = GPIO_DIRECTION_OUT;
    ret = _gpio_controller_ctx->ops.register_context(_gpio_controller_ctx, &gpio_port_param);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_param.in.signature);
        exit(-1);
    }

    strcpy(gpio_port_param.in.signature, "BELL");
    gpio_port_param.in.direction = GPIO_DIRECTION_OUT;
    ret = _gpio_controller_ctx->ops.register_context(_gpio_controller_ctx, &gpio_port_param);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_param.in.signature);
        exit(-1);
    }

    strcpy(gpio_port_param.in.signature, "AD-DRDY");
    gpio_port_param.in.direction = GPIO_DIRECTION_IN;
    ret = _gpio_controller_ctx->ops.register_context(_gpio_controller_ctx, &gpio_port_param);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_param.in.signature);
        exit(-1);
    }

    strcpy(gpio_port_param.in.signature, "LED-READY");
    gpio_port_param.in.direction = GPIO_DIRECTION_OUT;
    ret = _gpio_controller_ctx->ops.register_context(_gpio_controller_ctx, &gpio_port_param);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_param.in.signature);
        exit(-1);
    }

    strcpy(gpio_port_param.in.signature, "LED-RUN");
    gpio_port_param.in.direction = GPIO_DIRECTION_OUT;
    ret = _gpio_controller_ctx->ops.register_context(_gpio_controller_ctx, &gpio_port_param);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_param.in.signature);
        exit(-1);
    }

    strcpy(gpio_port_param.in.signature, "LED-ERROR");
    gpio_port_param.in.direction = GPIO_DIRECTION_OUT;
    ret = _gpio_controller_ctx->ops.register_context(_gpio_controller_ctx, &gpio_port_param);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_param.in.signature);
        exit(-1);
    }

    strcpy(gpio_port_param.in.signature, "AC-LOSS");
    gpio_port_param.in.direction = GPIO_DIRECTION_IN;
    ret = _gpio_controller_ctx->ops.register_context(_gpio_controller_ctx, &gpio_port_param);
    if(ret < 0) {
        printf("gpio_controller_register fail, port %s\n", gpio_port_param.in.signature);
        exit(-1);
    }


    cmd = GPIO_CONTROLLER_IOCTL_CMD_ACQUIRE_GPIO_CONTEX;
    _gpio_controller_ioctl_param = &manager->resources.gpio_controller_ioctl_param_rdry;
    strcpy(_gpio_controller_ioctl_param->acquire_gpio_ctx_param.in.signature, "AD-DRDY");
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, cmd, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to acquire gpio context\n");
        exit(-1);
    }

    cmd = GPIO_CONTROLLER_IOCTL_CMD_ACQUIRE_GPIO_CONTEX;
    _gpio_controller_ioctl_param = &manager->resources.gpio_controller_ioctl_param_led_ready;
    strcpy(_gpio_controller_ioctl_param->acquire_gpio_ctx_param.in.signature, "LED-READY");
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, cmd, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to acquire gpio context\n");
        exit(-1);
    }

    cmd = GPIO_CONTROLLER_IOCTL_CMD_ACQUIRE_GPIO_CONTEX;
    _gpio_controller_ioctl_param = &manager->resources.gpio_controller_ioctl_param_led_run;
    strcpy(_gpio_controller_ioctl_param->acquire_gpio_ctx_param.in.signature, "LED-RUN");
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, cmd, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to acquire gpio context\n");
        exit(-1);
    }

    cmd = GPIO_CONTROLLER_IOCTL_CMD_ACQUIRE_GPIO_CONTEX;
    _gpio_controller_ioctl_param = &manager->resources.gpio_controller_ioctl_param_led_error;
    strcpy(_gpio_controller_ioctl_param->acquire_gpio_ctx_param.in.signature, "LED-ERROR");
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, cmd, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to acquire gpio context\n");
        exit(-1);
    }

    cmd = GPIO_CONTROLLER_IOCTL_CMD_ACQUIRE_GPIO_CONTEX;
    _gpio_controller_ioctl_param = &manager->resources.gpio_controller_ioctl_param_ac_loss;
    strcpy(_gpio_controller_ioctl_param->acquire_gpio_ctx_param.in.signature, "AC-LOSS");
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, cmd, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to acquire gpio context\n");
        exit(-1);
    }

    cmd = GPIO_CONTROLLER_IOCTL_CMD_ACQUIRE_GPIO_CONTEX;
    _gpio_controller_ioctl_param = &manager->resources.gpio_controller_ioctl_param_bell;
    strcpy(_gpio_controller_ioctl_param->acquire_gpio_ctx_param.in.signature, "BELL");
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, cmd, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to acquire gpio context\n");
        exit(-1);
    }

    cmd = GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL;
    _gpio_controller_ioctl_param->set_level_param.in.level = GPIO_LEVEL_HIGH;
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, cmd, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        exit(-1);
    }

    sleep(1);

    cmd = GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL;
    _gpio_controller_ioctl_param->set_level_param.in.level = GPIO_LEVEL_LOW;
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, cmd, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        exit(-1);
    }

    cmd = GPIO_CONTROLLER_IOCTL_CMD_ACQUIRE_GPIO_CONTEX;
    _gpio_controller_ioctl_param = &gpio_controller_ioctl_param_temp;
    strcpy(_gpio_controller_ioctl_param->acquire_gpio_ctx_param.in.signature, "RESET");
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, cmd, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to acquire gpio context\n");
        exit(-1);
    }

    cmd = GPIO_CONTROLLER_IOCTL_CMD_SET_LEVEL;
    _gpio_controller_ioctl_param->set_level_param.in.level = GPIO_LEVEL_HIGH;
    ret = _gpio_controller_ctx->ops.ioctl(_gpio_controller_ctx, cmd, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to set gpio level\n");
        exit(-1);
    }
    
}

void init_spi_resources(Manager *manager)
{
    int ret = 0, cmd;
    Manager *_manager = manager;
    spi_controller_context *_spi_controller_ctx = NULL;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param = NULL;
    register_spi_bus_parameters spi_bus_param_bus0_node0;
    register_spi_bus_parameters spi_bus_param_bus0_node1;
    register_spi_bus_parameters spi_bus_param_bus0_node2;
    register_spi_bus_parameters spi_bus_param_bus0_node3;
    register_spi_bus_parameters spi_bus_param_bus0_node4;
    register_spi_bus_parameters spi_bus_param_bus0_node5;
    register_spi_bus_parameters spi_bus_param_bus0_node6;
    
    _spi_controller_ctx = &_manager->resources.spi_controller_ctx;

    // Initialize SPI Controller
    ret = spi_controller_init(_spi_controller_ctx, &_manager->log_controller_ctx, &_manager->log_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_init fail\n");
        exit(-1);
    }

    strcpy(spi_bus_param_bus0_node0.in.signature, "/dev/spidev0.0");
    spi_bus_param_bus0_node0.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node0.in.speed = SPI_SPEED_16MHZ;
    spi_bus_param_bus0_node0.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node0.in.bits_per_word = SPI_BPW_08BITS;
    ret = _spi_controller_ctx->ops.register_context(_spi_controller_ctx, &spi_bus_param_bus0_node0);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        exit(-1);
    }

    strcpy(spi_bus_param_bus0_node1.in.signature, "/dev/spidev0.1");
    spi_bus_param_bus0_node1.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node1.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node1.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node1.in.bits_per_word = SPI_BPW_08BITS;
    ret = _spi_controller_ctx->ops.register_context(_spi_controller_ctx, &spi_bus_param_bus0_node1);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        exit(-1);
    }

    strcpy(spi_bus_param_bus0_node2.in.signature, "/dev/spidev0.2");
    spi_bus_param_bus0_node2.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node2.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node2.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node2.in.bits_per_word = SPI_BPW_08BITS;
    ret = _spi_controller_ctx->ops.register_context(_spi_controller_ctx, &spi_bus_param_bus0_node2);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        exit(-1);
    }

    strcpy(spi_bus_param_bus0_node3.in.signature, "/dev/spidev0.3");
    spi_bus_param_bus0_node3.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node3.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node3.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node3.in.bits_per_word = SPI_BPW_08BITS;
    ret = _spi_controller_ctx->ops.register_context(_spi_controller_ctx, &spi_bus_param_bus0_node3);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        exit(-1);
    }

    strcpy(spi_bus_param_bus0_node4.in.signature, "/dev/spidev0.4");
    spi_bus_param_bus0_node4.in.mode = SPI_MODE_00;
    spi_bus_param_bus0_node4.in.speed = SPI_SPEED_4MHZ;
    spi_bus_param_bus0_node4.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node4.in.bits_per_word = SPI_BPW_08BITS;
    ret = _spi_controller_ctx->ops.register_context(_spi_controller_ctx, &spi_bus_param_bus0_node4);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        exit(-1);
    }

    strcpy(spi_bus_param_bus0_node5.in.signature, "/dev/spidev0.5");
    spi_bus_param_bus0_node5.in.mode = SPI_MODE_01;
    spi_bus_param_bus0_node5.in.speed = SPI_SPEED_16MHZ;
    spi_bus_param_bus0_node5.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node5.in.bits_per_word = SPI_BPW_08BITS;
    ret = _spi_controller_ctx->ops.register_context(_spi_controller_ctx, &spi_bus_param_bus0_node5);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        exit(-1);
    }

    strcpy(spi_bus_param_bus0_node6.in.signature, "/dev/spidev0.6");
    spi_bus_param_bus0_node6.in.mode = SPI_MODE_01;
    spi_bus_param_bus0_node6.in.speed = SPI_SPEED_16MHZ;
    spi_bus_param_bus0_node6.in.cs_polarity = SPI_CS_POLARITY_LOW;
    spi_bus_param_bus0_node6.in.bits_per_word = SPI_BPW_08BITS;
    ret = _spi_controller_ctx->ops.register_context(_spi_controller_ctx, &spi_bus_param_bus0_node6);
    if(ret < 0) {
        printf("spi_controller_register_spi_bus, fail\n");
        exit(-1);
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX;
    _spi_controller_ioctl_param = &manager->resources.spi_controller_ioctl_param_node0;
    strcpy(_spi_controller_ioctl_param->acquire_spi_ctx_param.in.signature, "/dev/spidev0.0");
    ret = _spi_controller_ctx->ops.ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->acquire_spi_context fail\n");
        exit(-1);
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX;
    _spi_controller_ioctl_param = &manager->resources.spi_controller_ioctl_param_node1;
    strcpy(_spi_controller_ioctl_param->acquire_spi_ctx_param.in.signature, "/dev/spidev0.1");
    ret = _spi_controller_ctx->ops.ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->acquire_spi_context fail\n");
        exit(-1);
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX;
    _spi_controller_ioctl_param = &manager->resources.spi_controller_ioctl_param_node2;
    strcpy(_spi_controller_ioctl_param->acquire_spi_ctx_param.in.signature, "/dev/spidev0.2");
    ret = _spi_controller_ctx->ops.ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->acquire_spi_context fail\n");
        exit(-1);
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX;
    _spi_controller_ioctl_param = &manager->resources.spi_controller_ioctl_param_node3;
    strcpy(_spi_controller_ioctl_param->acquire_spi_ctx_param.in.signature, "/dev/spidev0.3");
    ret = _spi_controller_ctx->ops.ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->acquire_spi_context fail\n");
        exit(-1);
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX;
    _spi_controller_ioctl_param = &manager->resources.spi_controller_ioctl_param_node4;
    strcpy(_spi_controller_ioctl_param->acquire_spi_ctx_param.in.signature, "/dev/spidev0.4");
    ret = _spi_controller_ctx->ops.ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->acquire_spi_context fail\n");
        exit(-1);
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX;
    _spi_controller_ioctl_param = &manager->resources.spi_controller_ioctl_param_node5;
    strcpy(_spi_controller_ioctl_param->acquire_spi_ctx_param.in.signature, "/dev/spidev0.5");
    ret = _spi_controller_ctx->ops.ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->acquire_spi_context fail\n");
        exit(-1);
    }

    cmd = SPI_CONTROLLER_IOCTL_CMD_ACQUIRE_SPI_CONTEX;    
    _spi_controller_ioctl_param = &manager->resources.spi_controller_ioctl_param_node6;
    strcpy(_spi_controller_ioctl_param->acquire_spi_ctx_param.in.signature, "/dev/spidev0.6");
    ret = _spi_controller_ctx->ops.ioctl(_spi_controller_ctx, cmd, _spi_controller_ioctl_param);
    if(ret < 0) {
        printf("spi_controller_ioctl->acquire_spi_context fail\n");
        exit(-1);
    }

    _spi_controller_ctx->ops.list_resources(_spi_controller_ctx);

}

void init_i2c_resources(Manager *manager)
{
    int ret = 0, cmd;
    Manager *_manager = manager;
    i2c_controller_context *_i2c_controller_ctx = NULL;
    i2c_controller_ioctl_parameters *_i2c_controller_ioctl_param = NULL;
    register_i2c_bus_parameters i2c_bus_param_bus0_node1;

    _i2c_controller_ctx = &_manager->resources.i2c_controller_ctx;

    // Initialize i2c Controller
    ret = i2c_controller_init(_i2c_controller_ctx, &_manager->log_controller_ctx, &_manager->log_controller_ioctl_param);
    if(ret < 0) {
        printf("i2c_controller_init fail\n");
        exit(-1);
    }

    strcpy(i2c_bus_param_bus0_node1.in.signature, "i2c-1");
    ret = _i2c_controller_ctx->ops.register_context(_i2c_controller_ctx, &i2c_bus_param_bus0_node1);
    if(ret < 0) {
        printf("i2c_controller->register_context fail\n");
        exit(-1);
    }

    cmd = I2C_CONTROLLER_IOCTL_CMD_ACQUIRE_I2C_CONTEX;
    _i2c_controller_ioctl_param = &manager->resources.i2c_controller_ioctl_param_node1;
    strcpy(_i2c_controller_ioctl_param->acquire_i2c_ctx_param.in.signature, "i2c-1");
    ret = _i2c_controller_ctx->ops.ioctl(_i2c_controller_ctx, cmd, _i2c_controller_ioctl_param);
    if(ret < 0) {
        printf("i2c_controller->ioctl fail\n");
        exit(-1);
    }

    _i2c_controller_ctx->ops.list_resources(_i2c_controller_ctx);

}


void init_io_expander_resources(Manager *manager)
{
    Manager *_manager = manager;
    int ret, cmd;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    register_io_expander_parameters io_expander_param;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param = NULL;
    io_expander_controller_context *_io_expander_controller_ctx = NULL;
    io_expander_controller_ioctl_parameters *_io_expander_controller_ioctl_param = NULL;
    spi_controller_context *_spi_controller_ctx = NULL;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param = NULL;

    _spi_controller_ctx = &_manager->resources.spi_controller_ctx;
    _spi_controller_ioctl_param = &_manager->resources.spi_controller_ioctl_param_node1;
    _io_expander_controller_ctx = &_manager->resources.io_expander_controller_ctx;

    ret = io_expander_controller_init(_io_expander_controller_ctx, &_manager->log_controller_ctx, &_manager->log_controller_ioctl_param);
    if(ret < 0) {
        printf("io_expander_controller_init fail\n");
        exit(-1);
    }

    //printf("node signature %s\n", manager->resources.spi_controller_ioctl_param_node1.temp.spi_ctx->signature);

    strcpy(io_expander_param.in.signature, "U1");
    io_expander_param.in.bus_type = IO_EXPANDER_BUS_TYPE_SPI;
    io_expander_param.in.spi_controller_ctx = _spi_controller_ctx;
    io_expander_param.in.spi_controller_ioctl_param = _spi_controller_ioctl_param;
    strcpy(io_expander_param.in_out.io_ports[0].terminal, "KZ_H1");
    io_expander_param.in_out.io_ports[0].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[1].terminal, "KZ_M1");
    io_expander_param.in_out.io_ports[1].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[2].terminal, "KZ_L1");
    io_expander_param.in_out.io_ports[2].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[3].terminal, "JC1");
    io_expander_param.in_out.io_ports[3].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[4].terminal, "CF1");
    io_expander_param.in_out.io_ports[4].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[5].terminal, "CK1");
    io_expander_param.in_out.io_ports[5].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[6].terminal, "CLED1");
    io_expander_param.in_out.io_ports[6].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[7].terminal, "FLED1");
    io_expander_param.in_out.io_ports[7].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[8].terminal, "KZ_H2");
    io_expander_param.in_out.io_ports[8].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[9].terminal, "KZ_M2");
    io_expander_param.in_out.io_ports[9].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[10].terminal, "KZ_L2");
    io_expander_param.in_out.io_ports[10].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[11].terminal, "JC2");
    io_expander_param.in_out.io_ports[11].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[12].terminal, "CF2");
    io_expander_param.in_out.io_ports[12].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[13].terminal, "CK2");
    io_expander_param.in_out.io_ports[13].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[14].terminal, "CLED2");
    io_expander_param.in_out.io_ports[14].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[15].terminal, "FLED2");
    io_expander_param.in_out.io_ports[15].direction = IO_EXPANDER_DIRECTION_OUT;
    io_expander_param.in.nb_io_ports = 16;
    ret = _io_expander_controller_ctx->ops.register_context(_io_expander_controller_ctx, &io_expander_param);
    if(ret < 0) {
        printf("io_expander_controller_register_io_expander fail\n");
        exit(-1);
    }

    _io_expander_controller_ioctl_param = &manager->resources.io_expander_controller_ioctl_param_U1;
    cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_ACQUIRE_IO_EXPANDER_CONTEX;
    strcpy(_io_expander_controller_ioctl_param->acquire_io_expander_ctx_param.in.signature, "U1"); 
    ret = _io_expander_controller_ctx->ops.ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
    if(ret < 0) {
        printf("io_expander_controller_ioctl fail\n");
        exit(-1);
    }

    strcpy(io_expander_param.in.signature, "U2");
    io_expander_param.in.bus_type = IO_EXPANDER_BUS_TYPE_SPI;
    io_expander_param.in.spi_controller_ctx = _spi_controller_ctx;
    io_expander_param.in.spi_controller_ioctl_param = &manager->resources.spi_controller_ioctl_param_node2;
    strcpy(io_expander_param.in_out.io_ports[0].terminal, "KZ_H4");
    io_expander_param.in_out.io_ports[0].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[1].terminal, "KZ_M4");
    io_expander_param.in_out.io_ports[1].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[2].terminal, "KZ_L4");
    io_expander_param.in_out.io_ports[2].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[3].terminal, "JC4");
    io_expander_param.in_out.io_ports[3].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[4].terminal, "CF4");
    io_expander_param.in_out.io_ports[4].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[5].terminal, "CK4");
    io_expander_param.in_out.io_ports[5].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[6].terminal, "CLED4");
    io_expander_param.in_out.io_ports[6].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[7].terminal, "FLED4");
    io_expander_param.in_out.io_ports[7].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[8].terminal, "KZ_H3");
    io_expander_param.in_out.io_ports[8].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[9].terminal, "KZ_M3");
    io_expander_param.in_out.io_ports[9].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[10].terminal, "KZ_L3");
    io_expander_param.in_out.io_ports[10].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[11].terminal, "JC3");
    io_expander_param.in_out.io_ports[11].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[12].terminal, "CF3");
    io_expander_param.in_out.io_ports[12].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[13].terminal, "CK3");
    io_expander_param.in_out.io_ports[13].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[14].terminal, "CLED3");
    io_expander_param.in_out.io_ports[14].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[15].terminal, "FLED3");
    io_expander_param.in_out.io_ports[15].direction = IO_EXPANDER_DIRECTION_OUT;
    io_expander_param.in.nb_io_ports = 16;
    ret = _io_expander_controller_ctx->ops.register_context(_io_expander_controller_ctx, &io_expander_param);
    if(ret < 0) {
        printf("io_expander_controller_register_io_expander fail\n");
        exit(-1);
    }

    _io_expander_controller_ioctl_param = &manager->resources.io_expander_controller_ioctl_param_U2;
    cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_ACQUIRE_IO_EXPANDER_CONTEX;
    strcpy(_io_expander_controller_ioctl_param->acquire_io_expander_ctx_param.in.signature, "U2"); 
    ret = _io_expander_controller_ctx->ops.ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
    if(ret < 0) {
        printf("io_expander_controller_ioctl fail\n");
        exit(-1);
    }

    strcpy(io_expander_param.in.signature, "U3");
    io_expander_param.in.bus_type = IO_EXPANDER_BUS_TYPE_SPI;
    io_expander_param.in.spi_controller_ctx = _spi_controller_ctx;
    io_expander_param.in.spi_controller_ioctl_param = &manager->resources.spi_controller_ioctl_param_node3;
    strcpy(io_expander_param.in_out.io_ports[0].terminal, "KZ_H5");
    io_expander_param.in_out.io_ports[0].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[1].terminal, "KZ_M5");
    io_expander_param.in_out.io_ports[1].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[2].terminal, "KZ_L5");
    io_expander_param.in_out.io_ports[2].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[3].terminal, "JC5");
    io_expander_param.in_out.io_ports[3].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[4].terminal, "CF5");
    io_expander_param.in_out.io_ports[4].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[5].terminal, "CK5");
    io_expander_param.in_out.io_ports[5].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[6].terminal, "CLED5");
    io_expander_param.in_out.io_ports[6].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[7].terminal, "FLED5");
    io_expander_param.in_out.io_ports[7].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[8].terminal, "KZ_H6");
    io_expander_param.in_out.io_ports[8].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[9].terminal, "KZ_M6");
    io_expander_param.in_out.io_ports[9].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[10].terminal, "KZ_L6");
    io_expander_param.in_out.io_ports[10].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[11].terminal, "JC6");
    io_expander_param.in_out.io_ports[11].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[12].terminal, "CF6");
    io_expander_param.in_out.io_ports[12].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[13].terminal, "CK6");
    io_expander_param.in_out.io_ports[13].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[14].terminal, "CLED6");
    io_expander_param.in_out.io_ports[14].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[15].terminal, "FLED6");
    io_expander_param.in_out.io_ports[15].direction = IO_EXPANDER_DIRECTION_OUT;
    io_expander_param.in.nb_io_ports = 16;
    ret = _io_expander_controller_ctx->ops.register_context(_io_expander_controller_ctx, &io_expander_param);
    if(ret < 0) {
        printf("io_expander_controller_register_io_expander fail\n");
        exit(-1);
    }

    _io_expander_controller_ioctl_param = &manager->resources.io_expander_controller_ioctl_param_U3;
    cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_ACQUIRE_IO_EXPANDER_CONTEX;
    strcpy(_io_expander_controller_ioctl_param->acquire_io_expander_ctx_param.in.signature, "U3"); 
    ret = _io_expander_controller_ctx->ops.ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
    if(ret < 0) {
        printf("io_expander_controller_ioctl fail\n");
        exit(-1);
    }

    strcpy(io_expander_param.in.signature, "U4");
    io_expander_param.in.bus_type = IO_EXPANDER_BUS_TYPE_SPI;
    io_expander_param.in.spi_controller_ctx = _spi_controller_ctx;
    io_expander_param.in.spi_controller_ioctl_param = &manager->resources.spi_controller_ioctl_param_node4;
    strcpy(io_expander_param.in_out.io_ports[0].terminal, "KZ_H7");
    io_expander_param.in_out.io_ports[0].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[1].terminal, "KZ_M7");
    io_expander_param.in_out.io_ports[1].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[2].terminal, "KZ_L7");
    io_expander_param.in_out.io_ports[2].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[3].terminal, "JC7");
    io_expander_param.in_out.io_ports[3].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[4].terminal, "CF7");
    io_expander_param.in_out.io_ports[4].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[5].terminal, "CK7");
    io_expander_param.in_out.io_ports[5].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[6].terminal, "CLED7");
    io_expander_param.in_out.io_ports[6].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[7].terminal, "FLED7");
    io_expander_param.in_out.io_ports[7].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[8].terminal, "KZ_H8");
    io_expander_param.in_out.io_ports[8].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[9].terminal, "KZ_M8");
    io_expander_param.in_out.io_ports[9].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[10].terminal, "KZ_L8");
    io_expander_param.in_out.io_ports[10].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[11].terminal, "JC8");
    io_expander_param.in_out.io_ports[11].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[12].terminal, "CF8");
    io_expander_param.in_out.io_ports[12].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[13].terminal, "CK8");
    io_expander_param.in_out.io_ports[13].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[14].terminal, "CLED8");
    io_expander_param.in_out.io_ports[14].direction = IO_EXPANDER_DIRECTION_OUT;
    strcpy(io_expander_param.in_out.io_ports[15].terminal, "FLED8");
    io_expander_param.in_out.io_ports[15].direction = IO_EXPANDER_DIRECTION_OUT;
    io_expander_param.in.nb_io_ports = 16;
    ret = _io_expander_controller_ctx->ops.register_context(_io_expander_controller_ctx, &io_expander_param);
    if(ret < 0) {
        printf("io_expander_controller_register_io_expander fail\n");
        exit(-1);
    }

    _io_expander_controller_ioctl_param = &manager->resources.io_expander_controller_ioctl_param_U4;
    cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_ACQUIRE_IO_EXPANDER_CONTEX;
    strcpy(_io_expander_controller_ioctl_param->acquire_io_expander_ctx_param.in.signature, "U4"); 
    ret = _io_expander_controller_ctx->ops.ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
    if(ret < 0) {
        printf("io_expander_controller_ioctl fail\n");
        exit(-1);
    }

#if 0
    while(1) {

        cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
        strcpy(_io_expander_controller_ioctl_param->set_level_param.in.terminal, "KZ_H1");
        _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_HIGH;
        ret = _io_expander_controller_ctx->ops.ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
        if(ret < 0) {
            printf("io_expander_controller_ioctl fail\n");
            //return -1;
        }

        sleep(5);

        cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
        strcpy(_io_expander_controller_ioctl_param->set_level_param.in.terminal, "KZ_H1");
        _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_LOW;
        ret = _io_expander_controller_ctx->ops.ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
        if(ret < 0) {
            printf("io_expander_controller_ioctl fail\n");
            //return -1;
        }

        sleep(5);
    }
#endif

}

void init_adc_resources(Manager * manager)
{
    Manager *_manager = manager;
    int ret, cmd;
    register_adc_parameters register_adc_param;
    adc_controller_context *_adc_controller_ctx = NULL;
    adc_controller_ioctl_parameters *_adc_controller_ioctl_param = NULL;
    spi_controller_context *_spi_controller_ctx = NULL;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param = NULL;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param = NULL;

    _gpio_controller_ctx = &_manager->resources.gpio_controller_ctx;
    _gpio_controller_ioctl_param = &_manager->resources.gpio_controller_ioctl_param_rdry;
    _spi_controller_ctx = &_manager->resources.spi_controller_ctx;
    _spi_controller_ioctl_param = &_manager->resources.spi_controller_ioctl_param_node0;
    _adc_controller_ioctl_param = &_manager->resources.adc_controller_ioctl_param_U1;

    _adc_controller_ctx = &_manager->resources.adc_controller_ctx;

    ret = adc_controller_init(_adc_controller_ctx, &_manager->log_controller_ctx, &_manager->log_controller_ioctl_param);
    if(ret < 0) {
        printf("adc_controller_init fail\n");
        exit(-1);
    }

    // Register ADC Resources
    strcpy(register_adc_param.in.signature, "U1");
    register_adc_param.in.bus_type = ADC_BUS_TYPE_SPI;
    register_adc_param.in.spi_controller_ctx = _spi_controller_ctx;
    register_adc_param.in.gpio_controller_ctx = _gpio_controller_ctx;
    register_adc_param.in.gpio_controller_ioctl_param_drdy = _gpio_controller_ioctl_param;
    register_adc_param.in.spi_controller_ioctl_param = _spi_controller_ioctl_param;
    strcpy(register_adc_param.in_out.analog_inputs[0].terminal,  "BV1");
    strcpy(register_adc_param.in_out.analog_inputs[1].terminal,  "CV1");
    strcpy(register_adc_param.in_out.analog_inputs[2].terminal,  "BV2");
    strcpy(register_adc_param.in_out.analog_inputs[3].terminal,  "CV2");
    strcpy(register_adc_param.in_out.analog_inputs[4].terminal,  "BV3");
    strcpy(register_adc_param.in_out.analog_inputs[5].terminal,  "CV3");
    strcpy(register_adc_param.in_out.analog_inputs[6].terminal,  "BV4");
    strcpy(register_adc_param.in_out.analog_inputs[7].terminal,  "CV4");
    strcpy(register_adc_param.in_out.analog_inputs[8].terminal,  "BV5");
    strcpy(register_adc_param.in_out.analog_inputs[9].terminal,  "CV5");
    strcpy(register_adc_param.in_out.analog_inputs[10].terminal,  "BV6");
    strcpy(register_adc_param.in_out.analog_inputs[11].terminal,  "CV6");
    strcpy(register_adc_param.in_out.analog_inputs[12].terminal,  "BV7");
    strcpy(register_adc_param.in_out.analog_inputs[13].terminal,  "CV7");
    strcpy(register_adc_param.in_out.analog_inputs[14].terminal,  "BV8");
    strcpy(register_adc_param.in_out.analog_inputs[15].terminal,  "CV8");
    register_adc_param.in.nb_terminals = 16;
    ret = _adc_controller_ctx->ops.register_context(_adc_controller_ctx, &register_adc_param);
    if(ret < 0) {
        printf("dac_controller_register_dac fail\n");
        exit(-1);
    }

    // Initialize ADC resources
    cmd = ADC_CONTROLLER_IOCTL_CMD_ACQUIRE_ADC_CONTEX;
    strcpy(_adc_controller_ioctl_param->acquire_adc_ctx_param.in.signature, "U1"); 
    ret = _adc_controller_ctx->ops.ioctl(_adc_controller_ctx, cmd, _adc_controller_ioctl_param);
    if(ret < 0) {
        printf("adc_controller_ioctl fail\n");
        exit(-1);
    }

}

void init_dac_resources(Manager * manager)
{
    Manager *_manager = manager;
    int ret, cmd;
    dac_controller_context *_dac_controller_ctx = NULL;
    dac_controller_ioctl_parameters *_dac_controller_ioctl_param2 = NULL;
    dac_controller_ioctl_parameters *_dac_controller_ioctl_param3 = NULL;
    spi_controller_context *_spi_controller_ctx = NULL;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param5 = NULL;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param6 = NULL;
    register_dac_parameters dac_param;

    _dac_controller_ctx = &_manager->resources.dac_controller_ctx;
    _spi_controller_ctx = &_manager->resources.spi_controller_ctx;
    _spi_controller_ioctl_param5 = &_manager->resources.spi_controller_ioctl_param_node5;
    _spi_controller_ioctl_param6 = &_manager->resources.spi_controller_ioctl_param_node6;
    _dac_controller_ioctl_param2 = &_manager->resources.dac_controller_ioctl_param_U2;
    _dac_controller_ioctl_param3 = &_manager->resources.dac_controller_ioctl_param_U3;

    ret = dac_controller_init(_dac_controller_ctx, &_manager->log_controller_ctx, &_manager->log_controller_ioctl_param);
    if(ret < 0) {
        printf("dac_controller_init fail\n");
        exit(-1);
    }

    strcpy(dac_param.in.signature, "U2");
    dac_param.in.bus_type = DAC_BUS_TYPE_SPI;
    dac_param.in.nb_terminals = 8;
    dac_param.in.spi_controller_ctx = _spi_controller_ctx;
    dac_param.in.spi_controller_ioctl_param = _spi_controller_ioctl_param5;
    strcpy(dac_param.in_out.analog_outputs[0].terminal,  "VI1");
    strcpy(dac_param.in_out.analog_outputs[1].terminal,  "VI2");
    strcpy(dac_param.in_out.analog_outputs[2].terminal,  "VI3");
    strcpy(dac_param.in_out.analog_outputs[3].terminal,  "VI4");
    strcpy(dac_param.in_out.analog_outputs[4].terminal,  "VU1");
    strcpy(dac_param.in_out.analog_outputs[5].terminal,  "VU2");
    strcpy(dac_param.in_out.analog_outputs[6].terminal,  "VU3");
    strcpy(dac_param.in_out.analog_outputs[7].terminal,  "VU4");
    ret = _dac_controller_ctx->ops.register_context(_dac_controller_ctx, &dac_param);
    if(ret < 0) {
        printf("dac_controller_register_dac fail\n");
        exit(-1);
    }

    strcpy(dac_param.in.signature, "U3");
    dac_param.in.bus_type = DAC_BUS_TYPE_SPI;
    dac_param.in.nb_terminals = 8;
    dac_param.in.spi_controller_ctx = _spi_controller_ctx;
    dac_param.in.spi_controller_ioctl_param = _spi_controller_ioctl_param6;
    strcpy(dac_param.in_out.analog_outputs[0].terminal,  "VI5");
    strcpy(dac_param.in_out.analog_outputs[1].terminal,  "VI6");
    strcpy(dac_param.in_out.analog_outputs[2].terminal,  "VI7");
    strcpy(dac_param.in_out.analog_outputs[3].terminal,  "VI8");
    strcpy(dac_param.in_out.analog_outputs[4].terminal,  "VU5");
    strcpy(dac_param.in_out.analog_outputs[5].terminal,  "VU6");
    strcpy(dac_param.in_out.analog_outputs[6].terminal,  "VU7");
    strcpy(dac_param.in_out.analog_outputs[7].terminal,  "VU8");
    ret = _dac_controller_ctx->ops.register_context(_dac_controller_ctx, &dac_param);
    if(ret < 0) {
        printf("dac_controller_register_dac fail\n");
        exit(-1);
    }

    cmd = DAC_CONTROLLER_IOCTL_CMD_ACQUIRE_DAC_CONTEX;
    strcpy(_dac_controller_ioctl_param2->acquire_dac_ctx_param.in.signature, "U2");
    ret = _dac_controller_ctx->ops.ioctl(_dac_controller_ctx, cmd, _dac_controller_ioctl_param2);
    if(ret < 0) {
        printf("dac_controller_ioctl fail\n");
        exit(-1);
    }

    cmd = DAC_CONTROLLER_IOCTL_CMD_ACQUIRE_DAC_CONTEX;
    _dac_controller_ioctl_param3 = &manager->resources.dac_controller_ioctl_param_U3; 
    strcpy(_dac_controller_ioctl_param3->acquire_dac_ctx_param.in.signature, "U3");
    ret = _dac_controller_ctx->ops.ioctl(_dac_controller_ctx, cmd, _dac_controller_ioctl_param3);
    if(ret < 0) {
        printf("dac_controller_ioctl fail\n");
        exit(-1);
    }
    
}

void init_ssd1306_oled_resources(Manager * manager)
{
    Manager *_manager = manager;
    int ret, cmd;
    ssd1306_oled_controller_context *_ssd1306_oled_controller_ctx = NULL;
    ssd1306_oled_controller_ioctl_parameters *_ssd1306_oled_controller_ioctl_param_ssd1306_oled_1 = NULL;

    i2c_controller_context *_i2c_controller_ctx = NULL;
    i2c_controller_ioctl_parameters *_i2c_controller_ioctl_param1 = NULL;
    register_ssd1306_oled_parameters ssd1306_oled_param;

    _ssd1306_oled_controller_ctx = &_manager->resources.ssd1306_oled_controller_ctx;
    _i2c_controller_ctx = &_manager->resources.i2c_controller_ctx ;
    _i2c_controller_ioctl_param1 = &_manager->resources.i2c_controller_ioctl_param_node1;
    _ssd1306_oled_controller_ioctl_param_ssd1306_oled_1 = &_manager->resources.ssd1306_oled_controller_ioctl_param_ssd1306_oled_1;

    ret = ssd1306_oled_controller_init(_ssd1306_oled_controller_ctx, &_manager->log_controller_ctx, &_manager->log_controller_ioctl_param);
    if(ret < 0) {
        printf("ssd1306_oled_controller_init fail\n");
        exit(-1);
    }

    strcpy(ssd1306_oled_param.in.signature, "SSD1306_OLED");
    ssd1306_oled_param.in.i2c_controller_ctx = _i2c_controller_ctx;
    ssd1306_oled_param.in.i2c_controller_ioctl_param = _i2c_controller_ioctl_param1;
    ret = _ssd1306_oled_controller_ctx->ops.register_context(_ssd1306_oled_controller_ctx, &ssd1306_oled_param);
    if(ret < 0) {
        printf("ssd1306_oled_controller->register_context fail\n");
        exit(-1);
    }

    cmd = SSD1306_OLED_CONTROLLER_IOCTL_CMD_ACQUIRE_SSD1306_OLED_CONTEXT;
    strcpy(_ssd1306_oled_controller_ioctl_param_ssd1306_oled_1->acquire_ssd1306_oled_ctx_param.in.signature, "SSD1306_OLED");
    ret = _ssd1306_oled_controller_ctx->ops.ioctl(_ssd1306_oled_controller_ctx, cmd, _ssd1306_oled_controller_ioctl_param_ssd1306_oled_1);
    if(ret < 0) {
        printf("ssd1306_oled_controller->ioctl fail\n");
        exit(-1);
    }

    _ssd1306_oled_controller_ctx->ops.list_resources(_ssd1306_oled_controller_ctx);

}


int manager_load_resources(Manager * _manager)
{
    Manager *manager = _manager;
    int ret, cmd;
    gpio_controller_context *_gpio_controller_ctx = NULL;
    spi_controller_context *_spi_controller_ctx = NULL;
    dac_controller_context *_dac_controller_ctx = NULL;
    dac_controller_ioctl_parameters *_dac_controller_ioctl_param = NULL;
    io_expander_controller_context *_io_expander_controller_ctx = NULL;
    spi_controller_ioctl_parameters *_spi_controller_ioctl_param = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param = NULL;
    adc_controller_context *_adc_controller_ctx = NULL;
    adc_controller_ioctl_parameters *_adc_controller_ioctl_param = NULL;
    io_expander_controller_ioctl_parameters *_io_expander_controller_ioctl_param = NULL;
    register_spi_bus_parameters spi_bus_param_bus0_node0;
    register_spi_bus_parameters spi_bus_param_bus0_node1;
    register_spi_bus_parameters spi_bus_param_bus0_node2;
    register_spi_bus_parameters spi_bus_param_bus0_node3;
    register_spi_bus_parameters spi_bus_param_bus0_node4;
    register_spi_bus_parameters spi_bus_param_bus0_node5;
    register_spi_bus_parameters spi_bus_param_bus0_node6;
    //register_spi_bus_parameters spi_bus_param_bus0_node7;
    register_io_expander_parameters io_expander_param;
    register_gpio_port_parameters gpio_port_param;
    register_dac_parameters dac_param;
    register_adc_parameters register_adc_param;
    gpio_controller_ioctl_parameters gpio_controller_ioctl_param_temp;

    if(!manager) {
        printf("manager_resource_setup, fail\n");
        return -1;
    }

#if 1
    init_gpio_resources(manager);

    init_spi_resources(manager);

    init_i2c_resources(manager);

    init_io_expander_resources(manager);

    init_adc_resources(manager);

    init_dac_resources(manager);

    init_ssd1306_oled_resources(manager);
#endif

    return 0;

}

void manager_deinit(Manager *manager)
{
    Manager *_manager = manager;
    modbus_tcp_context *modbus_ctx = NULL;
    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param; 

    if(!_manager) {
        printf("manager_deinit, fail\n");
        return ;
    }

    _log_controller_ctx = &_manager->log_controller_ctx;
    _log_controller_ioctl_param = &_manager->log_controller_ioctl_param;
    
    modbus_ctx = &_manager->modbus_ctx;

    modbus_tcp_deinit(modbus_ctx);
}


int manager_run(Manager *manager)
{
    int ret = 0;
    Manager *_manager = manager;
    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param; 
    modbus_tcp_context *modbus_ctx = NULL;

    if(!_manager) {
        printf("manager_run, fail\n");
        return -1;
    }

    _log_controller_ctx = &_manager->log_controller_ctx;
    _log_controller_ioctl_param = &_manager->log_controller_ioctl_param;

    modbus_ctx = &_manager->modbus_ctx;

    _manager->status = MANAGER_STATUS_CONFIGURING;

    // set data model to default when power on
    
    ret = pthread_create(&_manager->modbus_thread_id, NULL, start_modbus_tcp_routine, (void*)_manager);
    if(ret != 0) {
        printf("manager_run-> pthread_create -> start_modbus_tcp_routine, fail\n");
        _manager->status = MANAGER_STATUS_EXCEPTION;
    }

#if 1
    ret = pthread_create(&_manager->ip_monitor_id, NULL, start_misc_monitor_routine, (void*)_manager);
    if(ret != 0) {
        printf("manager_run-> pthread_create -> start_physical_channel_monitor_routine, fail\n");
        manager->status = MANAGER_STATUS_EXCEPTION;
    }

    ret = pthread_create(&_manager->physical_channel_monitor_id, NULL, start_physical_channel_monitor_routine, (void*)_manager);
    if(ret != 0) {
        printf("manager_run-> pthread_create -> start_physical_channel_monitor_routine, fail\n");
        manager->status = MANAGER_STATUS_EXCEPTION;
    }
#endif

    ret = pthread_create(&_manager->logical_channel_manager_id, NULL, start_logical_channel_manager_routine, (void*)_manager);
    if(ret != 0) {
        printf("manager_run-> pthread_create -> start_physical_channel_monitor_routine, fail\n");
        _manager->status = MANAGER_STATUS_EXCEPTION;
    }

    _manager->status = MANAGER_STATUS_RUNNING;

    //sleep(30);

    //manager->status = MANAGER_STATUS_QUIT;

    pthread_join(_manager->physical_channel_monitor_id, NULL);
    
    pthread_join(_manager->logical_channel_manager_id, NULL);

    pthread_join(_manager->modbus_thread_id, NULL);

    return ret;
}

void init_register_mapping(Manager *_manager)
{
    Manager *manager = _manager;
    int channel_id, first_step_parameter_channel_id;    
    Register *_register = NULL;
    Data_Model *data_model = NULL;

   
    _register =  &manager->modbus_ctx._register;
    data_model = manager->modbus_ctx._register.data_model;
    
    // initialize logical channels and physical channels
    for(channel_id=LOGICAL_CHANNEL_1_ID; channel_id<NB_LOGIC_CHANNEL; channel_id++) {

        // initialize logical channel registers
        init_logical_channel_register_mapping(manager, channel_id);
        
        // initialize physical channel registers
        init_physical_channel_register_mapping(manager, channel_id);

        // initialize step parameter pointers
        first_step_parameter_channel_id = ((NB_LOGIC_CHANNEL-1) + (channel_id * NB_PARAMETER_CHANNELS_FOR_EACH_LOGICAL_CHANNEL) + 1);
        init_step_parameter_pointer_register_mapping(manager, channel_id, first_step_parameter_channel_id);

        // initialize step parameters
        init_step_parameter_register_mapping(manager, channel_id);
    }
}

void *start_modbus_tcp_routine(void *parameter)
{

    Manager *manager = NULL;
    modbus_tcp_context *modbus_ctx = NULL;

    if(!parameter) {
        printf("start_modbus_tcp_routine, incorrect parameters\n");
        return NULL;
    }

    manager = (Manager *) parameter;

    modbus_ctx = &manager->modbus_ctx;

    //printf("start_modbus_tcp_routine\n");
    
    while(1) {

        switch(manager->status) {

            case MANAGER_STATUS_POR:
                 printf("manager status, POR\n");
            break;

            case MANAGER_STATUS_CONFIGURING:
                 printf("manager status, configuring\n");
            break;

            case MANAGER_STATUS_RUNNING:
                 //printf("manager status, running\n");
                 modbus_tcp_run(modbus_ctx);
            break;

            case MANAGER_STATUS_STOP:
                 printf("manager status, stop\n");
            break;

            case MANAGER_STATUS_QUIT:
                 printf("manager status, quit\n");
                 return NULL;
            break;
        
        }
    }
}

void do_physical_channel_logic(Manager *_manager)
{
    Manager *manager = _manager;
    Register *_register = NULL;
    double temp;
    int ret = 0, physical_channel_id, cmd;
    uint16_t *kz_h, *kz_m, *kz_l, *jc, *cf, *ck, *cled, *fled, *bell, *led_ready, *led_run, *led_error, *ac_loss;
    uint16_t *last_kz_h, *last_kz_m, *last_kz_l;
    uint16_t *last_jc, *last_cf, *last_ck, *last_cled, *last_fled, *last_bell, *last_led_ready, *last_led_run, *last_led_error, *last_ac_loss;
    char *_da_terminal = NULL, *_io_expander_terminal = NULL;
    char format[8][15] = {"KZ_H%d", "KZ_M%d", "KZ_L%d", "JC%d", "CF%d", "CK%d", "CLED%d", "FLED%d"};
    double temp_vi, temp_vu, temp_64;

    _4paired_register_region *vi_region, *vu_region, *last_vi_region, *last_vu_region; 

    io_expander_controller_context *_io_expander_controller_ctx = NULL;
    io_expander_controller_ioctl_parameters *_io_expander_controller_ioctl_param = NULL;
    dac_controller_context *_dac_controller_ctx = NULL;
    dac_controller_ioctl_parameters *_dac_controller_ioctl_param = NULL;
    gpio_controller_ioctl_parameters *_gpio_controller_ioctl_param = NULL;

    _register = &manager->modbus_ctx._register;

    _io_expander_controller_ctx = &manager->resources.io_expander_controller_ctx;
    _dac_controller_ctx = &manager->resources.dac_controller_ctx;

    bell = _register->physical_channel[0].bell;
    led_ready = _register->physical_channel[0].led_ready;
    led_run = _register->physical_channel[0].led_run;
    led_error = _register->physical_channel[0].led_error;;
    ac_loss = _register->physical_channel[0].ac_loss;
    last_bell = _register->physical_channel[0].temp.last_bell;
    last_led_ready = _register->physical_channel[0].temp.last_led_ready;
    last_led_run = _register->physical_channel[0].temp.last_led_run;
    last_led_error = _register->physical_channel[0].temp.last_led_error;;
    last_ac_loss = _register->physical_channel[0].temp.last_ac_loss;

    // check if need to update bell
    if(*bell != *last_bell) {
        if(*bell == 1) {
            // alarm now  
            ret = bell_enable(manager);
            if(!ret) {
                *last_bell = *bell;
            }
        } else {
            // clear alarm
            ret = bell_disable(manager);
            if(!ret) {
                *last_bell = *bell;
            }
        }
    }

    if(*led_ready != *last_led_ready) {
        if(*led_ready == 1) {
            // alarm now  
            ret = led_ready_enable(manager);
            if(!ret) {
                *last_led_ready = *led_ready;
            }
        } else {
            // clear alarm
            ret = led_ready_disable(manager);
            if(!ret) {
                *last_led_ready = *led_ready;
            }
        }
    }
    
    if(*led_run != *last_led_run) {
        if(*led_run == 1) {
            // alarm now  
            ret = led_run_enable(manager);
            if(!ret) {
                *last_led_run = *led_run;
            }
        } else {
            // clear alarm
            ret = led_run_disable(manager);
            if(!ret) {
                *last_led_run = *led_run;
            }
        }
    }

    if(*led_error != *last_led_error) {
        if(*led_error == 1) {
            // alarm now  
            ret = led_error_enable(manager);
            if(!ret) {
                *last_led_error = *led_error;
            }
        } else {
            // clear alarm
            ret = led_error_disable(manager);
            if(!ret) {
                *last_led_error = *led_error;
            }
        }
    }

    _gpio_controller_ioctl_param = &_manager->resources.gpio_controller_ioctl_param_ac_loss;
    cmd = GPIO_CONTROLLER_IOCTL_CMD_GET_LEVEL;
    ret = gpio_controller_ioctl(&_manager->resources.gpio_controller_ctx, cmd, _gpio_controller_ioctl_param);
    if(ret < 0) {
        printf("gpio_controller_ioctl fail to acquire gpio context\n");
    }
    *ac_loss = _gpio_controller_ioctl_param->get_level_param.out.level;
    
    if(*ac_loss != *last_ac_loss){
        *last_ac_loss = *ac_loss;
    }

    if(*ac_loss == AC_LOSS_ON) {
        
    }

    update_bv_and_cv(manager);
    update_sample_voltage_and_current(manager);

#if 0
    // for debug purposes, please set binding to proper setting
    calculate_and_update_logical_voltage_and_current(manager, PHYSICAL_CHANNEL_1_ID);
#endif

    for(physical_channel_id=PHYSICAL_CHANNEL_1_ID; physical_channel_id<NB_PHYSICAL_CHANNEL; physical_channel_id++) {

        if(physical_channel_id == 0 || physical_channel_id == 1) {
            _io_expander_controller_ioctl_param = &manager->resources.io_expander_controller_ioctl_param_U1;
        } else if(physical_channel_id == 2 || physical_channel_id == 3) {
            _io_expander_controller_ioctl_param = &manager->resources.io_expander_controller_ioctl_param_U2;
        } else if(physical_channel_id == 4 || physical_channel_id == 5) {
            _io_expander_controller_ioctl_param = &manager->resources.io_expander_controller_ioctl_param_U3;
        } else if(physical_channel_id == 6 || physical_channel_id == 7) {
            _io_expander_controller_ioctl_param = &manager->resources.io_expander_controller_ioctl_param_U4;
        }

        _io_expander_terminal = _io_expander_controller_ioctl_param->set_level_param.in.terminal;

        if(physical_channel_id>=0 && physical_channel_id<=3) {
            _dac_controller_ioctl_param = &manager->resources.dac_controller_ioctl_param_U2;
        } else {
            _dac_controller_ioctl_param = &manager->resources.dac_controller_ioctl_param_U3;
        }
        _da_terminal = _dac_controller_ioctl_param->write_and_update_channel_param.in.terminal;
        
        vi_region = &_register->physical_channel[physical_channel_id].vi_region;
        vu_region = &_register->physical_channel[physical_channel_id].vu_region;

        kz_h = _register->physical_channel[physical_channel_id].kz_h;
        kz_m = _register->physical_channel[physical_channel_id].kz_m;
        kz_l = _register->physical_channel[physical_channel_id].kz_l;
        jc = _register->physical_channel[physical_channel_id].jc;
        cf = _register->physical_channel[physical_channel_id].cf;
        ck = _register->physical_channel[physical_channel_id].ck;
        cled = _register->physical_channel[physical_channel_id].cled;
        fled = _register->physical_channel[physical_channel_id].fled;

        last_vi_region = &_register->physical_channel[physical_channel_id].temp.last_vi_region;
        last_vu_region = &_register->physical_channel[physical_channel_id].temp.last_vu_region;

        last_kz_h = _register->physical_channel[physical_channel_id].temp.last_kz_h;
        last_kz_m = _register->physical_channel[physical_channel_id].temp.last_kz_m;
        last_kz_l = _register->physical_channel[physical_channel_id].temp.last_kz_l;
        last_jc = _register->physical_channel[physical_channel_id].temp.last_jc;
        last_cf = _register->physical_channel[physical_channel_id].temp.last_cf;
        last_ck = _register->physical_channel[physical_channel_id].temp.last_ck;
        last_cled = _register->physical_channel[physical_channel_id].temp.last_cled;
        last_fled = _register->physical_channel[physical_channel_id].temp.last_fled;

        // check if need to update vi
        ret = compare_4paired_register_region(vi_region, last_vi_region);
        if(ret) {
            printf("update vi, channel_id %d\n", physical_channel_id);
            snprintf(_da_terminal, 255, "VI%d", physical_channel_id + 1);
            
            temp_vi = combine_4paired_register_region(vi_region);
            temp = temp_vi / UNIT_FACTOR_BETWEEN_V_AND_10NV;
            printf("terminal:%s voltage %.15f\n", _da_terminal, temp);

            cmd = DAC_CONTROLLER_IOCTL_CMD_WRITE_AND_UPDATE_CHANNEL;
            _dac_controller_ioctl_param->write_and_update_channel_param.in.voltage = temp;
            ret = dac_controller_ioctl(_dac_controller_ctx, cmd, _dac_controller_ioctl_param);
            if(ret < 0) {
                printf("write_single_register->dac_controller_ioctl->write_and_update_channel fail\n");
            }

            copy_4paired_register_region(last_vi_region, vi_region);
        }

        // check if need to update vu
        ret = compare_4paired_register_region(vu_region, last_vu_region);
        if(ret) {
            printf("update vu, channel_id %d\n", physical_channel_id);
            snprintf(_da_terminal, 255, "VU%d", physical_channel_id + 1);
            
            temp_vu = combine_4paired_register_region(vu_region);
            temp = temp_vu / UNIT_FACTOR_BETWEEN_V_AND_10NV;
            printf("terminal:%s voltage %f\n", _da_terminal, temp);

            cmd = DAC_CONTROLLER_IOCTL_CMD_WRITE_AND_UPDATE_CHANNEL;
            _dac_controller_ioctl_param->write_and_update_channel_param.in.voltage = temp;
            ret = dac_controller_ioctl(_dac_controller_ctx, cmd, _dac_controller_ioctl_param);
            if(ret < 0) {
                printf("write_single_register->dac_controller_ioctl->write_and_update_channel fail\n");
            }

            copy_4paired_register_region(last_vu_region, vu_region);
        }

        // check if need to update kz_h
        if(*kz_h != *last_kz_h) {
            snprintf(_io_expander_terminal, 255, format[0], physical_channel_id + 1);
            if(*kz_h == 1) {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_HIGH;
            } else {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_LOW;
            }

            cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
            ret = io_expander_controller_ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
            if(ret < 0) {
                printf("write_single_coil->io_expander_controller_ioctl->set_level fail\n");
            }

            *last_kz_h = *kz_h;
        }

        // check if need to update kz_m
        if(*kz_m != *last_kz_m) {
            snprintf(_io_expander_terminal, 255, format[1], physical_channel_id + 1);
            if(*kz_m == 1) {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_HIGH;
            } else {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_LOW;
            }

            cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
            ret = io_expander_controller_ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
            if(ret < 0) {
                printf("write_single_coil->io_expander_controller_ioctl->set_level fail\n");
            }

            *last_kz_m = *kz_m;
        }

        // check if need to update kz_l
        if(*kz_l != *last_kz_l) {
            snprintf(_io_expander_terminal, 255, format[2], physical_channel_id + 1);
            if(*kz_l == 1) {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_HIGH;
            } else {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_LOW;
            }

            cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
            ret = io_expander_controller_ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
            if(ret < 0) {
                printf("write_single_coil->io_expander_controller_ioctl->set_level fail\n");
            }

            *last_kz_l = *kz_l;
        }

        // check if need to update jc
        if(*jc != *last_jc) {
            snprintf(_io_expander_terminal, 255, format[3], physical_channel_id + 1);
            if(*jc == 1) {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_HIGH;
            } else {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_LOW;
            }

            cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
            ret = io_expander_controller_ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
            if(ret < 0) {
                printf("write_single_coil->io_expander_controller_ioctl->set_level fail\n");
            }

            *last_jc = *jc;
        }

        // check if need to update cf
        if(*cf != *last_cf) {
            snprintf(_io_expander_terminal, 255, format[4], physical_channel_id + 1);
            if(*cf == 1) {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_HIGH;
            } else {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_LOW;
            }

            cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
            ret = io_expander_controller_ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
            if(ret < 0) {
                printf("write_single_coil->io_expander_controller_ioctl->set_level fail\n");
            }

            *last_cf = *cf;
        }

        // check if need to update ck
        if(*ck != *last_ck) {
            snprintf(_io_expander_terminal, 255, format[5], physical_channel_id + 1);
            if(*ck == 1) {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_HIGH;
            } else {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_LOW;
            }

            cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
            ret = io_expander_controller_ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
            if(ret < 0) {
                printf("write_single_coil->io_expander_controller_ioctl->set_level fail\n");
            }

            *last_ck = *ck;
        }

        // check if need to update cled
        if(*cled != *last_cled) {
            snprintf(_io_expander_terminal, 255, format[6], physical_channel_id + 1);
            if(*cled == 1) {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_HIGH;
            } else {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_LOW;
            }

            cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
            ret = io_expander_controller_ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
            if(ret < 0) {
                printf("write_single_coil->io_expander_controller_ioctl->set_level fail\n");
            }

            *last_cled = *cled;
        }

        // check if need to update fled
        if(*fled != *last_fled) {
            snprintf(_io_expander_terminal, 255, format[7], physical_channel_id + 1);
            if(*fled == 1) {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_HIGH;
            } else {
                _io_expander_controller_ioctl_param->set_level_param.in.level = IO_EXPANDER_PORT_LEVEL_LOW;
            }

            cmd = IO_EXPANDER_CONTROLLER_IOCTL_CMD_SET_LEVEL;
            ret = io_expander_controller_ioctl(_io_expander_controller_ctx, cmd, _io_expander_controller_ioctl_param);
            if(ret < 0) {
                printf("write_single_coil->io_expander_controller_ioctl->set_level fail\n");
            }

            *last_fled = *fled;
        }

    }
}

void do_misc_logic(Manager *_manager) {
    Manager *manager = _manager;
    int ret;

    ret = check_and_update_IP(_manager);
    if(ret != 0) {
        printf("do_misc_logic->check_and_update_IP fail\n");
        exit(-1);
    }

    ret = get_IP_and_update_oled(_manager);
    if(ret != 0) {
        printf("do_misc_logic->get_IP_and_update_oled fail\n");
        exit(-1);
    }

    check_and_update_tower_light(_manager);
    
    sleep(3);
}


void *start_misc_monitor_routine(void *parameter)
{
    Manager *manager = NULL;
    Data_Model *data_modle = NULL;

    if(!parameter) {
        printf("start_physical_channel_monitor_routine, incorrect parameters\n");
        return NULL;
    }

    manager = (Manager *) parameter;

    data_modle = manager->modbus_ctx._register.data_model;

    //printf("start_physical_channel_monitor_routine\n");
    
    while(1) {

        switch(manager->status) {

            case MANAGER_STATUS_POR:
                 printf("manager status, POR\n");
            break;

            case MANAGER_STATUS_CONFIGURING:
                 printf("manager status, configuring\n");
            break;

            case MANAGER_STATUS_RUNNING:
                 //printf("manager status, running\n");
                 do_misc_logic(manager);
            break;

            case MANAGER_STATUS_STOP:
                 printf("manager status, stop\n");
            break;

            case MANAGER_STATUS_QUIT:
                 printf("manager status, quit\n");
                 return NULL;
            break;
        
        }
    } 
}

void *start_physical_channel_monitor_routine(void *parameter)
{
    Manager *manager = NULL;
    Data_Model *data_modle = NULL;

    if(!parameter) {
        printf("start_physical_channel_monitor_routine, incorrect parameters\n");
        return NULL;
    }

    manager = (Manager *) parameter;

    data_modle = manager->modbus_ctx._register.data_model;

    //printf("start_physical_channel_monitor_routine\n");
    
    while(1) {

        switch(manager->status) {

            case MANAGER_STATUS_POR:
                 printf("manager status, POR\n");
            break;

            case MANAGER_STATUS_CONFIGURING:
                 printf("manager status, configuring\n");
            break;

            case MANAGER_STATUS_RUNNING:
                 //printf("manager status, running\n");
                 do_physical_channel_logic(manager);
            break;

            case MANAGER_STATUS_STOP:
                 printf("manager status, stop\n");
            break;

            case MANAGER_STATUS_QUIT:
                 printf("manager status, quit\n");
                 return NULL;
            break;
        
        }
    } 
}

void *start_logical_channel_manager_routine(void *parameter)
{
    Manager *manager = NULL;
    Data_Model *data_modle = NULL;

    if(!parameter) {
        printf("start_physical_channel_monitor_routine, incorrect parameters\n");
        return NULL;
    }

    manager = (Manager *) parameter;

    data_modle = manager->modbus_ctx._register.data_model;

    //printf("start_logical_channel_monitor_routine\n");
    
    while(1) {

        switch(manager->status) {

            case MANAGER_STATUS_POR:
                 printf("manager status, POR\n");
            break;

            case MANAGER_STATUS_CONFIGURING:
                 printf("manager status, configuring\n");
            break;

            case MANAGER_STATUS_RUNNING:
                 //printf("manager status, running\n");
                 do_main_loop(manager);
            break;

            case MANAGER_STATUS_STOP:
                 printf("manager status, stop\n");
            break;

            case MANAGER_STATUS_QUIT:
                 printf("manager status, quit\n");
                 return NULL;
            break;
        
        }
    } 
}

int do_verification_logic(Manager *_manager)
{
    int ret = 0;

    return ret;
}

int check_step_parameters(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    uint16_t exp_code;
    Register *_register = NULL;
    int ret, logical_channel_id = channel_id;
    uint16_t *exp_code_reg, *mode_reg, *l_status_reg, *exp_message_reg, *step_type_reg;
    uint16_t *jump_condiction_enable_reg;
    uint16_t *step_pointer_reg, *step_quantity_reg, *step_definition_temp;
    
    _register = &manager->modbus_ctx._register;

    mode_reg = _register->logical_channel[logical_channel_id].mode;
    l_status_reg = _register->logical_channel[logical_channel_id].status;
    exp_code_reg = _register->logical_channel[logical_channel_id].exception_code;
    step_pointer_reg = _register->logical_channel[logical_channel_id].step_pointer;
    exp_message_reg = _register->logical_channel[logical_channel_id].exception_message;
    jump_condiction_enable_reg = _register->logical_channel[logical_channel_id].step_param.jump.enable;
    step_type_reg = _register->logical_channel[logical_channel_id].step_param.step_type;

    *exp_message_reg = 0;

    switch(*step_type_reg) {

        case FORMATION_STEP_TYPE_REST:
             // check REST parameters
             ret = check_REST_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check REST parameters fail\n");
                 return ret;
             }

             init_REST_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check_step_parameters->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;

        break;

        case FORMATION_STEP_TYPE_CCC:
             // check CCC parameters
             ret = check_CCC_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CCC parameters fail\n");
                 return ret;
             }

             init_CCC_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check_step_parameters->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCD:
             // check CCD parameters
             ret = check_CCD_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CCD parameters fail\n");
                 return ret;
             }

             init_CCD_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check_step_parameters->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCCCVC:
             // check CCCCVC parameters
             ret = check_CCCCVC_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CCCCVC parameters fail\n");
                 return ret;
             }

             init_CCCCVC_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check_step_parameters->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCDCVD:
             // check CCDCVD parameters
             ret = check_CCDCVD_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CCDCVD parameters fail\n");
                 return ret;
             }

             init_CCDCVD_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check_step_parameters->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CPC:
             // check CPC parameters
             ret = check_CPC_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CCDCVD parameters fail\n");
                 return ret;
             }

             init_CPC_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check_step_parameters->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CPD:
             // check CPD parameters
             ret = check_CPD_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CCDCVD parameters fail\n");
                 return ret;
             }

             init_CPD_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check_step_parameters->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CRD:
             // check CPC parameters
             ret = check_CRD_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check CCDCVD parameters fail\n");
                 return ret;
             }

             init_CRD_registers(manager, logical_channel_id);

             ret = init_physical_channel(manager, logical_channel_id);
             if(ret < 0) {
                 printf("check_step_parameters->init_physical_channel fail\n");
                 return ERROR_PHYSICAL_CHANNEL_LOST_CONTROL;
             }

             update_step_established_time(manager, logical_channel_id);

             update_holding_time_left_boundary_now(manager, logical_channel_id);

             return 0;
        break;

        case FORMATION_STEP_TYPE_CYCLE:
             
             return 0;
        break;

        case FORMATION_STEP_TYPE_GOTO:
             printf("check_step_parameters->check_goto_parameters\n");
             return 0;
        break;

        default:
             return ERROR_UNKNOWN_STEP_TYPE;
        break;
    }
    
    return 0;
}

int update_step_parameters(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    uint16_t exp_code;
    Register *_register = NULL;
    int ret, logical_channel_id = channel_id;
    uint16_t *exp_code_reg, *mode_reg, *l_status_reg, *exp_message_reg, *step_type_reg;
    uint16_t *jump_condiction_enable_reg;
    uint16_t *step_pointer_reg, *step_quantity_reg, *step_definition_temp;
    uint16_t *cycle_begin, *cycle_number, *cycle_count;

    _register = &manager->modbus_ctx._register;

    mode_reg = _register->logical_channel[logical_channel_id].mode;
    l_status_reg = _register->logical_channel[logical_channel_id].status;
    exp_code_reg = _register->logical_channel[logical_channel_id].exception_code;
    step_pointer_reg = _register->logical_channel[logical_channel_id].step_pointer;
    exp_message_reg = _register->logical_channel[logical_channel_id].exception_message;
    jump_condiction_enable_reg = _register->logical_channel[logical_channel_id].step_param.jump.enable;
    step_type_reg = _register->logical_channel[logical_channel_id].step_param.step_type;

    *exp_message_reg = 0;

    printf("update_step_parameters step_pointer %d\n", *step_pointer_reg);

    if((*step_pointer_reg >= 1) && (*step_pointer_reg <= 655)) {
         // do nothing if step_pointer in the first parameter channel
         step_definition_temp = _register->logical_channel[logical_channel_id].step_param_pointers.nb_1_655;
     } else if((*step_pointer_reg >= 656) && (*step_pointer_reg <= 1310)) {
         step_definition_temp = _register->logical_channel[logical_channel_id].step_param_pointers.nb_656_1310;
     } else if((*step_pointer_reg >= 1311) && (*step_pointer_reg <= 1965)) {
         step_definition_temp = _register->logical_channel[logical_channel_id].step_param_pointers.nb_1311_1965;
     } else if((*step_pointer_reg >= 1966) && (*step_pointer_reg <= 2620)) {
         step_definition_temp = _register->logical_channel[logical_channel_id].step_param_pointers.nb_1966_2620;
     } else if((*step_pointer_reg >= 2621) && (*step_pointer_reg <= 3275)) {
         step_definition_temp = _register->logical_channel[logical_channel_id].step_param_pointers.nb_2621_3275;
     } else if((*step_pointer_reg >= 3276) && (*step_pointer_reg <= 3930)) {
         step_definition_temp = _register->logical_channel[logical_channel_id].step_param_pointers.nb_3276_3930;
     } else if((*step_pointer_reg >= 3931) && (*step_pointer_reg <= 4585)) {
         step_definition_temp = _register->logical_channel[logical_channel_id].step_param_pointers.nb_3931_4585;
     } else if((*step_pointer_reg >= 4586) && (*step_pointer_reg <= 5240)) {
         step_definition_temp = _register->logical_channel[logical_channel_id].step_param_pointers.nb_4586_5240;
     } else if((*step_pointer_reg >= 5241) && (*step_pointer_reg <= 5895)) {
         step_definition_temp = _register->logical_channel[logical_channel_id].step_param_pointers.nb_5241_5895;
     } else if((*step_pointer_reg >= 5896) && (*step_pointer_reg <= 6550)) {
         step_definition_temp = _register->logical_channel[logical_channel_id].step_param_pointers.nb_5896_6550;
     } else if((*step_pointer_reg >= 6551) && (*step_pointer_reg <= 7205)) {
         step_definition_temp = _register->logical_channel[logical_channel_id].step_param_pointers.nb_6551_7205;
     } else if((*step_pointer_reg >= 7206) && (*step_pointer_reg <= 7860)) {
         step_definition_temp = _register->logical_channel[logical_channel_id].step_param_pointers.nb_7206_7860;
     } else {
         return ERROR_INCORRECT_STEP_POINTER;
     }

    //step_definition_temp = step_definition_temp + (*step_pointer_reg - 1) * HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET;
    step_definition_temp = step_definition_temp + ((*step_pointer_reg - 1)%655) * HOLDING_STEP_DEFINITION_REGISTER_END_OFFSET;

    switch(*step_type_reg) {

        case FORMATION_STEP_TYPE_CRD:
        case FORMATION_STEP_TYPE_CPC:
        case FORMATION_STEP_TYPE_CPD:
        case FORMATION_STEP_TYPE_GOTO:
        case FORMATION_STEP_TYPE_CYCLE:
        case FORMATION_STEP_TYPE_REST:
        case FORMATION_STEP_TYPE_CCC:
        case FORMATION_STEP_TYPE_CCD:
        case FORMATION_STEP_TYPE_CCCCVC:
        case FORMATION_STEP_TYPE_CCDCVD:

             update_step_parameter_register_mapping(manager, logical_channel_id, step_definition_temp);

             return 0;
        break;

        default:
             printf("update_step_parameters step_type %d\n", *step_type_reg);
             return ERROR_UNKNOWN_STEP_TYPE;
        break;
        
    }

    return 0;
}

int do_formation_goto(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    uint16_t exp_code;
    Register *_register = NULL;
    int ret, logical_channel_id = channel_id;
    uint16_t *exp_code_reg, *mode_reg, *l_status_reg, *exp_message_reg, *step_type;
    uint16_t *step_pointer_reg, *step_quantity_reg;
    uint16_t *cycle_begin, *cycle_number, *cycle_count;
    uint16_t *goto_next_step_id_reg;

    _register = &manager->modbus_ctx._register;

    mode_reg = _register->logical_channel[logical_channel_id].mode;
    l_status_reg = _register->logical_channel[logical_channel_id].status;
    exp_code_reg = _register->logical_channel[logical_channel_id].exception_code;
    step_pointer_reg = _register->logical_channel[logical_channel_id].step_pointer;
    exp_message_reg = _register->logical_channel[logical_channel_id].exception_message;

    step_type = _register->logical_channel[logical_channel_id].step_param.step_type;

    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_GOTO;

    // initialize step parameter registers
    switch(*step_type) {

        case FORMATION_STEP_TYPE_CRD:
        case FORMATION_STEP_TYPE_CPC:
        case FORMATION_STEP_TYPE_CPD:
        case FORMATION_STEP_TYPE_REST:
        case FORMATION_STEP_TYPE_CCC:
        case FORMATION_STEP_TYPE_CCD:
        case FORMATION_STEP_TYPE_CCCCVC:
        case FORMATION_STEP_TYPE_CCDCVD:
          
             *step_pointer_reg = *step_pointer_reg + 1;

             ret = update_step_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("do_formation_goto->update_step_parameters\n");
                 return ret;
             }
           
             ret = check_step_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("do_formation_goto->check_step_parameters 1\n");
                 return ret;
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CYCLE:
             // get count, number, and begin pointer, first
             cycle_begin = _register->logical_channel[logical_channel_id].step_param.info.cycle.begin;
             cycle_number = _register->logical_channel[logical_channel_id].step_param.info.cycle.number;
             cycle_count = _register->logical_channel[logical_channel_id].step_param.info.cycle.count;

             // check if need to jump to the next step
             if(*cycle_count == *cycle_number) {
                 *step_pointer_reg = *step_pointer_reg + 1;
             } else {
                 *cycle_count = *cycle_count + 1;
                 *step_pointer_reg = *_register->logical_channel[logical_channel_id].step_param.info.cycle.begin;
             }

             ret = update_step_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("do_formation_goto->update_step_parameters\n");
                 return ret;
             }

             ret = check_step_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("do_formation_goto->check_step_parameters 2\n");
                 return ret;
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_GOTO:

             goto_next_step_id_reg = _register->logical_channel[logical_channel_id].step_param.info.goingto.next;

             *step_pointer_reg = *goto_next_step_id_reg;

             // next step, or jump to step index?
             ret = update_step_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("do_formation_goto->update_step_parameters\n");
                 return ret;
             }
           
             ret = check_step_parameters(manager, logical_channel_id);
             if(ret < 0) {
                 printf("do_formation_goto->check_step_parameters 1\n");
                 return ret;
             }
             return 0;
        break;

        default:
             return ERROR_UNKNOWN_STEP_TYPE;
        break;
    }

    return 0;
}



int do_formation_transforming(Manager *_manager, int channel_id, int jump)
{
    Manager *manager = _manager;
    uint16_t exp_code;
    Register *_register = NULL;
    int ret, logical_channel_id = channel_id, flag;
    uint16_t *exp_code_reg, *mode_reg, *l_status_reg, *exp_message_reg, *step_type_reg;
    uint16_t *step_pointer_reg, *step_quantity_reg;
    uint16_t *cycle_begin, *cycle_number, *cycle_count;
    uint16_t *goto_behavior, *goto_operator, *goto_next_step_id, *goto_next_step_index;
    uint16_t *jump_enable, *action;
    _4paired_register_region *goto_operand2_region, *goto_operand1_region;
    uint64_t goto_operand1, goto_operand2;

    _register = &manager->modbus_ctx._register;

    action = _register->logical_channel[logical_channel_id].action;
    mode_reg = _register->logical_channel[logical_channel_id].mode;
    l_status_reg = _register->logical_channel[logical_channel_id].status;
    exp_code_reg = _register->logical_channel[logical_channel_id].exception_code;
    step_pointer_reg = _register->logical_channel[logical_channel_id].step_pointer;
    step_quantity_reg = _register->logical_channel[logical_channel_id].step_quantity;
    exp_message_reg = _register->logical_channel[logical_channel_id].exception_message;
    
    step_type_reg = _register->logical_channel[logical_channel_id].step_param.step_type;

    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_TRANSFORMING;

    reset_one_logical_channel_to_prepare_state(manager, logical_channel_id);

    // set all physical channels to prepare state
    ret = reset_all_bound_physical_channels_to_prepare_state(manager, logical_channel_id);
    if(ret < 0) {
        printf("do_formation_transforming->reset_all_bound_physical_channels_to_prepare_state fail\n");
        return ret;
    }

    // check if all physical channels are in the prepare state
    ret = check_if_bound_physical_channels_in_prepare_state(manager, logical_channel_id);
    if(ret < 0) {
        printf("do_formation_transforming->check_if_bound_physical_channels_in_prepare_state fail\n");
        return ret;
    }

    switch(*step_type_reg) {

        case FORMATION_STEP_TYPE_REST:
             if(jump == JUMP_CONDICTION_TIME) { // timeout
                 if(*step_pointer_reg != *step_quantity_reg) {
                     ret = do_formation_goto(manager, logical_channel_id);
                     if(ret < 0) {
                         printf("do_formation_transforming->do_formation_goto, fail, ret %d\n", ret);
                         return ret;
                     }

                     *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                     return 0;
                 }

                 *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;

                 return 0;
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCC:
             if(jump == JUMP_CONDICTION_TIME || jump == JUMP_CONDICTION_VOLTAGE || jump == JUMP_CONDICTION_CAPACITY) {
                 if(*step_pointer_reg != *step_quantity_reg) {
                     ret = do_formation_goto(manager, logical_channel_id);
                     if(ret < 0) {
                         printf("do_formation_transforming->do_formation_goto, fail, ret %d\n", ret);
                         return ret;
                     }

                     *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                     return 0;
                 }

                 *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;

                 return 0;
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCD:
             if(jump == JUMP_CONDICTION_TIME || jump == JUMP_CONDICTION_VOLTAGE || jump == JUMP_CONDICTION_CAPACITY) {
                 if(*step_pointer_reg != *step_quantity_reg) {
                     ret = do_formation_goto(manager, logical_channel_id);
                     if(ret < 0) {
                         printf("do_formation_transforming->do_formation_goto, fail, ret %d\n", ret);
                         return ret;
                     }

                     *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                     return 0;
                 }

                 *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;

                 return 0;
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCCCVC:
             if(jump == JUMP_CONDICTION_TIME || jump == JUMP_CONDICTION_CURRENT || jump == JUMP_CONDICTION_CAPACITY) {
                 if(*step_pointer_reg != *step_quantity_reg) {
                     ret = do_formation_goto(manager, logical_channel_id);
                     if(ret < 0) {
                         printf("do_formation_transforming->do_formation_goto, fail, ret %d\n", ret);
                         return ret;
                     }

                     *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                     return 0;
                 }

                 *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;

                 return 0;
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CCDCVD:
             if(jump == JUMP_CONDICTION_TIME || jump == JUMP_CONDICTION_CURRENT || jump == JUMP_CONDICTION_CAPACITY) {
                 if(*step_pointer_reg != *step_quantity_reg) {
                     ret = do_formation_goto(manager, logical_channel_id);
                     if(ret < 0) {
                         printf("do_formation_transforming->do_formation_goto, fail, ret %d\n", ret);
                         return ret;
                     }

                     *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                     return 0;
                 }

                 *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;

                 return 0;
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CRD:
             if(jump == JUMP_CONDICTION_TIME || jump == JUMP_CONDICTION_VOLTAGE || jump == JUMP_CONDICTION_CAPACITY) {
                 if(*step_pointer_reg != *step_quantity_reg) {
                     ret = do_formation_goto(manager, logical_channel_id);
                     if(ret < 0) {
                         printf("do_formation_transforming->do_formation_goto, fail, ret %d\n", ret);
                         return ret;
                     }

                     *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                     return 0;
                 }

                 *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;

                 return 0;
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CPC:
             if(jump == JUMP_CONDICTION_TIME || jump == JUMP_CONDICTION_VOLTAGE || jump == JUMP_CONDICTION_CAPACITY) {
                 if(*step_pointer_reg != *step_quantity_reg) {
                     ret = do_formation_goto(manager, logical_channel_id);
                     if(ret < 0) {
                         printf("do_formation_transforming->do_formation_goto, fail, ret %d\n", ret);
                         return ret;
                     }

                     *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                     return 0;
                 }

                 *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;

                 return 0;
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CPD:
             if(jump == JUMP_CONDICTION_TIME || jump == JUMP_CONDICTION_VOLTAGE || jump == JUMP_CONDICTION_CAPACITY) {
                 if(*step_pointer_reg != *step_quantity_reg) {
                     ret = do_formation_goto(manager, logical_channel_id);
                     if(ret < 0) {
                         printf("do_formation_transforming->do_formation_goto, fail, ret %d\n", ret);
                         return ret;
                     }

                     *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                     return 0;
                 }

                 *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;

                 return 0;
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_CYCLE:
             if(jump == JUMP_CONDICTION_CYCLE) {
                 if(*step_pointer_reg != *step_quantity_reg) {
                     ret = do_formation_goto(manager, logical_channel_id);
                     if(ret < 0) {
                         printf("do_formation_transforming->do_formation_goto, fail, ret %d\n", ret);
                         return ret;
                     }

                     *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                     return 0;

                 } else {
                     cycle_begin = _register->logical_channel[logical_channel_id].step_param.info.cycle.begin;
                     cycle_number = _register->logical_channel[logical_channel_id].step_param.info.cycle.number;
                     cycle_count = _register->logical_channel[logical_channel_id].step_param.info.cycle.count;

                     if(*cycle_count == *cycle_number) {
                         *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;
                         return 0;
                     } else {
                         // do loop again, but need to decrease the cycle_count
                         ret = do_formation_goto(manager, logical_channel_id);
                         if(ret < 0) {
                             printf("do_formation_transforming->do_formation_goto, fail, ret %d\n", ret);
                             return ret;
                         }

                         *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                         return 0;
                     }
                 }
             }

             return 0;
        break;

        case FORMATION_STEP_TYPE_GOTO:
             if(jump == JUMP_CONDICTION_GOTO) {

                 flag = 0;
                 jump_enable = _register->logical_channel[logical_channel_id].step_param.jump.enable;
                 goto_behavior = _register->logical_channel[logical_channel_id].step_param.info.goingto.behavior;
                 goto_operator = _register->logical_channel[logical_channel_id].step_param.info.goingto.operator;
                 goto_next_step_id = _register->logical_channel[logical_channel_id].step_param.info.goingto.next;
                 goto_next_step_index = _register->logical_channel[logical_channel_id].step_param.info.goingto.step_index;
                 goto_operand2_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.operand2_region;

                 update_goto_parameters(manager, logical_channel_id);

                 // update last_xxx

                 if(*jump_enable & STEP_JUMP_CONDICTION_TIMEOUT) {
                     goto_operand1_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_holding_time_region;
                 } else if(*jump_enable & STEP_JUMP_CONDICTION_VOLTAGE) {
                     goto_operand1_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_voltage_region;
                 } else if(*jump_enable & STEP_JUMP_CONDICTION_CURRENT) {
                     goto_operand1_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_current_region;
                 } else if(*jump_enable & STEP_JUMP_CONDICTION_CAPACITY) {
                     goto_operand1_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_capacity_region;
                 } else if(*jump_enable & STEP_JUMP_CONDICTION_ENERGY) {
                     goto_operand1_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_energy_region;
                 } else if(*jump_enable & STEP_JUMP_CONDICTION_TEMPERATURE) {
                     goto_operand1_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_temperature_region;
                 }

                 goto_operand1 = combine_4paired_register_region(goto_operand1_region);
                 goto_operand2 = combine_4paired_register_region(goto_operand2_region);

                 if(*goto_operator == GOTO_OPERATOR_GT) {
                     if(goto_operand1 > goto_operand2) {
                         flag = 1;
                     }
                 } else if(*goto_operator == GOTO_OPERATOR_GE) {
                     if(goto_operand1 >= goto_operand2) {
                         flag = 1;
                     }
                 } else if(*goto_operator == GOTO_OPERATOR_LT) {
                     if(goto_operand1 < goto_operand2) {
                         flag = 1;
                     }
                 } else if(*goto_operator == GOTO_OPERATOR_LE) {
                     if(goto_operand1 <= goto_operand2) {
                         flag = 1;
                     }
                 }

                 //printf("operand1 %"PRIu64" operand2 %"PRIu64" operator %d\n", goto_operand1, goto_operand2, *goto_operator);
                 printf("goto_next_step_id %d\n", *goto_next_step_id);
                 printf("behavior=%d, flag=%d\n", *goto_behavior, flag);

                 if(*goto_behavior == GOTO_BEHAVIOR_PAUSE) {
                     if(flag) {
                         ret = formation_pause(manager, logical_channel_id);
                         if(ret < 0) {
                             printf("do_formation_transforming\n");
                             return ret;
                         }

                         *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_PAUSING;
                         
                         *action = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;
    
                         return 0;
                     } else {
                         if(*step_pointer_reg != *step_quantity_reg) {
                             // jump to next step
                             *goto_next_step_id = *step_pointer_reg + 1;
                             ret = do_formation_goto(manager, logical_channel_id);
                             if(ret < 0) {
                                 printf("do_formation_goto, fail, ret %d\n", ret);
                                 return ret;
                             }

                             *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                             return 0;
                         } else {
                             // finish, now
                             *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;
                             return 0;
                         }
                     }
                 } else if(*goto_behavior == GOTO_BEHAVIOR_FINISH) {
                     if(flag) {

                         *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;

                         *action = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;

                         return 0;
                     } else {
                         if(*step_pointer_reg != *step_quantity_reg) {
                             // jump to next step
                             *goto_next_step_id = *step_pointer_reg + 1;
                             ret = do_formation_goto(manager, logical_channel_id);
                             if(ret < 0) {
                                 printf("do_formation_goto, fail, ret %d\n", ret);
                                 return ret;
                             }

                             *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                             return 0;
                         } else {
                             // finish, now
                             *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;
                             return 0;
                         }
                     }
                 } else if(*goto_behavior == GOTO_BEHAVIOR_JUMP) {
                     if(flag) {

                         *goto_next_step_id = *goto_next_step_index;

                         // jump to specific step id       
                         ret = do_formation_goto(manager, logical_channel_id);
                         if(ret < 0) {
                             printf("do_formation_goto fail, ret %d\n", ret);
                             return ret;
                         }

                         *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                         return 0;

                     } else {
                         if(*step_pointer_reg != *step_quantity_reg) {
                             // jump to next step
                             *goto_next_step_id = *step_pointer_reg + 1;
                             ret = do_formation_goto(manager, logical_channel_id);
                             if(ret < 0) {
                                 printf("do_formation_goto, fail, ret %d\n", ret);
                                 return ret;
                             }

                             *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;

                             return 0;
                         } else {
                             // finish, now
                             *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED;
                             return 0;
                         }
                     }
                 } else {
                     return ERROR_UNKNOWN_GOTO_BEHAVIOR;
                 }
             }

             return 0;
        break;

        default:
             return ERROR_UNKNOWN_STEP_TYPE;
        break;

    }

    return 0;
}

static int update_last_info(Manager *_manager, int channel_id)
{
    Manager *manager = _manager;
    int logical_channel_id = channel_id;
    Register *_register = NULL;
    uint64_t holding_time;
    _4paired_register_region *holding_time_region;
    _4paired_register_region *logic_current_region, *logic_voltage_region;
    _4paired_register_region *accumulated_logic_capacity_region, *logic_energy_region;
    _4paired_register_region *last_logic_energy_region, *last_logic_current_region;
    _4paired_register_region *last_logic_voltage_region, *last_accumulated_step_logic_capacity_region;
    _4paired_register_region *last_logic_holding_time_region, *last_logic_temperature_region;
    
    _4paired_register_region *accumulated_step_logic_capacity_region;
    _4paired_register_region *step_temp_accumulated_logic_capacity_region, *step_temp_accumulated_step_logic_capacity_region;

    _register = &manager->modbus_ctx._register;

    last_logic_energy_region = &_register->logical_channel[logical_channel_id].temp.last_logic_energy_region;
    last_logic_voltage_region = &_register->logical_channel[logical_channel_id].temp.last_logic_voltage_region;
    last_logic_current_region = &_register->logical_channel[logical_channel_id].temp.last_logic_current_region;
    last_accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_step_logic_capacity_region;
    last_logic_temperature_region = &_register->logical_channel[logical_channel_id].temp.last_logic_temperature_region;
    last_logic_holding_time_region = &_register->logical_channel[logical_channel_id].temp.last_logic_holding_time_region;

    step_temp_accumulated_logic_capacity_region = &_register->logical_channel[logical_channel_id].step_param.temp.accumulated_logic_capacity_region;
    step_temp_accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].step_param.temp.accumulated_step_logic_capacity_region;
   
    logic_energy_region = &_register->logical_channel[logical_channel_id].logic_energy_region;
    logic_voltage_region = &_register->logical_channel[logical_channel_id].logic_voltage_region;
    logic_current_region = &_register->logical_channel[logical_channel_id].logic_current_region;
    accumulated_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_logic_capacity_region;
    accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].accumulated_step_logic_capacity_region;
    
    copy_4paired_register_region(last_logic_voltage_region, logic_voltage_region);
    copy_4paired_register_region(last_logic_current_region, logic_current_region);
    copy_4paired_register_region(last_accumulated_step_logic_capacity_region, accumulated_step_logic_capacity_region);
    copy_4paired_register_region(last_logic_energy_region, logic_energy_region);

    copy_4paired_register_region(step_temp_accumulated_logic_capacity_region, accumulated_logic_capacity_region);
    copy_4paired_register_region(step_temp_accumulated_step_logic_capacity_region, accumulated_step_logic_capacity_region);
    
    holding_time_region = &_register->logical_channel[logical_channel_id].temp.holding_time_region;
    holding_time = combine_4paired_register_region(holding_time_region);
    update_4paired_register_region(last_logic_holding_time_region, holding_time);

    //
    
    return 0;
}

static int update_goto_parameters(Manager *_manager, int channel_id)
{
    Register *_register = NULL;
    Manager *manager = _manager;
    int logical_channel_id = channel_id;
    
    _4paired_register_region *last_logic_holding_time_region;
    _4paired_register_region *last_logic_temperature_region;    
    _4paired_register_region *last_logic_energy_region, *last_accumulated_step_logic_capacity_region;
    _4paired_register_region *last_logic_current_region, *last_logic_voltage_region;
    
    _4paired_register_region *goto_last_logic_holding_time_region;
    _4paired_register_region *goto_last_logic_energy_region;
    _4paired_register_region *goto_last_logic_current_region;
    _4paired_register_region *goto_last_logic_voltage_region;
    _4paired_register_region *goto_last_logic_capacity_region;
    _4paired_register_region *goto_last_logic_temperature_region;
    
    _register = &manager->modbus_ctx._register;
    
    last_logic_energy_region = &_register->logical_channel[logical_channel_id].temp.last_logic_energy_region;
    last_logic_current_region = &_register->logical_channel[logical_channel_id].temp.last_logic_current_region;
    last_logic_voltage_region = &_register->logical_channel[logical_channel_id].temp.last_logic_voltage_region;
    last_accumulated_step_logic_capacity_region = &_register->logical_channel[logical_channel_id].temp.last_accumulated_step_logic_capacity_region;
    last_logic_temperature_region = &_register->logical_channel[logical_channel_id].temp.last_logic_temperature_region;   
    last_logic_holding_time_region = &_register->logical_channel[logical_channel_id].temp.last_logic_holding_time_region;
    
    goto_last_logic_energy_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_energy_region;
    goto_last_logic_current_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_current_region;
    goto_last_logic_voltage_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_voltage_region;
    goto_last_logic_capacity_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_capacity_region;
    goto_last_logic_temperature_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_temperature_region;
    goto_last_logic_holding_time_region = &_register->logical_channel[logical_channel_id].step_param.info.goingto.last_logic_holding_time_region;    
    
    copy_4paired_register_region(goto_last_logic_energy_region, last_logic_energy_region);
    copy_4paired_register_region(goto_last_logic_current_region, last_logic_current_region);
    copy_4paired_register_region(goto_last_logic_voltage_region, last_logic_voltage_region);
    copy_4paired_register_region(goto_last_logic_capacity_region, last_accumulated_step_logic_capacity_region);
    copy_4paired_register_region(goto_last_logic_temperature_region, last_logic_temperature_region);
    copy_4paired_register_region(goto_last_logic_holding_time_region, last_logic_holding_time_region);
    
    return 0;
}

static int do_main_loop(Manager *_manager)
{
    Manager *manager = _manager;
    uint16_t exp_code;
    Register *_register = NULL;
    int ret, logical_channel_id;
    uint16_t *exp_code_reg, *action_reg, *mode_reg, *l_status_reg;
    uint16_t *step_pointer_reg, *step_quantity_reg;

    _register = &manager->modbus_ctx._register;

    // check all the present modes of all channels
    for(logical_channel_id=LOGICAL_CHANNEL_1_ID; logical_channel_id<NB_LOGIC_CHANNEL; logical_channel_id++) {

        mode_reg = _register->logical_channel[logical_channel_id].mode;
        action_reg = _register->logical_channel[logical_channel_id].action;
        l_status_reg = _register->logical_channel[logical_channel_id].status;
        exp_code_reg = _register->logical_channel[logical_channel_id].exception_code;
        step_pointer_reg = _register->logical_channel[logical_channel_id].step_pointer;

        if(*mode_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_CONFIGURATION) {

            if(*action_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RESET) {

                configuration_reset(manager, logical_channel_id);
      
                *mode_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_CONFIGURATION;

                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;
            }

            if(*action_reg != LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE) 
                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;

            if(*l_status_reg != LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_IDEL) 
                *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_IDEL;
            
            continue;
        }

        if(*mode_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_FORMATION) {

            //printf("mode: formation, action %d\n", *action_reg);

            if(*action_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RESET) {
                
                formation_reset(manager, logical_channel_id);

                *mode_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_CONFIGURATION;

                continue;
            }

            if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_FINISHED) {
                // do nothing
                //printf("finished, and wait for reset!!!\n");
                if(*action_reg != LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RESET \
                   && *action_reg != LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE) 
                    *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;

                continue;
            }

            if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING \
                && *action_reg != LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RESET \
                && *action_reg != LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_PAUSE ) {

                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;
            }

            if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_PAUSING \
                && *action_reg != LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RESET \
                && *action_reg != LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RESUME ) {

                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;
            }

            if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING && *action_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE) {
            
                ret = formation_run(manager, logical_channel_id);
                if(ret < 0) {
                    printf("do_main_loop->formation_run fail\n");
                    // set channel status to be in exception
                    exp_code = ret * -1;
                    *exp_code_reg = exp_code;
                    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_EXCEPTION;
                } else if(ret > 0){

                    update_last_info(manager, logical_channel_id);

                    ret = do_formation_transforming(manager, logical_channel_id, ret);
                    if(ret < 0) {
                        exp_code = ret * -1;
                        *exp_code_reg = exp_code;
                        *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_EXCEPTION;
                    }
                }

                continue;
            }

            // if the action is start, then we setup a formation task 
            if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_IDEL && *action_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_START) {
                ret = formation_start(manager, logical_channel_id);
                if(ret < 0) {
                    printf("do_main_loop->formation_start fail\n");
                    // set channel status to be in exception
                    exp_code = ret * -1;
                    *exp_code_reg = exp_code;
                    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_EXCEPTION;
                } else {
                    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;
                }
                
                *step_pointer_reg = 1;

                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;

                continue;
            }

            if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING && *action_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_PAUSE) {
                ret = formation_pause(manager, logical_channel_id);
                if(ret < 0) {
                    printf("do_main_loop->formation_pause fail\n");
                    exp_code = ret * -1;
                    *exp_code_reg = exp_code;
                    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_EXCEPTION;
                } else {
                    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_PAUSING;
                }

                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;

                continue;
            }

            if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_PAUSING && *action_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RESUME) {

                ret = formation_resume(manager, logical_channel_id);
                if(ret < 0) {
                    printf("do_main_loop->formation_resume fail\n");
                    exp_code = ret * -1;
                    *exp_code_reg = exp_code;
                    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_EXCEPTION;
                } else {
                    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;
                }

                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;

                continue;
            }

            continue;
        }

        if(*mode_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_DEBUG ) {

            if(*action_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RESET) {

                debug_reset(manager, logical_channel_id);
      
                *mode_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_CONFIGURATION;

                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;
            }

            ret = do_debug_logic(manager, logical_channel_id);
            if(ret < 0) {
                printf("do_main_loop->do_debug_logic fail\n");
                return -1;
            }
        }

        if(*mode_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_CALIBRATION) {

            if(*action_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RESET) {

                calibration_reset(manager, logical_channel_id);
      
                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;
                *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_IDEL;
                *mode_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_CONFIGURATION;
            }

            if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_IDEL \
                && *action_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_START) {

                ret = calibration_start(manager, logical_channel_id);
                if(ret < 0) {
                    printf("do_main_loop->calibration_start fail\n");
                    // set channel status to be in exception
                    exp_code = ret * -1;
                    *exp_code_reg = exp_code;
                    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_EXCEPTION;
                } else {
                    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING;
                }
                
                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;
                
                continue;

            }

            if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING \
                && *action_reg != LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RESET ) {
                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;
            }

            if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_RUNNING) {

                ret = calibration_run(manager, logical_channel_id);
                if(ret < 0) {
                    exp_code = ret * -1;
                    *exp_code_reg = exp_code;
                    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_EXCEPTION;
                }
            }
        }

        if(*mode_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_VERIFICATION) {
            ret = do_verification_logic(manager);
            if(ret < 0) {
                printf("do_main_loop->do_verification_logic fail\n");
                return -1;
            }
        }

        if(*mode_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_RECOVERY) {

            if(*action_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RESET) {

                recovery_logic_reset(manager, logical_channel_id);
      
                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;
                *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_IDEL;
                *mode_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_MODE_CONFIGURATION;
            }
#if 0
            if(*action_reg != LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RESET \
                && *action_reg != LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RECOVERY ) {

                *action_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_NONE;
            }
#endif
            if(*l_status_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_IDEL \
                && *action_reg == LOGICAL_CHANNEL_HOLDING_REGISTER_ACTION_RECOVERY) {

                ret = do_one_logical_channel_recovery(manager, logical_channel_id);
                if(ret < 0) {
                    printf("do_main_loop->do_one_logical_channel_recovery fail\n");
                    exp_code = ret * -1;
                    *exp_code_reg = exp_code;
                    *l_status_reg = LOGICAL_CHANNEL_HOLDING_REGISTER_STATUS_EXCEPTION;
                    return -1;
                }
            }
        }

    }

    return 0;
}

