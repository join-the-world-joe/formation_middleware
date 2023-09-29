/**
* @file     middleware.c
* @brief    This file is the entry of formation state machine
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     20/09/2018
*/

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/select.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "channel.h"
#include "log_controller.h"
#include "registers.h"
#include "tcp.h"
#include "modbus_tcp.h"
#include "spi_controller.h"
#include "i2c_controller.h"
#include "gpio_controller.h"
#include "dac_controller.h"
#include "io_expander_controller.h"
#include "adc_controller.h"
#include "ssd1306_oled_controller.h"
#include "manager.h"

void game_over() {
    printf("game over!!!!\n");
    exit(-1);
}

#if 1
int main(int argc, char *argv[])
{
    int ret = 0;
    static Manager manager;

    ret = manager_init(&manager);
    if(ret < 0) {
        printf("manager_init fail\n");
        return -1;
    }

    ret = manager_run(&manager);
    if(ret < 0) {
        printf("manager_run fail\n");
        return -1;
    }

    manager_deinit(&manager);

    return ret;
}
#endif

#if 0
int main(int argc, char *argv[])
{
    int ret = 0;

    modbus_tcp_context modbus_ctx;
    logging_ctx log_ctx;
    pthread_mutex_t log_mutex;

    signal(SIGINT, game_over);

    ret = logging_init(&log_ctx, "test.log", &log_mutex);
    if(ret < 0) {
        printf("logging_init fail\n");
        return ret;
    }

    ret = modbus_tcp_init(&modbus_ctx, &log_ctx);
    if(ret < 0) {
        printf("modbus_tcp_init fail\n");
        return -1;
    }

    ret = modbus_tcp_run(&modbus_ctx);
    if(ret < 0) {
        printf("modbus_tcp_run fail\n");
        modbus_tcp_deinit(&modbus_ctx);
        return -1;
    }

    return ret;
}
#endif

