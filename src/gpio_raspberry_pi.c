
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "log_controller.h"

#include "gpio_raspberry_pi.h"

int raspberry_pi_gpio_init(register_gpio_port_parameters *gpio_port_param)
{
    register_gpio_port_parameters *_gpio_port_param = gpio_port_param;
    char temp[255], direction_string[] = "in\0out", *_signature = NULL;
    uint8_t _direction = -1, pos = 0, length = 0;
    int ret = 0, gpio_number = -1, byte_count = -1, retries = 5;
    int export_fd = -1, direction_fd = -1, gpio_fd = -1;

    _direction = _gpio_port_param->in.direction;
    _signature = _gpio_port_param->in.signature;

    if(strcmp(_signature, GPIO_SIGNATURE_AD_DRDY) == 0) {
        gpio_number = GPIO_PIN_NB_AD_DRDY;
#if 0
    } else if(strcmp(_signature, GPIO_SIGNATURE_AD_START) == 0) {
        gpio_number = GPIO_PIN_NB_AD_START;
#endif
    } else if(strcmp(_signature, GPIO_SIGNATURE_DA_LDAC)==0) {
        gpio_number = GPIO_PIN_NB_DA_LDAC;
    } else if(strcmp(_signature, GPIO_SIGNATURE_RESET)==0) {
        gpio_number = GPIO_PIN_NB_RESET;
    } else if(strcmp(_signature, GPIO_SIGNATURE_BELL)==0) {
        gpio_number = GPIO_PIN_NB_BELL;
    } else if(strcmp(_signature, GPIO_SIGNATURE_LED_READY)==0) {
        gpio_number = GPIO_PIN_NB_LED_READY;
    } else if(strcmp(_signature, GPIO_SIGNATURE_LED_RUN)==0) {
        gpio_number = GPIO_PIN_NB_LED_RUN;
    } else if(strcmp(_signature, GPIO_SIGNATURE_LED_ERROR)==0) {
        gpio_number = GPIO_PIN_NB_LED_ERROR;
    } else if(strcmp(_signature, GPIO_SIGNATURE_AC_LOSS)==0) {
        gpio_number = GPIO_PIN_NB_AC_LOSS;
    } else {
        printf("unknown signature %s\n", _signature);
        return -1;
    }

    // export gpio to sysfs
    export_fd = open("/sys/class/gpio/export", O_WRONLY);
	if (export_fd < 0) {
		printf("open /sys/class/gpio/export fail\n");
		return -1;
	}

    memset(temp, '\0', 255);

    byte_count = snprintf(temp, 3, "%d", gpio_number);

    write(export_fd, temp, byte_count);

	close(export_fd);

    //

    // set gpio direction
    memset(temp, '\0', 255);

    snprintf(temp, 255, "/sys/class/gpio/gpio%d/direction", gpio_number);

    retries = 5;
    while(retries) {
        direction_fd = open(temp, O_RDWR);
	    if (direction_fd < 0) {
            retries--;
            usleep(EXPORT_TO_DIRECTION_DELAY);
	    } else {
            // we've open file successfully.
            printf("open %s done!!\n", temp);
            break;
        }        
    }

    if(retries == 0) {
        printf("open %s fail\n", temp);
        return -1;
    }

    pos = _direction == GPIO_DIRECTION_IN?0:3;
    length = _direction == GPIO_DIRECTION_IN?2:3;

    ret = write(direction_fd, &direction_string[pos], length);
    if(ret<0) {
        printf("gpio_init set gpio direction fail\n");
        close(direction_fd);
        return -1;
    }
    close(direction_fd);

    // open and record gpio file descriptor
    snprintf(temp, 255, "/sys/class/gpio/gpio%d/value", gpio_number);

    gpio_fd = open(temp, O_RDWR);
    if(gpio_fd<0) {
        printf("open %s fail\n", temp);
        return -1;
    }

    _gpio_port_param->out.dev_fd = gpio_fd;

    return 0;
}

int raspberry_pi_gpio_deinit(unregister_gpio_port_parameters *gpio_port_param)
{
    int ret = 0;
    unregister_gpio_port_parameters *_gpio_port_param = gpio_port_param;
    int _dev_fd = -1;

    _dev_fd = _gpio_port_param->temp.dev_fd;

    close(_dev_fd);

    return 0;
}

int gpio_set_level(gpio_ioctl_parameters *gpio_ioctl_param)
{
    int ret = 0, _dev_fd;
    gpio_ioctl_parameters *_ioctl_param = gpio_ioctl_param;
    char level_string[] = "01";
    uint8_t _level;
    int _gpio_fd = -1;
    
    _level = _ioctl_param->set_level_param.in.level ==GPIO_LEVEL_HIGH?1:0;
    _dev_fd = _ioctl_param->set_level_param.in.dev_fd;
    
    ret = write(_dev_fd, &level_string[_level], 1);
    if(ret<0) {
        printf("gpio_set_level_>write fail, ret %d, fd = %d\n", ret, _dev_fd);
        return -1;
    }

    return 0;
}

int gpio_get_level(gpio_ioctl_parameters *gpio_ioctl_param)
{
    int ret = 0, _dev_fd;
    gpio_ioctl_parameters *_ioctl_param = gpio_ioctl_param;
    char level_string[3] = {};
    int *_level;
    int _gpio_fd = -1;
    
    _level = _ioctl_param->get_level_param.out.level;
    _dev_fd = _ioctl_param->get_level_param.temp.dev_fd;
    
    //printf("_dev_fd: %d\n", _dev_fd);

    lseek(_dev_fd, 0, SEEK_SET);

    ret = read(_dev_fd, level_string, 3);
    if(ret<0) {
        printf("gpio_set_level_>write fail, ret %d, fd = %d\n", ret, _dev_fd);
        return -1;
    }

    //printf("level_string: %s, length: %d\n", level_string, strlen(level_string));

    if( level_string[0] == '1' ) {
        *_level = GPIO_LEVEL_HIGH;
    } else if( level_string[0] == '0' ){
        *_level = GPIO_LEVEL_LOW;
    } else {
        printf("gpio_get_level, unknow level_string\n");
        return -1;
    }

    //printf("gpio_get_level, level %s\n", level_string);

    return 0;
}

#if 0
int gpio_get_level(gpio_ioctl_parameter *ioctl_param)
{
    int ret = 0;
    gpio_ioctl_parameter *_ioctl_param = ioctl_param;
    char level_string[3] = {0};
    uint8_t level = GPIO_LEVEL_UNKNOWN;
    int _gpio_fd = -1;

    if(!_ioctl_param) {
        printf("gpio_set_level fail, incorrect parameters\n");
        return -1;
    }

    _gpio_fd = _ioctl_param->init_param.out_param_set.fd;

    if(_gpio_fd > 0) {
        ret = read(_gpio_fd, level_string, 3);
        if(ret < 0) {
            printf("gpio_get_level -> read fail\n");
            return -1;
        }
    } else {
        printf("gpio_get_level incorrect parameters\n");
        return -1;
    }

    printf("level_string = %s\n", level_string);

    level = atoi(level_string);

    printf("level = %d\n", level);

    _ioctl_param->get_param.out_param_set.level = level;

    return 0;
}
#endif
