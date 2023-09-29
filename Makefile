
CC := gcc
CFLAGS := -I./include
LDFLAGS := -lpthread

OBJS := platform_endian.o debug.o
OBJS += formation_logic.o calibration_logic.o verification_logic.o debug_logic.o configuration_logic.o
OBJS += middleware.o manager.o modbus_tcp.o tcp.o registers.o function_code_handler.o 
OBJS += gpio_controller.o  	spi_controller.o  	io_expander_controller.o	dac_controller.o	adc_controller.o	i2c_controller.o 	ssd1306_oled_controller.o
OBJS += hal_gpio.o  		hal_spi.o 			hal_io_expander.o			hal_dac.o			hal_adc.o			hal_i2c.o			hal_ssd1306_oled.o
OBJS += gpio_raspberry_pi.o spi_raspberry_pi.o	io_expander_mcp23s17.o	    dac_ti8568.o   		adc_ti1258.o		i2c_linux.o			ssd1306_oled_raspberry_pi.o
OBJS += logical_channel.o   physical_channel.o  step_timming_parameters.o	set_get_ip.o
OBJS += eeprom_controller.o hal_eeprom.o eeprom_24c0x.o 
OBJS += log_controller.o log_posix.o hal_log.o
OBJS += misc.o char_matrix.o
OBJS += recovery_logic.o

objs_path := ./objs

APP := middleware

$(APP):$(OBJS)
	$(CC) -g ./$(objs_path)/*.o $(CFLAGS) $(LDFLAGS) -o ./$(APP)

$(OBJS):%.o:./src/%.c
	$(CC) $(CFLAGS) -c -g $< -o ./$(objs_path)/$@

clean:
	rm ./$(objs_path)/*.o ./$(APP)

