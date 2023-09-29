
#ifndef __EEPROM_24C0X_H__
#define __EEPROM_24C0X_H__

#define EEPROM_SIGNATURE_24C0X_U6 "U6"

#define EEPROM_SIGNATURE_24C0X_U6_I2C_SLAVE_ADDR 0x50

int eeprom_24c0x_init(register_eeprom_parameters *eeprom_param);
int eeprom_24c0x_deinit(unregister_eeprom_parameters *);


#endif // __EEPROM_24C0X_H__