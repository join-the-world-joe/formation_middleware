
#ifndef __HAL_EEPROM_H__
#define __HAL_EEPROM_H__

int hal_eeprom_init(register_eeprom_parameters *eeprom_param);
int hal_eeprom_deinit(unregister_eeprom_parameters *eeprom_param);

#endif // __HAL_EEPROM_H__

