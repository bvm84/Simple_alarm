#ifndef EEPROM_PASS_H
#define EEPROM_PASS_H
#include <avr/eeprom.h>

uint8_t * eeprom_get_pass(uint8_t *KBD_pass_current);
void eeprom_save_pass (uint8_t *KBD_pass_current);
#endif