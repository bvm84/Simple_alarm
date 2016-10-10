#include <eeprom_pass.h>
/*uint8_t get_pass_sym(uint8_t * adress)
{
  return eeprom_read_byte(adress);
}
void save_pass_sym(uint8_t * adress, uint8_t value_)
{
  eeprom_write_byte(ardess, value_)
}*/
uint8_t EEMEM KBD_pass[8];
uint8_t * eeprom_get_pass(uint8_t * KBD_pass_current)
{
  uint8_t i=0, symbol=0;
  while ((symbol!='#')&&(i<7))
  {
    eeprom_busy_wait();
    symbol=eeprom_read_byte(&KBD_pass[i]);
    i++;
    KBD_pass_current[i]=symbol;
  }
  return KBD_pass_current;
}
void eeprom_save_pass (uint8_t * KBD_pass_current)
{
  uint8_t i=0, symbol=0;
  while ((symbol!='#')&&(i<7))
  {
    eeprom_busy_wait();
    symbol=KBD_pass_current[i];
    eeprom_write_byte(&KBD_pass[i], symbol);
    i++;
    }
}
