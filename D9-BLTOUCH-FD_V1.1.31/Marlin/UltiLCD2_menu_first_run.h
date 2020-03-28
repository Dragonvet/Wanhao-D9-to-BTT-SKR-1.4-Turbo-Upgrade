#ifndef ULTI_LCD2_MENU_FIRST_RUN_H
#define ULTI_LCD2_MENU_FIRST_RUN_H
#define EEPROM_FIRST_RUN_DONE_OFFSET 0x400
#define IS_FIRST_RUN_DONE() ((eeprom_read_byte((const uint8_t*)EEPROM_FIRST_RUN_DONE_OFFSET) == 'U') && home_offset[2] != 0.0)
#define SET_FIRST_RUN_DONE() do { eeprom_write_byte((uint8_t*)EEPROM_FIRST_RUN_DONE_OFFSET, 'U'); } while(0)
void lcd_menu_bed_leveling_start();
void lcd_menu_zprobe_offset_calibrate_start(); 
#endif
