#ifndef EEPLOADSAVE_H_
#define EEPLOADSAVE_H_

#include <avr/eeprom.h>
#include "PID.h"
#include "BDLC.h"

void eeprom_save_pid_y(pidData_t PID_factors_Y);

void eeprom_save_pid_x(pidData_t PID_factors_X);

void eeprom_save_motor_x(Motor_Data_t Motor_Data_x);

void eeprom_save_motor_y(Motor_Data_t Motor_Data_y);

pidData_t eeprom_load_pid_y(void);

pidData_t eeprom_load_pid_x(void);

Motor_Data_t eeprom_load_motor_x(void);

Motor_Data_t eeprom_load_motor_y(void);

#endif /* EEPLOADSAVE_H_ */