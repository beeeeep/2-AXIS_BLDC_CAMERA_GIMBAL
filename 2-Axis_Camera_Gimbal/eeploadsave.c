#include "eeploadsave.h"

pidData_t PID_factors_X_eep EEMEM;
pidData_t PID_factors_Y_eep EEMEM;
Motor_Data_t Motor_Data_x_eep EEMEM;
Motor_Data_t Motor_Data_y_eep EEMEM;

void eeprom_save_pid_y(pidData_t PID_factors_Y)
{
	eeprom_write_block(&PID_factors_Y, &PID_factors_Y_eep, sizeof(PID_factors_Y));
}

void eeprom_save_pid_x(pidData_t PID_factors_X)
{
	eeprom_write_block(&PID_factors_X, &PID_factors_X_eep, sizeof(PID_factors_X));
}

void eeprom_save_motor_x(Motor_Data_t Motor_Data_x)
{
	eeprom_write_block(&Motor_Data_x, &Motor_Data_x_eep, sizeof(Motor_Data_x));
}

void eeprom_save_motor_y(Motor_Data_t Motor_Data_y)
{
	eeprom_write_block(&Motor_Data_y, &Motor_Data_y_eep, sizeof(Motor_Data_y));
}

pidData_t eeprom_load_pid_y(void)
{
	pidData_t PID_factors_Y;
	eeprom_read_block(&PID_factors_Y, &PID_factors_Y_eep, sizeof(PID_factors_Y));
	return PID_factors_Y;
}

pidData_t eeprom_load_pid_x(void)
{
	pidData_t PID_factors_X;
	eeprom_read_block(&PID_factors_X, &PID_factors_X_eep, sizeof(PID_factors_X));
	return PID_factors_X;
}

Motor_Data_t eeprom_load_motor_x(void)
{
	Motor_Data_t Motor_Data_x;
	eeprom_read_block(&Motor_Data_x, &Motor_Data_x_eep, sizeof(Motor_Data_x));

	return Motor_Data_x;
}

Motor_Data_t eeprom_load_motor_y(void)
{
	Motor_Data_t Motor_Data_y;
	eeprom_read_block(&Motor_Data_y, &Motor_Data_y_eep, sizeof(Motor_Data_y));

	return Motor_Data_y;
}
