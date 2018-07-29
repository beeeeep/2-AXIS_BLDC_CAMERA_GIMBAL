#include "setup.h"

pidData_t PID_factors_X;
pidData_t PID_factors_Y;
Motor_Data_t Motor_Data_x;
Motor_Data_t Motor_Data_y;

void menu()
{
	uint8_t loop = 1;

	while (loop)
	{
		serialWrite_newline();
		serialWrite_newline();
		serialWrite("Please choose one of the following options:");
		serialWrite_newline();
		serialWrite("1.X-axis PID tuning");
		serialWrite_newline();
		serialWrite("2.Y-axis PID tuning");
		serialWrite_newline();
		serialWrite("3.X-Motor Tuning");
		serialWrite_newline();
		serialWrite("4.Y-Motor Tuning");
		serialWrite_newline();
		serialWrite("5.Exit");
		serialWrite_newline();
		serialWrite_newline();
		serialWrite("Select option: ");
		uint8_t val = serialRead_int();
		serialWrite_newline();

		switch (val)
		{
		case 1:
			serialWrite("Enter P: ");
			PID_factors_X.P_Factor = serialRead_int();
			serialWrite_newline();
			serialWrite("Enter I: ");
			PID_factors_X.I_Factor = serialRead_int();
			serialWrite_newline();
			serialWrite("Enter D: ");
			PID_factors_X.D_Factor = serialRead_int();
			serialWrite_newline();

			eeprom_save_pid_x(PID_factors_X);

			break;

		case 2:
			serialWrite("Enter P: ");
			PID_factors_Y.P_Factor = serialRead_int();
			serialWrite_newline();
			serialWrite("Enter I: ");
			PID_factors_Y.I_Factor = serialRead_int();
			serialWrite_newline();
			serialWrite("Enter D: ");
			PID_factors_Y.D_Factor = serialRead_int();
			serialWrite_newline();
			eeprom_save_pid_y(PID_factors_Y);
			break;

		case 3:
			serialWrite("Enter speed curve value for X axis motor (1-20): ");
			val = serialRead_int();
			Motor_Data_x.speed_curve = val;
			serialWrite_newline();
			serialWrite("Enter motor power for X axis motor (0-255): ");
			val = serialRead_int();
			Motor_Data_x.power = val;
			serialWrite_newline();
			eeprom_save_motor_x(Motor_Data_x);
			break;

		case 4:
			serialWrite("Enter speed curve value for Y axis motor (1-20): ");
			val = serialRead_int();
			Motor_Data_y.speed_curve = val;
			serialWrite_newline();
			serialWrite("Enter motor power for Y axis motor (0-255): ");
			val = serialRead_int();
			Motor_Data_y.power = val;
			serialWrite_newline();
			eeprom_save_motor_y(Motor_Data_y);
			break;

		case 5:
			loop = 0;
			break;

		default:
			break;
		}
	}
	serialWrite_newline();
};