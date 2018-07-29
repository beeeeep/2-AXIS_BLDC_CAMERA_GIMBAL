#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "txSerial.h"
#include "mpu9150.h"
#include "BDLC.h"
#include "eeploadsave.h"
#include "imu.h"
#include "PID.h"
#include "setup.h"
#include "ADC.h"

/* Variables */
pidData_t PID_Factors_X;
pidData_t PID_Factors_Y;
Motor_Data_t Motor_Data_x;
Motor_Data_t Motor_Data_y;
xyz compl_angles_0;
xyz compl_angles_1;
int16_t PID_x;
uint8_t PID_y;
uint8_t Sensor_0_flag;
uint8_t Sensor_1_flag;
extern uint16_t motorUpdate;
extern uint8_t extmotorPos;
joy_t Joystick;



void setup(void)
	{
		USART_Init();
		serialWrite("Starting...");
	    ADC_SETUP();
		mpu9150_setup(0,DLPF_5_Hz);
		mpu9150_gyro_setup(MR_500,0);
		mpu9150_acc_setup(MR_2_g,0);
		
	
		/* Load last set values from eeprom */
		PID_Factors_X=eeprom_load_pid_x();
		PID_Factors_Y=eeprom_load_pid_y();
		Motor_Data_x=eeprom_load_motor_x();
		Motor_Data_y=eeprom_load_motor_y();	
		Motor_Data_y.speed_curve=15;
		Motor_Data_y.power=200;
		BDLC_Setup(&Motor_Data_x,&Motor_Data_y);
		
		pid_Init_x(&PID_Factors_X);
		pid_Init_y(&PID_Factors_Y);
		
		DDRA&=(0<<PINA2);    //Set Button Pin     
		DDRA|=(1<<PINA7);
	}
	
	
	void wdt_init(void)
	{
		MCUSR = 0;
		wdt_disable();
		return;
	}



int main(void)
{
	wdt_init(); //Deactivate watchdog
	setup();
	
	while(1)
	{
		   
			compl_angles_0=mpu9150_read_acc_xyz_0();
			ADC_get_values(&Joystick);
			PID_x=pid_Controller_x(Joystick.x,compl_angles_0.x,&PID_Factors_X); //0.19ms
			PID_y=pid_Controller_y(Joystick.y,compl_angles_0.y,&PID_Factors_Y); //0.19ms
			BDLC_MotorX_Run(PID_x);
			BDLC_MotorY_Run(PID_y);
			
		   
			
			
		
		 

		
		/*Call menu routine (if Button is pressed) */
		if((PINA & (1<<PINA2)))
		{
		
			
			_delay_ms(100);       //Debounce
			PID_y++;
			BDLC_MotorX_Run(0);   //Freeze motors
			BDLC_MotorY_Run(0);
			
			menu();               //Call settings menu
			setup();
			serialWrite("Resuming...");
			serialWrite_newline();
		}
		
	
	
		
	}
}






