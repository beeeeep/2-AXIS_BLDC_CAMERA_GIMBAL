#ifndef BDLC_H_
#define BDLC_H_

#include <avr/io.h>
#include <avr/interrupt.h>

typedef struct MOTOR_DATA
{
	uint8_t speed_curve;
	uint8_t power;

} Motor_Data_t;

void BDLC_Setup(Motor_Data_t *Motor_Data_x, Motor_Data_t *Motor_Data_y);

void BDLC_MotorX_Run(int16_t speed);

void BDLC_MotorY_Run(int16_t speed);

void BDLC_as_Servo_Run_X(double base_sensor, double camera_sensor, int16_t desired_pos, uint8_t speed);

void BDLC_MotorX_Run_inst(uint16_t motorPos);
void BDLC_MotorY_Run_inst(uint16_t motorPos);

void MoveMotorPosSpeed(uint8_t motorNumber, int MotorPos, uint16_t maxPWM);

void BDLC_MotorY_Run_once(uint8_t speed, uint8_t steps);
#endif /* BDLC_H_ */