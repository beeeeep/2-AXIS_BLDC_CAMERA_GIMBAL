#include "BDLC.h"
#include "math.h"

uint8_t sineArraySize;

/*Main mode table */

uint8_t sinewave_data_x[255];
uint8_t sinewave_data_y[255];
// const uint8_t sinewave_data[] =
// {
// 	127,130,133,136,140,143,146,149,152,155,158,161,164,167,170,173,176,179,182,185,187,190,193,195,
// 	198,201,203,206,208,211,213,215,217,220,222,224,226,228,230,232,233,235,237,238,240,241,242,244,
// 	245,246,247,248,249,250,251,252,252,253,253,254,254,254,254,254,254,254,254,254,254,253,253,252,
// 	252,251,250,250,249,248,247,246,244,243,242,240,239,237,236,234,232,231,229,227,225,223,221,219,
// 	216,214,212,209,207,204,202,199,197,194,191,189,186,183,180,177,175,172,169,166,163,160,157,154,
// 	150,147,144,141,138,135,132,129,125,122,119,116,113,110,107,104,100,97,94,91,88,85,82,79,77,74,
// 	71,68,65,63,60,57,55,52,50,47,45,42,40,38,35,33,31,29,27,25,23,22,20,18,17,15,14,12,11,10,8,7,6,
// 	5,4,4,3,2,2,1,1,0,0,0,0,0,0,0,0,0,0,1,1,2,2,3,4,5,6,7,8,9,10,12,13,14,16,17,19,21,22,24,26,28,30,
// 	32,34,37,39,41,43,46,48,51,53,56,59,61,64,67,69,72,75,78,81,84,87,90,93,96,99,102,105,108,111,114,
// 118,121,124,127};

/* Servo mode table//
const uint8_t sinewave_data[]=
{
	254, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 249, 249, 248, 247, 245, 244, 243, 242, 240, 239, 238, 236, 234, 233, 231, 229, 227, 225, 223, 221, 219, 217, 215, 212, 210, 208, 205, 203, 200, 198, 195, 192, 190, 187, 184, 181, 178, 176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 139, 136, 133, 130, 127, 124, 121, 118, 115, 111, 108, 105, 102, 99, 96, 93, 90, 87, 84, 81, 78, 76, 73, 70, 67, 64, 62, 59, 56, 54, 51, 49, 46, 44, 42, 39, 37, 35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 5, 4, 3, 2, 2,
	1, 1, 1, 0, 0, 0,0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 9, 10, 11, 12, 14, 15, 16, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 42, 44, 46, 49, 51, 54, 56, 59, 62, 64, 67, 70, 73, 76, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 115, 118, 121, 124, 127, 130, 133, 136, 139, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173, 176, 178, 181, 184, 187, 190, 192, 195, 198, 200, 203, 205, 208, 210, 212, 215, 217, 219, 221, 223, 225, 227, 229, 231, 233, 234, 236, 238, 239, 240, 242, 243, 244, 245, 247, 248, 249, 249, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255,
	255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 249, 249, 248, 247, 245, 244, 243, 242, 240, 239, 238, 236, 234, 233, 231, 229, 227, 225, 223, 221, 219, 217, 215, 212, 210, 208, 205, 203, 200, 198, 195, 192, 190, 187, 184, 181, 178, 176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 139, 136, 133, 130, 127, 124, 121, 118, 115, 111, 108, 105, 102, 99, 96, 93, 90, 87, 84, 81, 78, 76, 73, 70, 67, 64, 62, 59, 56, 54, 51, 49, 46, 44, 42, 39, 37, 35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0,
	0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 9, 10, 11, 12, 14, 15, 16, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 42, 44, 46, 49, 51, 54, 56, 59, 62, 64, 67, 70, 73, 76, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 115, 118, 121, 124, 127, 130, 133, 136, 139, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173, 176, 178, 181, 184, 187, 190, 192, 195, 198, 200, 203, 205, 208, 210, 212, 215, 217, 219, 221, 223, 225, 227, 229, 231, 233, 234, 236, 238, 239, 240, 242, 243, 244, 245, 247, 248, 249, 249, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 254,
	254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 249, 249, 248, 247, 245, 244, 243, 242, 240, 239, 238, 236, 234, 233, 231, 229, 227, 225, 223, 221, 219, 217, 215, 212, 210, 208, 205, 203, 200, 198, 195, 192, 190, 187, 184, 181, 178, 176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 139, 136, 133, 130, 127, 124, 121, 118, 115, 111, 108, 105, 102, 99, 96, 93, 90, 87, 84, 81, 78, 76, 73, 70, 67, 64, 62, 59, 56, 54, 51, 49, 46, 44, 42, 39, 37, 35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0,
	0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 9, 10, 11, 12, 14, 15, 16, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 42, 44, 46, 49, 51, 54, 56, 59, 62, 64, 67, 70, 73, 76, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 115, 118, 121, 124, 127, 130, 133, 136, 139, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173, 176, 178, 181, 184, 187, 190, 192, 195, 198, 200, 203, 205, 208, 210, 212, 215, 217, 219, 221, 223, 225, 227, 229, 231, 233, 234, 236, 238, 239, 240, 242, 243, 244, 245, 247, 248, 249, 249, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255,
	254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 249, 249, 248, 247, 245, 244, 243, 242, 240, 239, 238, 236, 234, 233, 231, 229, 227, 225, 223, 221, 219, 217, 215, 212, 210, 208, 205, 203, 200, 198, 195, 192, 190, 187, 184, 181, 178, 176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 139, 136, 133, 130, 127, 124, 121, 118, 115, 111, 108, 105, 102, 99, 96, 93, 90, 87, 84, 81, 78, 76, 73, 70, 67, 64, 62, 59, 56, 54, 51, 49, 46, 44, 42, 39, 37, 35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0,
};
*/

//Brugi Mode table//
//int8_t sinewave_data[] =
//{6, 9, 12, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46, 49, 51, 54, 57, 60, 63, 65, 68, 71, 73, 76, 78, 81, 83, 85, 88, 90, 92,
//94,96,98,100,102,104,106,107,109,111,112,113,115,116,117,118,120,121,122,122,123,124,125,125,126,126,126,127,127,127,127,127,
//127,127,126,126,126,125,125,124,123,122,122,121,120,118,117,116,115,113,112,111,109,107,106,104,102,100,98,96,94,92,90,88,85,
//83,81,78,76,73,71,68,65,63,60,57,54,51,49,46,43,40, 37, 34, 31, 28, 25, 22, 19, 16, 12, 9, 6,3,0,-3,-6, -9,-12,-16,
//-19, -22, -25, -28, -31, -34, -37, -40, -43, -46, -49, -51, -54, -57, -60, -63, -65, -68,-71, -73, -76, -78, -81, -83, -85, -88, -90,-92,-94,-96,-98,-100,-102,-104,-106,-107,-109,-111,-112,-113,-115,-116,-117,-118,-120,-121,-122,-122,-123,-124,-125,-125,-126,-126,-126,
//-127,-127,-127,-127,-127,-127,-127,-126,-126,-126,-125,-125,-124,-123,-122,-122,-121,-120,-118,-117,-116,-115,-113,-112,-111,-109,-107,
//-106,-104,-102,-100,-98,-96,-94,-92,-90, -88, -85, -83, -81, -78, -76, -73, -71, -68, -65, -63, -60, -57, -54,-51, -49, -46, -43, -40,
//-37, -34, -31, -28, -25, -22, -19, -16, -12, -9, -6, -3,
//};

const float min_speed_table[] = {0, 5010, 9500, 9000, 8500, 8000, 7500, 7000, 6500, 6000,
								 5500, 5000, 4500, 4000, 3000, 2500, 2000, 1500, 501, 500};

float speed_data_x[255];
float speed_data_y[255];
uint8_t phaseshift;
uint8_t motorx_phase1;
uint8_t motorx_phase2;
uint8_t motorx_phase3;
uint8_t motory_phase1;
uint8_t motory_phase2;
uint8_t motory_phase3;
int8_t increment_x;
int8_t increment_y;
uint16_t speed_delay_x;
uint16_t speed_delay_y;
volatile unsigned long microseconds = 0;
volatile unsigned long oldtime_x;
volatile unsigned long oldtime_y;
volatile uint8_t int_counter = 0;
uint16_t pwm_a_motor0;
uint16_t pwm_b_motor0;
uint16_t pwm_c_motor0;
uint16_t pwm_a_motor1;
uint16_t pwm_b_motor1;
uint16_t pwm_c_motor1;
uint16_t motorUpdate;
uint8_t extmotorPos;
static int sensor_last;


void BDLC_Setup(Motor_Data_t *Motor_Data_x, Motor_Data_t *Motor_Data_y)
{
	/*Calculate Speed curve
	
	
      max_speed_table:
	 
	        (delay us) 
	             |
	   Motor_data|*
			     |  *
			     |    *
			     |      *
		      500|        *
			     |___________      
			     0       255   (table pointer)      
	*/

	uint16_t i = 0;
	do
	{

		speed_data_x[i] = ((600 - min_speed_table[Motor_Data_x->speed_curve]) / 1024) * i + min_speed_table[Motor_Data_x->speed_curve];
		speed_data_x[0] = 0;
		speed_data_y[i] = ((600 - min_speed_table[Motor_Data_y->speed_curve]) / 1024) * i + min_speed_table[Motor_Data_y->speed_curve];
		speed_data_y[0] = 0;

	} while (i++ != 255);

	uint8_t n = 0;
	float pulse;

	do
	{

		pulse = Motor_Data_x->power * sin((1.411764706 * n) * 3.1415926 / 180) / 2 + (Motor_Data_x->power / 2);

		sinewave_data_x[n] = (uint8_t)round(pulse);

	} while (n++ != 255);

	n = 0;
	do
	{

		pulse = Motor_Data_y->power * sin((1.411764706 * n) * 3.1415926 / 180) / 2 + (Motor_Data_y->power / 2);

		sinewave_data_y[n] = (uint8_t)round(pulse);

	} while (n++ != 255);

	/* Set timer0/1/2 for phase-correct PWM */
	DDRB = (1 << PINB3) | (1 << PINB4);
	DDRD = (1 << PIND4) | (1 << PIND5) | (1 << PIND6) | (1 << PIND7);

	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00);
	TCCR0B = (1 << CS00);

	TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20);
	TCCR2B = (1 << CS20);

	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << CS10);

	TIMSK1 |= (1 << TOIE1);

	sei();

	/* Each SinePWM has 120deg diff with each other (per motor) */

	motorx_phase1 = 0;
	motorx_phase2 = 85;
	motorx_phase3 = 170;

	motory_phase1 = 0;
	motory_phase2 = 85;
	motory_phase3 = 170;

	/*Start motors(in order for the first interrupt to occur) */
	OCR0A = sinewave_data_x[motorx_phase1];
	OCR2A = sinewave_data_x[motorx_phase2];
	OCR1A = sinewave_data_x[motorx_phase3];

	OCR0B = sinewave_data_y[motory_phase1];
	OCR2B = sinewave_data_y[motory_phase2];
	OCR1B = sinewave_data_y[motory_phase3];
}

void BDLC_as_Servo_Run_X(double base_sensor, double camera_sensor, int16_t desired_pos, uint8_t speed)
{
	uint8_t current_pos;
	int sensor = (int)camera_sensor;

	if (sensor_last != sensor)
	{
		sensor_last = sensor;
		if (sensor > 0 && sensor < 19)
		{
			current_pos = (4.57 * camera_sensor) + 168;
		}
		if (sensor > 19 && sensor < 74)
		{
			current_pos = (camera_sensor - 19) * 4.61;
		}
		if (sensor < 0 && sensor > -38)
		{
			current_pos = 4.57 * (camera_sensor + 38);
		}

		if (sensor < -38 && sensor > -87)
		{
			current_pos = (camera_sensor + 87) * 4.73;
		}
	}
	if (camera_sensor > desired_pos)
	{
		//
	}
	if (camera_sensor < desired_pos)
	{
		//	current_pos--;
	}
	//current_pos++;
	serialWrite_int(current_pos);
	serialWrite_newline();
	motorx_phase1 = current_pos;
	motorx_phase2 = motorx_phase1 + 85;
	motorx_phase3 = motorx_phase1 + 170;
}
void BDLC_MotorX_Run_inst(uint16_t motorPos)
{
	motorPos = motorPos & 0xFF;

	motorx_phase1 = sinewave_data_x[motorPos];
	motorx_phase2 = sinewave_data_x[motorPos + 85];
	motorx_phase3 = sinewave_data_x[motorPos + 170];
}

void BDLC_MotorY_Run_inst(uint16_t motorPos)
{
	motorPos = motorPos & 0xFF;

	motory_phase1 = sinewave_data_y[motorPos];
	motory_phase2 = sinewave_data_y[motorPos + 85];
	motory_phase3 = sinewave_data_y[motorPos + 170];
}

void BDLC_MotorX_Run(int16_t speed)

{
	/*Check if direction is clockwise or counter clockwise */
	if (speed > 0)
	{

		increment_x = 1;
		if (speed > 255)
			speed = 255;
	}
	if (speed < 0)
	{
		if (speed < -255)
			speed = -255;
		speed *= -1;
		increment_x = -1;
	}
	if (speed == 0)
	{
		increment_x = 0;
	}

	speed_delay_x = speed_data_x[speed];
}

void BDLC_MotorY_Run(int16_t speed)
{

	/*Check if direction is clockwise or counter clockwise */
	if (speed > 0)
	{

		increment_y = 1;
		if (speed > 255)
			speed = 255;
	}
	if (speed < 0)
	{
		if (speed < -255)
			speed = -255;
		speed *= -1;
		increment_y = -1;
	}
	if (speed == 0)
	{
		increment_y = 0;
	}
	speed_delay_y = speed_data_y[speed];
}

void BDLC_MotorY_Run_once(uint8_t speed, uint8_t steps)
{
	cli();
	increment_y = 1;
	motory_phase1 += increment_y;
	motory_phase2 += increment_y;
	motory_phase3 += increment_y;

	OCR0B = sinewave_data_y[motory_phase1];
	OCR2B = sinewave_data_y[motory_phase2];
	OCR1B = sinewave_data_y[motory_phase3];
	sei();
}

//ISR(TIMER1_OVF_vect)
//{
//int_counter++;
//if (int_counter>3)
//{
//microseconds+=128;
//int_counter=0;
//}
//
//if(microseconds-oldtime_x>speed_delay_x)
//{
//
//
//
//
//OCR0A=sinewave_data_x[motorx_phase1];
//OCR2A=sinewave_data_x[motorx_phase2];
//OCR1A=sinewave_data_x[motorx_phase3];
//motorx_phase1+=increment_x;
//motorx_phase2+=increment_x;
//motorx_phase3+=increment_x;
//motorUpdate=sinewave_data_x[motorx_phase1];
//
//++microseconds;
//oldtime_x=microseconds;
//
//}
//if(microseconds-oldtime_y>speed_delay_y)
//{
//
//
//
//motorUpdate=1;
//
//OCR0B=sinewave_data_y[motory_phase1];
//OCR2B=sinewave_data_y[motory_phase2];
//OCR1B=sinewave_data_y[motory_phase3];
//motory_phase1+=increment_y;
//motory_phase2+=increment_y;
//motory_phase3+=increment_y;
//
//
//++microseconds;
//oldtime_y=microseconds;
//
//}
//
//}

ISR(TIMER1_OVF_vect)
{
	int_counter++;
	if (int_counter > 64) //480hz
	{
		OCR0A = motorx_phase1;
		OCR2A = motorx_phase2;
		OCR1A = motorx_phase3;

		OCR0B = motory_phase1;
		OCR2B = motory_phase2;
		OCR1B = motory_phase3;
		extmotorPos++;
		motory_phase1 = sinewave_data_y[extmotorPos];
		motory_phase2 = sinewave_data_y[extmotorPos + 85];
		motory_phase3 = sinewave_data_y[extmotorPos + 170];

		motorUpdate = 1;
		int_counter = 0;
	}
}