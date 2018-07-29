/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief General PID implementation for AVR.
 *
 * Discrete PID controller implementation. Set up by giving P/I/D terms
 * to Init_PID(), and uses a struct PID_DATA to store internal values.
 *
 * - File:               pid.c
 * - Compiler:           IAR EWAAVR 4.11A
 * - Supported devices:  All AVR devices can be used.
 * - AppNote:            AVR221 - Discrete PID controller
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support email: avr@atmel.com
 *
 * $Name$
 * $Revision: 456 $
 * $RCSfile$
 * $Date: 2006-02-16 12:46:13 +0100 (to, 16 feb 2006) $
 *****************************************************************************/

#include "pid.h"
#include "stdint.h"

/*! \brief Initialisation of PID controller parameters.
 *
 *  Initialise the variables used by the PID algorithm.
 *
 *  \param p_factor  Proportional term.
 *  \param i_factor  Integral term.
 *  \param d_factor  Derivate term.
 *  \param pid  Struct with PID status.
 */
void pid_Init_x(struct PID_DATA *pid)
// Set up PID controller parameters
{
	// Start values for PID controller
	pid->sumError = 0;
	pid->lastProcessValue = 0;
	// Tuning constants for PID loop
	//pid->P_Factor = p_factor;
	//pid->I_Factor = i_factor;
	//pid->D_Factor = d_factor;
	// Limits to avoid overflow
	pid->maxError = MAX_INT / (pid->P_Factor + 1);
	pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);
}

void pid_Init_y(struct PID_DATA *pid)
// Set up PID controller parameters
{
	// Start values for PID controller
	pid->sumError = 0;
	pid->lastProcessValue = 0;
	// Tuning constants for PID loop
	//pid->P_Factor = p_factor;
	//pid->I_Factor = i_factor;
	//pid->D_Factor = d_factor;
	// Limits to avoid overflow
	pid->maxError = MAX_INT / (pid->P_Factor + 1);
	pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);

	pid_output_filter_init(); // initialize the output filter of the pid y
}

/*! \brief PID control algorithm.
 *
 *  Calculates output from setpoint, process value and PID status.
 *
 *  \param setPoint  Desired value.
 *  \param processValue  Measured value.
 *  \param pid_st  PID status struct.
 */
uint16_t pid_Controller_y(int16_t setPoint, double processValue, struct PID_DATA *pid_st)
{
	int16_t error, p_term, d_term;
	int32_t i_term, ret, temp;
	processValue *= 10;
	error = setPoint - processValue;

	//if (error==0)
	//{
	//pid_st->sumError=0;
	//}

	// Calculate Pterm and limit error overflow
	if (error > pid_st->maxError)
	{
		p_term = MAX_INT;
	}
	else if (error < -pid_st->maxError)
	{
		p_term = -MAX_INT;
	}
	else
	{
		p_term = pid_st->P_Factor * error;
	}

	// Calculate Iterm and limit integral runaway
	temp = (pid_st->sumError + error);
	if (temp > pid_st->maxSumError)
	{
		i_term = MAX_I_TERM;
		pid_st->sumError = pid_st->maxSumError;
	}
	else if (temp < -pid_st->maxSumError)
	{
		i_term = -MAX_I_TERM;
		pid_st->sumError = -pid_st->maxSumError;
	}
	else
	{
		pid_st->sumError = temp;
		i_term = pid_st->sumError * pid_st->I_Factor;
	}

	// Calculate Dterm
	d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue);

	pid_st->lastProcessValue = processValue;
	//serialWrite_int(p_term);
	//serialWrite(" ");
	//serialWrite_long(i_term);
	//serialWrite(" ");
	//serialWrite_int(d_term);
	//serialWrite_newline();

	ret = (p_term + i_term + d_term) / SCALING_FACTOR;
	if (ret > MAX_INT)
	{
		ret = MAX_INT;
	}
	else if (ret < -MAX_INT)
	{
		ret = -MAX_INT;
	}
	ret = (int32_t)pid_output_filter(ret);
	return ((uint16_t)ret);
}

#define FILTER_LENGTH 10
static int pid_output_list_index = 0;
static float pid_output_filter_coefficients[FILTER_LENGTH];
static float pid_output_list[FILTER_LENGTH];
/************************************************************************/
/* This is the filter that goes after the pid output                                                                     */
/************************************************************************/
float pid_output_filter(uint16_t last_output)
{
	if (pid_output_list_index < FILTER_LENGTH)
	{
		pid_output_list[pid_output_list_index++] = (float)last_output;
	}
	else
	{
		pid_output_list_index = 0;
		pid_output_list[pid_output_list_index++] = (float)last_output;
	}
	int temp_index = pid_output_list_index;
	float filter_output = 0;
	int coefficients_index = FILTER_LENGTH - 1;
	while (temp_index > 0)
	{
		filter_output += pid_output_list[--temp_index] * pid_output_filter_coefficients[coefficients_index];
	}
	temp_index = FILTER_LENGTH;
	while (temp_index > pid_output_list_index)
	{
		filter_output += pid_output_list[--temp_index] * pid_output_filter_coefficients[coefficients_index];
	}
	return filter_output;
}
/************************************************************************/
/* This function initializes the filter coefficients and the list
 * of pid outputs
 * It should be called before running the pid function and/or the 
 * pid_output_filter function                                                                     */
/************************************************************************/
void pid_output_filter_init()
{
	int i;
	float sum = 0;
	for (i = 0; i < FILTER_LENGTH; ++i)
	{
		pid_output_list[i] = 0;
		pid_output_filter_coefficients[i] = (FILTER_LENGTH - i);
		sum += i;
	}
	for (i = 0; i < FILTER_LENGTH; ++i)
	{
		pid_output_filter_coefficients[i] /= sum;
	}
}
int16_t pid_Controller_x(int16_t setPoint, double processValue, struct PID_DATA *pid_st)
{
	int16_t error, p_term, d_term;
	int32_t i_term, ret, temp;

	error = setPoint - processValue;

	if (error == 0)
	{
		pid_st->sumError = 0;
	}

	// Calculate Pterm and limit error overflow
	if (error > pid_st->maxError)
	{
		p_term = MAX_INT;
	}
	else if (error < -pid_st->maxError)
	{
		p_term = -MAX_INT;
	}
	else
	{
		p_term = pid_st->P_Factor * error;
	}

	// Calculate Iterm and limit integral runaway

	temp = pid_st->sumError + error;

	if (temp > pid_st->maxSumError)
	{
		i_term = MAX_I_TERM;
		pid_st->sumError = pid_st->maxSumError;
	}
	else if (temp < -pid_st->maxSumError)
	{
		i_term = -MAX_I_TERM;
		pid_st->sumError = -pid_st->maxSumError;
	}
	else
	{
		pid_st->sumError = temp;
		i_term = pid_st->sumError * pid_st->I_Factor;
	}

	// Calculate Dterm

	d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue);

	pid_st->lastProcessValue = processValue;

	ret = (p_term + i_term + d_term) / SCALING_FACTOR;
	if (ret > MAX_INT)
	{
		ret = MAX_INT;
	}
	else if (ret < -MAX_INT)
	{
		ret = -MAX_INT;
	}

	return ((int16_t)ret);
}

/*! \brief Resets the integrator.
 *
 *  Calling this function will reset the integrator in the PID regulator.
 */
void pid_Reset_Integrator(pidData_t *pid_st)
{
	pid_st->sumError = 0;
}

int32_t ComputePID(int16_t setPoint, double processValue, struct PID_DATA *pid_st)
{

	int32_t error = setPoint - processValue * 100;
	int32_t Ierr;

	Ierr = error * pid_st->I_Factor * 2;

	if (Ierr > 100000)
	{
		Ierr = 100000;
	}
	if (Ierr < -100000)
	{
		Ierr = -100000;
	}
	pid_st->sumError += Ierr;

	/*Compute PID Output*/
	int32_t out = (pid_st->P_Factor * error) + pid_st->sumError + pid_st->D_Factor * (error - pid_st->lastProcessValue) * 500;
	pid_st->lastProcessValue = error;

	out = out / 4096 / 8;

	return out;
}
