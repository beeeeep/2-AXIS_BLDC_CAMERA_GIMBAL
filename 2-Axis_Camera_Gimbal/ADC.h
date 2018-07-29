#ifndef ADC_H_
#define ADC_H_
#include <avr/io.h>
#include <avr/interrupt.h>
typedef struct
{
	int16_t x;
	int16_t y;

} joy_t;

void ADC_SETUP(void);

void ADC_get_values(joy_t *Joy);

#endif /* ADC_H_ */