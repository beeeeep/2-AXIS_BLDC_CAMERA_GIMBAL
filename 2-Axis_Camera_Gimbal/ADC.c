#include "ADC.h"

static volatile int16_t Joy_x;
void ADC_SETUP(void)
{
	ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << ADLAR);											   // ENABLE ADC0 WITH AREF VOLTAGE REFRENCE
	ADCSRA = (1 << ADSC) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN); //Set adc interrupt, prescaler to 128, start conversion
	DDRA = 0x00;
}

void ADC_get_values(joy_t *Joy)
{
	Joy_x -= 125;
	Joy_x *= 2;
	if (Joy_x > 250)
	{
		Joy_x = 250;
	}
	if (Joy_x < -250)
	{
		Joy_x = -250;
	}
	Joy->x = (Joy_x);
}

ISR(ADC_vect)
{
	Joy_x = ADCH;

	ADCSRA |= (1 << ADSC);
}