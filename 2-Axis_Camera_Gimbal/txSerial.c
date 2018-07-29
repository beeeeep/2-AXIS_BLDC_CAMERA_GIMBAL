#include "txSerial.h"

char rxBuffer[RX_BUFFER_SIZE];
uint8_t rxReadPos = 0;
uint8_t rxWritePos = 0;

void USART_Init(void)
{
	UBRR0H = (BRC >> 8);
	UBRR0L = BRC;

	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00) | (1 << UPM00) | (1 << UPM01);

	sei();
}

void USARTWrite(char c)
{

	while (!(UCSR0A & (1 << UDRE0)))
		;

	UDR0 = c;
}

void serialWrite(char *data)
{
	uint8_t i = 0;
	while (data[i])
	{
		USARTWrite(data[i++]);
	}
}

void serialWrite_int(int n)
{

	char theIntAsString[7];
	int i;
	sprintf(theIntAsString, "%i", n);
	for (i = 0; i < strlen(theIntAsString); i++)
	{
		USARTWrite(theIntAsString[i]);
	}
}
void serialWrite_double(double n, uint8_t num_of_dcmls)
{

	static char buffer[31];
	char *print = dtostrf(n, 3, num_of_dcmls, buffer);
	serialWrite(print);
}

//void serialWrite_float(float n)
//{
//
//char thefloatAsString[7];
//int i;
//sprintf( thefloatAsString, "%f", n );
//for (i=0; i < strlen(thefloatAsString); i++)
//{
//USARTWrite(thefloatAsString[i]);
//}
//}
void serialWrite_uint(int n)
{

	char theIntAsString[7];
	int i;
	sprintf(theIntAsString, "%u", n);
	for (i = 0; i < strlen(theIntAsString); i++)
	{
		USARTWrite(theIntAsString[i]);
	}
}

void serialWrite_long(unsigned long n)
{

	char theIntAsString[7];
	int i;
	sprintf(theIntAsString, "%ld", n);
	for (i = 0; i < strlen(theIntAsString); i++)
	{
		USARTWrite(theIntAsString[i]);
	}
}

void serialWrite_newline(void)
{
	serialWrite("\n\r");
}

int serialRead_int(void)
{
	int number;
	uint8_t i = 0;
	char Read_Char;
	char data[RX_BUFFER_SIZE];

	while (Read_Char != 0x0D) //Read until "Enter" is pressed
	{
		Read_Char = getChar();
		if (Read_Char != '\0')
		{
			data[i] = Read_Char;
			i++;
		}
	}

	number = atoi(data);

	return number;
}

int serialRead_int_timed(void)
{
	unsigned int current_time;
	unsigned int old_time = 0;
	int number;
	uint8_t i = 0;
	char Read_Char;
	char data[RX_BUFFER_SIZE];

	while (Read_Char != 0x0D) //Read until "Enter" is pressed
	{
		current_time = millis_get();
		if (current_time - old_time > 5000)
		{
			break;
		}

		Read_Char = getChar();
		if (Read_Char != '\0')
		{
			data[i] = Read_Char;
			i++;
		}
	}

	number = atoi(data);

	return number;
}

char getChar(void)
{
	char ret = '\0';

	if (rxReadPos != rxWritePos)
	{
		ret = rxBuffer[rxReadPos];

		rxReadPos++;

		if (rxReadPos >= RX_BUFFER_SIZE)
		{
			rxReadPos = 0;
		}
	}

	return ret;
}

ISR(USART0_RX_vect)
{

	rxBuffer[rxWritePos] = UDR0;

	rxWritePos++;

	if (rxWritePos >= RX_BUFFER_SIZE)
	{
		rxWritePos = 0;
	}
}