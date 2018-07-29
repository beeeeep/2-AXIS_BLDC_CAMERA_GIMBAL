
#ifndef TXSERIAL_H_
#define TXSERIAL_H_
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "millis.h"
#define BAUD 9600
#define BRC ((F_CPU / (16UL * BAUD)) - 1)
#define RX_BUFFER_SIZE 128
void USART_Init(void);

void USARTWrite(char c);

void serialWrite(char *data);

void serialWrite_int(int n);

void serialWrite_double(double n, uint8_t num_of_dcmls);

//void serialWrite_float(float n);

void serialWrite_uint(int n);

void serialWrite_long(unsigned long n);

void serialWrite_ulong(unsigned long n);

void serialWrite_newline(void);

int serialRead_int(void);

int serialRead_int_timed(void);

char getChar(void);

#endif /* TXSERIAL_H_ */
