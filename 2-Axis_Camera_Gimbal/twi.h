/*
 * twi.h
 *
 *  Created on: 5 sept. 2013
 *      Author: ldo
 */

#ifndef I2C_H_
#define I2C_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "txSerial.h"
#if defined(__AVR_ATmega32A__)
#define DDR_TWI DDRC
#define PORT_TWI PORTC
#define DDR_SCL PC0
#define DDR_SDA PC1
#endif
#if defined(__AVR_ATmega644P__)
#define DDR_TWI DDRC
#define PORT_TWI PORTC
#define DDR_SCL PC0
#define DDR_SDA PC1
#endif
#if defined(__AVR_ATmega644__)
#define DDR_TWI DDRC
#define PORT_TWI PORTC
#define DDR_SCL PC0
#define DDR_SDA PC1
#endif


#if defined(__AVR_ATmega2560__)
#define DDR_TWI DDRD
#define PORT_TWI PORTD
#define DDR_SCL PD0
#define DDR_SDA PD1
#endif
#if defined(__AVR_ATmega1284__)
#define DDR_TWI DDRC
#define PORT_TWI PORTC
#define DDR_SCL PC0
#define DDR_SDA PC1
#endif
/* Master */
#define TW_START     0x08    /* start condition transmitted */
#define TW_REP_START 0x10    /* repeated START condition transmitted */
#define TW_MT_ARB_LOST  0x38    /* arbitration lost in SLA+RW, SLA+R received, ACK returned */

/* Master Transmitter */
#define TW_MT_SLA_ACK   0x18 /* SLA+W transmitted; ACK received */
#define TW_MT_SLA_NACK  0x20 /* SLA+W transmitted; NACK received */
#define TW_MT_DATA_ACK  0x28 /* SLA+R has been transmitted; ACK has been received */
#define TW_MT_DATA_NACK 0x30 /* SLA+R has been transmitted; ACK has been received */

/* master receiver */
#define TW_MR_SLA_ACK   0x40    /* SLA+R has been transmitted; ACK has been received */
#define TW_MR_SLA_NACK  0x48    /* SLA+R has been transmitted; ACK has been received */
#define TW_MR_DATA_ACK  0x50    /* data received, ACK transmitted */
#define TW_MR_DATA_NACK 0x58    /* data received, NACK transmitted */

/* Slave Receiver */
#define TW_SR_SLA_ACK           0x60    /* SLA+W received, ACK returned */
#define TW_SR_ARB_LOST_SLA_ACK  0x68    /* arbitration lost in SLA+RW, SLA+W received, ACK returned */
#define TW_SR_GCALL_ACK         0x70    /* general call received, ACK returned */
#define TW_SR_ARB_LOST_GCALL_ACK       0x78 /* arbitration lost in SLA+RW, general call received, ACK returned */
#define TW_SR_DATA_ACK          0x80    /* data received, ACK returned */
#define TW_SR_DATA_NACK         0x88    /* data received, NACK returned */
#define TW_SR_GCALL_DATA_ACK    0x90    /* general call data received, ACK returned */
#define TW_SR_GCALL_DATA_NACK    0x98   /* general call data received, NACK returned */
#define TW_SR_STOP      0xA0    /* stop or repeated start condition received while selected */

/* slave transmitter mode */
#define TW_ST_SLA_ACK           0xA8    /* SLA+R received, ACK returned*/
#define TW_ST_ARB_LOST_SLA_ACK  0xB0    /* arbitration lost in SLA+RW, SLA+R received, ACK returned */
#define TW_ST_DATA_ACK          0xB8    /* data transmitted, ACK received */
#define TW_ST_DATA_NACK         0xC0    /* data transmitted, NACK received */
#define TW_ST_LAST_DATA         0xC8    /* last data byte transmitted, ACK received */

#define WRITE 0
#define READ 1

#define soft_reset()        \
do                          \
{                           \
	wdt_enable(WDTO_30MS);  \
	for(;;)                 \
	{                       \
	}                       \
} while(0)

void
twi_master_setup(void);
void
twi_slave_setup(uint8_t address, volatile uint8_t * buffer, volatile uint8_t *receive_buffer);


void
twi_start(void);
void
twi_address(void);
void
twi_data(uint8_t data);
void
twi_stop(void);

void
twi_write_bytes(uint8_t add, uint8_t nb_bytes, volatile uint8_t *buffer);
uint8_t
twi_read_bytes(uint8_t add, volatile uint8_t *reg, uint8_t nb_bytes,
    volatile uint8_t *buffer);

#endif /* TWI_H_ */
