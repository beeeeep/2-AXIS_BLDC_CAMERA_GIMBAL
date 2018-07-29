/*
 * twi.c
 *
 *  Created on: 3 oct. 2014
 *      Author: ldo
 */

#include "twi.h"
#include <util/delay.h>

volatile uint8_t slave_address;

static volatile uint8_t twi_flag_read_complete = 1;
static volatile uint8_t twi_flag_write_complete = 1;

static volatile uint8_t *twi_receive_buffer;
static volatile uint8_t *twi_transmit_buffer;
static volatile uint8_t twi_i = 0;

uint8_t twi_mode;
uint8_t twi_nb_data_to_transmit, twi_nb_data_to_receive;

ISR(TWI_vect)
{
  uint8_t status;

  status = TWSR & 0xF8;
  switch (status)
    {
    case (TW_START): /* 0x08 : start condition transmitted */
	
    case (TW_REP_START): /* 0x10 : repeated START condition transmitted */
      twi_address ();
      twi_i = 0;

      break;
    case (TW_MT_ARB_LOST): /* 0x38 : arbitration lost in SLA+RW, SLA+R received, ACK returned */
	 serialWrite("arb lost SLA+RW|SLA+R rcvd|ACK ret");
	 serialWrite("\t");
	 serialWrite_uint(slave_address);
	 serialWrite_newline();
	 soft_reset();
      break;

      /* Master Transmitter */
    case (TW_MT_SLA_ACK): /* 0x18 : SLA+W transmitted; ACK received */
	      TWDR = twi_transmit_buffer[0];
	      TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
	      twi_i++;
	      break;
    case (TW_MT_SLA_NACK): /* 0x20 : SLA+W transmitted; NACK received */
			
			serialWrite("SLAW+W transmitted, NACK recieved");
			 serialWrite("\t");
			 serialWrite_uint(slave_address);
			 serialWrite_newline();
			break;

    case (TW_MT_DATA_ACK):/* 0x28 : data has been transmitted; ACK has been received */
	  if (twi_i == twi_nb_data_to_transmit)
	  {
		  if (twi_mode == WRITE)
		  {
			  twi_stop ();
			  twi_flag_write_complete = 1;
		  }
		  else // READ
		  {
			  slave_address++;
			  twi_start ();
		  }
	  }
	  else
	  {
		  twi_data (twi_transmit_buffer[twi_i]);
		  twi_i++;
	  }
	  break;
    case (TW_MT_DATA_NACK): /* 0x30 : data has been transmitted; NACK has been received */
	      serialWrite("data tranmited, NACK recieved");
		   serialWrite("\t");
		   serialWrite_uint(slave_address);
		   serialWrite_newline();
		   soft_reset();
		  break;
    

      /* Master Receiver */
    case (TW_MR_SLA_ACK):/* 0x40 : SLA+R has been transmitted; ACK has been received */
		   if (twi_nb_data_to_receive == 1)
		   {
			   TWCR &= ~(1 << TWEA); /* desactiver ACK pour la derniere demande */
		   }
		   break;
    case (TW_MR_SLA_NACK): /* 0x48 : SLA+R has been transmitted; NACK has been received */
		serialWrite("SLA+R transmitted, NACK received");
		 serialWrite("\t");
		 serialWrite_uint(slave_address);
		 serialWrite_newline();
		break;
    case (TW_MR_DATA_ACK):/* 0x50 : data received, ACK transmitted */
    case (TW_MR_DATA_NACK): /* 0x58 : data received, NACK transmitted */
      twi_receive_buffer[twi_i] = TWDR;
      twi_i++;

      if (twi_i == (twi_nb_data_to_receive - 1))
	{
	  TWCR &= ~(1 << TWEA); /* desactiver ACK pour la derniere demande */
	}
      if (twi_i == twi_nb_data_to_receive)
	{
	  twi_flag_read_complete = 1;
	  twi_stop ();
	}
      break;

      /* Slave Receiver */
    case (TW_SR_SLA_ACK):/* 0x60 : SLA+W received, ACK returned */
      break;
    case (TW_SR_ARB_LOST_SLA_ACK):/*0x68 : arbitration lost in SLA+RW, SLA+W received, ACK returned */
	     serialWrite("arb lost SLA+RW|SLA+W rcvd|ACK ret");
		  serialWrite("\t");
		  serialWrite_uint(slave_address);
		  serialWrite_newline();
		   cli();
		   _delay_ms(100);
		   twi_master_setup();
		   sei();
      break;
    case (TW_SR_GCALL_ACK): /*0x70 : general call received, ACK returned */
      break;
    case (TW_SR_ARB_LOST_GCALL_ACK): /* 0x78 : arbitration lost in SLA+RW, general call received, ACK returned */
		serialWrite("arb lost SLA+RW|GC rcvd|ACK ret");
		 serialWrite("\t");
		 serialWrite_uint(slave_address);
		 serialWrite_newline();
		  cli();
		  _delay_ms(100);
		  twi_master_setup();
		  sei();
      break;
    case (TW_SR_DATA_ACK): /* 0x80 : data received, ACK returned */
    case (TW_SR_DATA_NACK): /* 0x88 : data received, NACK returned */
    case (TW_SR_GCALL_DATA_ACK): /* 0x90 : general call data received, ACK returned */
    case (TW_SR_GCALL_DATA_NACK):/* 0x98 : general call data received, NACK returned */
      twi_receive_buffer[twi_i] = TWDR;
      twi_i++;
      break;
    case (TW_SR_STOP):/* 0xA0 : stop or repeated start condition received while selected */
      twi_flag_write_complete = 1;
      twi_i = 0;
      break;

      /* Slave Transmitter */
    case (TW_ST_SLA_ACK): /* 0xA8 : SLA+R received, ACK returned*/
      twi_i = 0;
      TWDR = twi_transmit_buffer[twi_i];
      twi_i++;
      break;
    case (TW_ST_ARB_LOST_SLA_ACK): /* 0xB0 : arbitration lost in SLA+RW, SLA+R received, ACK returned */
		serialWrite("arb lost SLA+RW|SLA+R rcvd|ACK ret");
		 serialWrite("\t");
		 serialWrite_uint(slave_address);
		 serialWrite_newline();
		 cli();
		 _delay_ms(100);
		 twi_master_setup();
		 sei();
      break;
    case (TW_ST_DATA_ACK): /* 0xB8 : data transmitted, ACK received */
      TWDR = twi_transmit_buffer[twi_i];
      twi_i++;
      break;
    case (TW_ST_DATA_NACK):/*0xC0 : data transmitted, NACK received */
	  serialWrite("data trasmitted, NACK recieved");
	   serialWrite("\t");
	   serialWrite_uint(slave_address);
	   serialWrite_newline();
      break;
    case (TW_ST_LAST_DATA): /* 0xC8 : last data byte transmitted, ACK received */
      break;

    default:
      break;
    }

  TWCR |= (1 << TWINT); // TWINT flag bit is cleared
}

/*
 * \fn void twi_master_setup(void)
 * \brief twi master setup
 * SCLfreq = CPUfreq / (16 + 2 * TWBR * presc)
 * */
void
twi_master_setup (/*uint16_t scl_freq*/void)
{
  PORT_TWI = (1 << DDR_SDA) | (1 << DDR_SCL); // activate internal pull_ups for twi

  TWSR = 0x00;    // no prescaler

#if (F_CPU == 16000000UL)
TWBR = 72; // 100kHz @16Mhz
//TWBR = 0x0C;    // 400 KHz @16MHz
#endif
#if (F_CPU == 20000000UL)
  TWBR = 92; // 100kHz @20Mhz
#endif

  TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
  SREG |= (1 << SREG_I);
}

/* \fn void twi_slave_setup(uint8_t address, volatile uint8_t *transmit_buffer, volatile uint8_t *receive_buffer)
 * \brief TWI slave setup
 * \param address : address on 7 bits
 * \param transmit buffer : adresse du buffer pour la transmission
 * \param receive_buffer : adresse du buffer pour la reception
 */
void
twi_slave_setup (uint8_t address, volatile uint8_t *transmit_buffer,
		 volatile uint8_t *receive_buffer)
{
  PORT_TWI = (1 << DDR_SDA) | (1 << DDR_SCL); // activate internal pull_ups for twi

  //PRR &= ~(1 << PRTWI); // the PRTWI bit in PRR must be written to zero to enable the TWI
  TWAR = (address << 1) + 1; // Set own TWI slave address. Accept TWI General Calls.
  twi_transmit_buffer = transmit_buffer;
  twi_receive_buffer = receive_buffer;
  /* TWINT = 0 :
   * TWEA = 1 : enable ACK
   * TWSTA = 0: start condition
   * TWSTO = 0 : stop condition
   * TWEN = 1 : enable the TWI
   * TWIE = 1 : enable TWI interrupt */TWCR = (1 << TWEA) | (1 << TWEN)
      | (1 << TWIE);
  SREG |= (1 << SREG_I);
}

/*
 * \fn void twi_start(void)
 * \brief transmit START condition
 * */
void
twi_start (void)
{
  TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWSTA); /* send start condition */
}

/*
 * \fn void twi_address(void)
 * \brief transmit slave address SLA
 * */
void
twi_address (void)
{
  TWDR = slave_address; /* load SLA into TWDR */

  TWCR &= ~(1 << TWSTA); /* desactivate START */
}

/* \fn void twi_data(uint8_t data)
 * \brief transmit data
 * \param data to send
 * */
void
twi_data (uint8_t data)
{
  TWDR = data;
}

/*
 * \fn void twi_stop(void)
 * \brief transmit STOP condition
 * */
void
twi_stop (void)
{
  TWCR |= (1 << TWSTO); /* STOP */
}

/*
 * \fn void twi_write_bytes(uint8_t add, uint8_t nb_bytes, volatile uint8_t *buffer)
 * \brief send nb_bytes to slave
 * \param add : slave address
 * \param nb_bytes : nb bytes to send
 * \param buffer : transmit buffer address
 * */
void
twi_write_bytes (uint8_t add, uint8_t nb_bytes, volatile uint8_t *buffer)
{
  if (twi_flag_write_complete == 1)
    {
      twi_flag_write_complete = 0;

      slave_address = (add << 1);
      twi_nb_data_to_transmit = nb_bytes;
      twi_transmit_buffer = buffer;

      twi_mode = WRITE;
      twi_i = 0;
      twi_start ();

      while (twi_flag_write_complete == 0)
	;
    }
}

/**
 * \fn void twi_read_bytes (uint8_t add, volatile uint8_t *reg, uint8_t nb_bytes, volatile uint8_t *buffer)
 * \brief read nb_bytes to slave
 * \param add is 7-bits slave address
 * \param reg register to read
 * \param nb_bytes : nb bytes to read
 * \param buffer : receive buffer address
 * \return 0 if success, 1 if no slave response (timeout)
 */
uint8_t
twi_read_bytes (uint8_t add, volatile uint8_t *reg, uint8_t nb_bytes,
		volatile uint8_t *buffer)
{
  uint16_t timeout = 0;

  twi_flag_read_complete = 0;
  twi_flag_write_complete = 0;

  slave_address = (add << 1);
  twi_transmit_buffer = reg;
  twi_nb_data_to_transmit = 1;
  twi_nb_data_to_receive = nb_bytes;
  twi_receive_buffer = buffer;

  twi_mode = READ;
  twi_i = 0;
  twi_start ();

  while ((twi_flag_read_complete == 0) && (timeout < 5000))
    {
      timeout++;
    }
	if(timeout>5000)
	{
	 serialWrite("I2C timeout");
	  serialWrite("\t");
	  serialWrite_uint(slave_address);
	  serialWrite_newline();
	}
  return (timeout >= 5000);
}
