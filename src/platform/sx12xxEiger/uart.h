/*
 * Copyright (C) SEMTECH SA. All rights reserved.
 * Developed at SEMTECH SA, Neuchatel, Switzerland
 * All rights reserved. Reproduction in whole or part is
 * prohibited without the written permission of the copyright owner.
 *                                 
 */
/*! 
 *	\file		uart.h
 *	\brief		Implements the functions that manage the microcontroller UART's
 *
 *	\version	1.0
 *	\date		Feb 26 2007
 *	\author		MiL
 */
#ifndef __UART_H__
#define __UART_H__

#include <stdbool.h>

/*!
 * Functions return codes definition
 */
#define UART_OK                                       ( uint8_t ) 0x00
#define UART_ERROR                                    ( uint8_t ) 0x01
#define UART_BUSY                                     ( uint8_t ) 0x02
#define UART_EMPTY                                    ( uint8_t ) 0x03
//~ #define UART_DONE                                     ( uint8_t ) 0x04
//~ #define UART_TIMEOUT                                  ( uint8_t ) 0x05
//~ #define UART_UNSUPPORTED                              ( uint8_t ) 0x06
//~ #define UART_WAIT                                     ( uint8_t ) 0x07
//~ #define UART_CLOSE                                    ( uint8_t ) 0x08
//~ #define UART_ACK                                      ( uint8_t ) 0x09
//~ #define UART_NACK                                     ( uint8_t ) 0x0A
//~ #define UART_YES                                      ( uint8_t ) 0x0B
//~ #define UART_NO                                       ( uint8_t ) 0x0C

/*
 *	\brief	Initializes the UART peripheral
 *
 *	\retval	status	[OK]
 */
uint8_t UartInit( void );

void UartProcess( void );

/*
 *	\brief	Puts a char in the UART
 *
 *	\param [IN]:	data
 *	\retval	status	[UART_OK, UART_BUSY]
 */
uint8_t UartPutChar( uint8_t Data );

/*
 *	\brief	Gets a char from the UART
 *
 *	\param [OUT]:	data
 *	\retval	status	[UART_OK, UART_EMPTY]
 */
uint8_t UartGetChar( uint8_t *data );

/*
 *	\brief	Sends a buffer to the UART
 *
 *	\param [IN]:	buffer
 *	\param [IN]:	size
 *	\retval	status	[UART_OK, UART_ERROR]
 */
uint8_t UartPutBuffer( uint8_t *buffer, uint8_t size );

/*
 *	\brief	Receives a buffer from the UART,
 *          if size is not reached return the number
 *			of bytes read.
 *
 *	\param[IN]:	buffer
 *	\param[IN]:	size
 *	\param[OUT]:	nbReaddBytes
 *	\retval	status	[UART_OK, UART_EMPTY]
 */
uint8_t UartGetBuffer( uint8_t *buffer, uint8_t size, uint8_t *nbReaddBytes );

/*
 *	\brief	Flushes the UART fifos and RX/TX registers
 *
 *	\retval	status	[UART_OK]
 */
uint8_t UartFlush( void );

/*
 *	\brief	Tests if UART Tx Fifo is empty
 *
 *	\retval	status	[true, false]
 */
bool UartIsTxFifoEmpty( void );

/*
 *	\brief	Tests if UART Rx Fifo is empty
 *
 *	\retval	status	[true, false]
 */
bool UartIsRxFifoEmpty( void );

#endif // __UART_H__
