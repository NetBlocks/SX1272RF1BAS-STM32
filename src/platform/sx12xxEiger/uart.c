/*
 * Copyright (C) SEMTECH SA. All rights reserved.
 * Developed at SEMTECH SA, Neuchatel, Switzerland
 * All rights reserved. Reproduction in whole or part is
 * prohibited without the written permission of the copyright owner.
 *                                 
 */
/*! 
 *	\file		uart.c
 *	\brief		Implements the functions that manage the microcontroller UART's
 *
 *	\version	1.0
 *	\date		Feb 26 2007
 *	\author		MiL
 *
 *	\version	1.1
 *	\date		Apr 30 2007
 *	\author		MAL
 *	\remarks	- Adaptated to compile for the PIC16F877 
 *
 *	\version	1.2
 *	\date		Jun 1 2007
 *	\author		MAL
 *	\remarks	- Rx fifo and Rx interrupt handler added.
 */
#include "sx12xxEiger.h"
#include "fifo.h"
#include "uart.h"

#if defined( STM32F4XX ) || defined( STM32F2XX )

#include "usbd_cdc_core.h"
#include "usb_conf.h"
#include "usbd_desc.h"

#else

#include "usb_core.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_bsp.h"

#endif

/*!
 * FIFO buffers size
 */
#define FIFO_RX_SIZE								128
#define FIFO_TX_SIZE								128

#if defined( STM32F4XX ) || defined( STM32F2XX )

extern CDC_IF_Prop_TypeDef  APP_FOPS;

#else

extern  uint8_t USART_Rx_Buffer[];
extern uint32_t USART_Rx_ptr_in;
extern LINE_CODING linecoding;

#endif

tFifo FifoRx;
tFifo FifoTx;

uint16_t FifoRxBuffer[FIFO_RX_SIZE];
uint16_t FifoTxBuffer[FIFO_TX_SIZE];

uint8_t UartInit( void )
{
	FifoInit( &FifoRx, FifoRxBuffer, FIFO_RX_SIZE );
	FifoInit( &FifoTx, FifoTxBuffer, FIFO_TX_SIZE );

	return UART_OK;
}

void UartProcess( void )
{
	if( IsFifoEmpty( &FifoTx ) == false )
	{
#if defined( STM32F4XX ) || defined( STM32F2XX )
        
		APP_FOPS.pIf_DataTx( 0, 0 );
        
#else
        
        if( linecoding.datatype == 7 )
        {
            USART_Rx_Buffer[USART_Rx_ptr_in] = FifoPop( &FifoTx ) & 0x7F;
        }
        else if (linecoding.datatype == 8)
        {
            USART_Rx_Buffer[USART_Rx_ptr_in] = FifoPop( &FifoTx );
        }

        USART_Rx_ptr_in++;

        /* To avoid buffer overflow */
        if( USART_Rx_ptr_in == USART_RX_DATA_SIZE )
        {
            USART_Rx_ptr_in = 0;
        }	
        
#endif        
	}
}

uint8_t UartPutChar( uint8_t data )
{
	if( IsFifoFull( &FifoTx ) == true )
	{
		UartProcess( );
		//return BUSY;
	}
	FifoPush( &FifoTx, data );
	return UART_OK;
}

uint8_t UartGetChar( uint8_t *data )
{
	if( IsFifoEmpty( &FifoRx ) == true )
	{
		return UART_EMPTY;
	}
	*data = FifoPop( &FifoRx );
	return UART_OK;
}

uint8_t UartPutBuffer( uint8_t *buffer, uint8_t size )
{
	uint8_t i;

	for( i = 0; i < size; i++ )
	{
		if( IsFifoFull( &FifoTx ) == true )
		{
			return UART_BUSY;
		}
		FifoPush( &FifoTx, buffer[i] );
	}
	return UART_OK;	
}

uint8_t UartGetBuffer( uint8_t *buffer, uint8_t size, uint8_t *nbReadBytes )
{
	uint8_t i;

	for( i = 0; i < size; i++ )
	{
		if( IsFifoEmpty( &FifoRx ) == true )
		{
			*nbReadBytes = i;
			return UART_EMPTY;
		}
		buffer[i] = FifoPop( &FifoRx );
	}
	*nbReadBytes = i;
	return( UART_OK );
}

uint8_t UartFlush( void )
{
	FifoFlush( &FifoRx );
	FifoFlush( &FifoTx );
	return( UART_OK );
}

bool UartIsTxFifoEmpty( void )
{
	return IsFifoEmpty( &FifoTx );
}

bool UartIsRxFifoEmpty( void )
{
	return IsFifoEmpty( &FifoRx );
}
