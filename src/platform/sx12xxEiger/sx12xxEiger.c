/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 *
 * NetBlocks 
 * www.netblocks.eu   
 */
/*! 
 * \file       sx12xxEiger.c
 * \brief        
 *
 * \version    1.0
 * \date       Nov 21 2012
 * \author     Miguel Luis
 */
#include <stdint.h> 
#include "spi.h"
#include "i2c.h"
#include "ioe.h"
#include "led.h"
#include "uart.h"
#include "sx12xxEiger.h"

#if( defined( STM32F4XX ) || defined( STM32F2XX ) )

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

#else

#include "usb_regs.h"
#include "usb_core.h"
#include "usb_init.h"
#include "usb_pwr.h"
#include "usb_bsp.h"
#include "usb_sil.h"

#endif

// System tick (1ms)
volatile uint32_t TickCounter = 0;

#if( defined( STM32F4XX ) || defined( STM32F2XX ) )

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
   
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

#else

#endif

void BoardInit( void )
{
    uint8_t i;

    /* Setup SysTick Timer for 1 us interrupts ( not too often to save power ) */
    if( SysTick_Config( SystemCoreClock / 1000 ) )
    { 
        /* Capture error */ 
        while (1);
    }

    // Initialize SPI
    SpiInit( );
    
    // Initialize LED
    for( i = 0; i < LED_NB; i++ )
    {
        LedInit( ( tLed )i );
    }
		LedOff( LED_RED );
		LedOff( LED_GREEN );
}

void Delay ( uint32_t delay )
{
    // Wait delay ms
    uint32_t startTick = TickCounter;
    while( ( TickCounter - startTick ) < delay );   
}

void LongDelay ( uint8_t delay )
{
    uint32_t longDelay;
    uint32_t startTick;

    longDelay = delay * 1000;

    // Wait delay s
    startTick = TickCounter;
    while( ( TickCounter - startTick ) < longDelay );   
}
