/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       bleeper.c
 * \brief        
 *
 * \version    1.0
 * \date       Nov 21 2012
 * \author     Miguel Luis
 */
#include <stdint.h> 
#include "spi.h"
#include "i2c.h"
#include "led.h"
#include "bleeper.h"

// System tick (1ms)
volatile uint32_t TickCounter = 0;

void InitUnusedGPIO( void ) ;
void SelectorInit( void );
uint8_t GetSelectorValue( void );
void SetSelectorStandBy( void );
void SetSelectorSleep( void );

void BoardInit( void )
{
    uint8_t i;

    /* Setup SysTick Timer for 1 us interrupts ( not too often to save power ) */
    if( SysTick_Config( SystemCoreClock / 1000 ) )
    { 
        /* Capture error */ 
        while (1);
    }

    // Initialize unused GPIO to optimize power consumption
    InitUnusedGPIO( );

    // Initialize Selector
    SelectorInit( );

    // Initialize SPI
    SpiInit( );
    
    // Initialize LED
    for( i = 0; i < LED_NB; i++ )
    {
        LedInit( ( tLed )i );
    }

    LedOn( LED1 );
    LedOn( LED2 );
    LedOn( LED3 );
    LongDelay( 1 );
    LedOff( LED1 );
    LedOff( LED2 );
    LedOff( LED3 );
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

void InitUnusedGPIO( void ) 
{   
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // enable clock of the IO bank A
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); // enable clock of the IO bank B
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); // enable clock of the IO bank C
    
    GPIO_StructInit(&GPIO_InitStructure);
    // activate pullups on C8 to C12 (SDIO)
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    // -> save 10 to 50 µA
    
    // activate pullups on B10 and B11 (I²C)
    //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    //GPIO_Init(GPIOB, &GPIO_InitStructure);
    // -> dubious influence (might increase power a little bit)
    
    // activate pulldowns on B6, B7 and B12 to B15 (J2 and J5)
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // very little influence (<<5 µA)
    
    // activate pulldowns on A11 and A12 (USB)
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // very little influence (<<5 µA)
    
    // activate pulldowns on A10, B0, B1, C3 and C13 (unused pins)
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_13;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    // little influence? (~10 µA)
}


void SelectorInit( void )
{    
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOC, ENABLE);
    
    // configuration for selector IOs
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // enable pull-up resistors
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure); // configure bits 1 (LSB) and 2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure); // configure bits 4 and 8 (MSB)

}

uint8_t GetSelectorValue( void ) 
{
    uint8_t i = 0, j, k = 255;

    do {
        j = i;
        k = j;
        Delay(1); // 1 ms delay between check
        i  =     !GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8);
        i += 2 * !GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9);
        i += 4 * !GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6);
        i += 8 * !GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);
    } while( ( i != j ) && ( i != k ) ); // wait for 3 successive values to be equal 
    
    return i;
}


void SetSelectorStandBy( void ) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOC, ENABLE);
    
    // configuration for selector IOs
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // enable pull-up resistors
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure); // configure bits 1 (LSB) and 2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure); // configure bits 4 and 8 (MSB)
    
}

void SetSelectorSleep( void ) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // replace pullups by pulldowns to save power
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    
    // configure bits 1 (LSB) and 2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // configure bits 4 and 8 (MSB)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
}
