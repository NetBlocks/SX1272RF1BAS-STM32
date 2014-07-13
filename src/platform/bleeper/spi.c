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
 * \file       spi.c
 * \brief      SPI hardware driver
 *
 * \version    1.0
 * \date       Feb 12 2010
 * \author     Miguel Luis
 */
#include <stdint.h>
#if ( STM32F4XX )
#include "stm32f4xx_spi.h"
#elif defined( STM32F2XX )
#include "stm32f2xx_spi.h"
#elif defined( STM32F1XX )
#include "stm32f10x_spi.h"
#else
#include "stm32l1xx_spi.h"
#endif
#include "spi.h"

#if defined( STM32F4XX ) || defined( STM32F2XX )

#define SPI_INTERFACE                               SPI1
#define SPI_CLK                                     RCC_APB2Periph_SPI1

#define SPI_PIN_SCK_PORT                            GPIOB
#define SPI_PIN_SCK_PORT_CLK                        RCC_AHB1Periph_GPIOB
#define SPI_PIN_SCK                                 GPIO_Pin_3
#define SPI_PIN_SCK_AF_SOURCE                       GPIO_PinSource3
#define SPI_PIN_SCK_AF                              GPIO_AF_SPI1

#define SPI_PIN_MISO_PORT                           GPIOB
#define SPI_PIN_MISO_PORT_CLK                       RCC_AHB1Periph_GPIOB
#define SPI_PIN_MISO                                GPIO_Pin_4
#define SPI_PIN_MISO_AF_SOURCE                      GPIO_PinSource4
#define SPI_PIN_MISO_AF                             GPIO_AF_SPI1

#define SPI_PIN_MOSI_PORT                           GPIOA
#define SPI_PIN_MOSI_PORT_CLK                       RCC_AHB1Periph_GPIOA
#define SPI_PIN_MOSI                                GPIO_Pin_7
#define SPI_PIN_MOSI_AF_SOURCE                      GPIO_PinSource7
#define SPI_PIN_MOSI_AF                             GPIO_AF_SPI1

#elif defined( STM32L1XX_HD )

#define SPI_INTERFACE                               SPI1
#define SPI_CLK                                     RCC_APB2Periph_SPI1

#define SPI_PIN_SCK_PORT                            GPIOA
#define SPI_PIN_SCK_PORT_CLK                        RCC_AHBPeriph_GPIOA
#define SPI_PIN_SCK                                 GPIO_Pin_5
#define SPI_PIN_SCK_AF_SOURCE                       GPIO_PinSource5
#define SPI_PIN_SCK_AF                              GPIO_AF_SPI1

#define SPI_PIN_MISO_PORT                           GPIOA
#define SPI_PIN_MISO_PORT_CLK                       RCC_AHBPeriph_GPIOA
#define SPI_PIN_MISO                                GPIO_Pin_6
#define SPI_PIN_MISO_AF_SOURCE                      GPIO_PinSource6
#define SPI_PIN_MISO_AF                             GPIO_AF_SPI1

#define SPI_PIN_MOSI_PORT                           GPIOA
#define SPI_PIN_MOSI_PORT_CLK                       RCC_AHBPeriph_GPIOA
#define SPI_PIN_MOSI                                GPIO_Pin_7
#define SPI_PIN_MOSI_AF_SOURCE                      GPIO_PinSource7
#define SPI_PIN_MOSI_AF                             GPIO_AF_SPI1

#else

#define SPI_INTERFACE                               SPI3
#define SPI_CLK                                     RCC_APB1Periph_SPI3

#define SPI_PIN_SCK_PORT                            GPIOB
#define SPI_PIN_SCK_PORT_CLK                        RCC_APB2Periph_GPIOB
#define SPI_PIN_SCK                                 GPIO_Pin_3

#define SPI_PIN_MISO_PORT                           GPIOB
#define SPI_PIN_MISO_PORT_CLK                       RCC_APB2Periph_GPIOB
#define SPI_PIN_MISO                                GPIO_Pin_4

#define SPI_PIN_MOSI_PORT                           GPIOB
#define SPI_PIN_MOSI_PORT_CLK                       RCC_APB2Periph_GPIOB
#define SPI_PIN_MOSI                                GPIO_Pin_5

#endif

void SpiInit( void )
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

#if defined( STM32F4XX ) || defined( STM32F2XX )
    /* Enable peripheral clocks --------------------------------------------------*/
    /* Enable SPIy clock and GPIO clock for SPIy */
    RCC_AHB1PeriphClockCmd( SPI_PIN_MISO_PORT_CLK | SPI_PIN_MOSI_PORT_CLK |
                            SPI_PIN_SCK_PORT_CLK, ENABLE );
    RCC_APB2PeriphClockCmd( SPI_CLK, ENABLE );

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_PinAFConfig( SPI_PIN_SCK_PORT, SPI_PIN_SCK_AF_SOURCE, SPI_PIN_SCK_AF );
    GPIO_PinAFConfig( SPI_PIN_MOSI_PORT, SPI_PIN_MOSI_AF_SOURCE, SPI_PIN_MISO_AF );
    GPIO_PinAFConfig( SPI_PIN_MISO_PORT, SPI_PIN_MISO_AF_SOURCE, SPI_PIN_MOSI_AF );

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

#elif defined( STM32L1XX_HD )

    /* Enable peripheral clocks --------------------------------------------------*/
    /* Enable SPIy clock and GPIO clock for SPIy */
    RCC_AHBPeriphClockCmd( SPI_PIN_MISO_PORT_CLK | SPI_PIN_MOSI_PORT_CLK |
                            SPI_PIN_SCK_PORT_CLK, ENABLE );
    RCC_APB2PeriphClockCmd( SPI_CLK, ENABLE );

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_PinAFConfig( SPI_PIN_SCK_PORT, SPI_PIN_SCK_AF_SOURCE, SPI_PIN_SCK_AF );
    GPIO_PinAFConfig( SPI_PIN_MOSI_PORT, SPI_PIN_MOSI_AF_SOURCE, SPI_PIN_MISO_AF );
    GPIO_PinAFConfig( SPI_PIN_MISO_PORT, SPI_PIN_MISO_AF_SOURCE, SPI_PIN_MOSI_AF );

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

#else
    /* Enable peripheral clocks --------------------------------------------------*/
    /* Enable SPIy clock and GPIO clock for SPIy */
    RCC_APB2PeriphClockCmd( SPI_PIN_MISO_PORT_CLK | SPI_PIN_MOSI_PORT_CLK |
                            SPI_PIN_SCK_PORT_CLK, ENABLE );
    RCC_APB1PeriphClockCmd( SPI_CLK, ENABLE );

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
#endif

    GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK;
    GPIO_Init( SPI_PIN_SCK_PORT, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = SPI_PIN_MOSI;
    GPIO_Init( SPI_PIN_MOSI_PORT, &GPIO_InitStructure );

#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32L1XX_HD )
#else
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
#endif
    GPIO_InitStructure.GPIO_Pin = SPI_PIN_MISO;
    GPIO_Init( SPI_PIN_MISO_PORT, &GPIO_InitStructure );

#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32L1XX_HD )
#else
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );
    GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable, ENABLE );
#endif

    /* SPI_INTERFACE Config -------------------------------------------------------------*/
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    //SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // 5 MHz
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // 10 MHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( SPI_INTERFACE, &SPI_InitStructure );
    SPI_Cmd( SPI_INTERFACE, ENABLE );
}

uint8_t SpiInOut( uint8_t outData )
{
    /* Send SPIy data */
    SPI_I2S_SendData( SPI_INTERFACE, outData );
    while( SPI_I2S_GetFlagStatus( SPI_INTERFACE, SPI_I2S_FLAG_RXNE ) == RESET );
    return SPI_I2S_ReceiveData( SPI_INTERFACE );
}

