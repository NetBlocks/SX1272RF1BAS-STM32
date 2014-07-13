#include "bleeper.h"

#include "led.h"

GPIO_TypeDef* LedPort[LED_NB] = { 
                                LED1_GPIO_PORT,
                                LED2_GPIO_PORT,
                                LED3_GPIO_PORT,
#ifndef STM32L1XX_HD
                                LED4_GPIO_PORT,
#if defined( STM32F4XX ) || defined( STM32F2XX )
                                LED5_GPIO_PORT,
#endif
#endif
                                };
const uint16_t LedPin[LED_NB] = {
                                LED1_PIN,
                                LED2_PIN,
                                LED3_PIN,
#ifndef STM32L1XX_HD
                                LED4_PIN,
#if defined( STM32F4XX ) || defined( STM32F2XX )
                                LED5_PIN
#endif 
#endif
                                };
const uint32_t LedClk[LED_NB] = {
                                LED1_GPIO_CLK,
                                LED2_GPIO_CLK,
                                LED3_GPIO_CLK,
#ifndef STM32L1XX_HD
                                LED4_GPIO_CLK,
#if defined( STM32F4XX ) || defined( STM32F2XX )
                                LED5_GPIO_CLK
#endif
#endif
                                };

void LedInit( tLed led )
{
    GPIO_InitTypeDef  GPIO_InitStructure;

#if defined( STM32F2XX ) || defined( STM32F4XX )
    
    RCC_AHB1PeriphClockCmd( LedClk[led], ENABLE );

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

#elif defined( STM32L1XX_HD )

    RCC_AHBPeriphClockCmd( LedClk[led], ENABLE );

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

#else

    RCC_APB2PeriphClockCmd( LedClk[led], ENABLE );

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#endif

    GPIO_InitStructure.GPIO_Pin = LedPin[led];
    GPIO_Init( LedPort[led], &GPIO_InitStructure );
    GPIO_WriteBit( LedPort[led], LedPin[led], LED_OFF );
}

void LedOn( tLed led )
{
    GPIO_WriteBit( LedPort[led], LedPin[led], LED_ON );
}

void LedOff( tLed led )
{
    GPIO_WriteBit( LedPort[led], LedPin[led], LED_OFF ); 
}

void LedToggle( tLed led )
{
    LedPort[led]->ODR ^= LedPin[led]; 
}

#if defined( STM32L1XX_HD )
void LedStandBy( tLed led )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHBPeriphClockCmd( LedClk[led], ENABLE );
    
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // open-drain output
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_Pin = LedPin[led];
    GPIO_Init( LedPort[led], &GPIO_InitStructure );

}

void LedSleep( tLed led )
{
    GPIO_InitTypeDef GPIO_InitStructure;
        
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_PuPd_UP; // saves lots of power
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_Pin = LedPin[led];
    GPIO_Init( LedPort[led], &GPIO_InitStructure );

}
#endif
