#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"
#include <stdio.h>
#include "system_stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"


#define SOUND_PLUS_PIN 						GPIO_Pin_14
#define SOUND_PLUS_GPIO_PORT			GPIOC
#define SOUND_PLUS_GPIO_CLK       RCC_APB2Periph_GPIOC
#define SOUND_MINUS_PIN 					GPIO_Pin_15
#define SOUND_MINUS_GPIO_PORT			GPIOC
#define SOUND_MINUS_GPIO_CLK      RCC_APB2Periph_GPIOC

#define MOTOR_PIN 								GPIO_Pin_9
#define MOTOR_GPIO_PORT						GPIOB
#define MOTOR_GPIO_CLK      			RCC_APB2Periph_GPIOB

#define LED_PIN 								GPIO_Pin_7
#define LED_GPIO_PORT						GPIOB
#define LED_GPIO_CLK      			RCC_APB2Periph_GPIOB
#define LED_SET_ON()						GPIO_SetBits(LED_GPIO_PORT, LED_PIN)
#define LED_SET_OFF()						GPIO_ResetBits(LED_GPIO_PORT, LED_PIN)


#define USB_DP_PIN 								GPIO_Pin_12
#define USB_DP_GPIO_PORT					GPIOA
#define USB_DP_GPIO_CLK      			RCC_APB2Periph_GPIOA
#define USB_DM_PIN 								GPIO_Pin_11
#define USB_DM_GPIO_PORT					GPIOA
#define USB_DM_GPIO_CLK      			RCC_APB2Periph_GPIOA


#define SD_SPI								SPI2
#define SD_SPI_CLK						RCC_APB1Periph_SPI2

#define SD_SPI_SCK_PIN				GPIO_Pin_13
#define SD_SPI_SCK_GPIO_PORT	GPIOB
#define SD_SPI_SCK_GPIO_CLK		RCC_APB2Periph_GPIOB

#define SD_SPI_DO_PIN					GPIO_Pin_14
#define SD_SPI_DO_GPIO_PORT		GPIOB
#define SD_SPI_DO_GPIO_CLK		RCC_APB2Periph_GPIOB

#define SD_SPI_DI_PIN					GPIO_Pin_15
#define SD_SPI_DI_GPIO_PORT		GPIOB
#define SD_SPI_DI_GPIO_CLK		RCC_APB2Periph_GPIOB

#define SD_CS_PIN					GPIO_Pin_12
#define SD_CS_GPIO_PORT		GPIOB
#define SD_CS_GPIO_CLK		RCC_APB2Periph_GPIOB

#define SD_CD_PIN					GPIO_Pin_8
#define SD_CD_GPIO_PORT		GPIOA
#define SD_CD_GPIO_CLK		RCC_APB2Periph_GPIOA





extern void SysConfig(void);
void Interrupt_Button_Config (void);

#endif
