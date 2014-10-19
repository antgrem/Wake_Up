#include "main.h"

#include "stm32f10x_pwr.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "ffconf.h"
#include "diskio.h"
#include "ff.h"
#include "sd_spi_stm32.h"

#include "usb_istr.h"


void Led_Blink(uint16_t how_much, uint16_t period);
void Spi_Set_Speed(uint8_t Speed);
void SPI_Config(void);
void wait(uint16_t time);
void InitClk(void);

uint16_t Led_Flag;
uint8_t button_press, state;
char    buff[1024];


int main(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t i;
	FRESULT result;
	FATFS FATFS_Obj;
	FIL file;
	UINT nWritten;
	DSTATUS res;
	
	//SysConfig();
	InitClk();
	
// init Wake_up pin 
	PWR->CSR |= PWR_CSR_EWUP;
	
	Interrupt_Button_Config ();
	
	SysTick_Config(SystemFrequency / 1000);
	
	
//Init LED	
			RCC_APB2PeriphClockCmd(LED_GPIO_CLK, ENABLE);
	  	GPIO_InitStructure.GPIO_Pin = LED_PIN;
  		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  	GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
	
	Set_System();
//	Set_USBClock();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USB_Interrupts_Config();
	USB_Init();
	
//			SPI_Config();
			
			for(i=0; i<1024; i++)
				buff[i] = (char) (i & 0xFF);
				
//			res = disk_initialize(0);
			
	//		Led_Blink(2, 100);
			
//			result = f_mount(0, &FATFS_Obj);
//        if (result != FR_OK)
//        {
//                while(1);
//        }
				
				// create file write.txt
//        result = f_open(&file, "write.txt", FA_CREATE_ALWAYS | FA_WRITE);
//        if (result == FR_OK)
//        {
//                f_write(&file, &buff, 1023, &nWritten);
//                f_close(&file);
// 						if (nWritten == 0)
// 							Led_Blink(2, 100);
//         }
				
				
			
	while(1)
	{
		
		//		Led_Blink(5, 100);
		//		USART_To_USB_Send_Data('f');
		
		wait(200);
			
//		if (GPIO_ReadInputDataBit(SD_CD_GPIO_PORT, SD_CD_PIN) != 0)
///			LED_SET_ON();
//		else LED_SET_OFF();
		
//		PWR_EnterSTANDBYMode();
//		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
				
	

	}

}


void SPI_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(SD_SPI_SCK_GPIO_CLK | SD_CD_GPIO_CLK, ENABLE);
	
	//Chip Select
	GPIO_InitStructure.GPIO_Pin = SD_CS_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SD_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SD_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SD_SPI_DI_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SD_SPI_DI_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SD_SPI_DO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SD_SPI_DO_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SD_CD_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(SD_CD_GPIO_PORT, &GPIO_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	SPI_Cmd(SPI2, ENABLE);
	SPI_StructInit(&SPI_InitStructure);
	/* Initialize the SPI_Direction member */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  /* initialize the SPI_Mode member */
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  /* initialize the SPI_DataSize member */
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  /* Initialize the SPI_BaudRatePrescaler member */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  /* Initialize the SPI_FirstBit member */
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  /* Initialize the SPI_CRCPolynomial member */
  SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_CPHA = 0;
	SPI_InitStructure.SPI_CPOL = 0;
	
	SPI_Init(SPI2, &SPI_InitStructure);
}

void Spi_Set_Speed(uint8_t Speed)
{
	uint16_t tmpreg = 0;
	assert_param(IS_SPI_BAUDRATE_PRESCALER(Speed));
	
	tmpreg = SPI2->CR1;
	tmpreg &= ~(SPI_BaudRatePrescaler_256);// Clear prescaler
	tmpreg |= Speed;

}


void Interrupt_Button_Config (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the BUTTON Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect Button EXTI Line to Button GPIO Pin */
//    SYSCFG_EXTILineConfig(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);

    /* Configure Button EXTI line */
	  EXTI_DeInit();
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 
}


void SysConfig(void)
{
  /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits USB_Pre on*/
  RCC->CFGR |= (uint32_t)0x055C0000;
  
  /* Reset HSI_ON, CSS_ON and PLL_ON bits */
  RCC->CR |= (uint32_t)0x01080001;

  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
//  RCC->CFGR &= (uint32_t)0xFF80FFFF;

  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000;

}

//********************************************************************************
//Function: настройкa системы тактирования контроллера STM32F103xx              //
//          источник сигнала - внутренний RC генератор через умножитель         //
//********************************************************************************
void InitClk(void)
{
  //Частота  SystemCoreClock выше 24 MHz - разрешить буфер предварительной выборки FLASH
  FLASH->ACR|=  FLASH_ACR_PRFTBE;        //Включить буфер предварительной выборки
  FLASH->ACR&= ~FLASH_ACR_LATENCY;       //Очистить FLASH_ACR_LATENCY
  FLASH->ACR |= FLASH_ACR_LATENCY_1;     //Пропускать 1 такт

  //Настройка PLL 
  RCC->CFGR  &= ~RCC_CFGR_PLLSRC;        //Источником сигнала для PLL выбран HSI с делением на 2
  RCC->CR   &= ~RCC_CR_PLLON;            //Отключить генератор PLL
  RCC->CFGR &= ~RCC_CFGR_PLLMULL;        //Очистить PLLMULL
  RCC->CFGR |=  RCC_CFGR_PLLMULL12;      //Коефициент умножения = 12
	RCC->CFGR |=  RCC_CFGR_USBPRE; 					//Предделитель для USB равен 1
  RCC->CR   |=  RCC_CR_PLLON;            //Включить генератор PLL
  while((RCC->CR & RCC_CR_PLLRDY)==0) {} //Ожидание готовности PLL

  //Переключиться на тактирование от PLL
  RCC->CFGR &= ~RCC_CFGR_SW;             //Очистка битов выбора источника тактового сигнала
  RCC->CFGR |=  RCC_CFGR_SW_PLL;         //Выбрать источником тактового сигнала PLL
  while((RCC->CFGR&RCC_CFGR_SWS)!=0x08){}//Ожидание переключения на PLL
  //Настроить делитель для шины APB1
  RCC->CFGR &= ~RCC_CFGR_PPRE1;          //Очистка битов предделителя "APB1 Prescaler"
  RCC->CFGR |=  RCC_CFGR_PPRE1_DIV2;     //Установить "APB1 Prescaler" равным 2

  //Настроить делитель для ADC
//  RCC->CFGR &= ~CFGR_ADCPRE;             //Очистка битов предделителя "ADC Prescaler"
//  RCC->CFGR |=  CFGR_ADCPRE_DIV4;        //Установить "ADC Prescaler" равным 4

}




void Led_Blink(uint16_t how_much, uint16_t period)
{
	Led_Flag = 0;
	while (how_much != 0)
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_7);
			
			while (Led_Flag != 20)
				{};
					
			Led_Flag = 0;
			GPIO_ResetBits(GPIOB, GPIO_Pin_7);
					
			while (Led_Flag != period)
				{};
					
			Led_Flag = 0;
			how_much--;	
		};
}

void wait(uint16_t time)
{
	Led_Flag = 0;
	
		while (Led_Flag != time)
				{};
}



