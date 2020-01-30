#include "stm32f407xx.h"

#include <stdio.h>
#include <string.h>

char msg[1024] = "AHMET SALIH KANTAR\n\r";


USART_Handle_t usart2; 


void USART2_Init(void)
{
	usart2.pUSARTx = USART2;
	usart2.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2);
}

void 	USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpio;

	usart_gpio.pGPIOx = GPIOA;
	usart_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpio.GPIO_PinConfig.GPIO_PinAltFunMode =7;

	//USART2 TX
	usart_gpio.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpio);
	//USART2 RX
	usart_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpio);


}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIO_Btn,Gpio_Led;

	GPIO_Btn.pGPIOx = GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIO_Btn);

	Gpio_Led.pGPIOx = GPIOD;
	Gpio_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	Gpio_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpio_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpio_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	Gpio_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&Gpio_Led);

}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

	GPIO_ButtonInit();
	USART2_GPIOInit();
  USART2_Init();
  USART_PeripheralControl(USART2,ENABLE);

    while(1)
    {
		
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) ); // butona basana kadar bekle

		delay();

		USART_SendData(&usart2,(uint8_t*)msg,strlen(msg));

    }

	return 0;
}
