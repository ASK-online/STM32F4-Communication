#include "stm32f407xx.h"
#include <stdint.h>


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi){ // enable or disable
	
if(EnorDi == ENABLE){
	
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();     // 0. BITINI 1 YAPAR
			}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();     
			}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
			}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
			}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
			}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
			}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
			}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
			
			}
		
}
}	

// init de-init

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){ 

	
GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE); // sonradan ekledim uygulamada yazmamak için
	
	
uint32_t temp = 0;  
	
	//interrupt olmayan mod
	
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){ // analog 3' esit 
	
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // 2 ILE ÇARPARIZ 2 BIT OLDUGU IÇIN 
	  pGPIOHandle->pGPIOx->MODER &= ~ ( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //0 dan 3 e kadar yani 4 pini sileriz. clear
		pGPIOHandle->pGPIOx->MODER |=  temp; // SET EDERIZ
	
		}	
			
//interrupt  mod
	else{
		
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT ){  //FT
			
		EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			
		EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // rtsr bit clear yapar
			
		}
		
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){ // RT
			
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			
		}
	
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){  //RFT
		
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			
			
		} // hiçbiri degilse asagidaki komutlari aktif et
	
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		
		SYSCFG_PCLK_EN();
		
		SYSCFG->EXTICR[temp1] = portcode << (temp2 *4);
	
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	
temp=0;
	
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~ ( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |=  temp;
	
	
temp=0;
	
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~ ( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |=  temp;
	
	
temp=0;
	
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~ ( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	
	

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		
		uint8_t temp1 ,temp2;
		
		temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4*temp2 ));
		
	}

	}
	
	
	
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	

		if(pGPIOx == GPIOA){
			GPIOA_REG_RESET();     
			}
		else if(pGPIOx == GPIOB){
			GPIOB_REG_RESET();     
			}
		else if(pGPIOx == GPIOC){
			GPIOC_REG_RESET();
			}
		else if(pGPIOx == GPIOD){
			GPIOD_REG_RESET();
			}
		else if(pGPIOx == GPIOE){
			GPIOE_REG_RESET();
			}
		else if(pGPIOx == GPIOF){
			GPIOF_REG_RESET();
			}
		else if(pGPIOx == GPIOG){
			GPIOG_REG_RESET();
			}
		else if(pGPIOx == GPIOH){
			GPIOH_REG_RESET();
			}

	
}



//read - write

uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber){
	
	uint8_t value;       
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	
	
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	
	uint16_t value;   
	
	value = (uint16_t)(pGPIOx->IDR);
	
	
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber, uint8_t value ){
	
	if(value == GPIO_PIN_SET){
		
	pGPIOx->ODR ^= ( 1 << PinNumber); // XOR YAPARIZ 
}
	
	else{
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
	
	
	}	

	
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber, uint16_t value){
	
	pGPIOx->ODR=value;	
	
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber){


	pGPIOx->ODR ^=  (1 << PinNumber);
	
}





void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint64_t IRQPriority , uint8_t EnorDi){

if(EnorDi == ENABLE){

	if(IRQNumber <=31){
		*NVIC_ISER0 |= (1<<IRQNumber); 
	}
	
else if(IRQNumber > 31 && IRQNumber < 64 ){ // 32 TO 63	
	*NVIC_ISER1 |= (1<<IRQNumber); 
}
else if(IRQNumber >= 64 && IRQNumber < 96 ){
	*NVIC_ISER3 |= (1<<IRQNumber); 
	
}


}
else{
	
	
	if(IRQNumber <=31){
		*NVIC_ICER0 |= (1<<IRQNumber); 
	}
	
else if(IRQNumber > 31 && IRQNumber < 64 ){ // 32 TO 63	
	*NVIC_ICER1 |= (1<<IRQNumber); 
}
else if(IRQNumber >= 64 && IRQNumber < 96 ){
	*NVIC_ICER2 |= (1<<IRQNumber); 
	
}
}	
}				
		

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){

	uint8_t iprx = IRQNumber /4;
	uint8_t iprx_section = IRQNumber %4;
	*(NVIC_PR_BASE_ADDR + iprx*4) |= (IRQPriority << ( 8 * iprx_section) );

}

void GPIO_IRQHandling(uint8_t PinNumber){

if ( EXTI->PR & (1 << PinNumber)) {
EXTI->PR |= (1 << PinNumber);

}
}


	