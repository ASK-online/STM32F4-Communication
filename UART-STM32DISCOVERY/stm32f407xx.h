#ifndef  _STM32F407XX_H_
#define  _STM32F407XX_H_

#include  <stdint.h>
#define  __vo  volatile 


#define  NVIC_ISER0    ( (__vo uint32_t*) 0xE000E100 )
#define  NVIC_ISER1    ( (__vo uint32_t*) 0xE000E104 )
#define  NVIC_ISER2    ( (__vo uint32_t*) 0xE000E108 )
#define  NVIC_ISER3    ( (__vo uint32_t*) 0xE000E10C )
#define  NVIC_ICER0    ( (__vo uint32_t*) 0xE000E180 )
#define  NVIC_ICER1    ( (__vo uint32_t*) 0xE000E184 )
#define  NVIC_ICER2    ( (__vo uint32_t*) 0xE000E188 )
#define  NVIC_ICER3    ( (__vo uint32_t*) 0xE000E18C )
#define NVIC_PR_BASE_ADDR  ( (__vo uint32_t*)0xE000E400 )
#define NO_PR_BITS_IMPLEMENTED       4


#define FLASH_BASEADDR   0X08000000U  // ADRES DEGERINI FLASH_BASEADDR YAPAR
#define SRAM1_BASEADDR   0X20000000U  // SRAM1 114KB SIZE'I VAR
#define SRAM2_BASEADDR   0X20001C00U  //SRAM1+SIZEOF(SRAM1)
#define ROM_BASEADDR	   0X1FFF0000U
#define SRAM						 SRAM1_BASEADDR


#define PERIPH_BASE      0x40000000U
#define AHB1PERIPH_BASE  PERIPH_BASE
#define AHB2PERIPH_BASE  0x40010000U 

#define APB1PERIPH_BASE	 0x40020000U
#define APB2PERIPH_BASE	 0x50000000U



#define GPIOA_BASEADDR   (AHB1PERIPH_BASE + 0X0000)  //   (AHB1PERIPH_BASE+OFFSET)
#define GPIOB_BASEADDR   (AHB1PERIPH_BASE + 0X4000)  // hangi veriyolunu kullaniyorsa AHB1
#define GPIOC_BASEADDR   (AHB1PERIPH_BASE + 0X0800)
#define GPIOD_BASEADDR   (AHB1PERIPH_BASE + 0X0C00)
#define GPIOE_BASEADDR   (AHB1PERIPH_BASE + 0X1000)
#define GPIOF_BASEADDR   (AHB1PERIPH_BASE + 0X1400)
#define GPIOG_BASEADDR   (AHB1PERIPH_BASE + 0X1800)
#define GPIOH_BASEADDR   (AHB1PERIPH_BASE + 0X1C00)
#define GPIOI_BASEADDR   (AHB1PERIPH_BASE + 0X2000)

#define RCC_BASEADDR     (AHB1PERIPH_BASE + 0X3800)
#define EXTI_BASEADDR    (AHB1PERIPH_BASE + 0X3C00)
#define SYSCFG_BASEADDR  (AHB1PERIPH_BASE + 0X3800)



#define USART2_BASE    (APB1PERIPH_BASE + 0X4400)
#define USART3_BASE    (APB1PERIPH_BASE + 0X4800)
#define UART4_BASE     (APB1PERIPH_BASE + 0X4C00)
#define UART5_BASE     (APB1PERIPH_BASE + 0X5000)

#define EXT1_BASE      (APB2PERIPH_BASE + 0X3C00)
#define SPI1_BASE      (APB2PERIPH_BASE + 0X3000)
#define STSCFG_BASE    (APB2PERIPH_BASE + 0X3800)
#define USART1_BASE    (APB2PERIPH_BASE + 0X1000)
#define USART6_BASE    (APB2PERIPH_BASE + 0X1400)




typedef struct{

	  __vo uint32_t CR;
		__vo uint32_t PLLCFGR;
		__vo uint32_t CFGR;
		__vo uint32_t CIR;
		__vo uint32_t AHB1RSTR;
		__vo uint32_t AHB2RSTR;
		__vo uint32_t AHB3RSTR;
		uint32_t RESERVED0;
	  __vo uint32_t APB1RSTR;
		__vo uint32_t APB2RSTR;
		uint32_t RESERVED1[2];
		__vo uint32_t AHB1ENR;
		__vo uint32_t AHB2ENR;
		__vo uint32_t AHB3ENR;
		uint32_t RESERVED2;
	  __vo uint32_t APB1ENR;
		__vo uint32_t APB2ENR;
		uint32_t RESERVED3[2];
		__vo uint32_t AHB1LPENR;
		__vo uint32_t AHB2LPENR;
		__vo uint32_t AHB3LPENR;
		uint32_t RESERVED4;	
		__vo uint32_t APB1LPENR;
		__vo uint32_t APB2LPENR;
		uint32_t RESERVED5[2];
		__vo uint32_t BDCR;
		__vo uint32_t CSR;
		uint32_t RESERVED6[2];
		__vo uint32_t SSCGR;
		__vo uint32_t PLLI2SCFGR;
		__vo uint32_t PLLSAICFGR;
		__vo uint32_t DCKCFGR;
		__vo uint32_t CKGATENR;
		__vo uint32_t DCKCFGR2;
		
}RCC_RegDef_t;








typedef struct{
	
	__vo uint32_t MODER;
	__vo uint32_t OTYPER; 
	__vo uint32_t OSPEEDR; // SPEED
	__vo uint32_t PUPDR; // PULL UP/DOWN
	__vo uint32_t IDR; // INPUT DATA REGISTER
	__vo uint32_t ODR;  // OUTPUT DATA REGISTER
	__vo uint32_t BSRR; // BIT SET RESET
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2]; // AFR[0]=LOW AFR[1]=HIGH

}GPIO_RegDef_t;



typedef struct{

__vo uint32_t IMR;
__vo uint32_t EMR;
__vo uint32_t RTSR;
__vo uint32_t FTSR;
__vo uint32_t SWIER;
__vo uint32_t PR;
}EXTI_RegDef_t;


typedef struct{

__vo uint32_t MEMRMP;
__vo uint32_t PMC;
__vo uint32_t *EXTICR;
__vo uint32_t RESERVED1[2];
__vo uint32_t CMPCR;
__vo uint32_t RESERVED2[2];
__vo uint32_t CFGR;
	
}SYSCFG_RegDef_t;



typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
} USART_RegDef_t;



#define GPIOA  ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB  ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE  ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF  ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG  ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH  ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI  ((GPIO_RegDef_t*) GPIOI_BASEADDR)


#define RCC    ((RCC_RegDef_t*)    RCC_BASEADDR)
#define EXTI   ((EXTI_RegDef_t*)   EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)


#define USART1  			((USART_RegDef_t*)USART1_BASE)
#define USART2  			((USART_RegDef_t*)USART2_BASE)
#define USART3  			((USART_RegDef_t*)USART3_BASE)
#define UART4  				((USART_RegDef_t*)UART4_BASE)
#define UART5  				((USART_RegDef_t*)UART5_BASE)
#define USART6  			((USART_RegDef_t*)USART6_BASE)


// GPIO  ENABLE

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1<<7))



// USART-UART-SYSCFG ENABLE

#define USART1_PCCK_EN() (RCC->APB2ENR |= (1<<4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1<<17))
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1<<18))
#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1<<19))
#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1<<20))
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1<<5))
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1<<14))



//DISABLE

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~ (1<<2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~ (1<<3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~ (1<<4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~ (1<<5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~ (1<<6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~ (1<<7))

#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~ (1<<21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~ (1<<22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~ (1<<23))

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~ (1<<12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~ (1<<14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~ (1<<15))


#define USART1_PCCK_DI() (RCC->APB2ENR &= ~ (1<<4))
#define USART2_PCCK_DI() (RCC->APB1ENR &= ~ (1<<17))
#define USART3_PCCK_DI() (RCC->APB1ENR &= ~ (1<<18))
#define UART4_PCCK_DI()  (RCC->APB1ENR &= ~ (1<<19))
#define UART5_PCCK_DI()  (RCC->APB1ENR &= ~ (1<<20))
#define USART6_PCCK_DI() (RCC->APB1ENR &= ~ (1<<5))




#define GPIOA_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOC_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOD_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOE_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOF_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOG_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOH_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)



#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
                                        (x == GPIOB)?1:\
																				(x == GPIOC)?2:\
																				(x == GPIOB)?3:\
																				(x == GPIOA)?4:\
																				(x == GPIOB)?5:\
																				(x == GPIOA)?6:\
																				(x == GPIOC)?7:0 )




//IRQ MACRO 

#define IRQ_NO_EXTI0       6
#define IRQ_NO_EXTI1       7
#define IRQ_NO_EXTI2       8
#define IRQ_NO_EXTI3       9
#define IRQ_NO_EXTI4       10
#define IRQ_NO_EXTI9_5     23
#define IRQ_NO_EXTI15_10   40


#define NVIC_IRQ_PRI0      0
#define NVIC_IRQ_PRI15     15


//MACRO GENEL

#define ENABLE          1
#define DISABLE         0
#define SET          		ENABLE
#define RESET          	DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
#define FLAG_RESET      RESET
#define FLAG_SET        SET



#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15

#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11


#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_gpio_driver.h"



#endif 

