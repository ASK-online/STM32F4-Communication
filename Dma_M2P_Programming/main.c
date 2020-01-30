#include <stdint.h>
#include "stm32f446xx.h"

//prototipler
void button_init(void);
void uart2_init(void);
void dma1_init(void);
void send_data(void);
void enable_dma1_stream6(void);
void disable_dma1_stream6(void);
void dma1_interrupt_configuration(void);

//callback prototipler
void HT_Complete_callback(void);
void FT_Complete_callback(void);
void TE_error_callback(void);
void DME_error_callback(void);
void FE_error_callback(void);

#define BASE_GPIOC_PERI   GPIOC

char data[] ="SELAMMM\r\n";



int main(){
	
	button_init();
	uart2_init();
	dma1_init();
	dma1_interrupt_configuration();
	enable_dma1_stream6();
	


	
	while(1){	
		
	}
	
	return 0;
	
}



void send_data(void){
	
	char data[]="SELAMMM\n";
	
		
	USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	uint32_t len=sizeof(data);
	
	for(uint32_t i= 0; i<len; i++){
		
	while (! (pUART2->SR & (1 << 7))); //Flag TX bos isse devam et
	
	pUART2->DR = data[i];
		
}
}

void button_init(void){
	
	GPIO_TypeDef *pGPIOC;
	pGPIOC=BASE_GPIOC_PERI;
	
	RCC_TypeDef *pRCC;
  pRCC = RCC;
	
	EXTI_TypeDef *pEXTI;
	pEXTI = EXTI;
	
	SYSCFG_TypeDef *pSYSCFG;
	pSYSCFG = SYSCFG;
	
	//GPIOC peripheral	
	pRCC->AHB1ENR |= (1 << 2);
	
	//GPIO Pin input mode
	pGPIOC->MODER &= ~(0x3 << 26 ); // 26 ve 27 pini temizler input mode olmasi için 00 olmasi gerekir her zaman
	
	pEXTI->FTSR |= (1 << 13); //düsen kenarin 13. pini 1
	
	pEXTI->IMR |= (1 << 13);// interrupt mask register
	
	NVIC_EnableIRQ(EXTI15_10_IRQn); // 13. pin için
	
	//enable clock for SYSCFG
	pRCC->APB2ENR |= (1 << 14);
	
	//SYSCFG CR4 register
	pSYSCFG->EXTICR[3] &= ~(0xF << 4); //temizle
	pSYSCFG->EXTICR[3] |=  (0x2 << 4);  // set
			
}

void uart2_init(void)
{
	RCC_TypeDef *pRCC;
	pRCC = RCC;
	
	GPIO_TypeDef *pGPIOA;
	pGPIOA = GPIOA;
	
	USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	// enable uart2 
	pRCC->APB1ENR |= ( 1 << 17);
  //GPIOA peri
	pRCC->AHB1ENR |= ( 1 << 0);
  //PA2 ALTERNATIF FONKSIYON MOD
	pGPIOA->MODER &= ~(0x3 << 4); 
	pGPIOA->MODER |= (0x2 << 4);
	pGPIOA->AFR[0] &= ~( 0xF << 8);
	pGPIOA->AFR[0] |= ( 0x7 << 8);
  
 //pull up DIRENÇ GPIOA 2
  pGPIOA->PUPDR |= (0x1 << 4);

  //PA23 ALTERNATIF FONKSIYON MOD
	pGPIOA->MODER  &= ~(0x3 << 6); 
	pGPIOA->MODER  |=  (0x2 << 6);
	pGPIOA->AFR[0] &= ~( 0xF << 12);
	pGPIOA->AFR[0] |=  ( 0x7 << 12);
  
  //pull up DIRENÇ GPIOA 3	
	pGPIOA->PUPDR |= (0x1 << 6);
	
	//3 . Configure the baudrate 
	pUART2->BRR = 0x8B;
	
	//5.  Enable TX 
	pUART2->CR1 |= ( 1 << 3);
	
	//6.  Enable UART PERIPHERAL
	pUART2->CR1 |= ( 1 << 13);
		
}

void dma1_init(void)
{
	RCC_TypeDef *pRCC;
	pRCC = RCC;
	
	DMA_TypeDef *pDMA;
	pDMA=DMA1;
	
	DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	
	USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	//enable peripheral clock DMA1
	pRCC->AHB1ENR |= ( 1 << 21);
	
	//channel 4
	pSTREAM6->CR &= ~( 0x7 << 25);
	pSTREAM6->CR |=  ( 0x4 << 25);	
	
	// adresi memory
	pSTREAM6->M0AR = (uint32_t) data;
	
	//destination address peripheral
	pSTREAM6->PAR = (uint32_t) &pUART2->DR;
	
	//Program number of data items to send 
	uint32_t len = sizeof(data);
	pSTREAM6->NDTR = len;
	
	//M2P
	pSTREAM6->CR |= (0x1 << 6);
	
	//destination data width 
	pSTREAM6->CR &= ~(0x3 << 13); 
	pSTREAM6->CR &= ~(0x3 << 11);
	
	//enable memory oto artir
	pSTREAM6->CR |= ( 1 << 10);
	
	//direct mode or fifo mode 
	pSTREAM6->FCR |= ( 1 << 2);
	
	//Select fifo
	pSTREAM6->FCR &= ~(0x3 << 0); //clearing 
	pSTREAM6->FCR |=  (0x3 << 0); //setting
		
}

void  disable_dma1_stream6(void)
{
	DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	//Enable the stream
	pSTREAM6->CR &= ~( 1 << 0);
}

void enable_dma1_stream6(void)
{
	DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	//Enable the stream
	pSTREAM6->CR |= ( 1 << 0);
}

void dma1_interrupt_configuration(void)
{
	DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	
	//Half-transfer IE (HTIE)
	pSTREAM6->CR |= (1 << 3);
	
	//Transfer complete IE (TCIE)
	pSTREAM6->CR |= (1 << 4);
	
	//Transfer error IE (TEIE)
	pSTREAM6->CR |= (1 << 2);
	
	//FIFO overrun/underrun IE (FEIE)
	pSTREAM6->FCR |= (1 << 7);
	
	//Direct mode error (DMEIE)
	pSTREAM6->CR |= (1 << 1);
	
	//Enable IRQ for DMA1 stream6
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	
}

void HT_Complete_callback(void)
{

}

void FT_Complete_callback(void)	
{
	
	USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	 
	uint32_t len = sizeof(data);
	pSTREAM6->NDTR = len;
	
	pUART2->CR3 &= ~( 1 << 7);
	
	enable_dma1_stream6();
		
}



void TE_error_callback(void)
{
	while(1);
}
void FE_error_callback(void)
{
	
	while(1);
}
void DME_error_callback(void)
{
	while(1);
}
