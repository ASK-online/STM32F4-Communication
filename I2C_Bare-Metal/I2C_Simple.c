#include "stm32f4xx.h"

// SCL - PB8
// SDA - PB9

#define SLAVE_ADDR 0x68

void I2C_Init(void);
int I2C_Read(char saddr , char maddr , char *data);


int main(void){
	
	I2C_Init();
	char data[222]="ahmet salih ";
	
	while(1){
		I2C_Read(SLAVE_ADDR , 0 ,data);		
	}

}

void I2C_Init(void){
	
	RCC->AHB1ENR |= 2; // GPIOB
	RCC->APB1ENR |= 0x00200000; // enable clock I2C1
	
	GPIOB->MODER &= ~0x000F0000; 
	GPIOB->MODER |=	 0x000A0000; 
	
	GPIOB->AFR[1] &= ~0x000000FF;
	GPIOB->AFR[1] |= ~0x00000044;
	GPIOB->OTYPER |= ~0x00000300; //Open drain
	
	GPIOB->PUPDR &=~ 0x000F0000; // pull -ups
	GPIOB->PUPDR |=  0x00050000;
	
	//I2C config
	 
	I2C1->CR1 = 0x8000; // software reset
	I2C1->CR1  &=~0x8000;
	I2C1->CR2 = 0x0010;
	I2C1->CCR = 80; //standart mode 100kHz clock
	I2C1->TRISE = 17;
	I2C1->CR1 |= 0x0001;	
}

int I2C_Read(char saddr , char maddr , char *data){
		
	
	volatile int temp;
	
	while(I2C1->SR2 & 2){} // mesgul olmadigindan emin ol
		
		I2C1->CR1 |= 0x100; //start
		while(! (I2C1->SR1 & 1)) {}
			
		I2C1->DR = saddr << 1; 
		while(!(I2C1->SR1 & 2)){} 
		temp = I2C1->SR2;
			
			while(!(I2C1->SR1 & 0x80)){}
			I2C1->DR = maddr;
			while(!(I2C1->SR1 & 0x80)){}
	
				
I2C1->CR1 |= 0x100; // START
				while(! (I2C1->SR1 & 1)) {} // START BITI KONTROL EDILIR
				I2C1->DR = saddr << 1 ;
					while(!(I2C1->SR1 & 2)){}  // ADDR AKTIF ISE
						I2C1->CR1 &= ~0x400; // ack biti 0
						temp= I2C1->SR2;
I2C1->CR1 |= 0x200; // STOP
						
				while(!(I2C1->SR1 & 0x40)){}
					*data++ = I2C1->DR;
					
		
					
				return 0;		
}