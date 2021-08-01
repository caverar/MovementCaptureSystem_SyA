#include "uart.h"
#include <stm32f103xb.h>

#include "string.h"
#include "stdlib.h"
#include "stdarg.h"


#include <limits.h>



void initUART(void){
    //--Inicialización UART1-------------------------------------------------------------------------------

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 					// USART1EN=1, Activación de reloj UART1
	USART1->CR1 |= USART_CR1_UE;       						// UE=1, Activación UART1
	USART1->CR1 &= ~USART_CR1_M;  							// M=0, Tamaño de palabra 8 bits
	USART1->CR2 &= ~USART_CR2_STOP;    						// STOP=0, Seleccionar el número de bits de parada

	//	Inicializacion de pines (GPIOA)

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;						// IOPAEN = 1, Habilitar reloj de GPIOA
	GPIOA->CRH |= GPIO_CRH_MODE9_0;    						// MODE9 = 01, PA9 como salida a 10 MHz
	GPIOA->CRH |= GPIO_CRH_CNF9_1;     						// CNF9 = 10, PA9 como salida push pull alternada (salida de periférico)
	GPIOA->CRH &= ~GPIO_CRH_MODE10;    						// MODE10 = 00, PA10 como entrada
	GPIOA->CRH |= GPIO_CRH_CNF10_1;    						// CNF10 = 10, PA10 como entrada push pull

	//	Baudios 115200, Baudios = 72MHz/16*USARTDIV, USARTDIV = 39.0625, BRR = 0x0271 

	USART1->BRR = (0x0271); 								// Parte entera y decimal del preescaler de Baudios
	USART1->CR1 |= USART_CR1_TE;							// TE=1, Habilitar transmisor
	USART1->CR1 |= USART_CR1_RE;							// RE=1, Habilitar receptor
	//USART1->CR1 |= USART_CR1_RXNEIE;						// RXNEIE=1, Habilitar interrupciones de recepción

	// Configuración de interrupciones
	USART1->CR1 |= USART_CR1_RXNEIE;						// Activar interrupcion para RX
	USART1->CR1 |= USART_CR1_TXEIE;							// Activar interrupcion para TX terminado


	//	Primera transmisión (vaciá)

	USART1->DR = (0x04);									// Primera transmisión basura
	while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmisión
	
	initDMA();												// Inicializar DMA en caso de que este seleccionado UART_COMPLEX_MODE
}



void initDMAComplexMode(void){
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;						// Activar reloj de DMA
	USART1->CR3 |= USART_CR3_DMAT;							// Habilitar DMA para transmisión
	

	DMA1_Channel4->CCR |= DMA_CCR_TCIE;						// Habilitar interrupcion de transferencia completada
	DMA1_Channel4->CCR |= DMA_CCR_MINC;						// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
	DMA1_Channel4->CCR |= DMA_CCR_DIR;						// Dirección de transferencia Memoria->Periférico

	//DMA1_Channel4		

	//EXTI->IMR |=  EXTI_IMR_MR1;                         	// Mask interrupcion de linea 1
    //EXTI->RTSR |= EXTI_RTSR_TR1;                        	// Habilitar trigger con rising edge
    //DMA1->IFCR;
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);                    	// Habilitar interrupciones del canal 4 DMA1 
	
	uartTxEmptyBufferFlag = 0;								// Limpiar lag de trasferencia en curso					

}


void DMA1_Channel4_IRQHandler(void){
	uartTxEmptyBufferFlag = 0;								// Limpiar lag de trasferencia en curso	

	USART1->SR |= USART_CR1_TXEIE;
	DMA1->IFCR |= DMA_IFCR_CTCIF7;							// Limpliar flag de interrupcion
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;						// Desactivar transferencia DMA


}

void complexPrint(char *msg, ...){

	if(uartTxEmptyBufferFlag == 0){
		char buffer[100];
		va_list args;
		va_start(args,msg);
		vsprintf(buffer,msg,args);
		uartTxEmptyBufferFlag = 1;							// Activar Flag de transferencia en curso

		DMA1_Channel4->CMAR = (int)&buffer;			    	// Dirección de memoria de buffer de lectura
		DMA1_Channel4->CPAR = (int)&USART1->DR;				// Dirección de periférico
		DMA1_Channel4->CNDTR = strlen(buffer);				// Numero de Bytes a ser transferidos
		DMA1_Channel4->CCR |= DMA_CCR_EN;					// Habilitar canal DMA
	}
}
unsigned char getUartTxEmptyBufferFlag(void){
	return uartTxEmptyBufferFlag;
}


void simplePrint(char *msg, ...){
	char buffer[100];
	va_list args;
	va_start(args,msg);
	vsprintf(buffer,msg,args);

	for(int i=0; i<strlen(buffer); i++){
		USART1->DR = buffer[i];
		while(!(USART1->SR & USART_SR_TC));
	}
}



char printSignedShort(unsigned char figure, short value){
	if(figure == 0){
		if(value<0){
			return ' ';
		}else{
			return '-';
		}
	}else if(figure == 1){
		return ((short)value/10000) + 0x48;
	}else if(figure == 2){
		return ((short)value/1000) + 0x48;
	}else{
		return ' ';
	}
}

