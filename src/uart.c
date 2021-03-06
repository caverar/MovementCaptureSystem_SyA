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

	//	Baudios 230400, Baudios = 72MHz/16*USARTDIV, USARTDIV = 19.05, BRR = 0x0138, real: 230769

	USART1->BRR = (0x0271); 								// Parte entera y decimal del preescaler de Baudios
	USART1->CR1 |= USART_CR1_TE;							// TE=1, Habilitar transmisor
	USART1->CR1 |= USART_CR1_RE;							// RE=1, Habilitar receptor

	// Configuración de interrupciones
	//USART1->CR1 |= USART_CR1_RXNEIE;						// Activar interrupcion para RX
	//USART1->CR1 |= USART_CR1_TXEIE;						// Activar interrupcion para TX terminado


	//	Primera transmisión (vaciá)

	//USART1->DR = (0x04);									// Primera transmisión basura
	//while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmisión

	initDataTransferMethod();								// Inicializar DMA o Interrupciones, según el modo seleccionado
	
	
}

void initSimpleMode(void){
}


// void initInterruptMode(void){
// 	USART1->CR1 |= USART_CR1_TXEIE;							// Habilitar interrupcion de Transmit data register empty
// 	NVIC_EnableIRQ(USART1_IRQn);                   			// Habilitar interrupciones de USART1

// }


void initDMA(void){
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;						// Activar reloj de DMA
	USART1->CR3 |= USART_CR3_DMAT;							// Habilitar DMA para transmisión
	DMA1_Channel4->CCR |= DMA_CCR_PL_Msk;					// Maxima prioridad

	NVIC_EnableIRQ(DMA1_Channel4_IRQn);                   	// Habilitar interrupciones del canal 4 DMA1	
	uartTxEmptyBufferFlag = 0;								// Limpiar flag de trasferencia en curso					

}


void DMA1_Channel4_IRQHandler(void){
	uartTxEmptyBufferFlag = 0;								// Limpiar lag de trasferencia en curso
	
	//while(!(USART1->SR & USART_SR_TC));							
	DMA1->IFCR |= DMA_IFCR_CTCIF4;							// Limpliar flag de interrupcion de transferencia completa
	DMA1->IFCR |= DMA_IFCR_CGIF4;							// Limpliar todods los flag de interrupcion  
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;						// Desactivar transferencia DMA

	
}

void USART1_IRQHandler(){

	if(USART1->SR & USART_SR_TXE){
		if(uartTxCounter>0){		
			USART1->DR = uartTxBuffer[bufferSize-uartTxCounter];
			
			uartTxCounter -= 1;
		}else{
			uartTxEmptyBufferFlag = 0;								// Desactivar Flag de transmisión
			USART1->CR1 &= ~USART_CR1_TXEIE;						// Habilitar interrupcion de Transmit data register empty
			NVIC_DisableIRQ(USART1_IRQn);                   		// Habilitar interrupciones de USART1
		}
	}
	//USART1->SR &= ~USART_SR_TC;									// Limpiar bandera de interrupcion
}


/** 
* @brief Buffer limitado a ocupación de DMA.
*/	
void complexPrint(char *msg, ...){

	if(uartTxEmptyBufferFlag == 0){

		va_list args;
		va_start(args,msg);
		vsprintf(uartTxBuffer,msg,args);
		uartTxEmptyBufferFlag = 1;							// Activar Flag de transferencia en curso
		bufferSize = strlen(uartTxBuffer);
		uartTxBuffer[bufferSize]=0x00;						// Carácter de final de trama
		bufferSize += 1;

		DMA1_Channel4->CMAR = (unsigned int)&uartTxBuffer;	// Dirección de memoria de buffer de lectura
		DMA1_Channel4->CPAR = (unsigned int)&USART1->DR;	// Dirección de periférico

		DMA1_Channel4->CNDTR = bufferSize;					// Numero de Bytes a ser transferidos
		DMA1_Channel4->CCR |= DMA_CCR_TCIE;					// Habilitar interrupcion de transferencia completada
		DMA1_Channel4->CCR |= DMA_CCR_MINC;					// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
		DMA1_Channel4->CCR |= DMA_CCR_DIR;					// Dirección de transferencia Memoria->Periférico
		//DMA1_Channel4->CCR |= DMA_CCR_PL_Msk;				// Maxima prioridad
		//DMA1_Channel4->CCR |= DMA_CCR_CIRC;				// Modo circular


		USART1->SR &= ~USART_SR_TC;							// Escribir 0 en TC, para limpiar registro tranfer complete, según datasheet.

		DMA1_Channel4->CCR |= DMA_CCR_EN;					// Habilitar canal DMA

		while(!(DMA1->ISR |= DMA_ISR_TCIF4));
		DMA1->IFCR |= DMA_IFCR_CTCIF4;						// Limpliar flag de interrupcion

						
		
	}
}


void simplePrint(char *msg, ...){

	va_list args;
	va_start(args,msg);
	vsprintf(uartTxBuffer,msg,args);
	bufferSize = strlen(uartTxBuffer);

	uartTxBuffer[bufferSize]=0x00;							// Carácter de final de trama
	bufferSize += 1;

	for(int i=0; i<bufferSize; i++){
		USART1->DR = uartTxBuffer[i];
		while(!(USART1->SR & USART_SR_TC));
	}
}



void interruptPrint(char *msg, ...){

	
	va_list args;
	va_start(args,msg);
	vsprintf(uartTxBuffer,msg,args);

	
	bufferSize = strlen(uartTxBuffer);
	uartTxBuffer[bufferSize]=0x00;							// Carácter de final de trama
	uartTxCounter = bufferSize + 1;							// Contador toma el valor del tamaño del arreglo
	bufferSize += 1;										// Reajustar tamaño del buffer


	USART1->DR = uartTxBuffer[bufferSize-uartTxCounter];
	uartTxCounter -= 1;
	uartTxEmptyBufferFlag = 1;								// Activar Flag de transferencia en curso

	USART1->CR1 |= USART_CR1_TXEIE;							// Habilitar interrupcion de Transmit data register empty
	NVIC_EnableIRQ(USART1_IRQn);                   			// Habilitar interrupciones de USART1

}


void pollingOptimalPrint(char *msg, ...){
	va_list args;
	va_start(args,msg);
	vsprintf(uartTxBuffer,msg,args);

	bufferSize = strlen(uartTxBuffer);
	uartTxBuffer[bufferSize]=0x00;							// Carácter de final de trama
	uartTxCounter = bufferSize + 1;							// Contador toma el valor del tamaño del arreglo
	bufferSize += 1;										// Reajustar tamaño del buffer

	USART1->DR = uartTxBuffer[bufferSize-uartTxCounter];
	uartTxCounter -= 1;
	uartTxEmptyBufferFlag = 1;								// Activar Flag de transferencia en curso

}

void pollingOptimalPrintManager(void){
	if(USART1->SR & USART_SR_TC){
		if(uartTxCounter>0){
			USART1->DR = uartTxBuffer[bufferSize-uartTxCounter];
			uartTxCounter -= 1;
		}else{
			uartTxEmptyBufferFlag = 0;						// Desactivar Flag de transmisión
		}
	} 					

}