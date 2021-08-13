#ifndef __I2C_H
#define __I2C_H

#include <stm32f103xb.h>
#include "i2c.h"
#include "timer4.h"

volatile unsigned char I2C_Aux = 0;

void initI2C(void){

    //--Inicialización I2C---------------------------------------------------------------------------------

    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;						// Habilitar reloj I2C2


    // Ajuste de pines (pin B11-SDA, B10-SCL)
    RCC->APB2ENR |=RCC_APB2ENR_IOPBEN;						// IOPBEN=1, Habilitar reloj del Puerto B
    GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);      // Limpiar registros de configuración PB10
    GPIOB->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);      // Limpiar registros de configuración PB10
    GPIOB->CRH |= GPIO_CRH_MODE10;							// MODE10=0b11, PB10 Modo salida 50 MHz
    GPIOB->CRH |= GPIO_CRH_CNF10;							// CNF10=0b11, PB10 Modo alternado open drain
    GPIOB->ODR |= GPIO_ODR_ODR10;							// ODR10=1 Habilitar resistencia de PullUp
    GPIOB->CRH |= GPIO_CRH_MODE11;							// MODE11=0b11, PB11 Modo salida 50 MHz
    GPIOB->CRH |= GPIO_CRH_CNF11;							// CNF11=0b11, PB11 Modo alternado open drain
    GPIOB->ODR |= GPIO_ODR_ODR11;							// ODR11=1 Habilitar resistencia de PullUp

    // Configuración de DMA
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;						// Habilitar reloj DMA
    //DMA1_Channel5->CMAR = (int)I2C_rxBuffer;				// Dirección de memoria
    //DMA1_Channel5->CPAR = (int)&I2C2->DR;					// Dirección de periférico
    //DMA1_Channel5->CNDTR = 3;								// Numero de Bytes a ser transferidos
    //DMA1_Channel5->CCR |= DMA_CCR_TCIE;					// Habilitar interrupcion de transferencia completada
    //DMA1_Channel5->CCR |= DMA_CCR_MINC;					// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
    //DMA1_Channel5->CCR |= DMA_CCR_CIRC;					// Activar modo circular
    //DMA1_Channel5->CCR |= DMA_CCR_EN;						// Habilitar canal DMA



    // Configuración de periférico
    //I2C2->CCR |= I2C_CCR_FS;								// Activar modo I2C de alta velocidad
    I2C2->CR2 |= 36;										// Frecuencia de Bus APB1
    I2C2->CCR |= 180;										// Ajuste de frecuencia i2c a 400 khz
    I2C2->TRISE |= 37;										// y otros ajustes de velocidad

    I2C2->CR1 |= I2C_CR1_ACK;								// Enable ACK
    I2C2->CR2 |= I2C_CR2_DMAEN;								// Habilitar llamado a DMA
    //I2C2->CR2 |= I2C_CR2_LAST;							// Habilitar NACK, cuando se llena el buffer DMA
    I2C2->CR1 |= I2C_CR1_PE;								// Habilitar periférico
}


void writeI2C(unsigned char deviceAddress ,unsigned char address, unsigned char size, unsigned char *data){

    I2C2->CR1 |= I2C_CR1_START;								// Secuencia de inicio
    while(!(I2C2->SR1 & I2C_SR1_SB));						// SB=1?, Esperar a que se complete la secuencia de inicio
    I2C2->DR = deviceAddress;							    // Escribir dirección de escritura de periférico
    while(!(I2C2->SR1 & I2C_SR1_ADDR));						// ADDR=1?, ACK de dirección de periférico
    I2C_Aux = I2C2->SR2;								    // Lectura basura para limpiar ADDR
    I2C2->DR = address;										// Escribir dirección de registro de periférico
    while(!(I2C2->SR1 & I2C_SR1_TXE));						// TX=1?, Esperar a que el registro de datos TX este vació (transmisión)

    for(int i=0; i<size;i++){
        I2C2->DR = data[i];									// Escribir dato
        while(!(I2C2->SR1 & I2C_SR1_TXE));					// TX=1?, Esperar a que el registro de datos TX este vació (transmisión)
    }
    I2C2->CR1 |= I2C_CR1_STOP;								// Secuencia de parada
    wait_us(4);										        // Espera de 200 us
}




unsigned char* readI2C(unsigned char deviceAddress, unsigned char address, unsigned char size){
    static unsigned char readData[30];
    if(size==1){
        // Este código aun no funciona correctamente, por ahora leer dos datos en vez de 1.

        I2C2->CR1 |= I2C_CR1_START;							// Secuencia de inicio
        while(!(I2C2->SR1 & I2C_SR1_SB));					// SB=1?, Esperar a que se complete la secuencia de inicio
        I2C2->DR = deviceAddress;						    // Escribir dirección de escritura de periférico
        while(!(I2C2->SR1 & I2C_SR1_ADDR));					// ADDR=1?, ACK de dirección de periférico
        I2C_Aux= I2C2->SR2;							        // Lectura basura para limpiar ADDR
        I2C2->DR = address;									// Escribir dirección de registro de periférico
        while(!(I2C2->SR1 & I2C_SR1_TXE));					// TX=1?, Esperar a que el registro de datos TX este vació (transmisión)
        I2C2->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
        wait_us(4);										    // Espera de 200 us

        // Reajuste DMA
        DMA1_Channel5->CMAR = (int)&readData;			    // Dirección de memoria de buffer de lectura
        DMA1_Channel5->CPAR = (int)&I2C2->DR;				// Dirección de periférico
        DMA1_Channel5->CNDTR = size;						// Numero de Bytes a ser transferidos
        DMA1_Channel5->CCR |= DMA_CCR_TCIE;					// Habilitar interrupcion de transferencia completada
        //DMA1_Channel5->CCR &= !DMA_CCR_MINC;				// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
        //DMA1_Channel5->CCR |= DMA_CCR_CIRC;				// Activar modo circular
        DMA1_Channel5->CCR |= DMA_CCR_EN;					// Habilitar canal DMA

        // Mandar instrucción de lectura y lectura de datos
        I2C2->CR1 &= !I2C_CR1_ACK;							// Disable ACK
        //I2C2->CR2 |= I2C_CR2_LAST;						// Habilitar NACK, cuando se llena el buffer DMA
        I2C2->CR1 |= I2C_CR1_START;							// Secuencia de inicio
        while(!(I2C2->SR1 & I2C_SR1_SB));					// SB=1?, Esperar a que se complete la secuencia de inicio
        I2C2->DR = deviceAddress + 1;					    // Escribir dirección de lectura de periférico
        while(!(I2C2->SR1 & I2C_SR1_ADDR));					// ADDR=1?, ACK de dirección de periférico
        I2C_Aux = I2C2->SR2;								// Lectura basura para limpiar ADDR
        while(!(DMA1->ISR & DMA_ISR_TCIF5));				// TCIF5=1?, Esperar a completar la transferencia de datos por DMA
        DMA1->IFCR |= DMA_IFCR_CTCIF5;						// TCIF5=1, Limpiar bandera de transferencia de datos por DMA completada
        //I2C2->CR1 &= !I2C_CR1_ACK;						// Disable ACK
        //I2C2->CR2 &= !I2C_CR2_LAST;						// Deshabilitar NACK, cuando se llena el buffer DMA
        I2C2->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
        DMA1_Channel5->CCR &= !DMA_CCR_EN;					// Deshabilitar canal DMA
        wait_us(4);										    // Espera de 4 us
    }else{
        // Solo funciona si size >=2

        I2C2->CR1 &= ~I2C_CR1_PE;							// Deshabilitar periférico
        I2C2->CR1 |= I2C_CR1_PE;							// Habilitar periférico

        // Mandar instrucción de escritura y dirección de lectura

        //I2C2->CR2 |= I2C_CR2_LAST;						// Habilitar NACK, cuando se llena el buffer DMA
        I2C2->CR1 |= I2C_CR1_START;							// Secuencia de inicio
        while(!(I2C2->SR1 & I2C_SR1_SB));					// SB=1?, Esperar a que se complete la secuencia de inicio
        I2C2->DR = deviceAddress;			                // Escribir dirección de escritura de periférico
        while(!(I2C2->SR1 & I2C_SR1_ADDR));					// ADDR=1?, ACK de dirección de periférico
        I2C_Aux= I2C2->SR2;							        // Lectura basura para limpiar ADDR
        I2C2->DR = address;									// Escribir dirección de registro de periférico
        while(!(I2C2->SR1 & I2C_SR1_TXE));					// TX=1?, Esperar a que el registro de datos TX este vació (transmisión)
        I2C2->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
        wait_us(4);										    // Espera de 200 us
        

        // Reajuste DMA
        DMA1_Channel5->CMAR = (unsigned int)&readData;		// Dirección de memoria
        DMA1_Channel5->CPAR = (unsigned int)&I2C2->DR;	    // Dirección de periférico
        DMA1_Channel5->CNDTR = size;						// Numero de Bytes a ser transferidos
        DMA1_Channel5->CCR |= DMA_CCR_TCIE;					// Habilitar interrupcion de transferencia completada
        DMA1_Channel5->CCR |= DMA_CCR_MINC;					// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
        

        //DMA1_Channel5->CCR |= DMA_CCR_CIRC;				// Activar modo circular
        DMA1_Channel5->CCR |= DMA_CCR_EN;					// Habilitar canal DMA

        // Mandar instrucción de lectura y lectura de datos
        I2C2->CR1 |= I2C_CR1_ACK;							// Enable ACK
        I2C2->CR2 |= I2C_CR2_LAST;							// Habilitar NACK, cuando se llena el buffer DMA
        I2C2->CR1 |= I2C_CR1_START;							// Secuencia de inicio
        while(!(I2C2->SR1 & I2C_SR1_SB));					// SB=1?, Esperar a que se complete la secuencia de inicio
        I2C2->DR = deviceAddress + 1;						// Escribir dirección de lectura de periférico
        while(!(I2C2->SR1 & I2C_SR1_ADDR));					// ADDR=1?, ACK de dirección de periférico
        I2C_Aux = I2C2->SR2;								// Lectura basura para limpiar ADDR
        while(!(DMA1->ISR & DMA_ISR_TCIF5));				// TCIF5=1?, Esperar a completar la transferencia de datos por DMA
        DMA1->IFCR |= DMA_IFCR_CTCIF5;						// TCIF5=1, Limpiar bandera de transferencia de datos por DMA completada
        //I2C2->CR1 &= !I2C_CR1_ACK;						// Disable ACK
        //I2C2->CR2 &= !I2C_CR2_LAST;						// Deshabilitar NACK, cuando se llena el buffer DMA
        I2C2->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
        DMA1_Channel5->CCR &= !DMA_CCR_EN;					// Deshabilitar canal DMA
        wait_us(4);										    // Espera de 4 us
    }

    return readData;
}

// Lectura I2C sin IDLE

void readI2C_part1(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C2->CR1 &= ~I2C_CR1_PE;							// Deshabilitar periférico
        I2C2->CR1 |= I2C_CR1_PE;							// Habilitar periférico

        // Mandar instrucción de escritura y dirección de lectura

        //I2C2->CR2 |= I2C_CR2_LAST;						// Habilitar NACK, cuando se llena el buffer DMA
        I2C2->CR1 |= I2C_CR1_START;							// Secuencia de inicio

        // Siguiente paso:  while(!(I2C2->SR1 & I2C_SR1_SB)) SB=1?, Esperar a que se complete la secuencia de inicio
    }
}

void readI2C_part2(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C2->DR = deviceAddress;			                // Escribir dirección de escritura de periférico

        // Siguiente paso:  while(!(I2C2->SR1 & I2C_SR1_ADDR))  ADDR=1?, ACK de dirección de periférico
    }
}

void readI2C_part3(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C_Aux= I2C2->SR2;							        // Lectura basura para limpiar ADDR
        I2C2->DR = address;									// Escribir dirección de registro de periférico
        while(!(I2C2->SR1 & I2C_SR1_TXE));					// TX=1?, Esperar a que el registro de datos TX este vació (transmisión)

        // Siguiente paso:  while(!(I2C2->SR1 & I2C_SR1_TXE)) TX=1?, Esperar a que el registro de datos TX este vació (transmisión)
    }  
}


unsigned char* readI2C_part4(unsigned char deviceAddress, unsigned char address, unsigned char size){

    static unsigned char bufferPointer[30];

    if(size==1){
        //TODO
        return 0;
    }else{
        
        I2C2->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
        wait_us(4);										    // Espera de 200 us        

        // Reajuste DMA
        DMA1_Channel5->CMAR = (unsigned int)&bufferPointer;	// Dirección de memoria
        DMA1_Channel5->CPAR = (unsigned int)&I2C2->DR;	    // Dirección de periférico
        DMA1_Channel5->CNDTR = size;						// Numero de Bytes a ser transferidos
        DMA1_Channel5->CCR |= DMA_CCR_TCIE;					// Habilitar interrupcion de transferencia completada
        DMA1_Channel5->CCR |= DMA_CCR_MINC;					// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
        //DMA1_Channel5->CCR &= ~DMA_CCR_PL_Msk;			// minima prioridad
        //DMA1_Channel5->CCR |= DMA_CCR_CIRC;				// Activar modo circular
        DMA1_Channel5->CCR |= DMA_CCR_EN;					// Habilitar canal DMA

        // Mandar instrucción de lectura y lectura de datos
        I2C2->CR1 |= I2C_CR1_ACK;							// Enable ACK
        I2C2->CR2 |= I2C_CR2_LAST;							// Habilitar NACK, cuando se llena el buffer DMA
        I2C2->CR1 |= I2C_CR1_START;							// Secuencia de inicio

        // Siguiente paso:  while(!(I2C2->SR1 & I2C_SR1_SB)) SB=1?, Esperar a que se complete la secuencia de inicio
    }  

    return bufferPointer;
}

void readI2C_part5(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C2->DR = deviceAddress + 1;						// Escribir dirección de lectura de periférico

        // Siguiente paso:  while(!(I2C2->SR1 & I2C_SR1_ADDR)) ADDR=1?, ACK de dirección de periférico
    }  
}

void readI2C_part6(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C_Aux = I2C2->SR2;								// Lectura basura para limpiar ADDR

        // Siguiente paso:  while(!(DMA1->ISR & DMA_ISR_TCIF5)) TCIF5=1?, Esperar a completar la transferencia de datos por DMA
    }  
}

void readI2C_part7(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        DMA1->IFCR |= DMA_IFCR_CTCIF5;						// TCIF5=1, Limpiar bandera de transferencia de datos por DMA completada
        //I2C2->CR1 &= !I2C_CR1_ACK;						// Disable ACK
        //I2C2->CR2 &= !I2C_CR2_LAST;						// Deshabilitar NACK, cuando se llena el buffer DMA
        I2C2->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
        DMA1_Channel5->CCR &= ~DMA_CCR_EN;					// Deshabilitar canal DMA
        wait_us(4);										    // Espera de 4 us
    }  
}

#endif