#include <stm32f103xb.h>
#include "i2c.h"
#include "timer4.h"

volatile unsigned char I2C_Aux = 0;

//----------------------------------------------------------------------------------------------------------------------

void initI2C1(void){

    //--Inicialización I2C---------------------------------------------------------------------------------

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;						// Habilitar reloj I2C1


    // Ajuste de pines (pin B7-SDA, B6-SCL)
    RCC->APB2ENR |=RCC_APB2ENR_IOPBEN;						// IOPBEN=1, Habilitar reloj del Puerto B
    GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);        // Limpiar registros de configuración PB6
    GPIOB->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);        // Limpiar registros de configuración PB7
    GPIOB->CRL |= GPIO_CRL_MODE6;							// MODE6=0b11, PB6 Modo salida 50 MHz
    GPIOB->CRL |= GPIO_CRL_CNF6;							// CNF6=0b11, PB6 Modo alternado open drain
    GPIOB->ODR |= GPIO_ODR_ODR6;							// ODR6=1 Habilitar resistencia de PullUp
    GPIOB->CRL |= GPIO_CRL_MODE7;							// MODE7=0b11, PB7 Modo salida 50 MHz
    GPIOB->CRL |= GPIO_CRL_CNF7;							// CNF7=0b11, PB7 Modo alternado open drain
    GPIOB->ODR |= GPIO_ODR_ODR7;							// ODR7=1 Habilitar resistencia de PullUp

    // Configuración de DMA
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;						// Habilitar reloj DMA

    //DMA1_Channel7->CMAR = (int)I2C_rxBuffer;				// Dirección de memoria
    //DMA1_Channel7->CPAR = (int)&I2C2->DR;					// Dirección de periférico
    //DMA1_Channel7->CNDTR = 3;								// Numero de Bytes a ser transferidos
    //DMA1_Channel7->CCR |= DMA_CCR_TCIE;					// Habilitar interrupcion de transferencia completada
    //DMA1_Channel7->CCR |= DMA_CCR_MINC;					// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
    //DMA1_Channel7->CCR |= DMA_CCR_CIRC;					// Activar modo circular
    //DMA1_Channel7->CCR |= DMA_CCR_EN;						// Habilitar canal DMA



    // Configuración de periférico
    //I2C1->CCR |= I2C_CCR_FS;								// Activar modo I2C de alta velocidad
    I2C1->CR2 |= 36;										// Frecuencia de Bus APB1
    I2C1->CCR |= 180;										// Ajuste de frecuencia i2c a 400 khz
    I2C1->TRISE |= 37;										// y otros ajustes de velocidad

    I2C1->CR1 |= I2C_CR1_ACK;								// Enable ACK
    I2C1->CR2 |= I2C_CR2_DMAEN;								// Habilitar llamado a DMA
    //I2C1->CR2 |= I2C_CR2_LAST;							// Habilitar NACK, cuando se llena el buffer DMA
    I2C1->CR1 |= I2C_CR1_PE;								// Habilitar periférico
}



void writeI2C1(unsigned char deviceAddress ,unsigned char address, unsigned char size, unsigned char* data){

    I2C1->CR1 |= I2C_CR1_START;								// Secuencia de inicio
    while(!(I2C1->SR1 & I2C_SR1_SB));						// SB=1?, Esperar a que se complete la secuencia de inicio
    I2C1->DR = deviceAddress;							    // Escribir dirección de escritura de periférico
    while(!(I2C1->SR1 & I2C_SR1_ADDR));						// ADDR=1?, ACK de dirección de periférico
    I2C_Aux = I2C1->SR2;								    // Lectura basura para limpiar ADDR
    I2C1->DR = address;										// Escribir dirección de registro de periférico
    while(!(I2C1->SR1 & I2C_SR1_TXE));						// TX=1?, Esperar a que el registro de datos TX este vació (transmisión)

    for(int i=0; i<size;i++){
        I2C1->DR = data[i];									// Escribir dato
        while(!(I2C1->SR1 & I2C_SR1_TXE));					// TX=1?, Esperar a que el registro de datos TX este vació (transmisión)
    }
    I2C1->CR1 |= I2C_CR1_STOP;								// Secuencia de parada
    wait_us(4);										        // Espera de 200 us
}




unsigned char* readI2C1(unsigned char deviceAddress, unsigned char address, unsigned char size){

    static unsigned char readData[30];
    if(size==1){
        // Este código aun no funciona correctamente, por ahora leer dos datos en vez de 1.

        I2C1->CR1 |= I2C_CR1_START;							// Secuencia de inicio
        while(!(I2C1->SR1 & I2C_SR1_SB));					// SB=1?, Esperar a que se complete la secuencia de inicio
        I2C1->DR = deviceAddress;						    // Escribir dirección de escritura de periférico
        while(!(I2C1->SR1 & I2C_SR1_ADDR));					// ADDR=1?, ACK de dirección de periférico
        I2C_Aux= I2C1->SR2;							        // Lectura basura para limpiar ADDR
        I2C1->DR = address;									// Escribir dirección de registro de periférico
        while(!(I2C1->SR1 & I2C_SR1_TXE));					// TX=1?, Esperar a que el registro de datos TX este vació (transmisión)
        I2C1->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
        wait_us(4);										    // Espera de 200 us

        // Reajuste DMA
        DMA1_Channel7->CMAR = (int)&readData;			    // Dirección de memoria de buffer de lectura
        DMA1_Channel7->CPAR = (int)&I2C1->DR;				// Dirección de periférico
        DMA1_Channel7->CNDTR = size;						// Numero de Bytes a ser transferidos
        DMA1_Channel7->CCR |= DMA_CCR_TCIE;					// Habilitar interrupcion de transferencia completada
        //DMA1_Channel7->CCR &= !DMA_CCR_MINC;				// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
        //DMA1_Channel7->CCR |= DMA_CCR_CIRC;				// Activar modo circular
        DMA1_Channel7->CCR |= DMA_CCR_EN;					// Habilitar canal DMA

        // Mandar instrucción de lectura y lectura de datos
        I2C1->CR1 &= !I2C_CR1_ACK;							// Disable ACK
        //I2C2->CR2 |= I2C_CR2_LAST;						// Habilitar NACK, cuando se llena el buffer DMA
        I2C1->CR1 |= I2C_CR1_START;							// Secuencia de inicio
        while(!(I2C1->SR1 & I2C_SR1_SB));					// SB=1?, Esperar a que se complete la secuencia de inicio
        I2C1->DR = deviceAddress + 1;					    // Escribir dirección de lectura de periférico
        while(!(I2C1->SR1 & I2C_SR1_ADDR));					// ADDR=1?, ACK de dirección de periférico
        I2C_Aux = I2C1->SR2;								// Lectura basura para limpiar ADDR
        while(!(DMA1->ISR & DMA_ISR_TCIF7));				// TCIF7=1?, Esperar a completar la transferencia de datos por DMA
        DMA1->IFCR |= DMA_IFCR_CTCIF7;						// TCIF7=1, Limpiar bandera de transferencia de datos por DMA completada
        //I2C2->CR1 &= !I2C_CR1_ACK;						// Disable ACK
        //I2C2->CR2 &= !I2C_CR2_LAST;						// Deshabilitar NACK, cuando se llena el buffer DMA
        I2C1->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
        DMA1_Channel7->CCR &= !DMA_CCR_EN;					// Deshabilitar canal DMA
        wait_us(4);										    // Espera de 4 us
    }else{
        // Solo funciona si size >=2

        I2C1->CR1 &= ~I2C_CR1_PE;							// Deshabilitar periférico
        I2C1->CR1 |= I2C_CR1_PE;							// Habilitar periférico

        // Mandar instrucción de escritura y dirección de lectura

        //I2C2->CR2 |= I2C_CR2_LAST;						// Habilitar NACK, cuando se llena el buffer DMA
        I2C1->CR1 |= I2C_CR1_START;							// Secuencia de inicio
        while(!(I2C1->SR1 & I2C_SR1_SB));					// SB=1?, Esperar a que se complete la secuencia de inicio
        I2C1->DR = deviceAddress;			                // Escribir dirección de escritura de periférico
        while(!(I2C1->SR1 & I2C_SR1_ADDR));					// ADDR=1?, ACK de dirección de periférico
        I2C_Aux= I2C1->SR2;							        // Lectura basura para limpiar ADDR
        I2C1->DR = address;									// Escribir dirección de registro de periférico
        while(!(I2C1->SR1 & I2C_SR1_TXE));					// TX=1?, Esperar a que el registro de datos TX este vació (transmisión)
        I2C1->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
        wait_us(4);										    // Espera de 200 us
        

        // Reajuste DMA
        DMA1_Channel7->CMAR = (unsigned int)&readData;		// Dirección de memoria
        DMA1_Channel7->CPAR = (unsigned int)&I2C1->DR;	    // Dirección de periférico
        DMA1_Channel7->CNDTR = size;						// Numero de Bytes a ser transferidos
        DMA1_Channel7->CCR |= DMA_CCR_TCIE;					// Habilitar interrupcion de transferencia completada
        DMA1_Channel7->CCR |= DMA_CCR_MINC;					// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
        

        //DMA1_Channel7->CCR |= DMA_CCR_CIRC;				// Activar modo circular
        DMA1_Channel7->CCR |= DMA_CCR_EN;					// Habilitar canal DMA

        // Mandar instrucción de lectura y lectura de datos
        I2C1->CR1 |= I2C_CR1_ACK;							// Enable ACK
        I2C1->CR2 |= I2C_CR2_LAST;							// Habilitar NACK, cuando se llena el buffer DMA
        I2C1->CR1 |= I2C_CR1_START;							// Secuencia de inicio
        while(!(I2C1->SR1 & I2C_SR1_SB));					// SB=1?, Esperar a que se complete la secuencia de inicio
        I2C1->DR = deviceAddress + 1;						// Escribir dirección de lectura de periférico
        while(!(I2C1->SR1 & I2C_SR1_ADDR));					// ADDR=1?, ACK de dirección de periférico
        I2C_Aux = I2C1->SR2;								// Lectura basura para limpiar ADDR
        while(!(DMA1->ISR & DMA_ISR_TCIF7));				// TCIF5=1?, Esperar a completar la transferencia de datos por DMA
        DMA1->IFCR |= DMA_IFCR_CTCIF7;						// TCIF5=1, Limpiar bandera de transferencia de datos por DMA completada
        //I2C2->CR1 &= !I2C_CR1_ACK;						// Disable ACK
        //I2C2->CR2 &= !I2C_CR2_LAST;						// Deshabilitar NACK, cuando se llena el buffer DMA
        I2C1->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
        DMA1_Channel7->CCR &= !DMA_CCR_EN;					// Deshabilitar canal DMA
        wait_us(4);										    // Espera de 4 us
    }

    return readData;
}

// Lectura I2C sin IDLE

void readI2C1_part1(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C1->CR1 &= ~I2C_CR1_PE;							// Deshabilitar periférico
        I2C1->CR1 |= I2C_CR1_PE;							// Habilitar periférico

        // Mandar instrucción de escritura y dirección de lectura

        //I2C2->CR2 |= I2C_CR2_LAST;						// Habilitar NACK, cuando se llena el buffer DMA
        I2C1->CR1 |= I2C_CR1_START;							// Secuencia de inicio

        // Siguiente paso:  while(!(I2C1->SR1 & I2C_SR1_SB)) SB=1?, Esperar a que se complete la secuencia de inicio
    }
}

void readI2C1_part2(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C1->DR = deviceAddress;			                // Escribir dirección de escritura de periférico

        // Siguiente paso:  while(!(I2C1->SR1 & I2C_SR1_ADDR))  ADDR=1?, ACK de dirección de periférico
    }
}

void readI2C1_part3(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C_Aux= I2C1->SR2;							        // Lectura basura para limpiar ADDR
        I2C1->DR = address;									// Escribir dirección de registro de periférico
        while(!(I2C1->SR1 & I2C_SR1_TXE));					// TX=1?, Esperar a que el registro de datos TX este vació (transmisión)

        // Siguiente paso:  while(!(I2C1->SR1 & I2C_SR1_TXE)) TX=1?, Esperar a que el registro de datos TX este vació (transmisión)
    }  
}


unsigned char* readI2C1_part4(unsigned char deviceAddress, unsigned char address, unsigned char size){

    static unsigned char bufferPointer[30];

    if(size==1){
        //TODO
        return 0;
    }else{
        
        I2C1->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
        wait_us(4);										    // Espera de 200 us        

        // Reajuste DMA
        DMA1_Channel7->CMAR = (unsigned int)&bufferPointer;	// Dirección de memoria
        DMA1_Channel7->CPAR = (unsigned int)&I2C1->DR;	    // Dirección de periférico
        DMA1_Channel7->CNDTR = size;						// Numero de Bytes a ser transferidos
        DMA1_Channel7->CCR |= DMA_CCR_TCIE;					// Habilitar interrupcion de transferencia completada
        DMA1_Channel7->CCR |= DMA_CCR_MINC;					// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
        //DMA1_Channel7->CCR &= ~DMA_CCR_PL_Msk;			// minima prioridad
        //DMA1_Channel7->CCR |= DMA_CCR_CIRC;				// Activar modo circular
        DMA1_Channel7->CCR |= DMA_CCR_EN;					// Habilitar canal DMA

        // Mandar instrucción de lectura y lectura de datos
        I2C1->CR1 |= I2C_CR1_ACK;							// Enable ACK
        I2C1->CR2 |= I2C_CR2_LAST;							// Habilitar NACK, cuando se llena el buffer DMA
        I2C1->CR1 |= I2C_CR1_START;							// Secuencia de inicio

        // Siguiente paso:  while(!(I2C1->SR1 & I2C_SR1_SB)) SB=1?, Esperar a que se complete la secuencia de inicio
    }  

    return bufferPointer;
}

void readI2C1_part5(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C1->DR = deviceAddress + 1;						// Escribir dirección de lectura de periférico

        // Siguiente paso:  while(!(I2C1->SR1 & I2C_SR1_ADDR)) ADDR=1?, ACK de dirección de periférico
    }  
}

void readI2C1_part6(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C_Aux = I2C1->SR2;								// Lectura basura para limpiar ADDR

        // Siguiente paso:  while(!(DMA1->ISR & DMA_ISR_TCIF7)) TCIF7=1?, Esperar a completar la transferencia de datos por DMA
    }  
}

void readI2C1_part7(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        DMA1->IFCR |= DMA_IFCR_CTCIF7;						// TCIF7=1, Limpiar bandera de transferencia de datos por DMA completada
        //I2C1->CR1 &= !I2C_CR1_ACK;						// Disable ACK
        //I2C1->CR2 &= !I2C_CR2_LAST;						// Deshabilitar NACK, cuando se llena el buffer DMA
        I2C1->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
        DMA1_Channel7->CCR &= ~DMA_CCR_EN;					// Deshabilitar canal DMA
        wait_us(4);										    // Espera de 4 us
    }  
}

unsigned short getI2C1_SB(void){
    return (unsigned short) I2C1->SR1 & I2C_SR1_SB;
}

unsigned short getI2C1_ADDR(void){
    return (unsigned short)I2C1->SR1 & I2C_SR1_ADDR;
}

unsigned short getI2C1_AF(void){
    return (unsigned short)I2C1->SR1 & I2C_SR1_AF;
}

void clearI2C1_AF(void){
    I2C1->SR1 &= ~I2C_SR1_AF;    
}

unsigned short getI2C1_TXE(void){
    return (unsigned short) I2C1->SR1 & I2C_SR1_TXE;
}

unsigned int getI2C1_DMA_TCIF(void){

    return (unsigned int) DMA1->ISR & DMA_ISR_TCIF7;
}

//----------------------------------------------------------------------------------------------------------------------

void initI2C2(void){

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


void writeI2C2(unsigned char deviceAddress ,unsigned char address, unsigned char size, unsigned char* data){

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



unsigned char* readI2C2(unsigned char deviceAddress, unsigned char address, unsigned char size){
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

void readI2C2_part1(unsigned char deviceAddress, unsigned char address, unsigned char size){
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

void readI2C2_part2(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C2->DR = deviceAddress;			                // Escribir dirección de escritura de periférico

        // Siguiente paso:  while(!(I2C2->SR1 & I2C_SR1_ADDR))  ADDR=1?, ACK de dirección de periférico
    }
}

void readI2C2_part3(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C_Aux= I2C2->SR2;							        // Lectura basura para limpiar ADDR
        I2C2->DR = address;									// Escribir dirección de registro de periférico
        while(!(I2C2->SR1 & I2C_SR1_TXE));					// TX=1?, Esperar a que el registro de datos TX este vació (transmisión)

        // Siguiente paso:  while(!(I2C2->SR1 & I2C_SR1_TXE)) TX=1?, Esperar a que el registro de datos TX este vació (transmisión)
    }  
}


unsigned char* readI2C2_part4(unsigned char deviceAddress, unsigned char address, unsigned char size){

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

void readI2C2_part5(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C2->DR = deviceAddress + 1;						// Escribir dirección de lectura de periférico

        // Siguiente paso:  while(!(I2C2->SR1 & I2C_SR1_ADDR)) ADDR=1?, ACK de dirección de periférico
    }  
}

void readI2C2_part6(unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(size==1){
        //TODO
    }else{
        I2C_Aux = I2C2->SR2;								// Lectura basura para limpiar ADDR

        // Siguiente paso:  while(!(DMA1->ISR & DMA_ISR_TCIF5)) TCIF5=1?, Esperar a completar la transferencia de datos por DMA
    }  
}

void readI2C2_part7(unsigned char deviceAddress, unsigned char address, unsigned char size){
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

unsigned short getI2C2_SB(void){
    return (unsigned short) I2C2->SR1 & I2C_SR1_SB;
}

unsigned short getI2C2_ADDR(void){
    return (unsigned short)I2C2->SR1 & I2C_SR1_ADDR;
}

unsigned short getI2C2_AF(void){
    return (unsigned short)I2C2->SR1 & I2C_SR1_AF;
}

void clearI2C2_AF(void){
    I2C2->SR1 &= ~I2C_SR1_AF;    
}

unsigned short getI2C2_TXE(void){
    return (unsigned short) I2C2->SR1 & I2C_SR1_TXE;
}

unsigned int getI2C2_DMA_TCIF(void){
    return (unsigned int) DMA1->ISR & DMA_ISR_TCIF5;
}
//------------------------------------------------------------------------------------------------------------------------


void initI2C(unsigned char i2cPort){
    if(i2cPort==1){
        initI2C1();
    }else{
        initI2C2();
    }
}

unsigned char* readI2C(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(i2cPort==1){
        return (unsigned char*)readI2C1(deviceAddress, address, size);
    }else{
        return (unsigned char*)readI2C2(deviceAddress, address, size);
    }
}

void writeI2C(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size, unsigned char* data){
    if(i2cPort==1){
        writeI2C1(deviceAddress, address, size, data);
    }else{
        writeI2C2(deviceAddress, address, size, data);
    }   
}

void readI2C_part1(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(i2cPort==1){
        readI2C1_part1(deviceAddress, address, size);
    }else{
        readI2C2_part1(deviceAddress, address, size);
    }      
}

void readI2C_part2(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(i2cPort==1){
        readI2C1_part2(deviceAddress, address, size);
    }else{
        readI2C2_part2(deviceAddress, address, size);
    }      
}

void readI2C_part3(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(i2cPort==1){
        readI2C1_part3(deviceAddress, address, size);
    }else{
        readI2C2_part3(deviceAddress, address, size);
    }   
}

unsigned char* readI2C_part4(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(i2cPort==1){
        return (unsigned char*)readI2C1_part4(deviceAddress, address, size);
    }else{
        return (unsigned char*)readI2C2_part4(deviceAddress, address, size);
    }   
}

void readI2C_part5(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(i2cPort==1){
        readI2C1_part5(deviceAddress, address, size);
    }else{
        readI2C2_part5(deviceAddress, address, size);
    }   
}

void readI2C_part6(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(i2cPort==1){
        readI2C1_part6(deviceAddress, address, size);
    }else{
        readI2C2_part6(deviceAddress, address, size);
    }   
}

void readI2C_part7(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size){
    if(i2cPort==1){
        readI2C1_part7(deviceAddress, address, size);
    }else{
        readI2C2_part7(deviceAddress, address, size);
    }   
}


unsigned short getI2C_SB(unsigned char i2cPort){
    if(i2cPort==1){
        return getI2C1_SB();
    }else{
        return getI2C2_SB();
    }   
}

unsigned short getI2C_ADDR(unsigned char i2cPort){
    if(i2cPort==1){
        return getI2C1_ADDR();
    }else{
        return getI2C2_ADDR();
    }   
}

unsigned short getI2C_AF(unsigned char i2cPort){
    if(i2cPort==1){
        return getI2C1_AF();
    }else{
        return getI2C2_AF();
    }   
}

void clearI2C_AF(unsigned char i2cPort){
    if(i2cPort==1){
        clearI2C1_AF();
    }else{
        clearI2C2_AF();
    }    
}

unsigned short getI2C_TXE(unsigned char i2cPort){
    if(i2cPort==1){
        return getI2C1_TXE();
    }else{
        return getI2C2_TXE();
    }      
}

unsigned int getI2C_DMA_TCIF(unsigned char i2cPort){
    if(i2cPort==1){
        return getI2C1_DMA_TCIF();
    }else{
        return getI2C2_DMA_TCIF();
    }   
}


