#include <stm32f103xb.h>


extern "C"{
    #include "timer4.h"
    #include "uart.h"
    #include "i2c.h"
}


#include "helper_3dmath.h"

#include "dmpfirmware.h"

#define TRUE 0x01
#define FALSE 0x00

#define MPU_X_ACCEL_OFFSET_ADDRESS 0x06
#define MPU_Y_ACCEL_OFFSET_ADDRESS 0x08
#define MPU_Z_ACCEL_OFFSET_ADDRESS 0x0A
#define MPU_X_GYRO_OFFSET_ADDRESS 0x13
#define MPU_Y_GYRO_OFFSET_ADDRESS 0x15
#define MPU_Z_GYRO_OFFSET_ADDRESS 0x17

#define MPU0_X_ACCEL_OFFSET 0xFF58
#define MPU0_Y_ACCEL_OFFSET 0xF118
#define MPU0_Z_ACCEL_OFFSET 0x0C94
#define MPU0_X_GYRO_OFFSET 0xFF9C
#define MPU0_Y_GYRO_OFFSET 0xFFF0
#define MPU0_Z_GYRO_OFFSET 0x003C

#define MPU1_X_ACCEL_OFFSET 0xFF58
#define MPU1_Y_ACCEL_OFFSET 0xF118
#define MPU1_Z_ACCEL_OFFSET 0x0C94
#define MPU1_X_GYRO_OFFSET 0xFF9C
#define MPU1_Y_GYRO_OFFSET 0xFFF0
#define MPU1_Z_GYRO_OFFSET 0x003C

float MPUMaxYaw = 0;                // Parametors experimentales, dependen del angulo al que se quiere 1
float MPUMaxPitch = 0;
float MPUMaxRoll = 0;

#define SAMPLING_PERIOD_US 9952U
// 10ms

// Valores con signo en complemento a 2 de 16 bits
short MPU0xAccel = 0;
short MPU0yAccel = 0;
short MPU0zAccel = 0;
short MPU0xGyro = 0;
short MPU0yGyro = 0;
short MPU0zGyro = 0;
int MPU0wQuat = 0;
int MPU0xQuat = 0;
int MPU0yQuat = 0;
int MPU0zQuat = 0;

float MPU0Yaw = 0.0;
float MPU0Pitch = 0.0;
float MPU0Roll = 0.0;
float MPU0YawNormalized = 0.0;
float MPU0PitchNormalized = 0.0;
float MPU0RollNormalized = 0.0;


unsigned char MPU0InterruptionFlag = 0;
unsigned char MPU0InterruptionCounter = 0;
unsigned char* MPU0RawData;
unsigned char MPU0SampleReadyFlag = 0;
unsigned char Button0 =0;
Quaternion MPU0Quaternion;


short MPU1xAccel = 0;
short MPU1yAccel = 0;
short MPU1zAccel = 0;
short MPU1xGyro = 0;
short MPU1yGyro = 0;
short MPU1zGyro = 0;
int MPU1wQuat = 0;
int MPU1xQuat = 0;
int MPU1yQuat = 0;
int MPU1zQuat = 0;

float MPU1YawNormalized = 0.0;
float MPU1PitchNormalized = 0.0;
float MPU1RollNormalized = 0.0;

unsigned char MPU1InterruptionFlag = 1;
unsigned char MPU1InterruptionCounter = 0;    
unsigned char* MPU1RawData;
unsigned char MPU1SampleReadyFlag = 0;
unsigned char Button1 =0;
Quaternion MPU1Quaternion;

unsigned short uartTxCounter = 0;
extern unsigned char uartTxEmptyBufferFlag;


extern void DMA1_Channel4_IRQHandler();

void initMCU(void){

    //--Inicialización interface debug---------------------------------------------------------------------

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	                    // AFIOEN = 1, activar reloj de modulo AFIO
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;			            // SWJ_CFG = 010 JTAG disabled y SW-DP enable

	//--Inicializacion de reloj externo a 72 Mhz-----------------------------------------------------------

	FLASH->ACR |=FLASH_ACR_LATENCY_1;			            // Ajustar Latencia de memoria Flash en 2 (Debido al ajuste de alta frecuencia)
	RCC->CR |= RCC_CR_HSEON; 					            // HSE0N = 1, Habilitar reloj externo
    while(!(RCC->CR & RCC_CR_HSERDY));   		            // HSERDY = 1 ?, Esperar a que el reloj externo se estabilice
    RCC->CFGR |= RCC_CFGR_PLLSRC; 				            // PLLSRC=1, Seleccionar reloj externo como fuente de reloj de PLL
    RCC->CFGR |= RCC_CFGR_PLLMULL9;				            // PLLMUL=7, Seleccionar 9 como factor PLL
    RCC->CR	  |= RCC_CR_PLLON;					            // PLLON=1 Encender PLL
    while(!(RCC->CR & RCC_CR_PLLRDY));			            // PLLRDY=1 ? Esperar a que se ajuste el PLL
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; 			            // PPRE1=0b100, Ajustar el prescaler de APB1, para obtener 36Mhz
    RCC->CFGR |= RCC_CFGR_SW_1;    				            // SW=2, PLL seleccionado como reloj del sistema
    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)); 	            // SWS=2? , Esperar a que el reloj del sistema se configure

}

void initButtons(void){

    //--LEDBUILTIN-PC13--------------------------------------------------------------------------------------
    RCC->APB2ENR |=RCC_APB2ENR_IOPCEN;			            // IOPCEN=1, Habilitar reloj del Puerto C
    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);      // Limpiar registros de configuración    
    GPIOC->CRH |= GPIO_CRH_MODE13;				            // MODE13=0x11, PC13 en modo de salida
    GPIOC->CRH &= ~GPIO_CRH_CNF13;				            // CNF13=0x00, PC13 en modo de salida push-pull
    GPIOC->ODR |= GPIO_ODR_ODR13;				            // Salida en uno

    //--PA0 = Botón 1, PA1 = Botón 2--------------------------------------------------------------------------
    
    
    RCC->APB2ENR |=RCC_APB2ENR_IOPAEN;			            // IOPAEN=1, Habilitar reloj del Puerto A
    
    //--PA0
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);        // Limpiar registros de configuración    
    GPIOA->CRL &= ~GPIO_CRL_MODE0;				            // MODE0=0x00, PA0 en modo de entrada
    GPIOA->CRL |= GPIO_CRL_CNF0_1;				            // CNF0=0x10, PA0 con resistencias de pull-up/pull-down
    GPIOA->ODR &= ~GPIO_ODR_ODR0;				            // Resistencia de Pul-down

    //--PA1
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);        // Limpiar registros de configuración    
    GPIOA->CRL &= ~GPIO_CRL_MODE1;				            // MODE1=0x00, PA1 en modo de entrada
    GPIOA->CRL |= GPIO_CRL_CNF1_1;				            // CNF1=0x10, PA1 con resistencias de pull-up/pull-down
    GPIOA->ODR &= ~GPIO_ODR_ODR1;				            // Resistencia de Pul-down

}

void initMPU(unsigned char mpu6050Address, unsigned char interruptPin,
            short MPU_X_ACCEL_OFFSET, short MPU_Y_ACCEL_OFFSET,
            short MPU_Z_ACCEL_OFFSET, short MPU_X_GYRO_OFFSET,
            short MPU_Y_GYRO_OFFSET, short MPU_Z_GYRO_OFFSET){              
    
    
    //-----------------------Inicializar MPU------------------------------------------------------------------

    //int *inputI2CBuffer; 
    unsigned char outputI2CBuffer[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
    
    //inputI2CBuffer = readI2C(0xD0,0x6B,2);

    
    outputI2CBuffer[0] = 0x80;
    writeI2C(mpu6050Address,0x6B,1,outputI2CBuffer);                  // Reset MPU6050    
    
    outputI2CBuffer[0] = 0x00;                              
    writeI2C(mpu6050Address,0x6A,1,outputI2CBuffer);                  // Reset FIFO

    wait_ms(200);

    // Carga DMP

    outputI2CBuffer[0] = 0x00;                              
    writeI2C(mpu6050Address,0x6B,1,outputI2CBuffer);
    outputI2CBuffer[0] = 0x00;                              
    writeI2C(mpu6050Address,0x6C,1,outputI2CBuffer);    
    outputI2CBuffer[0] = 0x03;                              
    writeI2C(mpu6050Address,0x1A,1,outputI2CBuffer);
    outputI2CBuffer[0] = 0x18;                              
    writeI2C(mpu6050Address,0x1B,1,outputI2CBuffer);
    outputI2CBuffer[0] = 0x00;                              
    writeI2C(mpu6050Address,0x1C,1,outputI2CBuffer);
    outputI2CBuffer[0] = 0x00;                              
    writeI2C(mpu6050Address,0x23,1,outputI2CBuffer);
    outputI2CBuffer[0] = 0x00;                              
    writeI2C(mpu6050Address,0x38,1,outputI2CBuffer);
    outputI2CBuffer[0] = 0x04;                              
    writeI2C(mpu6050Address,0x6A,1,outputI2CBuffer);    
    outputI2CBuffer[0] = 0x04;                              
    writeI2C(mpu6050Address,0x19,1,outputI2CBuffer);   

    // firmware

    int i = 0;
    int j = 0;

    for(i = 0; i<192; i++){
        
        short data = 0x0000 + (i*0x10);
        outputI2CBuffer[0] = (data >> 8);
        outputI2CBuffer[1] = data; 
        writeI2C(mpu6050Address,0x6D,2,outputI2CBuffer);

        if(i<191){            
            for(j = 0; j<16; j++){
                outputI2CBuffer[j] = dmp_memory[(i*16) + j];   
            }
            writeI2C(0xD0,0x6F,16,outputI2CBuffer);
        }else{            
            for(j = 0; j<6; j++){
                outputI2CBuffer[j] = dmp_memory[(i*16) + j];   
            }
            writeI2C(mpu6050Address,0x6F,6,outputI2CBuffer);
        }    
    }

    outputI2CBuffer[0] = 0x04;
    outputI2CBuffer[1] = 0x00;                              
    writeI2C(mpu6050Address,0x70,2,outputI2CBuffer);


    // Ajustar valores de offset
    wait_us(300);

    outputI2CBuffer[0] = (unsigned char) (MPU_X_ACCEL_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_X_ACCEL_OFFSET;
    writeI2C(mpu6050Address,MPU_X_ACCEL_OFFSET_ADDRESS,2,outputI2CBuffer);

    outputI2CBuffer[0] = (unsigned char) (MPU_Y_ACCEL_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_Y_ACCEL_OFFSET;
    writeI2C(mpu6050Address,MPU_Y_ACCEL_OFFSET_ADDRESS,2,outputI2CBuffer);

    outputI2CBuffer[0] = (unsigned char) (MPU_Z_ACCEL_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_Z_ACCEL_OFFSET;
    writeI2C(mpu6050Address,MPU_Z_ACCEL_OFFSET_ADDRESS,2,outputI2CBuffer);

    outputI2CBuffer[0] = (unsigned char) (MPU_X_GYRO_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_X_GYRO_OFFSET;
    writeI2C(mpu6050Address,MPU_X_GYRO_OFFSET_ADDRESS,2,outputI2CBuffer);

    outputI2CBuffer[0] = (unsigned char) (MPU_Y_GYRO_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_Y_GYRO_OFFSET;
    writeI2C(mpu6050Address,MPU_Y_GYRO_OFFSET_ADDRESS,2,outputI2CBuffer);

    outputI2CBuffer[0] = (unsigned char) (MPU_Z_GYRO_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_Z_GYRO_OFFSET;
    writeI2C(mpu6050Address,MPU_Z_GYRO_OFFSET_ADDRESS,2,outputI2CBuffer);

    wait_ms(3);

    outputI2CBuffer[0] = (unsigned char) (MPU_X_ACCEL_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_X_ACCEL_OFFSET;
    outputI2CBuffer[2] = (unsigned char) (MPU_Y_ACCEL_OFFSET >> 8);
    outputI2CBuffer[3] = (unsigned char) MPU_Y_ACCEL_OFFSET;
    outputI2CBuffer[4] = (unsigned char) (MPU_Z_ACCEL_OFFSET >> 8);
    outputI2CBuffer[5] = (unsigned char) MPU_Z_ACCEL_OFFSET;
    writeI2C(mpu6050Address,MPU_X_ACCEL_OFFSET_ADDRESS,6,outputI2CBuffer);

    wait_ms(3);

    outputI2CBuffer[0] = (unsigned char) (MPU_X_GYRO_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_X_GYRO_OFFSET;
    outputI2CBuffer[2] = (unsigned char) (MPU_Y_GYRO_OFFSET >> 8);
    outputI2CBuffer[3] = (unsigned char) MPU_Y_GYRO_OFFSET;
    outputI2CBuffer[4] = (unsigned char) (MPU_Z_GYRO_OFFSET >> 8);
    outputI2CBuffer[5] = (unsigned char) MPU_Z_GYRO_OFFSET;
    writeI2C(mpu6050Address,MPU_X_GYRO_OFFSET_ADDRESS,6,outputI2CBuffer);

    wait_ms(3); 

    // Interruption pin bypass configuration
    outputI2CBuffer[0] = 0x02;
    writeI2C(mpu6050Address,0x37,1,outputI2CBuffer);

    // FIFO Buffer configuration
    outputI2CBuffer[0] = 0xC0;
    writeI2C(mpu6050Address,0x6A,1,outputI2CBuffer);

    // Interruption pin bypass configuration
    outputI2CBuffer[0] = 0x02;
    writeI2C(mpu6050Address,0x38,1,outputI2CBuffer);
    // First Read
    MPU0RawData = readI2C(0xD0,0x72,2);

    //wait_ms(5);


    //-----------------------Inicializar pin interrupcion-----------------------------------------------------

    if(interruptPin==0){
        // Pin B1
        
        RCC->APB2ENR |=RCC_APB2ENR_IOPBEN;			        // IOPBEN=1, Habilitar reloj del Puerto B
        GPIOB->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);    // Limpiar registros de configuración
        GPIOB->CRL &= ~GPIO_CRL_MODE1;				        // MODE1=0x00, PB1 en modo de entrada
        GPIOB->CRL |= GPIO_CRL_CNF1_1;				        // CNF1=0x10,  PB1 en modo pull-up/pull-down
        GPIOB->ODR &= ~GPIO_ODR_ODR1;						// ODR1=0 Habilitar resistencia de PullDown


        // Linea 1, dado que se usa B1. Ejemplos: Linea 2: EXT2, A2,C2,B2, etc

        RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                 // Habilitar reloj de AFIO
        AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PB;           // Habilitar interrupcion en PB1
        EXTI->IMR |=  EXTI_IMR_MR1;                         // Mask interrupcion de linea 1
        EXTI->RTSR |= EXTI_RTSR_TR1;                        // Habilitar trigger con rising edge
        NVIC_EnableIRQ(EXTI1_IRQn);                         // Habilitar interrupciones de la linea 1 

        //EXTI->SWIER |= EXTI_SWIER_SWI1;                   // Software interrupt 
        //EXTI->PR |= EXTI_PR_PR1;                          // Limpiar interrupcion pendiente   

    }else{
        // Pin B1
        
        RCC->APB2ENR |=RCC_APB2ENR_IOPBEN;			        // IOPBEN=1, Habilitar reloj del Puerto B
        GPIOB->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);    // Limpiar registros de configuración
        GPIOB->CRL &= ~GPIO_CRL_MODE1;				        // MODE1=0x00, PB1 en modo de entrada
        GPIOB->CRL |= GPIO_CRL_CNF1_1;				        // CNF1=0x10,  PB1 en modo pull-up/pull-down
        GPIOB->ODR &= ~GPIO_ODR_ODR1;						// ODR1=0 Habilitar resistencia de PullDown

        // Linea 1, dado que se usa B1. Ejemplos: Linea 2: EXT2, A2,C2,B2, etc
    
        RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                 // Habilitar reloj de AFIO
        AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PB;           // Habilitar interrupcion en PB1
        EXTI->IMR |=  EXTI_IMR_MR1;                         // Mask interrupcion de linea 1
        EXTI->RTSR |= EXTI_RTSR_TR1;                        // Habilitar trigger con rising edge
        NVIC_EnableIRQ(EXTI1_IRQn);                         // Habilitar interrupciones de la linea 1 
    
        //EXTI->SWIER |= EXTI_SWIER_SWI1;                   // Software interrupt 
        //EXTI->PR |= EXTI_PR_PR1;                          // Limpiar interrupcion pendiente   
    }
    
}

void getMPUData(unsigned char deviceAddress, unsigned char* MPUInterruptionCounter, unsigned char* MPUInterruptionFlag,
                int* MPUwQuat, int* MPUxQuat, int* MPUyQuat, int* MPUzQuat,
                short* MPUxGyro, short* MPUyGyro, short* MPUzGyro,
                short* MPUxAccel, short* MPUyAccel, short* MPUzAccel,
                unsigned char** MPURawData, unsigned char* MPUSampleReadyFlag){

    if(*MPUInterruptionFlag){

        if(*MPUInterruptionCounter == 0){
            readI2C_part1(deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 1){
            if(I2C2->SR1 & I2C_SR1_SB) (*MPUInterruptionCounter)++; 

        }else if(*MPUInterruptionCounter == 2){
            readI2C_part2(deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 3){
            if(I2C2->SR1 & I2C_SR1_ADDR) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 4){
            readI2C_part3(deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 5){
            if(I2C2->SR1 & I2C_SR1_TXE) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 6){
            (*MPURawData) = readI2C_part4(deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 7){
            if(I2C2->SR1 & I2C_SR1_SB) (*MPUInterruptionCounter)++;  
             
        }else if(*MPUInterruptionCounter == 8){
            readI2C_part5(deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 9){
            if(I2C2->SR1 & I2C_SR1_ADDR) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 10){
            readI2C_part5(deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 11){
            if(I2C2->SR1 & I2C_SR1_ADDR) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 12){
            readI2C_part6(deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 13){
            if(DMA1->ISR & DMA_ISR_TCIF5) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 14){

            readI2C_part7(deviceAddress,0x72,2);
                
            unsigned short fifoSize = ((*MPURawData)[0]<<8) + (*MPURawData)[1];
            if(fifoSize<28){
                *MPUInterruptionCounter = 0;
                *MPUInterruptionFlag = 0;

            }else{
                (*MPUInterruptionCounter)++;
            }
            // Lectura RawData x28
        }else if(*MPUInterruptionCounter == 15){
            readI2C_part1(deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 16){
            if(I2C2->SR1 & I2C_SR1_SB) (*MPUInterruptionCounter)++; 

        }else if(*MPUInterruptionCounter == 17){
            readI2C_part2(deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 18){
            if(I2C2->SR1 & I2C_SR1_ADDR) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 19){
            readI2C_part3(deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 20){
            if(I2C2->SR1 & I2C_SR1_TXE) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 21){
            (*MPURawData) = readI2C_part4(deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 22){
            if(I2C2->SR1 & I2C_SR1_SB) (*MPUInterruptionCounter)++;  
             
        }else if(*MPUInterruptionCounter == 23){
            readI2C_part5(deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 24){
            if(I2C2->SR1 & I2C_SR1_ADDR) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 25){
            readI2C_part5(deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 26){
            if(I2C2->SR1 & I2C_SR1_ADDR) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 27){
            readI2C_part6(deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 28){
            if(DMA1->ISR & DMA_ISR_TCIF5) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 29){

            readI2C_part7(deviceAddress,0x74,28);
            *MPUInterruptionCounter = 0;
            *MPUInterruptionFlag = 0;

            *MPUwQuat = ((int)(*MPURawData)[0] << 24) | ((int)(*MPURawData)[1] << 16) | ((int)(*MPURawData)[2] << 8) | (int)(*MPURawData)[3];
            *MPUxQuat = ((int)(*MPURawData)[4] << 24) | ((int)(*MPURawData)[5] << 16) | ((int)(*MPURawData)[6] << 8) | (int)(*MPURawData)[7];
            *MPUyQuat = ((int)(*MPURawData)[8] << 24) | ((int)(*MPURawData)[9] << 16) | ((int)(*MPURawData)[10] << 8) | (int)(*MPURawData)[11];
            *MPUzQuat = ((int)(*MPURawData)[12] << 24) | ((int)(*MPURawData)[13] << 16) | ((int)(*MPURawData)[14] << 8) | (int)(*MPURawData)[15];

            *MPUxGyro = ((*MPURawData)[16] << 8) | (*MPURawData)[17];
            *MPUyGyro = ((*MPURawData)[18] << 8) | (*MPURawData)[18];
            *MPUzGyro = ((*MPURawData)[20] << 8) | (*MPURawData)[21];

            *MPUxAccel = ((*MPURawData)[22] << 8) | (*MPURawData)[22];
            *MPUyAccel = ((*MPURawData)[24] << 8) | (*MPURawData)[25];
            *MPUzAccel = ((*MPURawData)[26] << 8) | (*MPURawData)[27];


            
                
                
            *MPUSampleReadyFlag = 1;
  

        }else{

        }
    }
}

void getQuaternion(Quaternion *q, const int* MPUwQuat,const int* MPUxQuat, const int* MPUyQuat, const int* MPUzQuat) {

	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	q -> w = (float)(*MPUwQuat >> 16) / 16384.0f;
	q -> x = (float)(*MPUxQuat >> 16) / 16384.0f;
	q -> y = (float)(*MPUyQuat >> 16) / 16384.0f;
	q -> z = (float)(*MPUzQuat >> 16) / 16384.0f;
}
void getEuler(Quaternion *q, float* MPUYaw, float* MPUPitch, float* MPURoll){
    *MPUYaw = atan2(2 * q -> x * q -> y - 2 * q -> w * q -> z, 2 * q -> w * q -> w + 2 * q -> x * q -> x - 1); // psi
	*MPUPitch = -asin(2 * q -> x * q -> z + 2 * q -> w * q -> y);                      // theta
	*MPURoll = atan2(2 * q -> y * q -> z - 2 * q -> w * q -> x, 2 * q -> w * q -> w + 2 * q -> z * q -> z - 1); // phi
}

void normalizeEuler(float* MPUYaw, float* MPUPitch, float* MPURoll, float* MPUYawNormalized, 
                    float* MPUPitchNormalized, float* MPURollNormalized){
    
    *MPUYawNormalized = *MPUYaw / MPUMaxYaw;
    *MPUPitchNormalized = *MPUPitch / MPUMaxPitch;
    *MPURollNormalized = *MPURoll / MPUMaxRoll;

    if(*MPUYawNormalized >1){
        *MPUYawNormalized = 1;
    }else if(*MPUYawNormalized < -1){
        *MPUYawNormalized = -1;
    }

    if(*MPUPitchNormalized >1){
        *MPUPitchNormalized = 1;
    }else if(*MPUPitchNormalized < -1){
        *MPUPitchNormalized = -1;
    }

    if(*MPURollNormalized >1){
        *MPURollNormalized = 1;
    }else if(*MPURollNormalized < -1){
        *MPURollNormalized = -1;
    }
}


void EXTI1_IRQHandler(){
    
    
    MPU0InterruptionFlag = 1;

    // Prototipo de la lectura de datos MPU sin multitarea
    //unsigned char* fifoState = readI2C(0xD0,0x72,2);
    //unsigned char* inputData;    
    //if(fifoState >= 28){
    //    inputData = readI2C(0xD0,0x74,28);
    //}

    EXTI->PR |= EXTI_PR_PR1;                                // Limpiar interrupcion pendiente    

}

void getButtonsData(unsigned char* Button0, unsigned char* Button1){

    *Button0 = GPIOA->ODR |= GPIO_ODR_ODR0;
    *Button1 = GPIOA->ODR |= GPIO_ODR_ODR1;
}


int main(void){

    //--MICROCONTROLADOR--------------------------------------------------------------------------------------
    
    initMCU();
    initButtons();

    //--PERIFÉRICOS INTERNOS----------------------------------------------------------------------------------
    
    initI2C();


    initUART();

    initTimer4();

    //--PERIFÉRICOS EXTERNOS----------------------------------------------------------------------------------
    initMPU(0xD0, 0,MPU0_X_ACCEL_OFFSET, MPU0_Y_ACCEL_OFFSET, MPU0_Z_ACCEL_OFFSET, 
            MPU0_X_GYRO_OFFSET, MPU0_Y_GYRO_OFFSET, MPU0_Z_GYRO_OFFSET);

/*  
    initMPU(0xD1, 0,MPU0_X_ACCEL_OFFSET, MPU0_Y_ACCEL_OFFSET, MPU0_Z_ACCEL_OFFSET, 
            MPU0_X_GYRO_OFFSET, MPU0_Y_GYRO_OFFSET, MPU0_Z_GYRO_OFFSET);
*/

    // Evaluar comunicación I2C con MPU 

    while(TRUE){
        //GPIOC->ODR ^= GPIO_ODR_ODR13;	                                    // Toggle PC13			
        //wait_ms(500);

        // Lectura de botones:

        getButtonsData(&Button0, &Button1);

        // I2C RX-TX:

        getMPUData(0xD0, &MPU0InterruptionCounter, &MPU0InterruptionFlag,   // MPU0
            &MPU0wQuat, &MPU0xQuat, &MPU0yQuat, &MPU0zQuat, 
            &MPU0xGyro, &MPU0yGyro, &MPU0zGyro, 
            &MPU0xAccel, &MPU0yAccel, &MPU0zAccel, 
            &MPU0RawData, &MPU0SampleReadyFlag);

/*         
        getMPUData(0xD0, &MPU0InterruptionCounter, &MPU0InterruptionFlag,   // MPU0
            &MPU0wQuat, &MPU0xQuat, &MPU0yQuat, &MPU0zQuat, 
            &MPU0xGyro, &MPU0yGyro, &MPU0zGyro, 
            &MPU0xAccel, &MPU0yAccel, &MPU0zAccel, 
            &MPU0RawData, &MPU0SampleReadyFlag); 
*/


        MPU1SampleReadyFlag = 1;
        
        // UART TX:           

        if(MPU0SampleReadyFlag && MPU1SampleReadyFlag && (uartTxEmptyBufferFlag == 0)){
            
            printf("%1d,%1d,%06d,%06d,%06d,%06d\r\n",Button0,Button1,MPU0xAccel,MPU0yAccel,MPU0zAccel,MPU0xGyro);

            //printf("%1d,%1d,%06d,%06d,%06d,%06d\r\n",Button0,Button1,MPU0xAccel,MPU0yAccel,MPU0zAccel,MPU0xGyro);
            MPU1SampleReadyFlag = 0;
            MPU0SampleReadyFlag = 0;

        }
    }   

    return 0;
}