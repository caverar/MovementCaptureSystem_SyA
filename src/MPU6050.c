#include <stm32f103xb.h>
#include "i2c.h"
#include "timer4.h"

#include "quaternion_utils.h"
#include "dmpfirmware.h"

#include "MPU6050.h"



void initMPUPart1(unsigned char i2cPort, unsigned char mpu6050Address){
    //int *inputI2CBuffer; 
    unsigned char outputI2CBuffer[2];

    
    outputI2CBuffer[0] = 0x80;
    writeI2C(i2cPort, mpu6050Address,0x6B,1,outputI2CBuffer);                  // Reset MPU6050    
    outputI2CBuffer[0] = 0x00;                              
    writeI2C(i2cPort, mpu6050Address,0x6A,1,outputI2CBuffer);                  // Reset FIFO

}

void initMPUPart2(unsigned char i2cPort, unsigned char mpu6050Address, 
                short MPU_X_ACCEL_OFFSET, short MPU_Y_ACCEL_OFFSET, short MPU_Z_ACCEL_OFFSET, 
                short MPU_X_GYRO_OFFSET, short MPU_Y_GYRO_OFFSET, short MPU_Z_GYRO_OFFSET){      
    
    
    //-----------------------Inicializar MPU------------------------------------------------------------------

    unsigned char outputI2CBuffer[16];

    // Carga DMP

    outputI2CBuffer[0] = 0x00;                              
    writeI2C(i2cPort, mpu6050Address,0x6B,1,outputI2CBuffer);
    outputI2CBuffer[0] = 0x00;                              
    writeI2C(i2cPort, mpu6050Address,0x6C,1,outputI2CBuffer);    
    outputI2CBuffer[0] = 0x03;                              
    writeI2C(i2cPort, mpu6050Address,0x1A,1,outputI2CBuffer);
    outputI2CBuffer[0] = 0x18;                              
    writeI2C(i2cPort, mpu6050Address,0x1B,1,outputI2CBuffer);
    outputI2CBuffer[0] = 0x00;                              
    writeI2C(i2cPort, mpu6050Address,0x1C,1,outputI2CBuffer);
    outputI2CBuffer[0] = 0x00;                              
    writeI2C(i2cPort, mpu6050Address,0x23,1,outputI2CBuffer);
    outputI2CBuffer[0] = 0x00;                              
    writeI2C(i2cPort, mpu6050Address,0x38,1,outputI2CBuffer);
    outputI2CBuffer[0] = 0x04;                              
    writeI2C(i2cPort, mpu6050Address,0x6A,1,outputI2CBuffer);    
    outputI2CBuffer[0] = 0x04;                              
    writeI2C(i2cPort, mpu6050Address,0x19,1,outputI2CBuffer);   

    // firmware

    int i = 0;
    int j = 0;

    for(i = 0; i<192; i++){
        
        short data = 0x0000 + (i*0x10);
        outputI2CBuffer[0] = (data >> 8);
        outputI2CBuffer[1] = data; 
        writeI2C(i2cPort, mpu6050Address,0x6D,2,outputI2CBuffer);

        if(i<191){            
            for(j = 0; j<16; j++){
                outputI2CBuffer[j] = dmp_memory[(i*16) + j];   
            }
            writeI2C(i2cPort, 0xD0,0x6F,16,outputI2CBuffer);
        }else{            
            for(j = 0; j<6; j++){
                outputI2CBuffer[j] = dmp_memory[(i*16) + j];   
            }
            writeI2C(i2cPort, mpu6050Address,0x6F,6,outputI2CBuffer);
        }    
    }

    outputI2CBuffer[0] = 0x04;
    outputI2CBuffer[1] = 0x00;                              
    writeI2C(i2cPort, mpu6050Address,0x70,2,outputI2CBuffer);


    // Ajustar valores de offset
    wait_us(300);

    outputI2CBuffer[0] = (unsigned char) (MPU_X_ACCEL_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_X_ACCEL_OFFSET;
    writeI2C(i2cPort, mpu6050Address,MPU_X_ACCEL_OFFSET_ADDRESS,2,outputI2CBuffer);

    outputI2CBuffer[0] = (unsigned char) (MPU_Y_ACCEL_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_Y_ACCEL_OFFSET;
    writeI2C(i2cPort, mpu6050Address,MPU_Y_ACCEL_OFFSET_ADDRESS,2,outputI2CBuffer);

    outputI2CBuffer[0] = (unsigned char) (MPU_Z_ACCEL_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_Z_ACCEL_OFFSET;
    writeI2C(i2cPort, mpu6050Address,MPU_Z_ACCEL_OFFSET_ADDRESS,2,outputI2CBuffer);

    outputI2CBuffer[0] = (unsigned char) (MPU_X_GYRO_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_X_GYRO_OFFSET;
    writeI2C(i2cPort, mpu6050Address,MPU_X_GYRO_OFFSET_ADDRESS,2,outputI2CBuffer);

    outputI2CBuffer[0] = (unsigned char) (MPU_Y_GYRO_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_Y_GYRO_OFFSET;
    writeI2C(i2cPort, mpu6050Address,MPU_Y_GYRO_OFFSET_ADDRESS,2,outputI2CBuffer);

    outputI2CBuffer[0] = (unsigned char) (MPU_Z_GYRO_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_Z_GYRO_OFFSET;
    writeI2C(i2cPort, mpu6050Address,MPU_Z_GYRO_OFFSET_ADDRESS,2,outputI2CBuffer);

    wait_ms(3);

    outputI2CBuffer[0] = (unsigned char) (MPU_X_ACCEL_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_X_ACCEL_OFFSET;
    outputI2CBuffer[2] = (unsigned char) (MPU_Y_ACCEL_OFFSET >> 8);
    outputI2CBuffer[3] = (unsigned char) MPU_Y_ACCEL_OFFSET;
    outputI2CBuffer[4] = (unsigned char) (MPU_Z_ACCEL_OFFSET >> 8);
    outputI2CBuffer[5] = (unsigned char) MPU_Z_ACCEL_OFFSET;
    writeI2C(i2cPort, mpu6050Address,MPU_X_ACCEL_OFFSET_ADDRESS,6,outputI2CBuffer);

    wait_ms(3);

    outputI2CBuffer[0] = (unsigned char) (MPU_X_GYRO_OFFSET >> 8);
    outputI2CBuffer[1] = (unsigned char) MPU_X_GYRO_OFFSET;
    outputI2CBuffer[2] = (unsigned char) (MPU_Y_GYRO_OFFSET >> 8);
    outputI2CBuffer[3] = (unsigned char) MPU_Y_GYRO_OFFSET;
    outputI2CBuffer[4] = (unsigned char) (MPU_Z_GYRO_OFFSET >> 8);
    outputI2CBuffer[5] = (unsigned char) MPU_Z_GYRO_OFFSET;
    writeI2C(i2cPort, mpu6050Address,MPU_X_GYRO_OFFSET_ADDRESS,6,outputI2CBuffer);

    wait_ms(3); 

    // // Interruption pin bypass configuration
    // outputI2CBuffer[0] = 0x02;
    // writeI2C(mpu6050Address,0x37,1,outputI2CBuffer);

    // // FIFO Buffer configuration
    // outputI2CBuffer[0] = 0xC0;
    // writeI2C(mpu6050Address,0x6A,1,outputI2CBuffer);

    // // Interruption pin bypass configuration
    // outputI2CBuffer[0] = 0x02;
    // writeI2C(mpu6050Address,0x38,1,outputI2CBuffer);
    // // First Read
    // MPU0RawData = readI2C(mpu6050Address,0x72,2);

    
}

void initMPUPart3(unsigned char i2cPort, unsigned char mpu6050Address, unsigned char interruptPin){
    unsigned char outputI2CBuffer[2];

    // Interruption pin bypass configuration
    outputI2CBuffer[0] = 0x02;
    writeI2C(i2cPort, mpu6050Address,0x37,1,outputI2CBuffer);
    // FIFO Buffer configuration
    outputI2CBuffer[0] = 0xC0;
    writeI2C(i2cPort, mpu6050Address,0x6A,1,outputI2CBuffer);
    // Interruption pin bypass configuration
    outputI2CBuffer[0] = 0x02;
    writeI2C(i2cPort, mpu6050Address,0x38,1,outputI2CBuffer);
    // First Read
    dummyMPURead = readI2C(i2cPort, 0xD0,0x72,2);

    //-----------------------Inicializar pin interrupcion-----------------------------------------------------

    if(interruptPin==1){

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
        //NVIC_EnableIRQ(EXTI1_IRQn);                       // Habilitar interrupciones de la linea 1 

        //EXTI->SWIER |= EXTI_SWIER_SWI1;                   // Software interrupt 
        //EXTI->PR |= EXTI_PR_PR1;                          // Limpiar interrupcion pendiente   

    }else{
        // Pin B0
        
        RCC->APB2ENR |=RCC_APB2ENR_IOPBEN;			        // IOPBEN=1, Habilitar reloj del Puerto B
        GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);    // Limpiar registros de configuración
        GPIOB->CRL &= ~GPIO_CRL_MODE0;				        // MODE0=0x00, PB0 en modo de entrada
        GPIOB->CRL |= GPIO_CRL_CNF0_1;				        // CNF0=0x10,  PB0 en modo pull-up/pull-down
        GPIOB->ODR &= ~GPIO_ODR_ODR0;						// ODR0=0 Habilitar resistencia de PullDown

        // Linea 0, dado que se usa B0. Ejemplos: Linea 2: EXT2, A2,C2,B2, etc
    
        RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                 // Habilitar reloj de AFIO
        AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PB;           // Habilitar interrupcion en PB0
        EXTI->IMR |=  EXTI_IMR_MR0;                         // Mask interrupcion de linea 0
        EXTI->RTSR |= EXTI_RTSR_TR0;                        // Habilitar trigger con rising edge
        //NVIC_EnableIRQ(EXTI0_IRQn);                       // Habilitar interrupciones de la linea 0 
    
        //EXTI->SWIER |= EXTI_SWIER_SWI0;                   // Software interrupt 
        //EXTI->PR |= EXTI_PR_PR0;                          // Limpiar interrupcion pendiente  
    }
}

void getMPUData(unsigned char i2cPort, unsigned char deviceAddress, unsigned char* MPUInterruptionCounter, unsigned char* MPUInterruptionFlag,
                int* MPUwQuat, int* MPUxQuat, int* MPUyQuat, int* MPUzQuat,
                short* MPUxGyro, short* MPUyGyro, short* MPUzGyro,
                short* MPUxAccel, short* MPUyAccel, short* MPUzAccel,
                unsigned char** MPURawData, unsigned char* MPUSampleReadyFlag, struct Quaternion* MPUQuaternion, struct Gravity* MPUGravity,
                float* MPUYaw, float* MPUPitch, float* MPURoll,
                float* averageMPUYaw, float* averageMPUPitch, float* averageMPURoll, 
                int* counter, float* offsetMPUYaw, float* offsetMPUPitch, float* offsetMPURoll){

    if(*MPUInterruptionFlag){

        if(*MPUInterruptionCounter == 0){
            readI2C_part1(i2cPort, deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 1){
            if(getI2C_SB(i2cPort)) (*MPUInterruptionCounter)++; 

        }else if(*MPUInterruptionCounter == 2){
            readI2C_part2(i2cPort, deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 3){
            if(getI2C_ADDR(i2cPort)){
                (*MPUInterruptionCounter)++;
            }else if(getI2C_AF(i2cPort)){
                clearI2C_AF(i2cPort);
                *MPUInterruptionCounter = 0;
                *MPUInterruptionFlag = 0;
            }

        }else if(*MPUInterruptionCounter == 4){
            readI2C_part3(i2cPort, deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 5){
            if(getI2C_TXE(i2cPort)) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 6){
            (*MPURawData) = readI2C_part4(i2cPort, deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 7){
            if(getI2C_SB(i2cPort)) (*MPUInterruptionCounter)++;  
             
        }else if(*MPUInterruptionCounter == 8){
            readI2C_part5(i2cPort, deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 9){
            if(getI2C_ADDR(i2cPort)) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 10){
            readI2C_part6(i2cPort, deviceAddress,0x72,2);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 11){
            if(getI2C_DMA_TCIF(i2cPort)) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 12){

            readI2C_part7(i2cPort, deviceAddress,0x72,2);
                
            unsigned short fifoSize = ((*MPURawData)[0]<<8) + (*MPURawData)[1];
            if(fifoSize<28){
                *MPUInterruptionCounter = 0;
                *MPUInterruptionFlag = 0;
            }else{
                (*MPUInterruptionCounter)++;
            }

        // Lectura RawData x28

        }else if(*MPUInterruptionCounter == 13){
            readI2C_part1(i2cPort, deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 14){
            if(getI2C_SB(i2cPort)) (*MPUInterruptionCounter)++; 

        }else if(*MPUInterruptionCounter == 15){
            readI2C_part2(i2cPort, deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 16){
            if(getI2C_ADDR(i2cPort)) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 17){
            readI2C_part3(i2cPort, deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 18){
            if(getI2C_TXE(i2cPort)) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 19){
            (*MPURawData) = readI2C_part4(i2cPort, deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 20){
            if(getI2C_SB(i2cPort)) (*MPUInterruptionCounter)++;  
             
        }else if(*MPUInterruptionCounter == 21){
            readI2C_part5(i2cPort, deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 22){
            if(getI2C_ADDR(i2cPort)) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 23){
            readI2C_part6(i2cPort, deviceAddress,0x74,28);
            (*MPUInterruptionCounter)++;
        }else if(*MPUInterruptionCounter == 24){
            if(getI2C_DMA_TCIF(i2cPort)) (*MPUInterruptionCounter)++;

        }else if(*MPUInterruptionCounter == 25){

            readI2C_part7(i2cPort, deviceAddress,0x74,28);
            *MPUInterruptionCounter = 0;
            *MPUInterruptionFlag = 0;

            *MPUwQuat = ((*MPURawData)[0] << 24) | ((*MPURawData)[1] << 16) | ((*MPURawData)[2] << 8) | (*MPURawData)[3];
            *MPUxQuat = ((*MPURawData)[4] << 24) | ((*MPURawData)[5] << 16) | ((*MPURawData)[6] << 8) | (*MPURawData)[7];
            *MPUyQuat = ((*MPURawData)[8] << 24) | ((*MPURawData)[9] << 16) | ((*MPURawData)[10] << 8) | (*MPURawData)[11];
            *MPUzQuat = ((*MPURawData)[12] << 24) | ((*MPURawData)[13] << 16) | ((*MPURawData)[14] << 8) | (*MPURawData)[15];

            //*MPUwQuat = ((*MPURawData)[0] << 8) | (*MPURawData)[1]; 
            //*MPUxQuat = ((*MPURawData)[4] << 8) | (*MPURawData)[5]; 
            //*MPUyQuat = ((*MPURawData)[8] << 8) | (*MPURawData)[9];
            //*MPUzQuat = ((*MPURawData)[12] << 8) | (*MPURawData)[13];



            *MPUxGyro = ((*MPURawData)[16] << 8) | (*MPURawData)[17];
            *MPUyGyro = ((*MPURawData)[18] << 8) | (*MPURawData)[18];
            *MPUzGyro = ((*MPURawData)[20] << 8) | (*MPURawData)[21];

            *MPUxAccel = ((*MPURawData)[22] << 8) | (*MPURawData)[22];
            *MPUyAccel = ((*MPURawData)[24] << 8) | (*MPURawData)[25];
            *MPUzAccel = ((*MPURawData)[26] << 8) | (*MPURawData)[27];

            getQuaternion(MPUQuaternion,MPUwQuat, MPUxQuat,MPUyQuat, MPUzQuat);
            getGravity(MPUQuaternion, MPUGravity);
            getYawPitchRoll(MPUQuaternion, MPUGravity, MPUYaw, MPUPitch, MPURoll);

            filterYawPitchRoll(MPUYaw, MPUPitch, MPURoll, averageMPUYaw, averageMPUPitch, averageMPURoll, 
                                counter, offsetMPUYaw, offsetMPUPitch, offsetMPURoll);


                
            *MPUSampleReadyFlag = 1;
            


        }else{

        }
    }
}


