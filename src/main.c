#include <stm32f103xb.h>

#include "timer4.h"
#include "uart.h"

#include "i2c.h"
#include "quaternion_utils.h"
// #include "dmpfirmware.h"

#include "MPU6050.h"

#define TRUE 0x01
#define FALSE 0x00



#define MPU0_X_ACCEL_OFFSET -2606
#define MPU0_Y_ACCEL_OFFSET -548
#define MPU0_Z_ACCEL_OFFSET 2554
#define MPU0_X_GYRO_OFFSET 60
#define MPU0_Y_GYRO_OFFSET 1
#define MPU0_Z_GYRO_OFFSET -47

#define MPU1_X_ACCEL_OFFSET -118
#define MPU1_Y_ACCEL_OFFSET -2610
#define MPU1_Z_ACCEL_OFFSET 2750
#define MPU1_X_GYRO_OFFSET 3
#define MPU1_Y_GYRO_OFFSET 14
#define MPU1_Z_GYRO_OFFSET -26

float MPUMaxYaw = 0;                                        // Parameters experimentales, dependen del angulo al que se quiere 1
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
float averageMPU0Yaw = 0.0;
float averageMPU0Pitch = 0.0; 
float averageMPU0Roll = 0.0; 
float offsetMPU0Yaw = 0.0; 
float offsetMPU0Pitch = 0.0; 
float offsetMPU0Roll = 0.0;
int filterCounterMPU0 = 0;



unsigned char MPU0InterruptionFlag = 0;
unsigned char MPU0InterruptionCounter = 0;
unsigned char* MPU0RawData;
unsigned char MPU0SampleReadyFlag = 0;
unsigned char Button0 =0;
struct Quaternion MPU0Quaternion;
struct Gravity MPU0Gravity;


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

float MPU1Yaw = 0.0;
float MPU1Pitch = 0.0;
float MPU1Roll = 0.0;
float averageMPU1Yaw = 0.0;
float averageMPU1Pitch = 0.0; 
float averageMPU1Roll = 0.0; 
float offsetMPU1Yaw = 0.0; 
float offsetMPU1Pitch = 0.0; 
float offsetMPU1Roll = 0.0;
int filterCounterMPU1 = 0;



unsigned char MPU1InterruptionFlag = 0;
unsigned char MPU1InterruptionCounter = 0;    
unsigned char* MPU1RawData;
unsigned char MPU1SampleReadyFlag = 0;
unsigned char Button1 =0;
struct Quaternion MPU1Quaternion;
struct Gravity MPU1Gravity;


unsigned char i2cMPUCurrentUser;

extern unsigned char uartTxEmptyBufferFlag;

unsigned char uartSamplingCounter = 0;                            // Contador para muestreo de datos, cada 5 muestras reales. (50ms)


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




void EXTI1_IRQHandler(){    
    
    MPU1InterruptionFlag = 1;
    EXTI->PR |= EXTI_PR_PR1;                                // Limpiar interrupcion pendiente    

}

void EXTI0_IRQHandler(){    
    
    MPU0InterruptionFlag = 1;
    EXTI->PR |= EXTI_PR_PR0;                                // Limpiar interrupcion pendiente    

}

void getButtonsData(unsigned char* Button0, unsigned char* Button1){

    *Button0 = GPIOA->IDR & GPIO_IDR_IDR0;
    *Button1 = (GPIOA->IDR & GPIO_IDR_IDR1) >> 1;
}


int main(void){

    //--MICROCONTROLADOR--------------------------------------------------------------------------------------
    
    initMCU();
    initButtons();

    //--PERIFÉRICOS INTERNOS----------------------------------------------------------------------------------
    
    initI2C(1);
    initI2C(2);
    initUART();
    initTimer4();


    simplePrint("Preparese, 5 segundos para iniciar calibracion...\r\n");
    wait_ms(1000);    
    simplePrint("4 segundos para iniciar calibracion...\r\n");
    wait_ms(1000);
    simplePrint("3 segundos para iniciar calibracion...\r\n");
    wait_ms(1000); 
     simplePrint("2 segundos para iniciar calibracion...\r\n");
    wait_ms(1000);     
    simplePrint("1 segundo para iniciar calibracion...\r\n");
    wait_ms(1000);


    //--PERIFÉRICOS EXTERNOS----------------------------------------------------------------------------------

    initMPUPart1(1, 0xD0);                                                                // MPU0
    initMPUPart1(2, 0xD0);                                                                  // MPU1
    wait_ms(200);
    initMPUPart2(1, 0xD0, MPU0_X_ACCEL_OFFSET, MPU0_Y_ACCEL_OFFSET, MPU0_Z_ACCEL_OFFSET,  // MPU0
                MPU0_X_GYRO_OFFSET, MPU0_Y_GYRO_OFFSET, MPU0_Z_GYRO_OFFSET);
    initMPUPart2(2, 0xD0, MPU1_X_ACCEL_OFFSET, MPU1_Y_ACCEL_OFFSET, MPU1_Z_ACCEL_OFFSET,    // MPU1
                MPU1_X_GYRO_OFFSET, MPU1_Y_GYRO_OFFSET, MPU1_Z_GYRO_OFFSET);

    initMPUPart3(1, 0xD0, 0);                                                             // MPU0
    wait_us(200);
    initMPUPart3(2, 0xD0, 1);                                                               // MPU1
    wait_us(200);   

    i2cMPUCurrentUser = 0;

    
    NVIC_EnableIRQ(EXTI0_IRQn);                                                             // MPU0
    NVIC_EnableIRQ(EXTI1_IRQn);                                                             // MPU1
    
    printf("Porfavor no se mueva, Calibrando...\r\n");
    // Evaluar comunicación I2C con MPU 

    while(TRUE){
        //GPIOC->ODR ^= GPIO_ODR_ODR13;	                                                    // Toggle PC13			
        //wait_ms(500);

        // Lectura de botones:

        getButtonsData(&Button0, &Button1);

        // I2C RX-TX:


        getMPUData(1, 0xD0, &MPU0InterruptionCounter, &MPU0InterruptionFlag,              // MPU0
            &MPU0wQuat, &MPU0xQuat, &MPU0yQuat, &MPU0zQuat, 
            &MPU0xGyro, &MPU0yGyro, &MPU0zGyro, 
            &MPU0xAccel, &MPU0yAccel, &MPU0zAccel, 
            &MPU0RawData, &MPU0SampleReadyFlag, &MPU0Quaternion, &MPU0Gravity,
            &MPU0Yaw, &MPU0Pitch, &MPU0Roll,
            &averageMPU0Yaw, &averageMPU0Pitch, &averageMPU0Roll, 
            &filterCounterMPU0, &offsetMPU0Yaw, &offsetMPU0Pitch, &offsetMPU0Roll); 


         
        getMPUData(2, 0xD0, &MPU1InterruptionCounter, &MPU1InterruptionFlag,                // MPU1
            &MPU1wQuat, &MPU1xQuat, &MPU1yQuat, &MPU1zQuat, 
            &MPU1xGyro, &MPU1yGyro, &MPU1zGyro, 
            &MPU1xAccel, &MPU1yAccel, &MPU1zAccel, 
            &MPU1RawData, &MPU1SampleReadyFlag, &MPU1Quaternion, &MPU1Gravity,
            &MPU1Yaw, &MPU1Pitch, &MPU1Roll,
            &averageMPU1Yaw, &averageMPU1Pitch, &averageMPU1Roll, 
            &filterCounterMPU1, &offsetMPU1Yaw, &offsetMPU1Pitch, &offsetMPU1Roll);  



        //MPU1SampleReadyFlag = 1;        
        //filterCounterMPU1 = 600;
         


        
        // UART TX:           
        printManager();
        if(MPU0SampleReadyFlag && MPU1SampleReadyFlag && (uartTxEmptyBufferFlag == 0)){
            
            //printf("%1d,%1d,%06d,%06d,%06d,%06d\r\n",Button0,Button1,MPU0Yaw,MPU0Pitch,MPU1Yaw,MPU1Pitch);
            //printf("%1d,%1d,%06d,%06d,%06d,%06d\r\n",Button0,Button1,MPU0Yaw,MPU0Pitch,MPU0Roll,MPU0xGyro);            
            
            //printf("%1d,%1d,%05d,%05d,%05d,%05d\r\n", Button0, Button1, MPU0PitchSendable, MPU0RollSendable, MPU1PitchSendable, MPU1RollSendable);
            

            if((filterCounterMPU0 >=600) && (filterCounterMPU1 >=600)){

                if(uartSamplingCounter<4){
                    uartSamplingCounter++;
                }else{
                    //printf("%d,%d\r\n",(int)(MPU0Pitch*1000),(int)(MPU0Roll*1000));
                    printf("%1d,%1d,%05d,%05d,%05d,%05d\r\n", Button0, Button1, (int)(MPU0Pitch*1000), (int)(MPU0Roll*1000), (int)(MPU1Pitch*1000), (int)(MPU1Roll*1000));
                    uartSamplingCounter = 0;
                }

              
            }

            MPU1SampleReadyFlag = 0;
            MPU0SampleReadyFlag = 0;

        }
    }   

    return 0;
}