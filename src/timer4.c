#ifndef __TIMER4_H
#define __TIMER4_H

#include "timer4.h"
#include <stm32f103xb.h>

void initTimer4(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;                     // Habilitar reloj
    TIM4->PSC = 72;                                         // Prescaler en para reloj de 1Mhz
    TIM4->ARR = 0xFFFF;
    TIM4->CR1 |= TIM_CR1_URS;                               // Update interrupt enable Habilitar interrupciones o eventos en overflow
    TIM4->DIER |= TIM_DIER_UIE;                             // Habilitar interrupcion UI
    TIM4->EGR |= TIM_EGR_UG;								// Reiniciar contador y actualizar registros
}
void wait_ms(unsigned int time){
    


    for(int i=0; i<time; i++){

        TIM4->ARR = 1000;
        TIM4->EGR |= TIM_EGR_UG;							// Reiniciar contador y actualizar registros
        TIM4->SR &= ~TIM_SR_UIF;                            // Limpiar flag
	    TIM4->CR1 |= TIM_CR1_CEN;							// Habilitar contador
        TIM4->SR &= ~TIM_SR_UIF;                            // Limpiar flag
        while(!(TIM4->SR & TIM_SR_UIF));                    // Revisar si ya se cumplido dio el overflow  
        TIM4->SR &= ~TIM_SR_UIF;                            // Limpiar flag
        TIM4->CR1 &= ~TIM_CR1_CEN;							// Deshabilitar contador


    /*  TIM4->CR1 &= ~TIM_CR1_CEN;							// Deshabilitar contador 
        TIM4->ARR = 0xFFFF;
        TIM4->EGR |= TIM_EGR_UG;							// Reiniciar contador y actualizar registros
        TIM4->CR1 |= TIM_CR1_CEN;							// Habilitar contador
        TIM4->SR = 0;                                       // Limpiar flag
        while(TIM4->CNT<1000);                              // Revisar si no se ha cumplido el overflow   
        TIM4->CR1 &= ~TIM_CR1_CEN;							// Deshabilitar contador 
    */
    }
}
void wait_us(unsigned short time){
    
    //TIM4->ARR = time;
    //TIM4->EGR |= TIM_EGR_UG;							// Reiniciar contador y actualizar registros
    //TIM4->SR &= ~TIM_SR_UIF;                          // Limpiar flag
	//TIM4->CR1 |= TIM_CR1_CEN;							// Habilitar contador
    //TIM4->SR &= ~TIM_SR_UIF;                          // Limpiar flag
    //while(!(TIM4->SR & TIM_SR_UIF));                  // Revisar si ya se cumplió dio el overflow  
    //TIM4->SR &= ~TIM_SR_UIF;                          // Limpiar flag
    //TIM4->CR1 &= ~TIM_CR1_CEN;						// Deshabilitar contador


    TIM4->CR1 &= ~TIM_CR1_CEN;							// Deshabilitar contador 
    TIM4->ARR = 0xFFFF;
    TIM4->EGR |= TIM_EGR_UG;							// Reiniciar contador y actualizar registros
    TIM4->CR1 |= TIM_CR1_CEN;							// Habilitar contador
    TIM4->SR = 0;                                       // Limpiar flag
    while(TIM4->CNT<time);                              // Revisar si no se ha cumplido el overflow   
    TIM4->CR1 &= ~TIM_CR1_CEN;							// Deshabilitar contador 
}

#endif