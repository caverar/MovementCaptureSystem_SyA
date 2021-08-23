#ifndef __UART_H
#define __UART_H

#define UART_COMPLEX_MODE

void initUART(void);
char printSignedShort(unsigned char figure, short value);
void DMA1_Channel4_IRQHandler();
unsigned char uartTxEmptyBufferFlag;
void initDMAComplexMode(void);
void initDMASimpleMode(void);
char uartTxBuffer[300];
unsigned int bufferSize;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef UART_COMPLEX_MODE
#define initDMA(void) initDMAComplexMode(void);
void complexPrint(char *msg, ...);
#define printf(msg, args...) complexPrint((char*)msg, ##args);
#endif


#ifndef UART_COMPLEX_MODE //UART_SIMPLE_MODE
#define initDMA(void) initDMASimpleMode(void);
void simplePrint(char *msg, ...);
#define printf(msg, args...) simplePrint(msg, ##args);
#endif


#endif

#ifdef __cplusplus
}
#endif