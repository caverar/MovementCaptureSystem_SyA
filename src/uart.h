#ifndef __UART_H
#define __UART_H


void initUART(void);
char printSignedShort(unsigned char figure, short value);
void DMA1_Channel4_IRQHandler();
//typedef void printFunction(char *msg, ...);
unsigned char uartTxEmptyBufferFlag;

#ifdef UART_COMPLEX_MODE

void initDMAComplexMode(void);
#define initDMA() void initDMAComplexMode(void);

unsigned char getUartTxEmptyBufferFlag(void);
void complexPrint(char *msg, ...);
#define printf(msg, args...) complexPrint(msg, ##args);



#else //UART_SIMPLE_MODE
void initDMASimpleMode(void);
#define initDMA() void initDMASimpleMode(void);
void simplePrint(char *msg, ...);
#define printf(msg, args...) simplePrint(msg, ##args);
#endif

#endif

