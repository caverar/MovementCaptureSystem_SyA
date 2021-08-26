#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif


#define UART_POLLING_OPTIMAL_MODE

void initUART(void);
char printSignedShort(unsigned char figure, short value);
void DMA1_Channel4_IRQHandler();
unsigned char uartTxEmptyBufferFlag;
unsigned char dummyRead;
void initDMA(void);
void initSimpleMode(void);


char uartTxBuffer[300];
unsigned int bufferSize;
int uartTxCounter;


#ifdef UART_COMPLEX_MODE
#define initDataTransferMethod(void) initDMA(void);
void complexPrint(char *msg, ...);
#define printf(msg, args...) complexPrint((char*)msg, ##args);
#endif


#ifdef UART_SIMPLE_MODE 
#define initDataTransferMethod(void) initSimpleMode(void);
void simplePrint(char *msg, ...);
#define printf(msg, args...) simplePrint(msg, ##args);
#endif

#ifdef UART_INTERRUPT_MODE
void USART1_IRQHandler();
#define initDataTransferMethod(void) initSimpleMode(void);
void interruptPrint(char *msg, ...);
#define printf(msg, args...) interruptPrint(msg, ##args);
#endif

#ifdef UART_POLLING_OPTIMAL_MODE
#define initDataTransferMethod(void) initSimpleMode(void);
void pollingOptimalPrint(char *msg, ...);
#define printf(msg, args...) pollingOptimalPrint(msg, ##args);
void pollingOptimalPrintManager(void);
#define printManager(void) pollingOptimalPrintManager(void);

#endif


#ifdef __cplusplus
}
#endif
#endif

