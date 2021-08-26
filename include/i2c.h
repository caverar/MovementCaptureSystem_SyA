#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif


void initI2C1(void);

unsigned char* readI2C1(unsigned char deviceAddress, unsigned char address, unsigned char size);
void writeI2C1(unsigned char deviceAddress, unsigned char address, unsigned char size, unsigned char* data);

void readI2C1_part1(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C1_part2(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C1_part3(unsigned char deviceAddress, unsigned char address, unsigned char size);
unsigned char* readI2C1_part4(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C1_part5(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C1_part6(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C1_part7(unsigned char deviceAddress, unsigned char address, unsigned char size);

unsigned short getI2C1_SB(void);
unsigned short getI2C1_ADDR(void);
unsigned short getI2C1_AF(void);
void clearI2C1_AF(void);
unsigned short getI2C1_TXE(void);
unsigned int getI2C1_DMA_TCIF(void);

void initI2C2(void);

unsigned char* readI2C2(unsigned char deviceAddress, unsigned char address, unsigned char size);
void writeI2C2(unsigned char deviceAddress, unsigned char address, unsigned char size, unsigned char* data);

void readI2C2_part1(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C2_part2(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C2_part3(unsigned char deviceAddress, unsigned char address, unsigned char size);
unsigned char* readI2C2_part4(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C2_part5(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C2_part6(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C2_part7(unsigned char deviceAddress, unsigned char address, unsigned char size);

unsigned short getI2C2_SB(void);
unsigned short getI2C2_ADDR(void);
unsigned short getI2C2_AF(void);
void clearI2C2_AF(void);
unsigned short getI2C2_TXE(void);
unsigned int getI2C2_DMA_TCIF(void);


void initI2C(unsigned char i2cPort);
unsigned char* readI2C(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size);
void writeI2C(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size, unsigned char* data);

void readI2C_part1(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C_part2(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C_part3(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size);
unsigned char* readI2C_part4(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C_part5(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C_part6(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C_part7(unsigned char i2cPort, unsigned char deviceAddress, unsigned char address, unsigned char size);

unsigned short getI2C_SB(unsigned char i2cPort);
unsigned short getI2C_ADDR(unsigned char i2cPort);
unsigned short getI2C_AF(unsigned char i2cPort);
void clearI2C_AF(unsigned char i2cPort);
unsigned short getI2C_TXE(unsigned char i2cPort);
unsigned int getI2C_DMA_TCIF(unsigned char i2cPort);


#endif

#ifdef __cplusplus
}
#endif