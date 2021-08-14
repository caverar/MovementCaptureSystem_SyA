#ifndef __I2C_H
#define __I2C_H

void initI2C(void);
unsigned char* readI2C(unsigned char deviceAddress, unsigned char address, unsigned char size);
void writeI2C(unsigned char deviceAddress, unsigned char address, unsigned char size, unsigned char *data);

void readI2C_part1(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C_part2(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C_part3(unsigned char deviceAddress, unsigned char address, unsigned char size);
unsigned char* readI2C_part4(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C_part5(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C_part6(unsigned char deviceAddress, unsigned char address, unsigned char size);
void readI2C_part7(unsigned char deviceAddress, unsigned char address, unsigned char size);

#endif

