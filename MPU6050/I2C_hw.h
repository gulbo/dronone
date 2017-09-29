#ifndef I2C_hw_h
#define I2C_hw_h
 
#include "mbed.h"
 
#define I2C_SDA p9
#define I2C_SCL p10
 
class I2C_hw
{
private:
    I2C i2c;
 
public:
    Serial debugSerial;
    I2C_hw();
    I2C_hw(PinName i2cSda, PinName i2cScl);
 
    int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
    int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
    int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
    int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
 
    bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
};
 
#endif