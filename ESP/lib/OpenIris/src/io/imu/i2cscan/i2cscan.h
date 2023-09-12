#ifndef _I2CSCAN_H_
#define _I2CSCAN_H_ 1

#include <Arduino.h>
#include <Wire.h>

namespace I2CSCAN {
    bool isI2CExist(uint8_t addr);
    int clearBus(uint8_t SDA, uint8_t SCL);
}

#endif // _I2CSCAN_H_
