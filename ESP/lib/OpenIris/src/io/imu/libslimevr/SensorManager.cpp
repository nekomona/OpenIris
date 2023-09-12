/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 TheDevMinerTV

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "SensorManager.h"
#include "ErroneousSensor.h"
#include "driver/i2c.h"
#include "io/imu/i2cscan/i2cscan.h"

namespace SlimeVR
{
    namespace Sensors
    {
        void SensorManager::buildSensor(uint8_t imuType, uint8_t address, int extraParam)
        {
            running = false;

            Serial.printf("Building IMU with: \n\
                            imuType=0x%02X, address=0x%02X,\n\
                            extraParam=%d\n",
                            imuType, address,
                            extraParam);

            // Now start detecting and building the IMU
            m_Sensor = nullptr;

            if (I2CSCAN::isI2CExist(address)) {
                Serial.printf("IMU found at address 0x%02X\n", address);
            } else {
                m_Sensor = new ErroneousSensor(imuType);
                Serial.printf("Can't find I2C device on provided address 0x%02X\n", address);
                return;
            }

            switch (imuType) {
            case IMU_BNO080: case IMU_BNO085: case IMU_BNO086:
                // Extra param used as interrupt pin
                {
                uint8_t intPin = extraParam;
                m_Sensor = new BNO080Sensor(imuType, address, intPin);
                }
                break;
            default:
                {
                m_Sensor = new ErroneousSensor(imuType);
                Serial.printf("IMU type $d not recognized\n", imuType);
                return;
                }
            }

            m_Sensor->motionSetup();

            if (m_Sensor->isWorking()) {
                Serial.println("IMU sensor configured");
            } else {
                Serial.println("IMU sensor init failed");
            }

            running = true;
        }

        void SensorManager::update() {
            if (m_Sensor && m_Sensor->isWorking()) {
                m_Sensor->motionLoop();
            }
        }
    }
}
