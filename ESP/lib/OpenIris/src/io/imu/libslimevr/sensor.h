/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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

#ifndef SLIMEVR_SENSOR_H_
#define SLIMEVR_SENSOR_H_

#include <Arduino.h>

// #define DEBUG_SENSOR

#define DATA_TYPE_NORMAL 1
#define DATA_TYPE_CORRECTION 2

#define IMU_UNKNOWN 0
#define IMU_BNO080 3
#define IMU_BNO085 4
#define IMU_BNO086 7
#define IMU_DEV_RESERVED 250 // Reserved, should not be used in any release firmware

enum class SensorStatus : uint8_t {
    SENSOR_OFFLINE = 0,
    SENSOR_OK = 1,
    SENSOR_ERROR = 2
};

namespace SlimeVR
{
    namespace Sensors
    {
        class Sensor
        {
        public:
            Sensor(const char *sensorName, uint8_t type, uint8_t address)
                : addr(address), sensorType(type) {}

            virtual ~Sensor(){};
            virtual void motionSetup(){};
            virtual void postSetup(){};
            virtual void motionLoop(){};
            virtual SensorStatus getSensorState();

            bool isWorking() {
                return working;
            };
            uint8_t getSensorType() {
                return sensorType;
            };
            bool hasNewDataToSend() {
                return newAcceleration;
            };
            const float * getAcceleration() {
                return acceleration;
            };
            const float * getangularVelocity() {
                return angularVelocity;
            };
            void takeData() {
                newAcceleration = false;
            }

            bool hadData = false;
        protected:
            uint8_t addr = 0;
            uint8_t sensorType = 0;
            bool configured = false;
            bool working = false;
            uint8_t gyroAccuracy = 0;
            uint8_t accelAccuracy = 0;

            bool newAcceleration = false;
            float acceleration[3]{};
            float angularVelocity[3]{};
        };

        const char * getIMUNameByType(int imuType);
    }
}

#endif // SLIMEVR_SENSOR_H_
