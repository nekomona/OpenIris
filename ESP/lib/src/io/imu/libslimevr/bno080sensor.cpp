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

#include "bno080sensor.h"

namespace SlimeVR
{
    namespace Sensors
    {
        void BNO080Sensor::motionSetup()
        {
        #ifdef DEBUG_SENSOR
            imu.enableDebugging(Serial);
        #endif
            if(!imu.begin(addr, Wire, m_IntPin)) {
                Serial.printf("Can't connect to %s at address 0x%02x", getIMUNameByType(sensorType), addr);
                return;
            }

            Serial.printf("Connected to %s on 0x%02x. "
                        "Info: SW Version Major: 0x%02x "
                        "SW Version Minor: 0x%02x "
                        "SW Part Number: 0x%02x "
                        "SW Build Number: 0x%02x "
                        "SW Version Patch: 0x%02x\n",
                        getIMUNameByType(sensorType),
                        addr,
                        imu.swMajor,
                        imu.swMinor,
                        imu.swPartNumber,
                        imu.swBuildNumber,
                        imu.swVersionPatch
                        );

            // Only output calibrated sensor data
            imu.enableGyro(10);
            imu.enableAccelerometer(10);

            lastReset = 0;
            lastData = millis();
            working = true;
            configured = true;
        }

        void BNO080Sensor::motionLoop()
        {
            //Look for reports from the IMU
            while (imu.dataAvailable())
            {
                hadData = true;

                lastReset = 0;
                lastData = millis();

                if (imu.hasNewAccel()) {
                    imu.getAccel(this->acceleration[0], this->acceleration[1], this->acceleration[2], this->accelAccuracy);
                    imu.getGyro(this->angularVelocity[0], this->angularVelocity[1], this->angularVelocity[2], this->gyroAccuracy);

                    newAcceleration = true;
                }

                if (m_IntPin == 255 || imu.I2CTimedOut())
                    break;
            }

            if (lastData + 1000 < millis() && configured)
            {
                while(true) {
                    BNO080Error error = imu.readError();
                    if(error.error_source == 255)
                        break;
                    lastError = error;
                    Serial.printf("BNO08X error. Severity: %d, seq: %d, src: %d, err: %d, mod: %d, code: %d\n",
                        error.severity, error.error_sequence_number, error.error_source, error.error, error.error_module, error.error_code);
                }
                working = false;
                lastData = millis();
                uint8_t rr = imu.resetReason();
                Serial.printf("Sensor doesn't respond. Last reset reason: %d\n", rr);
                Serial.printf("Last error: %d, seq: %d, src: %d, err: %d, mod: %d, code: %d\n",
                        lastError.severity, lastError.error_sequence_number, lastError.error_source, lastError.error, lastError.error_module, lastError.error_code);
            }
        }

        SensorStatus BNO080Sensor::getSensorState() {
            return lastReset > 0 ? SensorStatus::SENSOR_ERROR : isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_OFFLINE;
        }
    }
}
