#include "imuServer.hpp"
#include <Arduino.h>
#include <Wire.h>
#include <libb64/cencode.h>

constexpr static const char *IMU_CONTENT_TYPE = "multipart/form-data;boundary=" IMU_PART_BOUNDARY;
constexpr static const char *IMU_BOUNDARY = "\r\n--" IMU_PART_BOUNDARY "\r\n";

esp_err_t IMUHelpers::response(httpd_req_t *req)
{
    esp_err_t res = ESP_OK;

    SlimeVR::Sensors::SensorManager *sensorManager = (SlimeVR::Sensors::SensorManager *)req->user_ctx;
    SlimeVR::Sensors::Sensor *sensor = sensorManager->getSensor();
    const float *sensorAcc = sensor->getAcceleration();
    const float *sensorGyro = sensor->getangularVelocity();

    IMUResponse imuResponse;

    char imuEncoded[40];
    int imuEncodedLen = 0;

    res = httpd_resp_set_type(req, IMU_CONTENT_TYPE);
    if (res != ESP_OK)
        return res;

    // Start timeout counting before sending first data
    sensor->postSetup();
    while (true)
    {
        sensorManager->update();
        if (sensor->hasNewDataToSend()) {
            // Get quaternion + acc
            imuResponse.acc_x = sensorAcc[0]; imuResponse.acc_y = sensorAcc[1]; imuResponse.acc_z = sensorAcc[2];
            imuResponse.gyro_x = sensorGyro[0]; imuResponse.gyro_y = sensorGyro[1]; imuResponse.gyro_z = sensorGyro[2];
            // Encode data into base64
            imuEncodedLen = base64_encode_chars((char *)&imuResponse, sizeof(imuResponse), imuEncoded);
            // Clear data valid flag
            sensor->takeData();
        } else {
            continue;
        }

        // Only reaching here if hasNewDataToSend() == true
        if (res == ESP_OK)
            res = httpd_resp_send(req, IMU_BOUNDARY, strlen(IMU_BOUNDARY));
        if (res == ESP_OK)
            res = httpd_resp_send(req, imuEncoded, imuEncodedLen);
        if (res != ESP_OK)
            break;
    }
    return res;
}

IMUServer::IMUServer(const int IMU_PORT) : IMU_SERVER_PORT(IMU_PORT) {}

void IMUServer::setupIMU()
{
    // using `static_cast` here seems to be better, because there are 2 similar function signatures
    Wire.begin(static_cast<int>(IMU_SDA_GPIO_NUM), static_cast<int>(IMU_SCL_GPIO_NUM));

    // Counterpart on ESP32 to ClockStretchLimit
    Wire.setTimeOut(150);
    Wire.setClock(400000);

    sensorManager.buildSensor(IMU_BNO085, IMU_BNO_ADDRESS_AD0_HI, IMU_INT_GPIO_NUM);
}

int IMUServer::startIMUServer()
{
    setupIMU();

    // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //! Turn-off the 'brownout detector'
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 20480;
    config.max_uri_handlers = 1;
    config.server_port = this->IMU_SERVER_PORT;
    config.ctrl_port = this->IMU_SERVER_PORT;
    config.stack_size = 20480;

    httpd_uri_t imu_page = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = &IMUHelpers::response,
        .user_ctx = &this->sensorManager};

    int status = httpd_start(&imu_stream, &config);

    if (status != ESP_OK)
        return -1;
    else
    {
        httpd_register_uri_handler(imu_stream, &imu_page);
        Serial.println("IMU server initialized");
        switch (wifiStateManager.getCurrentState())
        {
        case WiFiState_e::WiFiState_ADHOC:
            Serial.printf("\n\rThe IMU is under: http://%s:%i\n\r", WiFi.softAPIP().toString().c_str(), this->IMU_SERVER_PORT);
            break;
        default:
            Serial.printf("\n\rThe IMU is under: http://%s:%i\n\r", WiFi.localIP().toString().c_str(), this->IMU_SERVER_PORT);
            break;
        }
        return 0;
    }
}
