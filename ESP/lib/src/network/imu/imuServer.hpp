#pragma once
#ifndef IMU_SERVER_HPP
#define IMU_SERVER_HPP
#define IMU_PART_BOUNDARY "P"
#include <Arduino.h>
#include <WiFi.h>
#include "data/StateManager/StateManager.hpp"
#include <io/imu/libslimevr/SensorManager.h>

// Camera includes
#include "esp_http_server.h"
#include "esp_timer.h"

struct IMUResponse {
	float gyro_x, gyro_y, gyro_z;
	float acc_x, acc_y, acc_z;
};

namespace IMUHelpers
{
	esp_err_t response(httpd_req_t *req);
}
class IMUServer
{
private:
	httpd_handle_t imu_stream = nullptr;
	int IMU_SERVER_PORT;
	SlimeVR::Sensors::SensorManager sensorManager;

	void setupIMU();

public:
	IMUServer(const int IMU_PORT = 82);
	int startIMUServer();
};

#endif // IMU_SERVER_HPP
