#include <openiris.hpp>

/**
 * @brief ProjectConfig object
 * @brief This is the main configuration object for the project
 * @param name The name of the project config partition
 * @param mdnsName The mDNS hostname to use
 */
ProjectConfig deviceConfig("openiris", MDNS_HOSTNAME);

#ifdef CONFIG_CAMERA_MODULE_ESP32S3_XIAO_SENSE
LEDManager ledManager(LED_BUILTIN);
#else
LEDManager ledManager(33);
#endif  // ESP32S3_XIAO_SENSE

#ifndef SIM_ENABLED
CameraHandler cameraHandler(deviceConfig);
#endif  // SIM_ENABLED

#ifndef ETVR_EYE_TRACKER_USB_API
WiFiHandler wifiHandler(deviceConfig, WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
MDNSHandler mdnsHandler(deviceConfig);
#ifdef SIM_ENABLED
APIServer apiServer(deviceConfig, wifiStateManager, "/control");
#else
APIServer apiServer(deviceConfig, cameraHandler, "/control");
StreamServer streamServer;
IMUServer imuServer;
#endif  // SIM_ENABLED

void etvr_eye_tracker_web_init() {
  log_d("[SETUP]: Starting Network Handler");
  // deviceConfig.attach(wifiHandler);
  log_d("[SETUP]: Checking ADHOC Settings");
  wifiHandler._enable_adhoc = ENABLE_ADHOC;
  deviceConfig.attach(mdnsHandler);
  log_d("[SETUP]: Starting WiFi Handler");
  wifiHandler.begin();
  log_d("[SETUP]: Starting MDNS Handler");
  mdnsHandler.startMDNS();

  switch (wifiStateManager.getCurrentState()) {
    case WiFiState_e::WiFiState_Disconnected: {
      //! TODO: Implement
      break;
    }
    case WiFiState_e::WiFiState_ADHOC: {
#ifndef SIM_ENABLED
      log_d("[SETUP]: Starting Stream Server");
      streamServer.startStreamServer();
      log_d("[SETUP]: Starting IMU Server");
      imuServer.startIMUServer();
#endif  // SIM_ENABLED
      log_d("[SETUP]: Starting API Server");
      apiServer.setup();
      break;
    }
    case WiFiState_e::WiFiState_Connected: {
#ifndef SIM_ENABLED
      log_d("[SETUP]: Starting Stream Server");
      streamServer.startStreamServer();
      log_d("[SETUP]: Starting IMU Server");
      imuServer.startIMUServer();
#endif  // SIM_ENABLED
      log_d("[SETUP]: Starting API Server");
      apiServer.setup();
      break;
    }
    case WiFiState_e::WiFiState_Connecting: {
      //! TODO: Implement
      break;
    }
    case WiFiState_e::WiFiState_Error: {
      //! TODO: Implement
      break;
    }
  }
}
#endif  // ETVR_EYE_TRACKER_WEB_API

void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  Logo::printASCII();
  //  Serial.flush();
  ledManager.begin();

#ifndef SIM_ENABLED
  deviceConfig.attach(cameraHandler);
#endif  // SIM_ENABLED
  deviceConfig.load();

#ifndef ETVR_EYE_TRACKER_USB_API
  etvr_eye_tracker_web_init();
#else   // ETVR_EYE_TRACKER_WEB_API
  WiFi.disconnect(true);
  etvr_eye_tracker_usb_init();
#endif  // ETVR_EYE_TRACKER_WEB_API
}

void loop() {
  ledManager.handleLED();
#ifdef ETVR_EYE_TRACKER_USB_API
  etvr_eye_tracker_usb_loop();
#endif  // ETVR_EYE_TRACKER_USB_API
}
