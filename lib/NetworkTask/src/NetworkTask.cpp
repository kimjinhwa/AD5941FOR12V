#include <Arduino.h>
#include <WiFi.h>
#include "mainGrobal.h"

#ifdef WEBOTA
#include <WebServer.h>
#include "esp32WebOTA.h"

//#include <openVPNClinet.h>


const char *soft_ap_ssid ="IMP_";
const char *soft_ap_password = "87654321";

extern WebServer webServer;

void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("Connected to WiFi");
      printf(WiFi.localIP().toString().c_str());
      webServer.begin();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi");
      // WiFi 재연결 로직 추가 가능
      break;
  }
}
void NetworkTask(void *parameter)
{
    //WiFi.begin(systemDefaultValue.ssid, systemDefaultValue.ssid_password);
    WiFi.onEvent(onWiFiEvent);
    WiFi.begin("iptime_mbhong", "");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        printf(".");
    }
    printf("");
    printf("wifi connected");
    printf("ip address: ");
    printf(WiFi.localIP().toString().c_str());

  webInit(); 

    // String macAddress = String(soft_ap_ssid) + String(WiFi.macAddress());
    // printf("\r\nWiFi.softAP(soft_ap_ssid=%s, soft_ap_password=%s)", soft_ap_ssid, soft_ap_password);
    // // WiFi.softAP(macAddress.c_str(), soft_ap_password.c_str());
    // WiFi.softAP(macAddress.c_str(), "");
    // printf("\r\nWiFi.softAPConfig");
    // WiFi.softAPConfig(IPAddress(192, 168, 11, 1), IPAddress(192, 168, 11, 1), IPAddress(255, 255, 255, 0));
    // printf("\r\nWiFi.mode(WIFI_MODE_AP)");
    //WiFi.mode(WIFI_MODE_AP);

    for (;;)
    {
        #ifdef WEBOTA
        if(WiFi.isConnected()) webServer.handleClient();
        #endif
        delay(200);
    }
}
#endif