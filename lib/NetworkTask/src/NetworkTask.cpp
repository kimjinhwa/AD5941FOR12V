#include <Arduino.h>
#include <WiFi.h>
#include "mainGrobal.h"

//#include <openVPNClinet.h>

const char *soft_ap_ssid ="IMP_";
const char *soft_ap_password = "87654321";
void NetworkTask(void *parameter)
{
    //WiFi.begin(systemDefaultValue.ssid, systemDefaultValue.ssid_password);
    WiFi.begin("iptime_mbhong", "");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        printf(".");
    }
    printf("");
    printf("wifi connected");
    printf("ip address: ");
    printf(WiFi.localIP().toString().c_str());

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
        delay(500);
    }
}