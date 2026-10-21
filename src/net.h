//WiFi, setup hotspot (captive portal), mDNS and MQTT. Nothing in here blocks the OpenTherm loop for long.
#pragma once
#include <Arduino.h>

void net_begin(bool force_ap);
void net_loop();

//Settings changed via the web interface: reconnect with the new values
void net_wifi_changed();
void net_mqtt_changed();

bool wifi_connected();
bool ap_active();
String ap_ssid();

bool mqtt_publish(const char* topic, const char* payload, bool retain = false);
bool mqtt_connected();
int mqtt_state();                     // PubSubClient state, 0 = connected
unsigned long mqtt_last_rx_ms();      // millis() of the last received MQTT message, 0 = none

//Asynchronous WiFi network scan
struct ScanNet {
  char ssid[33];
  int8_t rssi;
  bool secure;
};
#define MAX_SCAN_NETS 15
extern ScanNet scan_nets[MAX_SCAN_NETS];
extern uint8_t scan_count;
void wifi_scan_start();
bool wifi_scan_busy();
