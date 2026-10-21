//Settings stored in flash (LittleFS /config.json), edited via the web interface.
//Replaces the compile time settings.h of version 1.2.3.
#pragma once
#include <Arduino.h>

extern const char* FW_VERSION;

//Debug message types, bit positions in Config::debug_flags (names as the serial_* flags of version 1.2.3)
enum DebugFlag : uint8_t {
  DBG_MONITOR = 0,   // serial_monitor : OpenTherm traffic
  DBG_MQTT,          // serial_mqtt    : outgoing MQTT messages
  DBG_MQTT_IN,       // serial_mqtt_in : incoming MQTT messages
  DBG_RANGE,         // serial_range   : range checks
  DBG_UPDATE,        // serial_update  : value updates
  DBG_CONVERT,       // serial_convert : f8.8 conversion
  DBG_ONEWIRE,       // serial_onewire : 1-Wire sensors
  DBG_DEBUG,         // serial_debug   : frame decode steps
  DBG_COUNT
};

struct Config {
  //WiFi parameters
  String wifi_ssid;
  String wifi_pass;
  uint8_t ap_fallback_min = 5;        // Start the setup hotspot after this many minutes without WiFi, 0 = never

  //MQTT parameters
  String mqtt_host;
  uint16_t mqtt_port = 1883;
  String mqtt_user;
  String mqtt_pass;
  String mqtt_client_id = "ECV";

  //Device
  String hostname = "soft-ecv";        // Page is reachable at http://<hostname>.local
  String admin_pass;                  // Empty until set on first use
  bool boiler_modulating = false;     // ID 3 control type: false = on/off (as version 1.2.3), true = modulating
  bool report_sensor_fault = true;    // No flow temperature: report a fault to the Elga (ID 0 fault bit, ID 5 OEM code 25)

  //1-Wire sensor assignment, all zero = not assigned
  uint8_t sensor_flow[8];
  uint8_t sensor_return[8];

  //Debug output
  uint8_t debug_flags = 0;
  bool debug_keep   = false;          // Keep debug_flags after a restart
  bool debug_serial = true;           // Write debug messages to the USB serial port
  bool debug_web    = true;           // Keep debug messages for the live log on the web page
};

extern Config config;

bool config_begin();                  // Mount LittleFS and load the settings
bool config_save();
void config_erase();                  // Remove all settings, defaults are restored

String addr_to_hex(const uint8_t* addr);
bool hex_to_addr(const String& hex, uint8_t* addr);
bool addr_is_set(const uint8_t* addr);
