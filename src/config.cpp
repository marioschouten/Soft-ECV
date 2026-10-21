#include "config.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

Config config;

static const char* CONFIG_FILE = "/config.json";

//Default sensor addresses, taken over from the hard coded sensor1 and sensor2 of version 1.2.3
static const uint8_t default_flow[8]   = {0x28, 0xE8, 0x88, 0x79, 0xA2, 0x00, 0x03, 0x03};
static const uint8_t default_return[8] = {0x28, 0x18, 0xCD, 0x79, 0xA2, 0x00, 0x03, 0x4A};

String addr_to_hex(const uint8_t* addr) {
  char buf[17];
  for (int i = 0; i < 8; i++) { sprintf(buf + i * 2, "%02X", addr[i]); }
  return String(buf);
}

bool hex_to_addr(const String& hex, uint8_t* addr) {
  if (hex.length() != 16) { return false; }
  for (int i = 0; i < 8; i++) {
    char byte_hex[3] = {hex[i * 2], hex[i * 2 + 1], 0};
    char* end;
    addr[i] = strtoul(byte_hex, &end, 16);
    if (*end) { return false; }
  }
  return true;
}

bool addr_is_set(const uint8_t* addr) {
  for (int i = 0; i < 8; i++) { if (addr[i]) { return true; } }
  return false;
}

static void set_defaults() {
  config = Config();
  memcpy(config.sensor_flow, default_flow, 8);
  memcpy(config.sensor_return, default_return, 8);
}

//Read a sensor address: key missing = keep default, empty string = not assigned
static void load_addr(JsonDocument& doc, const char* key, uint8_t* addr) {
  if (!doc[key].is<const char*>()) { return; }
  String hex = doc[key].as<const char*>();
  if (hex.length() == 0 || !hex_to_addr(hex, addr)) { memset(addr, 0, 8); }
}

static bool config_load() {
  set_defaults();
  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) { return false; }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { return false; }

  config.wifi_ssid         = doc["wifi_ssid"]       | "";
  config.wifi_pass         = doc["wifi_pass"]       | "";
  config.ap_fallback_min   = doc["ap_fallback_min"] | 5;
  config.mqtt_host         = doc["mqtt_host"]       | "";
  config.mqtt_port         = doc["mqtt_port"]       | 1883;
  config.mqtt_user         = doc["mqtt_user"]       | "";
  config.mqtt_pass         = doc["mqtt_pass"]       | "";
  config.mqtt_client_id    = doc["mqtt_client_id"]  | "ECV";
  config.hostname          = doc["hostname"]        | "soft-ecv";
  config.admin_pass        = doc["admin_pass"]      | "";
  config.boiler_modulating = doc["boiler_modulating"] | false;
  config.report_sensor_fault = doc["report_sensor_fault"] | true;
  load_addr(doc, "sensor_flow", config.sensor_flow);
  load_addr(doc, "sensor_return", config.sensor_return);
  config.debug_keep        = doc["debug_keep"]      | false;
  config.debug_serial      = doc["debug_serial"]    | true;
  config.debug_web         = doc["debug_web"]       | true;
  //Debug messages switch off at restart unless "keep after restart" is set
  config.debug_flags       = config.debug_keep ? (uint8_t)(doc["debug_flags"] | 0) : 0;
  return true;
}

bool config_begin() {
  if (!LittleFS.begin()) {
    //First start with this flash layout: format the file system
    LittleFS.format();
    if (!LittleFS.begin()) { set_defaults(); return false; }
  }
  return config_load();
}

bool config_save() {
  JsonDocument doc;
  doc["wifi_ssid"]         = config.wifi_ssid;
  doc["wifi_pass"]         = config.wifi_pass;
  doc["ap_fallback_min"]   = config.ap_fallback_min;
  doc["mqtt_host"]         = config.mqtt_host;
  doc["mqtt_port"]         = config.mqtt_port;
  doc["mqtt_user"]         = config.mqtt_user;
  doc["mqtt_pass"]         = config.mqtt_pass;
  doc["mqtt_client_id"]    = config.mqtt_client_id;
  doc["hostname"]          = config.hostname;
  doc["admin_pass"]        = config.admin_pass;
  doc["boiler_modulating"] = config.boiler_modulating;
  doc["report_sensor_fault"] = config.report_sensor_fault;
  doc["sensor_flow"]       = addr_is_set(config.sensor_flow) ? addr_to_hex(config.sensor_flow) : String();
  doc["sensor_return"]     = addr_is_set(config.sensor_return) ? addr_to_hex(config.sensor_return) : String();
  doc["debug_flags"]       = config.debug_flags;
  doc["debug_keep"]        = config.debug_keep;
  doc["debug_serial"]      = config.debug_serial;
  doc["debug_web"]         = config.debug_web;

  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) { return false; }
  serializeJson(doc, f);
  f.close();
  return true;
}

void config_erase() {
  LittleFS.remove(CONFIG_FILE);
  set_defaults();
}
