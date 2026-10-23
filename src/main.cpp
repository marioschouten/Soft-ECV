//Version 1.3.9 by Mario M.C. Schouten
//
//Arduino Wemo D1 mini based E-C(entral)H(eating) follower using OpenTherm protocol and Ihor Melnyk's slave Terminal adapter for communication.
//The OpenTherm leader is the Elga heat pump; the modulation is passed on via MQTT to OpenHAB, which switches the E-CV (7 steps, 9 kW).
//
//Implemented functionality is as follows:
// - The E-CV operational status is set via MQTT topic [ecv/status/fault|ch_mode|flame]
// - The various measurements of temperature, pressure, flow etc. are set via MQTT topic [ecv/sensors/...]
// - The commands to update setpoints are set via MQTT topic [ecv/command/...]
// - The modulation level requested by the Thermostat is publish via MQTT topic [ecv/thermostat/modulation]
// - The CH requested by the Thermostat is publish via MQTT topic [ecv/thermostat/ch_requested]
// - The CH setpoint by the Thermostat is publish via MQTT topic [ecv/thermostat/ch_setpoint]
// - The Thermostat raw data is publish via MQTT topic [ecv/thermostat/rawdata/rx|tx]
// - Web interface at http://<device name>.local: live values, settings, 1-Wire sensor assignment, debug log, firmware update
// - Setup hotspot SoftECV-Setup-xxxx on first start, when WiFi is lost, or after pressing reset twice within 3 seconds
//
//Source files:
// - boiler.cpp   OpenTherm follower: replies to the leader, frame recorder, MQTT values
// - sensors.cpp  DS18B20 1-Wire sensors
// - net.cpp      WiFi, setup hotspot, MQTT
// - web.cpp      Web interface and API, the page itself is web/index.html
// - config.cpp   Settings stored in flash (replaces settings.h)
// - debuglog.cpp Debug messages to serial and web
//
//Version 1.2.3 is kept in legacy/main-v1.2.3.cpp.

//Libraries
#include <Arduino.h>
extern "C" {
#include <user_interface.h>
}
#include "config.h"
#include "debuglog.h"
#include "boiler.h"
#include "sensors.h"
#include "net.h"
#include "web.h"

const char* FW_VERSION = "1.3.9";

//Double reset detection: pressing reset twice within 3 seconds starts the setup hotspot
#define DRD_MAGIC   0xEC0DD15Cul
#define DRD_ADDRESS 32            // RTC user memory block, survives a reset but not a power cycle
const unsigned long DRD_TIMEOUT = 3000;
static bool drd_cleared = false;

static bool double_reset_detected() {
  uint32_t value = 0;
  ESP.rtcUserMemoryRead(DRD_ADDRESS, &value, sizeof(value));
  bool detected = value == DRD_MAGIC && ESP.getResetInfoPtr()->reason == REASON_EXT_SYS_RST;
  value = DRD_MAGIC;
  ESP.rtcUserMemoryWrite(DRD_ADDRESS, &value, sizeof(value));
  return detected;
}

static void double_reset_clear() {
  uint32_t value = 0;
  ESP.rtcUserMemoryWrite(DRD_ADDRESS, &value, sizeof(value));
  drd_cleared = true;
}

//Serial commands for the self test (tools/ot_check.py), replies start with "OTJ "
//  inject <8 hex>         process a test frame, the reply is not sent on the OpenTherm line
//  sim <topic> <payload>  handle as received MQTT message
static void serial_commands() {
  static char line[96];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (len < sizeof(line) - 1) { line[len++] = c; }
      continue;
    }
    if (len == 0) { continue; }
    line[len] = 0;
    len = 0;

    if (!strncmp(line, "inject ", 7)) {
      char* end;
      uint32_t request = strtoul(line + 7, &end, 16);
      if (strlen(line + 7) != 8 || *end) { Serial.println("OTJ {\"error\":\"expected 8 hex characters\"}"); continue; }
      FrameRec rec = boiler_inject(request);
      Serial.printf("OTJ {\"s\":%lu,\"q\":\"%08lx\",", (unsigned long)rec.seq, (unsigned long)rec.request);
      if (rec.flags & FRAME_REPLIED) { Serial.printf("\"r\":\"%08lx\",", (unsigned long)rec.reply); }
      else { Serial.print("\"r\":null,"); }
      Serial.printf("\"f\":%u}\n", rec.flags);
    } else if (!strncmp(line, "sim ", 4)) {
      char* topic = line + 4;
      char* payload = strchr(topic, ' ');
      if (!payload) { Serial.println("OTJ {\"error\":\"expected topic and payload\"}"); continue; }
      *payload++ = 0;
      boiler_mqtt(topic, payload);
      Serial.println("OTJ {\"ok\":true}");
    } else if (!strcmp(line, "info")) {
      Serial.printf("OTJ {\"fw\":\"%s\",\"modulating\":%s,\"sensor_fault\":%s,\"delay\":%u,\"heap\":%u,\"heap_block\":%u}\n", FW_VERSION,
                    config.boiler_modulating ? "true" : "false", config.report_sensor_fault ? "true" : "false", timing, ESP.getFreeHeap(), ESP.getMaxFreeBlockSize());
    } else {
      Serial.println("OTJ {\"error\":\"unknown command\"}");
    }
  }
}

// -----------------------------------------------------------SETUP--------------------------------------------------------------
void setup() {
  //Set serial speed for serial monitor
  Serial.begin(115200);
  Serial.println();
  Serial.println("Start program.");

  // initialize digital pin LED_BUILTIN as an output: blinks while the setup hotspot is on, on when connected to WiFi
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  bool force_ap = double_reset_detected();
  config_begin();
  dbg(LOG_SYS, "Soft E-CV %s, reset reason: %s", FW_VERSION, ESP.getResetReason().c_str());
  uint32_t reason = ESP.getResetInfoPtr()->reason;
  if (reason >= REASON_WDT_RST && reason <= REASON_SOFT_WDT_RST) { dbg(LOG_SYS, "%s", ESP.getResetInfo().c_str()); }
  if (force_ap) { dbg(LOG_SYS, "Reset pressed twice: starting the setup hotspot"); }

  //OpenTherm first, so the leader gets replies as soon as possible
  boiler_begin();
  sensors_begin();
  net_begin(force_ap);
  web_begin();
}

//------------------------------------------------------------LOOP----------------------------------------------------------------
// LOOP Runs the main code. The OpenTherm reply is checked between the other tasks so it is never late.
void loop() {
  boiler_loop();
  net_loop();
  boiler_loop();
  sensors_loop();
  boiler_loop();
  web_loop();
  serial_commands();

  if (!drd_cleared && millis() > DRD_TIMEOUT) { double_reset_clear(); }
}
