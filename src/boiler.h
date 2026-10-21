//OpenTherm follower (boiler side): replies to the OpenTherm leader (the Elga heat pump) on behalf of the E-CV.
#pragma once
#include <Arduino.h>

//Values received from the OpenTherm leader
struct LeaderState {
  bool status_seen        = false;    // ID 0 received at least once
  bool ch_enable          = false;    // ID 0 HB bit 0
  bool dhw_enable         = false;    // ID 0 HB bit 1
  bool cooling_enable     = false;    // ID 0 HB bit 2
  bool otc_active         = false;    // ID 0 HB bit 3
  bool ch2_enable         = false;    // ID 0 HB bit 4
  bool setpoint_seen      = false;
  double control_setpoint = 0;        // ID 1
  double max_rel_mod      = 100;      // ID 14, caps the reported modulation
  bool room_setpoint_seen = false;
  double room_setpoint    = 0;        // ID 16
  bool room_temp_seen     = false;
  double room_temp        = 0;        // ID 24
  uint8_t config_flags    = 0;        // ID 2 HB
  uint8_t member_id       = 0;        // ID 2 LB
  double ot_version       = 0;        // ID 124
  uint16_t product        = 0;        // ID 126
};

//E-CV status and values set via MQTT (topics as version 1.2.3)
struct EcvState {
  bool fault   = false;                       // [ecv/status/fault]
  bool ch_mode = false;                       // [ecv/status/ch_mode]
  bool flame   = false;                       // [ecv/status/flame]
  double max_rel_modulation       = 100;      // [ecv/command/max_rel_modulation]
  double max_ch_water_setpoint    = 70;       // [ecv/command/max_ch_water_setpoint]
  double dhw_setpoint             = 65;       // [ecv/command/dhw_setpoint]
  double water_pressure_ch        = 2.00;     // [ecv/sensors/water_pressure_ch]
  double outside_temperature      = 0.00;     // [ecv/sensors/outside_temperature]
  double heater_flow_temperature  = 0.00;     // [ecv/sensors/heater_flow_temperature], used when the sensor pair is not usable
  double return_water_temperature = 0.00;     // [ecv/sensors/return_water_temperature], used when no return sensor reading
  double water_flow_dhw           = 0.00;     // [ecv/sensors/water_flow_dhw]
  double dhw_temperature          = 0.00;     // [ecv/sensors/dhw_temperature]
  bool heater_flow_received       = false;
  bool return_water_received      = false;
  double set_modulation           = 0;        // Calculated from control setpoint and flow temperature
  double temp_difference          = 0;
};

struct OtStats {
  uint32_t frames   = 0;    // Valid requests received
  uint32_t invalid  = 0;    // Requests rejected (parity, message type, damaged)
  uint32_t timeouts = 0;    // Incomplete frames reported by the library
  uint32_t unknown  = 0;    // Replies with UNKNOWN-DATAID
  uint32_t last_rx_ms = 0;
  uint16_t frames_last_minute = 0;
};

//Frame recorder flags
const uint8_t FRAME_REPLIED   = 0x01;
const uint8_t FRAME_INJECTED  = 0x02;   // Test frame, reply not sent on the OpenTherm line
const uint8_t FRAME_PARITY    = 0x04;   // Rejected: parity error
const uint8_t FRAME_BAD_TYPE  = 0x08;   // Rejected: not a leader to follower message type
const uint8_t FRAME_DAMAGED   = 0x10;   // Rejected: library reported an invalid frame

struct FrameRec {
  uint32_t seq;
  uint32_t ms;            // millis() at end of request
  uint32_t request;
  uint32_t reply;
  uint16_t gap_ms;        // Time since the previous request
  uint16_t latency_ms;    // End of request to start of reply
  uint8_t flags;
};

extern LeaderState leader;
extern EcvState ecv;
extern OtStats ot_stats;
extern const unsigned int timing;

void boiler_begin();
void boiler_loop();
bool boiler_quiet();                           // True when no frame is being received or answered

//Process a test frame exactly as a received frame, without sending the reply on the OpenTherm line
FrameRec boiler_inject(uint32_t request);

//Handle an MQTT message on one of the ecv/... topics
void boiler_mqtt(const char* topic, const char* payload);

//Values as reported to the leader, ok = false if not available
bool sensor_fault();          // No flow temperature and reporting enabled: fault to the Elga
bool sensors_in_use();        // Both 1-Wire sensors read 0-60 C, else flow and return come from MQTT
double flow_temperature(bool* ok);
double return_temperature(bool* ok);
double reported_modulation();

//Frame recorder and 30 minute history as JSON
void frames_write_json(Print& out, uint32_t after, uint16_t max_count);
uint32_t frames_last_seq();
void history_write_json(Print& out);
