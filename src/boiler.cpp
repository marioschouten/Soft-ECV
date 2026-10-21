#include "boiler.h"
#include <OpenTherm.h>
#include "config.h"
#include "debuglog.h"
#include "net.h"
#include "sensors.h"

//OpenTherm input and output wires connected to 4 and 5 pins on the OpenTherm Shield
const int inPin  = 12;  //for Arduino, 12 for ESP8266 (D6), 19 for ESP32
const int outPin = 13;  //for Arduino, 13 for ESP8266 (D7), 23 for ESP32
static OpenTherm ot(inPin, outPin, true);

//Openterm Leader Follower response timing (Min. 20ms - max. 800ms)
const unsigned int timing = 250;

//Modulation calculation on the difference between control setpoint and flow temperature, updated every 60 sec
const double upper_limit = 20.00;
const double lower_limit =  2.00;
const unsigned long MODULATION_INTERVAL = 60000;

//E-CV capacity reported on ID 15: 9 kW in 7 power steps, minimum modulation 1/7 = 14%
const uint8_t ECV_CAPACITY_KW    = 9;
const uint8_t ECV_MIN_MODULATION = 14;

//Adjustment bounds reported on ID 48 (DHW setpoint) and ID 49 (max CH water setpoint), upper and lower in C
const int8_t DHW_SETPOINT_UB    = 70;
const int8_t DHW_SETPOINT_LB    = 30;
const int8_t MAX_CH_SETPOINT_UB = 80;
const int8_t MAX_CH_SETPOINT_LB = 20;

//No domestic hot water on the E-CV (DHW_mode = "0" in version 1.2.3)
const bool DHW_PRESENT = false;

//Implemented OpenTherm version (ID 125) and product type/version (ID 127)
const double OT_VERSION       = 2.2;
const uint8_t PRODUCT_TYPE    = 1;
const uint8_t PRODUCT_VERSION = 3;

LeaderState leader;
EcvState ecv;
OtStats ot_stats;

//Frame recorder, the last 128 conversations (about 3 KB)
#define FRAME_LOG_SIZE 128
static FrameRec frame_log[FRAME_LOG_SIZE];
static uint32_t frame_seq = 0;

//History for the web page: one point every 10 sec for 30 minutes, value * 100, INT16_MIN = no value
#define HISTORY_SIZE 180
const unsigned long HISTORY_INTERVAL = 10000;
static int16_t hist_flow[HISTORY_SIZE], hist_return[HISTORY_SIZE], hist_setpoint[HISTORY_SIZE], hist_mod[HISTORY_SIZE];
static uint16_t hist_count = 0;
static uint16_t hist_pos = 0;

//Time of the last edge on the OpenTherm input, set by the interrupt handler
static volatile unsigned long last_edge_us = 0;

//Reply waiting for the protocol delay, sent from boiler_loop()
static struct {
  bool active;
  uint32_t request;
  uint32_t reply;
  unsigned long rx_us;
  uint32_t seq;
} pending;

static unsigned long last_modulation_update = 0;
static unsigned long last_ch_update         = 0;
static int ch_enabled_history               = 0;
static unsigned long last_temp_publish      = 0;
static unsigned long last_history           = 0;
static unsigned long minute_start           = 0;
static uint16_t minute_frames               = 0;

struct Reply {
  OpenThermMessageType type;
  uint16_t data;
};

//-------------------------------------------OpenTherm message FUNCTIONS--------------------------------------------------------
//OpenTherm interupt handler
void IRAM_ATTR handleInterrupt() {
  last_edge_us = micros();
  ot.handleInterrupt();
}

//f8.8 conversion, signed (negative temperatures are two's complement)
static uint16_t to_f88(double value) { return (uint16_t)(int16_t)lround(value * 256.0); }
static double from_f88(uint16_t data)  { return (int16_t)data / 256.0; }

//Supported data-ids, used for the descriptions in the logs and for INVALID-DATA requests
struct IdInfo { uint8_t id; bool f88; const char* name; };
static const IdInfo id_info[] = {
  {  0, false, "Status flags"},
  {  1, true,  "Control setpoint CH water temperature (C)"},
  {  2, false, "Leader config flags and MemberID"},
  {  3, false, "Follower config flags and MemberID"},
  {  5, false, "Application-specific and OEM fault flags"},
  { 14, true,  "Maximum relative modulation level setting (Percent)"},
  { 15, false, "Maximum capacity (kW) and minimum modulation (Percent)"},
  { 16, true,  "Room setpoint (C)"},
  { 17, true,  "Relative modulation level (Percent)"},
  { 18, true,  "Water pressure in CH circuit (bar)"},
  { 19, true,  "Water flow rate in DHW circuit (litres/minute)"},
  { 24, true,  "Room temperature (C)"},
  { 25, true,  "Boiler flow water temperature (C)"},
  { 26, true,  "DHW temperature (C)"},
  { 27, true,  "Outside temperature (C)"},
  { 28, true,  "Return water temperature (C)"},
  { 48, false, "DHW setpoint bounds (C)"},
  { 49, false, "Max CH water setpoint bounds (C)"},
  { 56, true,  "DHW setpoint (C)"},
  { 57, true,  "Maximum CH water setpoint (C)"},
  {124, true,  "Leader OpenTherm version"},
  {125, true,  "Follower OpenTherm version"},
  {126, false, "Leader product type and version"},
  {127, false, "Follower product type and version"},
};

static const IdInfo* find_id(uint8_t id) {
  for (const IdInfo& info : id_info) { if (info.id == id) { return &info; } }
  return nullptr;
}

static const char* type_name(OpenThermMessageType type) {
  switch (type) {
    case OpenThermMessageType::READ_DATA:       return "READ-DATA";
    case OpenThermMessageType::WRITE_DATA:      return "WRITE-DATA";
    case OpenThermMessageType::INVALID_DATA:    return "INVALID-DATA";
    case OpenThermMessageType::RESERVED:        return "RESERVED";
    case OpenThermMessageType::READ_ACK:        return "READ-ACK";
    case OpenThermMessageType::WRITE_ACK:       return "WRITE-ACK";
    case OpenThermMessageType::DATA_INVALID:    return "DATA-INVALID";
    case OpenThermMessageType::UNKNOWN_DATA_ID: return "UNKNOWN-DATAID";
  }
  return "?";
}

//Text of a frame as published on [ecv/thermostat/rawdata/rx] and [ecv/thermostat/rawdata/tx]
static void describe(char* buf, size_t size, char prefix, uint32_t frame) {
  uint8_t id = (frame >> 16) & 0xFF;
  uint16_t data = frame & 0xFFFF;
  const IdInfo* info = find_id(id);
  const char* name = info ? info->name : "Unsupported data-id";
  if (info && info->f88) {
    snprintf(buf, size, "%c-%08lx %s %s: %.2f", prefix, (unsigned long)frame, type_name(OpenTherm::getMessageType(frame)), name, from_f88(data));
  } else {
    snprintf(buf, size, "%c-%08lx %s %s: %04X", prefix, (unsigned long)frame, type_name(OpenTherm::getMessageType(frame)), name, data);
  }
}

//No usable flow temperature: the E-CV cannot be controlled, reported as fault with OEM fault code 25 (if enabled)
const uint8_t OEM_CODE_NO_FLOW_TEMP = 25;
bool sensor_fault() {
  bool ok;
  flow_temperature(&ok);
  return config.report_sensor_fault && !ok;
}

//Follower status flags (ID 0 LB). On a fault only the fault flag is set, CH mode and flame are switched off (as version 1.2.3)
static uint8_t follower_status() {
  if (ecv.fault || sensor_fault()) { return 0x01; }
  return (ecv.ch_mode ? 0x02 : 0) | (ecv.flame ? 0x08 : 0);
}

//Follower configuration flags (ID 3 HB): bit 0 DHW present, bit 1 control type (0 = modulating, 1 = on/off)
static uint8_t follower_config() {
  return (DHW_PRESENT ? 0x01 : 0) | (config.boiler_modulating ? 0 : 0x02);
}

//READ-DATA: reply the value, or DATA-INVALID if it is not available or outside the protocol range
static Reply read_value(uint8_t id, bool available, double value, double low, double high) {
  if (!available) {
    dbg(DBG_RANGE, "ID %u: no value available, reply DATA-INVALID", id);
    return {OpenThermMessageType::DATA_INVALID, 0};
  }
  if (value < low || value > high) {
    dbg(DBG_RANGE, "ID %u: value %.2f outside range %.0f..%.0f, reply DATA-INVALID", id, value, low, high);
    return {OpenThermMessageType::DATA_INVALID, 0};
  }
  uint16_t data = to_f88(value);
  dbg(DBG_CONVERT, "ID %u: value %.2f to f8.8 %04X", id, value, data);
  return {OpenThermMessageType::READ_ACK, data};
}

//WRITE-DATA: check the written value against the protocol range
static bool write_in_range(uint8_t id, uint16_t data, double low, double high, double* value) {
  *value = from_f88(data);
  if (*value < low || *value > high) {
    dbg(DBG_RANGE, "ID %u: written value %.2f outside range %.0f..%.0f, reply DATA-INVALID", id, *value, low, high);
    return false;
  }
  return true;
}

static Reply read_raw(uint16_t data)  { return {OpenThermMessageType::READ_ACK, data}; }
static Reply write_ack(uint16_t data) { return {OpenThermMessageType::WRITE_ACK, data}; }
static Reply data_invalid(uint16_t data) { return {OpenThermMessageType::DATA_INVALID, data}; }

//Reply to a READ-DATA or WRITE-DATA request. A request type the data-id does not support gets UNKNOWN-DATAID.
static Reply handle_request(OpenThermMessageType type, uint8_t id, uint16_t data) {
  bool is_read  = type == OpenThermMessageType::READ_DATA;
  bool is_write = type == OpenThermMessageType::WRITE_DATA;
  double value;
  bool ok;

  switch (id) {
    //ID 0 Status: leader flags in HB, reply with the follower flags in LB (mandatory)
    case 0:
      if (!is_read) { break; }
      leader.status_seen    = true;
      leader.ch_enable      = data & 0x0100;
      leader.dhw_enable     = data & 0x0200;
      leader.cooling_enable = data & 0x0400;
      leader.otc_active     = data & 0x0800;
      leader.ch2_enable     = data & 0x1000;
      return read_raw((data & 0xFF00) | follower_status());

    //ID 1 Control setpoint (mandatory)
    case 1:
      if (!is_write) { break; }
      if (!write_in_range(id, data, 0, 100, &value)) { return data_invalid(data); }
      leader.control_setpoint = value;
      leader.setpoint_seen = true;
      return write_ack(data);

    //ID 2 Leader configuration and MemberID
    case 2:
      if (!is_write) { break; }
      leader.config_flags = data >> 8;
      leader.member_id = data & 0xFF;
      return write_ack(data);

    //ID 3 Follower configuration, MemberID 0 (mandatory)
    case 3:
      if (!is_read) { break; }
      return read_raw(follower_config() << 8);

    //ID 5 Fault flags (HB) and OEM fault code (LB): "service request" (bit 0) when [ecv/status/fault] is set,
    //OEM fault code 25 when there is no flow temperature
    case 5:
      if (!is_read) { break; }
      return read_raw((ecv.fault ? 0x0100 : 0) | (sensor_fault() ? OEM_CODE_NO_FLOW_TEMP : 0));

    //ID 14 Maximum relative modulation, caps the reported modulation (mandatory)
    case 14:
      if (!is_write) { break; }
      if (!write_in_range(id, data, 0, 100, &value)) { return data_invalid(data); }
      leader.max_rel_mod = value;
      return write_ack(data);

    //ID 15 Maximum capacity (kW) and minimum modulation (%)
    case 15:
      if (!is_read) { break; }
      return read_raw((ECV_CAPACITY_KW << 8) | ECV_MIN_MODULATION);

    //ID 16 Room setpoint
    case 16:
      if (!is_write) { break; }
      if (!write_in_range(id, data, -40, 127, &value)) { return data_invalid(data); }
      leader.room_setpoint = value;
      leader.room_setpoint_seen = true;
      return write_ack(data);

    //ID 17 Relative modulation level (mandatory)
    case 17:
      if (!is_read) { break; }
      return read_value(id, true, reported_modulation(), 0, 100);

    //ID 18 Water pressure, ID 19 DHW flow rate
    case 18:
      if (!is_read) { break; }
      return read_value(id, true, ecv.water_pressure_ch, 0, 5);
    case 19:
      if (!is_read) { break; }
      return read_value(id, true, ecv.water_flow_dhw, 0, 16);

    //ID 24 Room temperature. Write-only in the spec, but the Elga sends it as READ-DATA with the temperature in the data
    //and keeps asking after UNKNOWN-DATAID: store it and reply READ-ACK with the same value
    case 24:
      if (!is_write && !is_read) { break; }
      if (!write_in_range(id, data, -40, 127, &value)) { return data_invalid(data); }
      leader.room_temp = value;
      leader.room_temp_seen = true;
      return is_write ? write_ack(data) : read_raw(data);

    //ID 25 Boiler flow temperature, DATA-INVALID on sensor fault (mandatory)
    case 25:
      if (!is_read) { break; }
      value = flow_temperature(&ok);
      return read_value(id, ok, value, -40, 127);

    //ID 26 DHW temperature, ID 27 outside temperature, ID 28 return water temperature
    case 26:
      if (!is_read) { break; }
      return read_value(id, true, ecv.dhw_temperature, -40, 127);
    case 27:
      if (!is_read) { break; }
      return read_value(id, true, ecv.outside_temperature, -40, 127);
    case 28:
      if (!is_read) { break; }
      value = return_temperature(&ok);
      return read_value(id, ok, value, -40, 127);

    //ID 48 and 49 Setpoint bounds, s8 upper bound in HB and lower bound in LB
    case 48:
      if (!is_read) { break; }
      return read_raw(((uint8_t)DHW_SETPOINT_UB << 8) | (uint8_t)DHW_SETPOINT_LB);
    case 49:
      if (!is_read) { break; }
      return read_raw(((uint8_t)MAX_CH_SETPOINT_UB << 8) | (uint8_t)MAX_CH_SETPOINT_LB);

    //ID 56 DHW setpoint and ID 57 max CH water setpoint. The E-CV values (set via MQTT) are leading,
    //a write is acknowledged with the actual value.
    case 56:
      if (is_read) { return read_value(id, true, ecv.dhw_setpoint, 0, 127); }
      if (!write_in_range(id, data, 0, 127, &value)) { return data_invalid(data); }
      return write_ack(to_f88(ecv.dhw_setpoint));
    case 57:
      if (is_read) { return read_value(id, true, ecv.max_ch_water_setpoint, 0, 127); }
      if (!write_in_range(id, data, 0, 127, &value)) { return data_invalid(data); }
      return write_ack(to_f88(ecv.max_ch_water_setpoint));

    //ID 124-127 OpenTherm and product versions
    case 124:
      if (!is_write) { break; }
      leader.ot_version = from_f88(data);
      return write_ack(data);
    case 125:
      if (!is_read) { break; }
      return read_value(id, true, OT_VERSION, 0, 127);
    case 126:
      if (!is_write) { break; }
      leader.product = data;
      return write_ack(data);
    case 127:
      if (!is_read) { break; }
      return read_raw((PRODUCT_TYPE << 8) | PRODUCT_VERSION);
  }

  dbg(DBG_DEBUG, "ID %u %s not supported, reply UNKNOWN-DATAID", id, type_name(type));
  return {OpenThermMessageType::UNKNOWN_DATA_ID, data};
}

//Check a received frame and build the reply. Returns false if the frame must not be answered.
static bool build_reply(uint32_t request, OpenThermResponseStatus status, FrameRec& rec) {
  //A frame with a parity error is rejected without reply (OpenTherm v2.2 section 4.5)
  if (OpenTherm::parity(request)) {
    rec.flags |= FRAME_PARITY;
    dbg(DBG_DEBUG, "Request %08lx: parity error, no reply", (unsigned long)request);
    return false;
  }

  //Only READ-DATA, WRITE-DATA and INVALID-DATA are leader to follower messages
  OpenThermMessageType type = OpenTherm::getMessageType(request);
  if (type != OpenThermMessageType::READ_DATA && type != OpenThermMessageType::WRITE_DATA && type != OpenThermMessageType::INVALID_DATA) {
    rec.flags |= FRAME_BAD_TYPE;
    dbg(DBG_DEBUG, "Request %08lx: message type %s, no reply", (unsigned long)request, type_name(type));
    return false;
  }

  //The library reports INVALID-DATA as invalid request, any other invalid report is a damaged frame
  if (status != OpenThermResponseStatus::SUCCESS && type != OpenThermMessageType::INVALID_DATA) {
    rec.flags |= FRAME_DAMAGED;
    dbg(DBG_DEBUG, "Request %08lx: damaged frame, no reply", (unsigned long)request);
    return false;
  }

  uint8_t id = (request >> 16) & 0xFF;
  uint16_t data = request & 0xFFFF;
  dbg(DBG_DEBUG, "Request %08lx: %s ID %u data %04X", (unsigned long)request, type_name(type), id, data);

  Reply reply;
  if (type == OpenThermMessageType::INVALID_DATA) {
    //Leader sends a value it marks as invalid: DATA-INVALID for a known data-id
    reply = find_id(id) ? data_invalid(data) : Reply{OpenThermMessageType::UNKNOWN_DATA_ID, data};
  } else {
    reply = handle_request(type, id, data);
  }

  rec.reply = OpenTherm::buildResponse(reply.type, (OpenThermMessageID)id, reply.data);
  rec.flags |= FRAME_REPLIED;
  return true;
}

static FrameRec& new_frame(uint32_t request) {
  FrameRec& rec = frame_log[frame_seq % FRAME_LOG_SIZE];
  rec = FrameRec();
  rec.seq = ++frame_seq;
  rec.ms = millis();
  rec.request = request;
  return rec;
}

//OpenTherm process received data, the reply is sent from boiler_loop() after the protocol delay
static void processRequest(unsigned long request, OpenThermResponseStatus status) {
  if (status == OpenThermResponseStatus::TIMEOUT) {
    ot_stats.timeouts++;
    return;
  }

  unsigned long rx_us = last_edge_us;
  FrameRec& rec = new_frame(request);
  rec.gap_ms = ot_stats.last_rx_ms ? std::min<uint32_t>(rec.ms - ot_stats.last_rx_ms, 65535) : 0;
  ot_stats.last_rx_ms = rec.ms;

  if (!build_reply(request, status, rec)) {
    ot_stats.invalid++;
    return;
  }
  ot_stats.frames++;
  minute_frames++;
  if (OpenTherm::getMessageType(rec.reply) == OpenThermMessageType::UNKNOWN_DATA_ID) { ot_stats.unknown++; }

  pending.active  = true;
  pending.request = request;
  pending.reply   = rec.reply;
  pending.rx_us   = rx_us;
  pending.seq     = rec.seq;
}

FrameRec boiler_inject(uint32_t request) {
  //Test frames must not change the values received from the leader
  LeaderState saved = leader;
  FrameRec& rec = new_frame(request);
  rec.flags |= FRAME_INJECTED;
  OpenThermResponseStatus status = OpenTherm::isValidRequest(request) ? OpenThermResponseStatus::SUCCESS : OpenThermResponseStatus::INVALID;
  build_reply(request, status, rec);
  leader = saved;
  return rec;
}

//-------------------------------------------E-CV FUNCTIONS---------------------------------------------------------------------
//The 1-Wire sensors are only used as a pair: both must read 0-60 C. Otherwise flow and return both come from MQTT
//[ecv/sensors/heater_flow_temperature and return_water_temperature]. A defect DS18B20 reads -127 (no answer) or 85 (power-on value).
const float SENSOR_MIN = 0;
const float SENSOR_MAX = 60;

bool sensors_in_use() {
  float flow = sensor_flow_temp(), ret = sensor_return_temp();
  return sensor_ok(flow) && sensor_ok(ret) && flow >= SENSOR_MIN && flow <= SENSOR_MAX && ret >= SENSOR_MIN && ret <= SENSOR_MAX;
}

double flow_temperature(bool* ok) {
  if (sensors_in_use()) { *ok = true; return sensor_flow_temp(); }
  //Use the MQTT provided temperature, if received
  *ok = ecv.heater_flow_received;
  return ecv.heater_flow_temperature;
}

double return_temperature(bool* ok) {
  if (sensors_in_use()) { *ok = true; return sensor_return_temp(); }
  *ok = ecv.return_water_received;
  return ecv.return_water_temperature;
}

//Modulation reported on ID 17, capped by the leader's maximum (ID 14) and the MQTT maximum
double reported_modulation() {
  double modulation = constrain(ecv.set_modulation, 0.0, 100.0);
  modulation = min(modulation, leader.max_rel_mod);
  modulation = min(modulation, ecv.max_rel_modulation);
  return modulation;
}

//Calculate the modulation from the difference between control setpoint and flow temperature
static void calculate_modulation() {
  bool ok;
  double flow = flow_temperature(&ok);
  if (!ok) {
    //Without a flow temperature the E-CV is not switched on blind
    ecv.temp_difference = 0;
    ecv.set_modulation = 0;
    dbg(LOG_SYS, "No flow temperature available, modulation set to 0");
  } else {
    ecv.temp_difference = leader.control_setpoint - flow;
    //Check if difference is > upper limit, < lower limit, or between lower and upper limit
    if (ecv.temp_difference > upper_limit) {
      ecv.set_modulation = 100.00;
    } else if (ecv.temp_difference < lower_limit) {
      ecv.set_modulation = 0.00;
    } else {
      ecv.set_modulation = min(100.0, ecv.temp_difference / (upper_limit - lower_limit) * 100);
    }
  }

  //Publish the calculation to MQTT [ecv/thermostat/rawdata/modulation]
  char msg[110];
  snprintf(msg, sizeof(msg), "Request: %.2f Heater flow: %.2f Difference:%.2f Set Modulation: %.2f",
           leader.control_setpoint, flow, ecv.temp_difference, ecv.set_modulation);
  mqtt_publish("ecv/thermostat/rawdata/modulation", msg);
}

//Log and publish a conversation after the reply was sent
static void after_reply(uint32_t request, uint32_t reply, unsigned long latency) {
  char rx[150], tx[170], msg[24];
  describe(rx, sizeof(rx), 'T', request);
  describe(tx, sizeof(tx), 'B', reply);

  //DEBUG_MONITOR: Print the OpenTherm request and reply
  dbg(DBG_MONITOR, "%s", rx);
  dbg(DBG_MONITOR, "%s", tx);

  //Publish the request to MQTT [ecv/thermostat/rawdata/rx] and the reply to [ecv/thermostat/rawdata/tx]
  mqtt_publish("ecv/thermostat/rawdata/rx", rx);
  size_t len = strlen(tx);
  snprintf(tx + len, sizeof(tx) - len, " Replied after: %lums.", latency);
  mqtt_publish("ecv/thermostat/rawdata/tx", tx);

  uint8_t id = (request >> 16) & 0xFF;
  OpenThermMessageType reply_type = OpenTherm::getMessageType(reply);

  //Publish CH requested to MQTT [ecv/thermostat/ch_requested] on change, else every 60 sec
  if (id == 0 && reply_type == OpenThermMessageType::READ_ACK) {
    int ch_enabled = leader.ch_enable ? 1 : 0;
    if (ch_enabled != ch_enabled_history || millis() - last_ch_update > 60000) {
      ch_enabled_history = ch_enabled;
      mqtt_publish("ecv/thermostat/ch_requested", ch_enabled ? "1" : "0");
      last_ch_update = millis();
    }
  }

  //Publish the CH setpoint to MQTT [ecv/thermostat/ch_setpoint] and update the modulation every 60 sec
  if (id == 1 && reply_type == OpenThermMessageType::WRITE_ACK) {
    snprintf(msg, sizeof(msg), "%.2f", leader.control_setpoint);
    mqtt_publish("ecv/thermostat/ch_setpoint", msg);
    if (millis() - last_modulation_update > MODULATION_INTERVAL) {
      calculate_modulation();
      last_modulation_update = millis();
    }
  }

  //Publish the modulation level to MQTT [ecv/thermostat/modulation]
  if (id == 17 && reply_type == OpenThermMessageType::READ_ACK) {
    snprintf(msg, sizeof(msg), "%.2f", reported_modulation());
    mqtt_publish("ecv/thermostat/modulation", msg);
  }

  //Publish the boiler temperature to MQTT [ecv/thermostat/boilertemp]
  if (id == 25) {
    snprintf(msg, sizeof(msg), "%.2f", sensor_flow_temp());
    mqtt_publish("ecv/thermostat/boilertemp", msg);
  }
}

static int16_t history_value(bool ok, double value) {
  return ok ? (int16_t)constrain(lround(value * 100), -32000L, 32000L) : INT16_MIN;
}

static void history_add() {
  bool ok;
  double value = flow_temperature(&ok);
  hist_flow[hist_pos] = history_value(ok, value);
  value = return_temperature(&ok);
  hist_return[hist_pos] = history_value(ok, value);
  hist_setpoint[hist_pos] = history_value(leader.setpoint_seen, leader.control_setpoint);
  hist_mod[hist_pos] = history_value(true, reported_modulation());
  hist_pos = (hist_pos + 1) % HISTORY_SIZE;
  if (hist_count < HISTORY_SIZE) { hist_count++; }
}

bool boiler_quiet() {
  return !pending.active && ot.isReady() && micros() - last_edge_us > 40000;
}

void boiler_begin() {
  //Init OpenTerm interupt handler
  ot.begin(handleInterrupt, processRequest);
  last_modulation_update = millis();
  minute_start = millis();
}

void boiler_loop() {
  //OpenTerm process
  ot.process();

  //Send the reply when the protocol delay has passed
  if (pending.active && micros() - pending.rx_us >= timing * 1000UL) {
    unsigned long latency = (micros() - pending.rx_us) / 1000;
    ot.sendResponse(pending.reply);
    pending.active = false;
    FrameRec& rec = frame_log[(pending.seq - 1) % FRAME_LOG_SIZE];
    if (rec.seq == pending.seq) { rec.latency_ms = latency; }
    after_reply(pending.request, pending.reply, latency);
    return;
  }
  if (pending.active) { return; }

  unsigned long now = millis();
  //Publish the boiler return temperature to MQTT [ecv/thermostat/returntemp] every 5 seconds
  if (now - last_temp_publish > 5000) {
    char msg[24];
    snprintf(msg, sizeof(msg), "%.2f", sensor_return_temp());
    mqtt_publish("ecv/thermostat/returntemp", msg);
    last_temp_publish = now;
  }
  if (now - last_history >= HISTORY_INTERVAL) {
    history_add();
    last_history = now;
  }
  if (now - minute_start >= 60000) {
    ot_stats.frames_last_minute = minute_frames;
    minute_frames = 0;
    minute_start = now;
  }
}

//------------------------------------------------------------MQTT---------------------------------------------------------------
static void set_flag(bool* flag, const char* payload, const char* name) {
  if (payload[0] == '0') { *flag = false; }
  else if (payload[0] == '1') { *flag = true; }
  else { return; }
  dbg(DBG_UPDATE, "%s set to %c", name, payload[0]);
}

static void set_value(double* var, const char* payload, const char* name) {
  double old_value = *var;
  *var = atof(payload);
  dbg(DBG_UPDATE, "%s changed from %.2f to %.2f", name, old_value, *var);
}

void boiler_mqtt(const char* topic, const char* payload) {
  //DEBUG_MQTT: Print the topic and payload of the received MQTT message
  dbg(DBG_MQTT_IN, "%s %s", topic, payload);

  if      (!strcmp(topic, "ecv/status/fault"))                    { set_flag(&ecv.fault, payload, "Fault status"); }
  else if (!strcmp(topic, "ecv/status/ch_mode"))                  { set_flag(&ecv.ch_mode, payload, "CH mode"); }
  else if (!strcmp(topic, "ecv/status/flame"))                    { set_flag(&ecv.flame, payload, "Flame status"); }
  else if (!strcmp(topic, "ecv/command/max_rel_modulation"))      { set_value(&ecv.max_rel_modulation, payload, "Max relative modulation"); }
  else if (!strcmp(topic, "ecv/command/max_ch_water_setpoint"))   { set_value(&ecv.max_ch_water_setpoint, payload, "Max CH water setpoint"); }
  else if (!strcmp(topic, "ecv/command/dhw_setpoint"))            { set_value(&ecv.dhw_setpoint, payload, "DHW setpoint"); }
  else if (!strcmp(topic, "ecv/sensors/water_pressure_ch"))       { set_value(&ecv.water_pressure_ch, payload, "Water pressure CH"); }
  else if (!strcmp(topic, "ecv/sensors/outside_temperature"))     { set_value(&ecv.outside_temperature, payload, "Outside temperature"); }
  else if (!strcmp(topic, "ecv/sensors/water_flow_dhw"))          { set_value(&ecv.water_flow_dhw, payload, "Water flow DHW"); }
  else if (!strcmp(topic, "ecv/sensors/dhw_temperature"))         { set_value(&ecv.dhw_temperature, payload, "DHW temperature"); }
  else if (!strcmp(topic, "ecv/sensors/heater_flow_temperature")) {
    set_value(&ecv.heater_flow_temperature, payload, "Boiler flow temperature");
    ecv.heater_flow_received = true;
  }
  else if (!strcmp(topic, "ecv/sensors/return_water_temperature")) {
    set_value(&ecv.return_water_temperature, payload, "Return water temperature");
    ecv.return_water_received = true;
  }
  //MQTT TOPIC is [ecv/rawdata/command]: payload of 8 hex characters is processed as a test frame (not sent on the line)
  else if (!strcmp(topic, "ecv/rawdata/command")) {
    char* end;
    uint32_t request = strtoul(payload, &end, 16);
    if (strlen(payload) != 8 || *end) {
      dbg(LOG_SYS, "ecv/rawdata/command: expected 8 hex characters, got \"%s\"", payload);
      return;
    }
    FrameRec rec = boiler_inject(request);
    char tx[170];
    if (rec.flags & FRAME_REPLIED) { describe(tx, sizeof(tx), 'B', rec.reply); }
    else { snprintf(tx, sizeof(tx), "T-%08lx rejected, no reply (flags %02X)", (unsigned long)request, rec.flags); }
    dbg(LOG_SYS, "Test frame: %s", tx);
    mqtt_publish("ecv/thermostat/rawdata/test", tx);
  }
}

//------------------------------------------------------------JSON---------------------------------------------------------------
uint32_t frames_last_seq() { return frame_seq; }

void frames_write_json(Print& out, uint32_t after, uint16_t max_count) {
  //The frame waiting for its reply is left out until the reply is sent
  uint32_t last = pending.active ? pending.seq - 1 : frame_seq;
  uint32_t oldest = frame_seq > FRAME_LOG_SIZE ? frame_seq - FRAME_LOG_SIZE + 1 : 1;
  uint32_t first = max(after + 1, oldest);
  out.print('[');
  for (uint32_t seq = first, n = 0; seq <= last && n < max_count; seq++) {
    const FrameRec& rec = frame_log[(seq - 1) % FRAME_LOG_SIZE];
    if (rec.seq != seq) { continue; }
    if (n++) { out.print(','); }
    out.printf("{\"s\":%lu,\"t\":%lu,\"q\":\"%08lx\",", (unsigned long)rec.seq, (unsigned long)rec.ms, (unsigned long)rec.request);
    if (rec.flags & FRAME_REPLIED) { out.printf("\"r\":\"%08lx\",", (unsigned long)rec.reply); }
    else { out.print("\"r\":null,"); }
    out.printf("\"g\":%u,\"l\":%u,\"f\":%u}", rec.gap_ms, rec.latency_ms, rec.flags);
  }
  out.print(']');
}

static void history_series(Print& out, const char* name, const int16_t* series) {
  out.printf("\"%s\":[", name);
  uint16_t start = (hist_pos + HISTORY_SIZE - hist_count) % HISTORY_SIZE;
  for (uint16_t i = 0; i < hist_count; i++) {
    int16_t value = series[(start + i) % HISTORY_SIZE];
    if (i) { out.print(','); }
    if (value == INT16_MIN) { out.print("null"); }
    else { out.printf("%.2f", value / 100.0); }
  }
  out.print(']');
}

void history_write_json(Print& out) {
  out.printf("{\"step\":%lu,", HISTORY_INTERVAL / 1000);
  history_series(out, "flow", hist_flow);   out.print(',');
  history_series(out, "ret", hist_return);  out.print(',');
  history_series(out, "sp", hist_setpoint); out.print(',');
  history_series(out, "mod", hist_mod);
  out.print('}');
}
