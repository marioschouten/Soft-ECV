#include "web.h"
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Updater.h>
#include "config.h"
#include "debuglog.h"
#include "net.h"
#include "boiler.h"
#include "sensors.h"
#include "web_index.h"   // Generated from web/index.html by tools/embed_web.py

static AsyncWebServer server(80);

//Sign-in sessions: a random token in a cookie, up to 4 browsers at the same time
#define MAX_SESSIONS 4
static String sessions[MAX_SESSIONS];
static uint8_t next_session = 0;
static uint8_t failed_logins = 0;
static unsigned long login_blocked_until = 0;

static bool restart_requested = false;
static unsigned long restart_at = 0;
static bool upload_allowed = false;

//Memory needed to build a response: the largest (frames) is about 3 KB and its buffer grows while it is written
const uint32_t MIN_FREE_HEAP  = 10000;
const uint32_t MIN_FREE_BLOCK = 6000;
const uint8_t MAX_ACTIVE_GETS = 3;       // Responses being sent at the same time
static uint8_t active_gets = 0;
static unsigned long active_changed = 0;   // A response that never reports its end would block all GETs: reset after 20 s
static uint32_t low_memory_count = 0;    // GET requests answered with "busy"

static void request_restart(unsigned long delay_ms) {
  restart_requested = true;
  restart_at = millis() + delay_ms;
}

//---------------------------------------------------AUTHENTICATION-------------------------------------------------------------
static String session_token(AsyncWebServerRequest* request) {
  const AsyncWebHeader* cookie = request->getHeader("Cookie");
  if (!cookie) { return String(); }
  const String& value = cookie->value();
  int start = value.indexOf("ecv=");
  if (start < 0) { return String(); }
  int end = value.indexOf(';', start);
  return value.substring(start + 4, end < 0 ? value.length() : end);
}

//Signed in, or no admin password set yet (first use)
static bool signed_in(AsyncWebServerRequest* request) {
  if (config.admin_pass.length() == 0) { return true; }
  String token = session_token(request);
  if (token.length() != 32) { return false; }
  for (const String& session : sessions) { if (session == token) { return true; } }
  return false;
}

static String new_session() {
  char token[33];
  for (int i = 0; i < 4; i++) { snprintf(token + i * 8, 9, "%08x", ESP.random()); }
  sessions[next_session] = token;
  next_session = (next_session + 1) % MAX_SESSIONS;
  return String(token);
}

static void send_session(AsyncWebServerRequest* request, const String& token) {
  AsyncWebServerResponse* response = request->beginResponse(200, "application/json", "{\"ok\":true}");
  response->addHeader("Set-Cookie", "ecv=" + token + "; Path=/; HttpOnly; SameSite=Strict");
  request->send(response);
}

//---------------------------------------------------HELPERS--------------------------------------------------------------------
static void send_json(AsyncWebServerRequest* request, JsonDocument& doc) {
  AsyncResponseStream* response = request->beginResponseStream("application/json");
  serializeJson(doc, *response);
  request->send(response);
}

static void send_ok(AsyncWebServerRequest* request) {
  request->send(200, "application/json", "{\"ok\":true}");
}

static void send_error(AsyncWebServerRequest* request, int code, const char* message) {
  AsyncResponseStream* response = request->beginResponseStream("application/json");
  response->setCode(code);
  response->print("{\"error\":");
  json_string(*response, message);
  response->print('}');
  request->send(response);
}

static bool require_signin(AsyncWebServerRequest* request) {
  if (signed_in(request)) { return true; }
  send_error(request, 401, "Sign in first.");
  return false;
}

static bool has_post(AsyncWebServerRequest* request, const char* name) {
  return request->hasParam(name, true);
}

static String post(AsyncWebServerRequest* request, const char* name) {
  const AsyncWebParameter* param = request->getParam(name, true);
  return param ? param->value() : String();
}

static uint32_t query_uint(AsyncWebServerRequest* request, const char* name) {
  const AsyncWebParameter* param = request->getParam(name);
  return param ? strtoul(param->value().c_str(), nullptr, 10) : 0;
}

//Value rounded to 2 decimals, or null when not available
//Template so obj["key"] stays a member proxy: a JsonVariant of a missing member is null and set() would drop the value
template <typename T>
static void set_value(T&& var, bool ok, double value) {
  if (ok) { var.set(round(value * 100) / 100.0); }
  else { var.set(nullptr); }
}

//---------------------------------------------------STATUS---------------------------------------------------------------------
static void handle_status(AsyncWebServerRequest* request) {
  JsonDocument doc;
  unsigned long now = millis();
  doc["fw"] = FW_VERSION;
  doc["up"] = now / 1000;
  doc["heap"] = ESP.getFreeHeap();
  doc["heap_block"] = ESP.getMaxFreeBlockSize();
  doc["busy"] = low_memory_count;
  doc["reset"] = ESP.getResetReason();
  //Watchdog or exception: the code addresses (epc1..) can be looked up in firmware.elf with addr2line
  uint32_t reason = ESP.getResetInfoPtr()->reason;
  if (reason >= REASON_WDT_RST && reason <= REASON_SOFT_WDT_RST) { doc["reset_info"] = ESP.getResetInfo(); }
  doc["core"] = ESP.getCoreVersion();
  doc["name"] = config.hostname;

  JsonObject wifi = doc["wifi"].to<JsonObject>();
  wifi["ok"] = wifi_connected();
  wifi["ssid"] = config.wifi_ssid;
  wifi["rssi"] = wifi_connected() ? WiFi.RSSI() : 0;
  wifi["ip"] = WiFi.localIP().toString();
  wifi["mac"] = WiFi.macAddress();
  wifi["ap"] = ap_active();
  wifi["ap_ssid"] = ap_ssid();
  wifi["ap_clients"] = ap_active() ? WiFi.softAPgetStationNum() : 0;

  JsonObject mqtt = doc["mqtt"].to<JsonObject>();
  mqtt["host"] = config.mqtt_host;
  mqtt["port"] = config.mqtt_port;
  mqtt["ok"] = mqtt_connected();
  mqtt["state"] = mqtt_state();
  mqtt["rx_age"] = mqtt_last_rx_ms() ? (long)((now - mqtt_last_rx_ms()) / 1000) : -1;

  JsonObject ot = doc["ot"].to<JsonObject>();
  ot["frames"] = ot_stats.frames;
  ot["invalid"] = ot_stats.invalid;
  ot["timeouts"] = ot_stats.timeouts;
  ot["unknown"] = ot_stats.unknown;
  ot["age_ms"] = ot_stats.last_rx_ms ? (long)(now - ot_stats.last_rx_ms) : -1;
  ot["fpm"] = ot_stats.frames_last_minute;
  ot["delay"] = timing;
  ot["seq"] = frames_last_seq();

  //Values received from the leader (the Elga)
  JsonObject l = doc["leader"].to<JsonObject>();
  l["seen"] = leader.status_seen;
  l["ch"] = leader.ch_enable;
  l["dhw"] = leader.dhw_enable;
  set_value(l["sp"], leader.setpoint_seen, leader.control_setpoint);
  set_value(l["room_sp"], leader.room_setpoint_seen, leader.room_setpoint);
  set_value(l["room_t"], leader.room_temp_seen, leader.room_temp);
  set_value(l["max_mod"], true, leader.max_rel_mod);

  //Values reported to the leader
  JsonObject e = doc["ecv"].to<JsonObject>();
  bool ok;
  double value = flow_temperature(&ok);
  set_value(e["flow"], ok, value);
  e["flow_src"] = sensors_in_use() ? "sensor" : (ok ? "mqtt" : "none");
  value = return_temperature(&ok);
  set_value(e["ret"], ok, value);
  e["ret_src"] = sensors_in_use() ? "sensor" : (ok ? "mqtt" : "none");
  set_value(e["mod"], true, reported_modulation());
  set_value(e["pres"], true, ecv.water_pressure_ch);
  set_value(e["out"], true, ecv.outside_temperature);
  set_value(e["dhw_t"], true, ecv.dhw_temperature);
  set_value(e["dhw_flow"], true, ecv.water_flow_dhw);
  set_value(e["max_ch"], true, ecv.max_ch_water_setpoint);
  set_value(e["dhw_sp"], true, ecv.dhw_setpoint);
  set_value(e["max_mod"], true, ecv.max_rel_modulation);
  e["fault"] = ecv.fault;
  e["sensor_fault"] = sensor_fault();
  e["ch_mode"] = ecv.ch_mode;
  e["flame"] = ecv.flame;
  e["modulating"] = config.boiler_modulating;

  JsonObject auth = doc["auth"].to<JsonObject>();
  auth["set"] = config.admin_pass.length() > 0;
  auth["ok"] = signed_in(request);
  doc["setup"] = config.wifi_ssid.length() == 0;
  send_json(request, doc);
}

static void handle_sensors(AsyncWebServerRequest* request) {
  JsonDocument doc;
  JsonArray list = doc["found"].to<JsonArray>();
  for (uint8_t i = 0; i < found_count; i++) {
    JsonObject s = list.add<JsonObject>();
    s["a"] = addr_to_hex(found_sensors[i].addr);
    set_value(s["t"], sensor_ok(found_sensors[i].temp), found_sensors[i].temp);
  }
  doc["flow"] = addr_is_set(config.sensor_flow) ? addr_to_hex(config.sensor_flow) : String();
  doc["ret"] = addr_is_set(config.sensor_return) ? addr_to_hex(config.sensor_return) : String();
  set_value(doc["flow_t"], sensor_ok(sensor_flow_temp()), sensor_flow_temp());
  set_value(doc["ret_t"], sensor_ok(sensor_return_temp()), sensor_return_temp());
  doc["scanning"] = sensors_scanning();
  send_json(request, doc);
}

static void handle_settings(AsyncWebServerRequest* request) {
  if (!require_signin(request)) { return; }
  JsonDocument doc;
  doc["wifi_ssid"] = config.wifi_ssid;
  doc["wifi_pass_set"] = config.wifi_pass.length() > 0;
  doc["ap_fallback_min"] = config.ap_fallback_min;
  doc["mqtt_host"] = config.mqtt_host;
  doc["mqtt_port"] = config.mqtt_port;
  doc["mqtt_user"] = config.mqtt_user;
  doc["mqtt_pass_set"] = config.mqtt_pass.length() > 0;
  doc["mqtt_client_id"] = config.mqtt_client_id;
  doc["hostname"] = config.hostname;
  doc["modulating"] = config.boiler_modulating;
  doc["report_sensor_fault"] = config.report_sensor_fault;
  send_json(request, doc);
}

static void handle_debug_get(AsyncWebServerRequest* request) {
  JsonDocument doc;
  doc["flags"] = config.debug_flags;
  doc["serial"] = config.debug_serial;
  doc["web"] = config.debug_web;
  doc["keep"] = config.debug_keep;
  send_json(request, doc);
}

//---------------------------------------------------ROUTES---------------------------------------------------------------------
static void setup_routes() {
  //Each response is kept in memory until the browser has received it. Many requests at once over a slow link ran the
  //D1 out of memory (OOM abort and restart): limit the GET responses in progress and answer "busy" when memory is low,
  //the page retries at the next poll
  server.addMiddleware([](AsyncWebServerRequest* request, ArMiddlewareNext next) {
    if (request->method() != HTTP_GET) { next(); return; }
    if (active_gets >= MAX_ACTIVE_GETS && millis() - active_changed > 20000) { active_gets = 0; }
    if (active_gets >= MAX_ACTIVE_GETS || ESP.getFreeHeap() < MIN_FREE_HEAP || ESP.getMaxFreeBlockSize() < MIN_FREE_BLOCK) {
      low_memory_count++;
      request->send(503, "text/plain", "busy");
      return;
    }
    active_gets++;
    active_changed = millis();
    request->onDisconnect([]() {
      if (active_gets) { active_gets--; }
      active_changed = millis();
    });
    next();
  });

  //Page, gzip compressed in flash
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* response = request->beginResponse(200, "text/html", index_html_gz, index_html_gz_len);
    response->addHeader("Content-Encoding", "gzip");
    response->addHeader("Cache-Control", "no-cache");
    request->send(response);
  });

  //Read-only data, no sign in needed
  server.on("/api/status", HTTP_GET, handle_status);
  server.on("/api/sensors", HTTP_GET, handle_sensors);
  server.on("/api/debug", HTTP_GET, handle_debug_get);
  server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncResponseStream* response = request->beginResponseStream("application/json");
    history_write_json(*response);
    request->send(response);
  });
  server.on("/api/frames", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncResponseStream* response = request->beginResponseStream("application/json");
    response->printf("{\"now\":%lu,\"delay\":%u,\"frames\":", millis(), timing);
    frames_write_json(*response, query_uint(request, "after"), 40);
    response->print('}');
    request->send(response);
  });
  server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncResponseStream* response = request->beginResponseStream("application/json");
    dbg_write_json(*response, query_uint(request, "after"));
    request->send(response);
  });
  server.on("/api/settings", HTTP_GET, handle_settings);

  //Sign in and out
  server.on("/api/login", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (millis() < login_blocked_until) { send_error(request, 429, "Too many attempts. Wait a minute and try again."); return; }
    if (config.admin_pass.length() == 0 || post(request, "pass") != config.admin_pass) {
      if (++failed_logins >= 5) { failed_logins = 0; login_blocked_until = millis() + 60000; }
      send_error(request, 403, "Wrong password.");
      return;
    }
    failed_logins = 0;
    send_session(request, new_session());
  });
  server.on("/api/logout", HTTP_POST, [](AsyncWebServerRequest* request) {
    String token = session_token(request);
    for (String& session : sessions) { if (session == token) { session = String(); } }
    AsyncWebServerResponse* response = request->beginResponse(200, "application/json", "{\"ok\":true}");
    response->addHeader("Set-Cookie", "ecv=; Path=/; Max-Age=0");
    request->send(response);
  });

  //Admin password: first use without sign in, later only when signed in
  server.on("/api/password", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    String pass = post(request, "pass");
    if (pass.length() < 8 || pass.length() > 63) { send_error(request, 400, "Use 8 to 63 characters."); return; }
    config.admin_pass = pass;
    config_save();
    for (String& session : sessions) { session = String(); }
    dbg(LOG_SYS, "Admin password changed");
    send_session(request, new_session());
  });

  //First setup from the hotspot: WiFi, MQTT and admin password in one go
  server.on("/api/setup", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    String ssid = post(request, "ssid");
    String admin = post(request, "admin");
    if (ssid.length() == 0) { send_error(request, 400, "Choose a WiFi network."); return; }
    if (config.admin_pass.length() == 0 && (admin.length() < 8 || admin.length() > 63)) { send_error(request, 400, "Use 8 to 63 characters for the admin password."); return; }
    config.wifi_ssid = ssid;
    if (has_post(request, "pass")) { config.wifi_pass = post(request, "pass"); }
    if (post(request, "mqtt_host").length()) {
      config.mqtt_host = post(request, "mqtt_host");
      config.mqtt_port = post(request, "mqtt_port").toInt() ?: 1883;
      config.mqtt_user = post(request, "mqtt_user");
      config.mqtt_pass = post(request, "mqtt_pass");
    }
    if (admin.length()) { config.admin_pass = admin; }
    config_save();
    dbg(LOG_SYS, "Setup saved");
    net_wifi_changed();
    net_mqtt_changed();
    send_session(request, new_session());
  });

  server.on("/api/settings/wifi", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    if (has_post(request, "ap_fallback_min")) {
      config.ap_fallback_min = constrain(post(request, "ap_fallback_min").toInt(), 0, 60);
    }
    bool reconnect = false;
    if (has_post(request, "ssid") && post(request, "ssid").length()) {
      String ssid = post(request, "ssid");
      //A new network needs its password, for the same network an empty password keeps the saved one
      if (ssid != config.wifi_ssid || post(request, "pass").length()) { config.wifi_pass = post(request, "pass"); }
      config.wifi_ssid = ssid;
      reconnect = true;
    }
    config_save();
    send_ok(request);
    if (reconnect) { net_wifi_changed(); }
  });

  server.on("/api/settings/mqtt", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    config.mqtt_host = post(request, "host");
    config.mqtt_port = post(request, "port").toInt() ?: 1883;
    config.mqtt_user = post(request, "user");
    if (post(request, "pass").length()) { config.mqtt_pass = post(request, "pass"); }
    if (has_post(request, "clear_pass")) { config.mqtt_pass = String(); }
    if (post(request, "client_id").length()) { config.mqtt_client_id = post(request, "client_id"); }
    config_save();
    send_ok(request);
    net_mqtt_changed();
  });

  server.on("/api/settings/device", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    if (post(request, "hostname").length()) {
      String name = post(request, "hostname");
      name.toLowerCase();
      String clean;
      for (char c : name) { if (isalnum(c) || c == '-') { clean += c; } }
      if (clean.length()) { config.hostname = clean; }
    }
    if (has_post(request, "modulating")) {
      bool modulating = post(request, "modulating") == "1";
      if (modulating != config.boiler_modulating) {
        dbg(LOG_SYS, "Boiler type set to %s (ID 3)", modulating ? "modulating" : "on/off");
      }
      config.boiler_modulating = modulating;
    }
    if (has_post(request, "sensor_fault")) { config.report_sensor_fault = post(request, "sensor_fault") == "1"; }
    config_save();
    send_ok(request);
  });

  server.on("/api/sensors/scan", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    sensors_scan();
    send_ok(request);
  });

  server.on("/api/sensors/assign", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    uint8_t flow[8] = {0}, ret[8] = {0};
    String flow_hex = post(request, "flow"), ret_hex = post(request, "ret");
    if ((flow_hex.length() && !hex_to_addr(flow_hex, flow)) || (ret_hex.length() && !hex_to_addr(ret_hex, ret))) {
      send_error(request, 400, "Invalid sensor address.");
      return;
    }
    if (flow_hex.length() && flow_hex == ret_hex) { send_error(request, 400, "Flow and return need different sensors."); return; }
    memcpy(config.sensor_flow, flow, 8);
    memcpy(config.sensor_return, ret, 8);
    config_save();
    dbg(LOG_SYS, "Sensors assigned: flow %s, return %s", flow_hex.length() ? flow_hex.c_str() : "none", ret_hex.length() ? ret_hex.c_str() : "none");
    send_ok(request);
  });

  server.on("/api/debug", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    if (has_post(request, "flags"))  { config.debug_flags = post(request, "flags").toInt(); }
    if (has_post(request, "serial")) { config.debug_serial = post(request, "serial") == "1"; }
    if (has_post(request, "web"))    { config.debug_web = post(request, "web") == "1"; }
    if (has_post(request, "keep"))   { config.debug_keep = post(request, "keep") == "1"; }
    config_save();
    send_ok(request);
  });

  server.on("/api/wifi/scan", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    wifi_scan_start();
    send_ok(request);
  });
  server.on("/api/wifi/scan", HTTP_GET, [](AsyncWebServerRequest* request) {
    JsonDocument doc;
    doc["busy"] = wifi_scan_busy();
    JsonArray list = doc["nets"].to<JsonArray>();
    for (uint8_t i = 0; i < scan_count; i++) {
      JsonObject net = list.add<JsonObject>();
      net["s"] = scan_nets[i].ssid;
      net["r"] = scan_nets[i].rssi;
      net["l"] = scan_nets[i].secure;
    }
    send_json(request, doc);
  });

  //Test frame: processed as a received frame, the reply is not sent on the OpenTherm line
  server.on("/api/inject", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    String hex = post(request, "q");
    char* end;
    uint32_t frame = strtoul(hex.c_str(), &end, 16);
    if (hex.length() != 8 || *end) { send_error(request, 400, "Expected 8 hex characters."); return; }
    FrameRec rec = boiler_inject(frame);
    AsyncResponseStream* response = request->beginResponseStream("application/json");
    response->printf("{\"s\":%lu,\"q\":\"%08lx\",", (unsigned long)rec.seq, (unsigned long)rec.request);
    if (rec.flags & FRAME_REPLIED) { response->printf("\"r\":\"%08lx\",", (unsigned long)rec.reply); }
    else { response->print("\"r\":null,"); }
    response->printf("\"f\":%u}", rec.flags);
    request->send(response);
  });

  //Simulated MQTT message, for tests
  server.on("/api/sim", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    boiler_mqtt(post(request, "topic").c_str(), post(request, "payload").c_str());
    send_ok(request);
  });

  server.on("/api/restart", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    send_ok(request);
    dbg(LOG_SYS, "Restart requested");
    request_restart(1000);
  });

  server.on("/api/erase", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!require_signin(request)) { return; }
    config_erase();
    send_ok(request);
    dbg(LOG_SYS, "All settings erased");
    request_restart(1000);
  });

  //Firmware update (.bin from PlatformIO)
  server.on("/api/update", HTTP_POST,
    [](AsyncWebServerRequest* request) {
      if (!require_signin(request)) { return; }
      if (Update.hasError() || !Update.isFinished()) { send_error(request, 500, "Firmware update failed. The current firmware keeps running."); return; }
      AsyncWebServerResponse* response = request->beginResponse(200, "application/json", "{\"ok\":true}");
      response->addHeader("Connection", "close");
      request->send(response);
      request_restart(1500);
    },
    [](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final) {
      if (index == 0) {
        upload_allowed = signed_in(request);
        if (!upload_allowed) { return; }
        dbg(LOG_SYS, "Firmware update started: %s", filename.c_str());
        Update.runAsync(true);
        uint32_t space = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if (!Update.begin(space, U_FLASH)) { dbg(LOG_SYS, "Firmware update: %s", Update.getErrorString().c_str()); }
      }
      if (!upload_allowed) { return; }
      if (!Update.hasError() && Update.write(data, len) != len) { dbg(LOG_SYS, "Firmware update: %s", Update.getErrorString().c_str()); }
      if (final) {
        if (Update.end(true)) { dbg(LOG_SYS, "Firmware update done, %u bytes, restarting", (unsigned)(index + len)); }
        else { dbg(LOG_SYS, "Firmware update: %s", Update.getErrorString().c_str()); }
      }
    });

  //Captive portal: on the setup hotspot every unknown address opens the setup page
  server.onNotFound([](AsyncWebServerRequest* request) {
    if (ap_active() && request->host() != WiFi.softAPIP().toString()) {
      request->redirect("http://192.168.4.1/");
      return;
    }
    send_error(request, 404, "Not found.");
  });
}

void web_begin() {
  setup_routes();
  server.begin();
  dbg(LOG_SYS, "HTTP server started");
}

void web_loop() {
  if (restart_requested && (long)(millis() - restart_at) >= 0) {
    ESP.restart();
  }
}
