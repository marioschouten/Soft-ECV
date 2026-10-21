#include "net.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <PubSubClient.h>
#include "config.h"
#include "debuglog.h"
#include "boiler.h"

//Set espClient to Wifi, "client" is the MQTT client
static WiFiClient espClient;
static PubSubClient client(espClient);
static DNSServer dns;
static const IPAddress ap_ip(192, 168, 4, 1);

//MQTT topics the E-CV listens to (as version 1.2.3)
static const char* const subscriptions[] = {
  "ecv/rawdata/command",
  "ecv/status/fault",
  "ecv/status/ch_mode",
  "ecv/status/flame",
  "ecv/command/max_rel_modulation",
  "ecv/command/max_ch_water_setpoint",
  "ecv/command/dhw_setpoint",
  "ecv/sensors/water_pressure_ch",
  "ecv/sensors/outside_temperature",
  "ecv/sensors/heater_flow_temperature",
  "ecv/sensors/return_water_temperature",
  "ecv/sensors/water_flow_dhw",
  "ecv/sensors/dhw_temperature",
};

const unsigned long MQTT_RETRY_INTERVAL   = 5000;
const unsigned long AP_STOP_DELAY         = 180000;   // Stop the hotspot 3 minutes after WiFi is back (if nobody is connected to it)
const unsigned long AP_STA_RETRY_INTERVAL = 180000;   // While the hotspot is on, try the WiFi network every 3 minutes
const unsigned long AP_STA_RETRY_TIME     = 20000;

static bool ap_on = false;
static bool mdns_on = false;
static bool sta_trying = false;              // Station connect attempt while the hotspot is on
static unsigned long sta_try_start = 0;
static unsigned long sta_next_try = 0;
static unsigned long sta_connected_since = 0;
static unsigned long sta_lost_since = 0;
static unsigned long last_mqtt_attempt = 0;
static bool mqtt_retry_now = true;
static unsigned long last_mqtt_rx = 0;
static int mqtt_fail_count = 0;
static bool scan_running = false;

ScanNet scan_nets[MAX_SCAN_NETS];
uint8_t scan_count = 0;

String ap_ssid() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char ssid[20];
  snprintf(ssid, sizeof(ssid), "SoftECV-Setup-%02X%02X", mac[4], mac[5]);
  return String(ssid);
}

bool wifi_connected()          { return WiFi.status() == WL_CONNECTED; }
bool ap_active()               { return ap_on; }
bool mqtt_connected()          { return client.connected(); }
int mqtt_state()               { return client.state(); }
unsigned long mqtt_last_rx_ms() { return last_mqtt_rx; }

//---------------------------------------------------Wi-FI FUNCTIONS------------------------------------------------------------
static void start_sta() {
  WiFi.begin(config.wifi_ssid.c_str(), config.wifi_pass.c_str());
}

//Setup hotspot: open on first use, protected with the admin password once that is set
static void start_ap() {
  if (ap_on) { return; }
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(ap_ip, ap_ip, IPAddress(255, 255, 255, 0));
  bool protect = config.admin_pass.length() >= 8;
  WiFi.softAP(ap_ssid().c_str(), protect ? config.admin_pass.c_str() : nullptr);
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", ap_ip);
  ap_on = true;

  //Station retries scan all channels and disturb the hotspot: retry only now and then
  WiFi.setAutoReconnect(false);
  sta_trying = false;
  sta_next_try = millis() + AP_STA_RETRY_INTERVAL;
  dbg(LOG_SYS, "Setup hotspot %s started at 192.168.4.1 (%s)", ap_ssid().c_str(), protect ? "password protected" : "open");
}

static void stop_ap() {
  if (!ap_on) { return; }
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  ap_on = false;
  dbg(LOG_SYS, "Setup hotspot stopped");
}

static void start_mdns() {
  if (mdns_on) { MDNS.notifyAPChange(); return; }
  if (MDNS.begin(config.hostname.c_str())) {
    MDNS.addService("http", "tcp", 80);
    mdns_on = true;
    dbg(LOG_SYS, "Reachable at http://%s.local", config.hostname.c_str());
  }
}

void net_wifi_changed() {
  WiFi.disconnect(false);
  WiFi.hostname(config.hostname);
  if (config.wifi_ssid.length() == 0) { return; }
  start_sta();
  sta_lost_since = millis();
  if (ap_on) {
    sta_trying = true;
    sta_try_start = millis();
  }
  dbg(LOG_SYS, "Connecting to WiFi network %s", config.wifi_ssid.c_str());
}

static void wifi_loop() {
  unsigned long now = millis();

  if (wifi_connected()) {
    if (!sta_connected_since) {
      sta_connected_since = now;
      sta_trying = false;
      dbg(LOG_SYS, "WiFi connected to %s, IP address %s, signal %d dBm", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
      start_mdns();
      mqtt_retry_now = true;
    }
    sta_lost_since = 0;
    if (ap_on && now - sta_connected_since > AP_STOP_DELAY && WiFi.softAPgetStationNum() == 0) { stop_ap(); }
  } else {
    if (sta_connected_since) {
      sta_connected_since = 0;
      dbg(LOG_SYS, "WiFi connection lost");
    }
    if (!sta_lost_since) { sta_lost_since = now; }

    //Start the setup hotspot when WiFi is not configured or lost for too long
    if (!ap_on) {
      if (config.wifi_ssid.length() == 0) {
        start_ap();
      } else if (config.ap_fallback_min && now - sta_lost_since > config.ap_fallback_min * 60000UL) {
        WiFi.disconnect(false);
        start_ap();
      }
    }

    //While the hotspot is on: try the WiFi network now and then, but not while someone is connected to the hotspot
    if (ap_on && config.wifi_ssid.length()) {
      if (sta_trying && now - sta_try_start > AP_STA_RETRY_TIME) {
        WiFi.disconnect(false);
        sta_trying = false;
        sta_next_try = now + AP_STA_RETRY_INTERVAL;
      } else if (!sta_trying && (long)(now - sta_next_try) >= 0 && WiFi.softAPgetStationNum() == 0) {
        start_sta();
        sta_trying = true;
        sta_try_start = now;
      }
    }
  }

  if (ap_on) { dns.processNextRequest(); }
  if (mdns_on) { MDNS.update(); }

  //Copy the scan results when an asynchronous scan is done
  if (scan_running) {
    int n = WiFi.scanComplete();
    if (n >= 0) {
      scan_count = 0;
      for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid.length() == 0) { continue; }
        //Keep the strongest entry of each network name
        int found = -1;
        for (int j = 0; j < scan_count; j++) { if (ssid == scan_nets[j].ssid) { found = j; } }
        if (found >= 0) {
          if (WiFi.RSSI(i) > scan_nets[found].rssi) { scan_nets[found].rssi = WiFi.RSSI(i); }
          continue;
        }
        if (scan_count >= MAX_SCAN_NETS) { continue; }
        strncpy(scan_nets[scan_count].ssid, ssid.c_str(), 32);
        scan_nets[scan_count].ssid[32] = 0;
        scan_nets[scan_count].rssi = WiFi.RSSI(i);
        scan_nets[scan_count].secure = WiFi.encryptionType(i) != ENC_TYPE_NONE;
        scan_count++;
      }
      WiFi.scanDelete();
      scan_running = false;
    } else if (n == WIFI_SCAN_FAILED) {
      scan_running = false;
    }
  }
}

void wifi_scan_start() {
  if (scan_running) { return; }
  WiFi.scanNetworks(true);
  scan_running = true;
}

bool wifi_scan_busy() { return scan_running; }

//---------------------------------------------------MQTT FUNCTIONS-------------------------------------------------------------
//MQTT message received: copy the payload as a string (PubSubClient does not end it with a zero)
static void callback(char* topic, byte* payload, unsigned int length) {
  char value[64];
  unsigned int len = min(length, (unsigned int)sizeof(value) - 1);
  memcpy(value, payload, len);
  value[len] = 0;
  last_mqtt_rx = millis();
  boiler_mqtt(topic, value);
}

bool mqtt_publish(const char* topic, const char* payload, bool retain) {
  if (!client.connected()) { return false; }
  //DEBUG_MQTT: Print the outgoing message
  dbg(DBG_MQTT, "Publish %s %s", topic, payload);
  return client.publish(topic, payload, retain);
}

void net_mqtt_changed() {
  client.disconnect();
  mqtt_retry_now = true;
  mqtt_fail_count = 0;
}

static void mqtt_connect() {
  IPAddress ip;
  if (ip.fromString(config.mqtt_host)) { client.setServer(ip, config.mqtt_port); }
  else { client.setServer(config.mqtt_host.c_str(), config.mqtt_port); }

  const char* user = config.mqtt_user.length() ? config.mqtt_user.c_str() : nullptr;
  const char* pass = config.mqtt_pass.length() ? config.mqtt_pass.c_str() : nullptr;
  //Last will: the broker publishes OFFLINE when the E-CV disappears
  if (client.connect(config.mqtt_client_id.c_str(), user, pass, "ecv/system", 0, true, "E-CV is OFFLINE")) {
    //Once connected publish birth message and subscribe to the E-CV topics
    client.publish("ecv/system", "E-CV is ONLINE", true);
    for (const char* topic : subscriptions) { client.subscribe(topic); }
    dbg(LOG_SYS, "MQTT connected to %s:%u", config.mqtt_host.c_str(), config.mqtt_port);
    mqtt_fail_count = 0;
  } else {
    //Report the first failure and then once a minute
    if (mqtt_fail_count++ % 12 == 0) {
      dbg(LOG_SYS, "MQTT connection to %s:%u failed, rc=%d, retrying every 5 seconds", config.mqtt_host.c_str(), config.mqtt_port, client.state());
    }
  }
}

static void mqtt_loop() {
  if (!wifi_connected() || config.mqtt_host.length() == 0) { return; }
  if (client.connected()) {
    client.loop();
    return;
  }
  //A connect attempt can take a few hundred ms: only start one when no OpenTherm frame is in progress
  unsigned long now = millis();
  if ((mqtt_retry_now || now - last_mqtt_attempt > MQTT_RETRY_INTERVAL) && boiler_quiet()) {
    mqtt_retry_now = false;
    last_mqtt_attempt = now;
    mqtt_connect();
  }
}

void net_begin(bool force_ap) {
  WiFi.persistent(false);
  WiFi.hostname(config.hostname);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  //Short timeouts so a missing broker does not hold up the OpenTherm loop
  espClient.setTimeout(400);
  client.setSocketTimeout(2);
  client.setBufferSize(512);
  client.setCallback(callback);

  if (config.wifi_ssid.length()) {
    start_sta();
    dbg(LOG_SYS, "Connecting to WiFi network %s", config.wifi_ssid.c_str());
  }
  sta_lost_since = millis();
  if (config.wifi_ssid.length() == 0 || force_ap) {
    start_ap();
    if (config.wifi_ssid.length()) {
      //Forced hotspot (double reset): keep trying the configured network as well
      sta_trying = true;
      sta_try_start = millis();
      start_sta();
    }
  }
}

//Blue LED: blinking = setup hotspot is on, on = connected to the WiFi network, off = no connection
static void led_loop() {
  bool led_on = ap_on ? (millis() / 500) % 2 : wifi_connected();
  digitalWrite(LED_BUILTIN, led_on ? LOW : HIGH);   // The LED is on when the pin is LOW
}

void net_loop() {
  wifi_loop();
  mqtt_loop();
  led_loop();
}
