#include "sensors.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "config.h"
#include "debuglog.h"
#include "boiler.h"

// OneWire DS18S20, DS18B20, DS1822 Temperature sensor integration
#define ONE_WIRE_PIN D3  // on pin D3 (a 4.7K resistor is necessary)

//Read temperature every 5 seconds, a 12 bit conversion takes 750ms
const unsigned long READ_INTERVAL   = 5000;
const unsigned long CONVERSION_TIME = 800;

static OneWire oneWire(ONE_WIRE_PIN);
static DallasTemperature sensors(&oneWire);

FoundSensor found_sensors[MAX_SENSORS];
uint8_t found_count = 0;

static float heater_temp = DEVICE_DISCONNECTED_C;
static float return_temp = DEVICE_DISCONNECTED_C;
static bool scan_requested = true;
static bool converting = false;
static unsigned long last_request = 0;

bool sensor_ok(float temp) { return temp > DEVICE_DISCONNECTED_C + 1; }
float sensor_flow_temp()   { return heater_temp; }
float sensor_return_temp() { return return_temp; }
void sensors_scan()        { scan_requested = true; }
bool sensors_scanning()    { return scan_requested; }

void sensors_begin() {
  sensors.begin();
  sensors.setWaitForConversion(false);
}

//Search the bus and list the sensors found
static void scan() {
  sensors.begin();
  sensors.setWaitForConversion(false);
  found_count = 0;
  DeviceAddress addr;
  uint8_t count = sensors.getDeviceCount();
  for (uint8_t i = 0; i < count && found_count < MAX_SENSORS; i++) {
    if (sensors.getAddress(addr, i)) {
      memcpy(found_sensors[found_count].addr, addr, 8);
      found_sensors[found_count].temp = DEVICE_DISCONNECTED_C;
      found_count++;
    }
  }
  dbg(LOG_SYS, "1-Wire: %u sensors found on D3", found_count);

  //Nothing found: log what the search does see, a damaged sensor or bad contact shows up as a CRC error
  if (found_count == 0) {
    bool presence = oneWire.reset();
    oneWire.reset_search();
    uint8_t seen = 0;
    while (seen < MAX_SENSORS && oneWire.search(addr)) {
      seen++;
      bool crc_ok = OneWire::crc8(addr, 7) == addr[7];
      dbg(LOG_SYS, "1-Wire search saw %s%s", addr_to_hex(addr).c_str(), crc_ok ? "" : " (CRC error)");
    }
    if (!seen) { dbg(LOG_SYS, "1-Wire search saw nothing, presence pulse %s", presence ? "yes" : "no"); }
  }
  for (uint8_t i = 0; i < found_count; i++) {
    dbg(DBG_ONEWIRE, "Sensor %u : %s", i + 1, addr_to_hex(found_sensors[i].addr).c_str());
  }
  //Read the new sensors straight away
  last_request = millis() - READ_INTERVAL;
  converting = false;
}

//Reading of an assigned sensor: from the scan list, or directly from the bus if it was not found in the last scan
static float read_assigned(const uint8_t* addr) {
  if (!addr_is_set(addr)) { return DEVICE_DISCONNECTED_C; }
  for (uint8_t i = 0; i < found_count; i++) {
    if (memcmp(found_sensors[i].addr, addr, 8) == 0) { return found_sensors[i].temp; }
  }
  return sensors.getTempC(addr);
}

void sensors_loop() {
  //1-Wire timing briefly disables interrupts, only use the bus while no OpenTherm frame is in progress
  if (!boiler_quiet()) { return; }

  unsigned long now = millis();
  if (scan_requested) {
    scan_requested = false;
    scan();
    return;
  }

  //Start a conversion on all sensors, read the results when the conversion is done
  if (!converting && now - last_request >= READ_INTERVAL) {
    sensors.requestTemperatures();
    converting = true;
    last_request = now;
    return;
  }

  if (converting && now - last_request >= CONVERSION_TIME) {
    converting = false;
    for (uint8_t i = 0; i < found_count; i++) {
      found_sensors[i].temp = sensors.getTempC(found_sensors[i].addr);
    }
    heater_temp = read_assigned(config.sensor_flow);
    return_temp = read_assigned(config.sensor_return);

    //DEBUG_ONEWIRE: Print the temperature readings
    dbg(DBG_ONEWIRE, "Temperature heater is: %.2f and return water is: %.2f celcius.", heater_temp, return_temp);
  }
}
