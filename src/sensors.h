//DS18B20 1-Wire temperature sensors, read without blocking the OpenTherm loop.
#pragma once
#include <Arduino.h>

#define MAX_SENSORS 8

struct FoundSensor {
  uint8_t addr[8];
  float temp;                 // Last reading, DEVICE_DISCONNECTED_C (-127) if the sensor did not answer
};

extern FoundSensor found_sensors[MAX_SENSORS];
extern uint8_t found_count;

void sensors_begin();
void sensors_loop();
void sensors_scan();          // Search the bus again at the next opportunity
bool sensors_scanning();

//Last reading of the assigned sensors, check with sensor_ok()
float sensor_flow_temp();
float sensor_return_temp();
bool sensor_ok(float temp);
