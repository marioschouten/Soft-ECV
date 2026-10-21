//Version 1.2.3 by Mario M.C. Schouten
//
//Arduino Wemo D1 mini based E-C(entral)H(eating) follower using OpenTherm protocol and Ihor Melnyk's slave Terminal adapter for communication.
//
//Implemented functionality is as follows:
// - The E-CV operational status is set via MQTT topic [ecv/status] with the format as described under HEATER SETTINGS
// - The various measurements of temperature, pressure, flow etc. are set via MTT topic [ecv/sensors] as described under HEATER SETTINGS
// - The commands to update setpoints are set via MQTT topic [ecv/command] in the format as described under HEATER SETTINGS
// - The modulation level requested by the Thermostat is publish via MQTT topic [ecv/thermostat/modulation]
// - The CH requested by the Thermostat is publish via MQTT topic [ecv/thermostat/ch_requested]
// - The CH setpoint by the Thermostat is publish via MQTT topic [ecv/thermostat/ch_setpoint]
// - The Thermostat raw data is publish via MQTT topic [ecv/thermostat/rawdata]


//Libraries
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <OpenTherm.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <settings.h>


//WiFi parameters
#ifndef STASSID
  const char* STASSID = WIFI_SSID;
  const char* STAPSK  = WIFI_PASSWORD;
#endif

//MQTT parameters
const char* mqtt_server   = MQTT_HOST;
const int   mqtt_port     = MQTT_PORT; 
const char* mqtt_user     = MQTT_USER;
const char* mqtt_password = MQTT_PASSWORD;

//OpenTherm input and output wires connected to 4 and 5 pins on the OpenTherm Shield
const int inPin = 12;  //for Arduino, 12 for ESP8266 (D6), 19 for ESP32
const int outPin = 13; //for Arduino, 13 for ESP8266 (D7), 23 for ESP32
OpenTherm ot(inPin, outPin, true);

// OneWire DS18S20, DS18B20, DS1822 Temperature sensor integration
#define ONE_WIRE_PIN D3  // on pin D3 (a 4.7K resistor is necessary)
//Openterm Leader Follower response timing (Min. 20ms - max. 800ms)
unsigned int timing       = 250; // Default timing is 25ms

//ECV STATUS SETTINGS - Default can be adjusted with MQTT message
const char* fault_indication = "0";         // Default = 0, updated with MQTT topic [ecv/status/fault]
const char* CH_mode = "0";                  // Default = 0, updated with MQTT topic [ecv/status/ch_mode]
const char* flame_status = "0";             // Default = 0, updated with MQTT topic [ecv/status/flame]
const char* DHW_mode = "0";                 // Default = 0, if DHW is present change to 1
const char* cooling_status = "0";           // Default = 0, no support in this software version for cooling status
const char* CH2_mode = "0";                 // Default = 0, no support in this software version for CH2 mode
const char* diagnostic = "0";               // Default = 0, no support in this software version for diagnostics
const char* msg_0_bit_7 = "0";              // Reserved

//ECV COMMAND SETTINGS - Default can be adjusted with MQTT message
double max_rel_modulation = 100;            // Default = 100, updated with MQTT topic [ecv/command/max_rel_modulation]
double max_ch_water_setpoint = 70;          // Default =  70, updated with MQTT topic [ecv/command/max_ch_water_setpoint]
double dhw_setpoint = 65;                   // Default =  65, updated with MQTT topic [ecv/command/dhw_setpoint]

//ECV SENSORS SETTINGS - Default can be adjusted with MQTT message
double water_pressure_ch = 2.00;            // Default =  2, updated with MQTT topic [ecv/sensors/water_pressure_ch]
double outside_temperature = 0.00;          // Default =  0, updated with MQTT topic [ecv/sensors/outside_temperature]
double heater_flow_temperature = 0.00;      // Default =  0, updated with MQTT topic [ecv/sensors/heater_flow_temperature]
double return_water_temperature = 0.00;     // Default =  0, updated with MQTT topic [ecv/sensors/return_water_temperature]
double water_flow_dhw = 0.00;               // Default =  0, updated with MQTT topic [ecv/sensors/water_flow_dhw]
double dhw_temperature = 0.00;              // Default =  0, updated with MQTT topic [ecv/sensors/dhw_temperature]

//DEBUG MESSAGE SETTING
const char* serial_monitor    = "0";        // Default = 0, if set to 1 the OpenTherm traffic will be shown on the serial monitor
const char* serial_mqtt       = "0";        // Default = 0, if set to 1 all outgoing MQTT related debug messages are shown on the serial terminal
const char* serial_mqtt_in    = "0";        // Default = 0, if set to 1 all incomming MQTT related debug messages are shown on the serial terminal
const char* serial_range      = "0";        // Default = 0, if set to 1 all range check debug messages are shown on the serial terminal
const char* serial_update     = "0";        // Default = 0, is set to 1 all value updates are shown on the serial terminal
const char* serial_convert    = "0";        // Default = 0, if set to 1 all value to hex conversion debug messages are shown on the serial terminal
const char* serial_onewire    = "0";        // Default = 0, if set to 1 the system will print a list of device addresses to the terminal
const char* serial_debug      = "0";        // Default = 0, if set to 1 debug messages are shown on the serial monitor


//Internal program variables, DO NOT CHANGE
//Init WiFi
const char* ssid     = STASSID;
const char* password = STAPSK;
AsyncWebServer server(80);


//Set espClient to Wifi
WiFiClient espClient;
//Set "client" to be the variable name for the MQTT client
PubSubClient client(espClient);

//One wire variables
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);

float heater_temp, return_temp;

uint8_t sensor1[8] = {0x28, 0xE8, 0x88, 0x79, 0xA2, 0x00, 0x03, 0x03};
uint8_t sensor2[8] = {0x28, 0x18, 0xCD, 0x79, 0xA2, 0x00, 0x03, 0x4A};

DeviceAddress Thermometer;

int deviceCount              = 0;
unsigned long last_temp              = millis();
unsigned long last_ch_update         = millis();
unsigned long last_modulation_update = millis();

//Setup message buffer size
#define MSG_BUFFER_SIZE (110)
char msg[MSG_BUFFER_SIZE];

//OpenTherm message ID 0 HB and LB variables
int leader_status[8]   = {0,0,0,0,0,0,0,0};
int follower_status[8] = {0,0,0,0,0,0,0,0};

//OpenTherm reply message bit parity counter
int f2l_parity          = 0;   // Parity counter
int parity_correction   = 0;   // Parity correction for message ID 03

//Counter for test message transmissions
long int value          = 0;

//Flag for MQTT modulation reporting
int ch_enabled          = 0;
int ch_enabled_history  = 0;

//Variables for modulation level
double set_modulation      =  0.00;
double control_ch_setpoint =  0.00;
double temp_difference     =  0.00;
double upper_limit         = 20.00;
double lower_limit         =  2.00;







//-------------------------------------------OpenTherm message FUNCTIONS--------------------------------------------------------
//OpenTherm interupt handler
void IRAM_ATTR handleInterrupt() {
  //OpenTerm interrupt handler
  ot.handleInterrupt();
}

//DECODE message flag flag8 status bits and return value
String decode_flag_flag8(String msg_value) {
  String original   = msg_value;
  String LB         = msg_value.substring(2 , 1);
  String HB         = msg_value.substring(1 , 0);
  String msg_lb     = "";
  String msg_hb     = "";

  //Save current parity
  parity_correction = f2l_parity;

  //Decode the LB
  if (LB == "0") { leader_status[4] = 0, leader_status[5] = 0; leader_status[6] = 0; leader_status[7] = 0;  msg_lb = "0000"; f2l_parity = f2l_parity + 0;}
  if (LB == "1") { leader_status[4] = 0, leader_status[5] = 0; leader_status[6] = 0; leader_status[7] = 1;  msg_lb = "0001"; f2l_parity = f2l_parity + 1;}
  if (LB == "2") { leader_status[4] = 0, leader_status[5] = 0; leader_status[6] = 1; leader_status[7] = 0;  msg_lb = "0010"; f2l_parity = f2l_parity + 1;}
  if (LB == "3") { leader_status[4] = 0, leader_status[5] = 0; leader_status[6] = 1; leader_status[7] = 1;  msg_lb = "0011"; f2l_parity = f2l_parity + 2;}
  if (LB == "4") { leader_status[4] = 0, leader_status[5] = 1; leader_status[6] = 0; leader_status[7] = 0;  msg_lb = "0100"; f2l_parity = f2l_parity + 1;}
  if (LB == "5") { leader_status[4] = 0, leader_status[5] = 1; leader_status[6] = 0; leader_status[7] = 1;  msg_lb = "0101"; f2l_parity = f2l_parity + 2;}
  if (LB == "6") { leader_status[4] = 0, leader_status[5] = 1; leader_status[6] = 1; leader_status[7] = 0;  msg_lb = "0110"; f2l_parity = f2l_parity + 2;}
  if (LB == "7") { leader_status[4] = 0, leader_status[5] = 1; leader_status[6] = 1; leader_status[7] = 1;  msg_lb = "0111"; f2l_parity = f2l_parity + 3;}
  if (LB == "8") { leader_status[4] = 1, leader_status[5] = 0; leader_status[6] = 0; leader_status[7] = 0;  msg_lb = "1000"; f2l_parity = f2l_parity + 1;}
  if (LB == "9") { leader_status[4] = 1, leader_status[5] = 0; leader_status[6] = 0; leader_status[7] = 1;  msg_lb = "1001"; f2l_parity = f2l_parity + 2;}
  if (LB == "A") { leader_status[4] = 1, leader_status[5] = 0; leader_status[6] = 1; leader_status[7] = 0;  msg_lb = "1010"; f2l_parity = f2l_parity + 2;}
  if (LB == "B") { leader_status[4] = 1, leader_status[5] = 0; leader_status[6] = 1; leader_status[7] = 1;  msg_lb = "1011"; f2l_parity = f2l_parity + 3;}
  if (LB == "C") { leader_status[4] = 1, leader_status[5] = 1; leader_status[6] = 0; leader_status[7] = 0;  msg_lb = "1100"; f2l_parity = f2l_parity + 2;}
  if (LB == "D") { leader_status[4] = 1, leader_status[5] = 1; leader_status[6] = 0; leader_status[7] = 1;  msg_lb = "1101"; f2l_parity = f2l_parity + 3;}
  if (LB == "E") { leader_status[4] = 1, leader_status[5] = 1; leader_status[6] = 1; leader_status[7] = 0;  msg_lb = "1110"; f2l_parity = f2l_parity + 3;}
  if (LB == "F") { leader_status[4] = 1, leader_status[5] = 1; leader_status[6] = 1; leader_status[7] = 1;  msg_lb = "1111"; f2l_parity = f2l_parity + 4;}

  //Decode the HB
  if (HB == "0") { leader_status[0] = 0, leader_status[1] = 0; leader_status[2] = 0; leader_status[3] = 0;  msg_hb = "0000"; f2l_parity = f2l_parity + 0;}
  if (HB == "1") { leader_status[0] = 0, leader_status[1] = 0; leader_status[2] = 0; leader_status[3] = 1;  msg_hb = "0001"; f2l_parity = f2l_parity + 1;}
  if (HB == "2") { leader_status[0] = 0, leader_status[1] = 0; leader_status[2] = 1; leader_status[3] = 0;  msg_hb = "0010"; f2l_parity = f2l_parity + 1;}
  if (HB == "3") { leader_status[0] = 0, leader_status[1] = 0; leader_status[2] = 1; leader_status[3] = 1;  msg_hb = "0011"; f2l_parity = f2l_parity + 2;}
  if (HB == "4") { leader_status[0] = 0, leader_status[1] = 1; leader_status[2] = 0; leader_status[3] = 0;  msg_hb = "0100"; f2l_parity = f2l_parity + 1;}
  if (HB == "5") { leader_status[0] = 0, leader_status[1] = 1; leader_status[2] = 0; leader_status[3] = 1;  msg_hb = "0101"; f2l_parity = f2l_parity + 2;}
  if (HB == "6") { leader_status[0] = 0, leader_status[1] = 1; leader_status[2] = 1; leader_status[3] = 0;  msg_hb = "0110"; f2l_parity = f2l_parity + 2;}
  if (HB == "7") { leader_status[0] = 0, leader_status[1] = 1; leader_status[2] = 1; leader_status[3] = 1;  msg_hb = "0111"; f2l_parity = f2l_parity + 3;}
  if (HB == "8") { leader_status[0] = 1, leader_status[1] = 0; leader_status[2] = 0; leader_status[3] = 0;  msg_hb = "1000"; f2l_parity = f2l_parity + 1;}
  if (HB == "9") { leader_status[0] = 1, leader_status[1] = 0; leader_status[2] = 0; leader_status[3] = 1;  msg_hb = "1001"; f2l_parity = f2l_parity + 2;}
  if (HB == "A") { leader_status[0] = 1, leader_status[1] = 0; leader_status[2] = 1; leader_status[3] = 0;  msg_hb = "1010"; f2l_parity = f2l_parity + 2;}
  if (HB == "B") { leader_status[0] = 1, leader_status[1] = 0; leader_status[2] = 1; leader_status[3] = 1;  msg_hb = "1011"; f2l_parity = f2l_parity + 3;}
  if (HB == "C") { leader_status[0] = 1, leader_status[1] = 1; leader_status[2] = 0; leader_status[3] = 0;  msg_hb = "1100"; f2l_parity = f2l_parity + 2;}
  if (HB == "D") { leader_status[0] = 1, leader_status[1] = 1; leader_status[2] = 0; leader_status[3] = 1;  msg_hb = "1101"; f2l_parity = f2l_parity + 3;}
  if (HB == "E") { leader_status[0] = 1, leader_status[1] = 1; leader_status[2] = 1; leader_status[3] = 0;  msg_hb = "1110"; f2l_parity = f2l_parity + 3;}
  if (HB == "F") { leader_status[0] = 1, leader_status[1] = 1; leader_status[2] = 1; leader_status[3] = 1;  msg_hb = "1111"; f2l_parity = f2l_parity + 4;}

  //Build new mgw_value from LB and HB
  msg_value = msg_hb + msg_lb;

  //Set parity_correction to current parity, value will be needed later to be subtracted when building the new leader message value
  parity_correction = f2l_parity - parity_correction; 

  //DEBUG_DEBUG: print the received OpenTHerm leader status message
  if (strcmp(serial_debug, "1") == 0 ) {
    Serial.print("Original msg_value: ");
    Serial.print(original);
    Serial.print(" LB: ");
    Serial.print(LB);
    Serial.print(" HB: ");
    Serial.print(HB);
    Serial.print(" msg_value: ");
    Serial.print(msg_value);
    Serial.print(" Leader status: ");
    for (int i = 0; i < 8; i++) { Serial.print(leader_status[i]); }
    Serial.println();
  }

  //Return the measurement value
  return msg_value;
}  

//DECODE message flag f8.8 measurements and return value
String decode_flag_f8(String msg_value) {
  //Initializing base value to 1, i.e 16^0, set the variable length fixed to 4 and the dec_val to 0
  int base = 1;  int len = 4; double dec_val = 0;

  //Extracting characters as digits from last character 
  for (int i=len-1; i>=0; i--) {
    // if character lies in '0'-'9', converting it to integral 0-9 by subtracting 48 from ASCII value.
    if (msg_value[i]>='0' && msg_value[i]<='9') {
      dec_val += (msg_value[i] - 48)*base;

      // incrementing base by power
      base = base * 16;
    } 

    // if character lies in 'A'-'F' , converting it to integral 10 - 15 by subtracting 55 from ASCII value
    else if (msg_value[i]>='a' && msg_value[i]<='f') {
        dec_val += (msg_value[i] - 87)*base;

        // incrementing base by power
        base = base*16;
    }
  }

  //DEBUG_DEBUG: Measurement in hex, decimal and converted to real value
  if (strcmp(serial_debug, "1") == 0 ) {
    Serial.print("Measurement in hexadecimal: ");
    Serial.print(msg_value);
    Serial.print(" decimal: ");
    Serial.print(dec_val);
    Serial.print(" and divided by 256: ");
    Serial.print(dec_val/256);
    Serial.println();
  }

  //Calculate the measurement and convert to a string
  dec_val = dec_val / 256;
  msg_value = String(dec_val, 2);

//DEBUG_DEBUG: Print the decoded measurement value
  if (strcmp(serial_debug, "1") == 0 ) {
    Serial.print("Converted measurement being returned in String msg_value is: ");
    Serial.print(msg_value);
    Serial.println();
  }
  
  //Return the measurement value
  return msg_value;
}  

//ENCODE message flag f8.8 measurements and return value
String encode_flag_f8(String msg_value) {
  //Convert string to double and multiply as per protocol
  long dec_val       = msg_value.toFloat() * 256;
  long dec_parity    = dec_val;
  long dec_received  = dec_val;

  //Calculate parity
  bool parity = 0; 
  while (dec_parity) { 
    parity = !parity; 
    dec_parity = dec_parity & (dec_parity - 1); 
  }

  //Update parity count
  if (parity == true) {
    f2l_parity = f2l_parity + 1;
  }

  //Covert decimal to hex
  int rem      =  0;
  msg_value    = "";

  if (dec_val != 0 ) {
    while (dec_val > 0)   // Do this whilst the quotient is greater than 0.
    {
      rem = dec_val % 16; // Get the remainder.
      if (rem > 9)
      {
        // Map the character given that the remainder is greater than 9.
        switch (rem)
        {
          case 10: msg_value = "A" + msg_value; break;
          case 11: msg_value = "B" + msg_value; break;
          case 12: msg_value = "C" + msg_value; break;
          case 13: msg_value = "D" + msg_value; break;
          case 14: msg_value = "E" + msg_value; break;
          case 15: msg_value = "F" + msg_value; break;
        }
      }
      else
      {
        msg_value = char(rem + 48) + msg_value; // Converts integer (0-9) to ASCII code.
        // x + 48 is the ASCII code for x digit (if 0 <= x <= 9)
      }
      dec_val = dec_val/16;
    }

    //Check if the Hex value is formated to ####
    while (msg_value.length() < 4) {
      msg_value = "0" + msg_value;
    }
  } else {
    msg_value = "0000";
  }

  //DEBUG_CONVERT: Measurement in decimal, convert to Hex
  if (strcmp(serial_convert, "1") == 0 ) {
    Serial.print("Measurement multiplied by 256 in decimal is: ");
    Serial.print(dec_received);
    Serial.print(" and in Hex: ");
    Serial.print(msg_value);
    Serial.print(" With parity of the measurement value: ");
    Serial.print(parity);
    Serial.print(" brings the total parity to: ");
    Serial.print(f2l_parity);
    Serial.println();
  }

  //Return the measurement value
  return msg_value;
}  



//---------------------------------------------------Wi-FI & MQTT FUNCTIONS-----------------------------------------------------
//FUNCTION: Setup WiFi connection, called from setup()
void setup_wifi() {
  delay(10);

  //DEBUG_MONITOR: We start by connecting to a WiFi network
  if (strcmp(serial_monitor, "1") == 0 ) {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
  }

  //Set WiFi-client mode
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //DEBUG_MONITOR: Print a . for every 500ms waiting loop finished
    if (strcmp(serial_monitor, "1") == 0 ) {
      Serial.print(".");
    }
  }

  randomSeed(micros());

  //DEBUG_MONITOR: Show Wi-Fi connection status on serial monitor
  if (strcmp(serial_monitor, "1") == 0 ) {
    Serial.println("");
    Serial.print("WiFi connected to ");
    Serial.print("IP address: ");
    Serial.print(WiFi.localIP());
    Serial.println();
  }

  //Switch ON the LED
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
}

//FUNCTION: Call-back on MQTT message, called from setup() to update variables with MQTT topic "sensors"  messages
void callback(char* topic, byte* payload, unsigned int length) {
  //DEBUG_MQTT: Print the topic of the received MQTT message
  if (strcmp(serial_mqtt_in, "1") == 0 ) {
    Serial.print("MQTT Message topic: ");
    Serial.print(topic);
  }

  //MQTT TOPIC is [ecv/sensors/fault], set the corresponding variables
  if (strcmp(topic, "ecv/status/fault") == 0) {
    if ((char)payload[0] == 48 ) {follower_status[7] = 0; }
    if ((char)payload[0] == 49 ) {follower_status[7] = 1; }
    //DEBUG_MQTT: Print payload of MQTT message with topic [ecv/sensors/fault]
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("   Fault status: ");
      Serial.print(payload[0]);
      Serial.print("   Follower status: ");
      for (int i = 0; i < 8; i++) {Serial.print(follower_status[i]);}
      Serial.println();
    }
  }

  //MQTT TOPIC is [ecv/sensors/ch_mode], set the corresponding variables
  if (strcmp(topic, "ecv/status/ch_mode") == 0) {
    if ((char)payload[0] == 48 ) {follower_status[6] = 0; }
    if ((char)payload[0] == 49 ) {follower_status[6] = 1; }
    //DEBUG_MQTT: Print payload of MQTT message with topic [ecv/sensors/ch_mode]
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("   CH-Mode status: ");
      Serial.print(payload[0]);
      Serial.print("   Follower status: ");
      for (int i = 0; i < 8; i++) {Serial.print(follower_status[i]);}
      Serial.println();
    }
  }

  //MQTT TOPIC is [ecv/sensors/flame], set the corresponding variables
  if (strcmp(topic, "ecv/status/flame") == 0) {
    //Set modulation level based on flame OFF
    if ((char)payload[0] == 48 ) {
      follower_status[4] = 0; 
    }
    //Set modulation level based on flame ON
    if ((char)payload[0] == 49 ) {
      follower_status[4] = 1; 
    }
    //DEBUG_MQTT: Print payload of MQTT message
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("   Flame status: ");
      Serial.print(payload[0]);
      Serial.print("   Follower status: ");
      for (int i = 0; i < 8; i++) {Serial.print(follower_status[i]);}
      Serial.println();
    }
  }

  //MQTT TOPIC is [ecv/command/max_rel_modulation], set the corresponding variables
  if (strcmp(topic, "ecv/command/max_rel_modulation") == 0) {
    String test = String((char*)payload);
    max_rel_modulation = test.toDouble();
    //DEBUG_MQTT: Print payload of MQTT message
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("   Set max relative modulation: ");
      Serial.print(max_rel_modulation);
      Serial.println();
    }
  }

  //MQTT TOPIC is [ecv/command/max_ch_water_setpoint], set the corresponding variables
  if (strcmp(topic, "ecv/command/max_ch_water_setpoint") == 0) {
    String test = String((char*)payload);
    max_ch_water_setpoint = test.toDouble();
    //DEBUG_MQTT: Print payload of MQTT message
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("   Set max CH water setpoint: ");
      Serial.print(max_ch_water_setpoint);
      Serial.println();
    }
  }

  //MQTT TOPIC is [ecv/command/dhw_setpoint], set the corresponding variables
  if (strcmp(topic, "ecv/command/dhw_setpoint") == 0) {
    dhw_setpoint = atoi((char *)payload);
    //DEBUG_MQTT: Print payload of MQTT message
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("   Set DHW setpoint: ");
      Serial.print(dhw_setpoint);
      Serial.println();
    }
  }

  //MQTT TOPIC is [ecv/sensors/water_pressure_ch], set the corresponding variables
  if (strcmp(topic, "ecv/sensors/water_pressure_ch") == 0) {
    String test = String((char*)payload);
    water_pressure_ch = test.toDouble();
    //DEBUG_MQTT: Print payload of MQTT message
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("   Water pressure CH: ");
      Serial.print(water_pressure_ch);
      Serial.println();
    }
  }

  //MQTT TOPIC is [ecv/sensors/outside_temperature], set the corresponding variables
  if (strcmp(topic, "ecv/sensors/outside_temperature") == 0) {
    String test = String((char*)payload);
    outside_temperature = test.toDouble();
    //DEBUG_MQTT: Print payload of MQTT message
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("   Outside temperature: ");
      Serial.print(outside_temperature);
      Serial.println();
    }
  }

  //MQTT TOPIC is [ecv/sensors/heater_flow_temperature], set the corresponding variables
  if (strcmp(topic, "ecv/sensors/heater_flow_temperature") == 0) {
    String test = String((char*)payload);
    heater_flow_temperature = test.toDouble();
    //DEBUG_MQTT: Print payload of MQTT message
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("   Boiler flow temperature: ");
      Serial.print(heater_flow_temperature);
      Serial.println();
    }
  }

  //MQTT TOPIC is [ecv/sensors/return_water_temperature], set the corresponding variables
  if (strcmp(topic, "ecv/sensors/return_water_temperature") == 0) {
    String test = String((char*)payload);
    return_water_temperature = test.toDouble();
    //DEBUG_MQTT: Print payload of MQTT message
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("   Return water temperature: ");
      Serial.print(return_water_temperature);
      Serial.println();
    }
  }

  //MQTT TOPIC is [ecv/sensors/water_flow_dhw], set the corresponding variables
  if (strcmp(topic, "ecv/sensors/water_flow_dhw") == 0) {
    String test = String((char*)payload);
    water_flow_dhw = test.toDouble();
    //DEBUG_MQTT: Print payload of MQTT message
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("   Water flow DHW: ");
      Serial.print(water_flow_dhw);
      Serial.println();
    }
  }

  //MQTT TOPIC is [ecv/sensors/dhw_temperature], set the corresponding variables
  if (strcmp(topic, "ecv/sensors/dhw_temperature") == 0) {
    String test = String((char*)payload);
    dhw_temperature = test.toDouble();
    //DEBUG_MQTT: Print payload of MQTT message
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("   DHW Temperature: ");
      Serial.print(dhw_temperature);
      Serial.println();
    }
  }

  //MQTT TOPIC is "ecv/rawdata/command", use the payload of 8 characters to test the analysis_respond software
  if (strcmp(topic, "ecv/rawdata/command") == 0) {
    //Transform MQTT payload to ASCII characters in pos[8]
    String msg_pos[8];

    byte position_0 = (char)payload[0];
    byte position_1 = (char)payload[1];
    byte position_2 = (char)payload[2];
    byte position_3 = (char)payload[3];
    byte position_4 = (char)payload[4];
    byte position_5 = (char)payload[5];
    byte position_6 = (char)payload[6];
    byte position_7 = (char)payload[7];

    msg_pos[0] = static_cast<char>(position_0);
    msg_pos[1] = static_cast<char>(position_1);
    msg_pos[2] = static_cast<char>(position_2);
    msg_pos[3] = static_cast<char>(position_3);
    msg_pos[4] = static_cast<char>(position_4);
    msg_pos[5] = static_cast<char>(position_5);
    msg_pos[6] = static_cast<char>(position_6);
    msg_pos[7] = static_cast<char>(position_7);
  
    //DEBUG_MQTT: On serial terminal report message arrived with content
    if (strcmp(serial_mqtt_in, "1") == 0 ) {
      Serial.print("MQTT Message arrived with topic [");
      Serial.print(topic);
      Serial.print("] and is converted and stored into pos[] with content: ");
      for (int i = 0; i < 8; i++) {Serial.print(msg_pos[i]);}
      Serial.println();
    }


    //Decode incoming message and send reply
    //processRequest(msg_pos);
  }
}

//FUNCTION: Reconnect MQTT, called from loop()
void reconnect() {
  //Loop until we're reconnected
  while (!client.connected()) {
    if (strcmp(serial_monitor, "1") == 0 ) {
      Serial.print("Attempting MQTT connection...");
    }

    // Attempt to connect
    if (client.connect("ECV", mqtt_user, mqtt_password)) {
      //Switch ON the LED
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)

      //Show connected on serial terminal
      if (strcmp(serial_monitor, "1") == 0 ) {
        Serial.println("connected");
      }

      //Once connected publish birth message on initial connection
      snprintf (msg, MSG_BUFFER_SIZE, "E-CV is ONLINE");
      client.publish("ecv/system",msg);
      client.subscribe("ecv/rawdata/command");
      client.subscribe("ecv/status/fault");
      client.subscribe("ecv/status/ch_mode");
      client.subscribe("ecv/status/flame");
      client.subscribe("ecv/command/max_rel_modulation");
      client.subscribe("ecv/command/max_ch_water_setpoint");
      client.subscribe("ecv/command/dhw_setpoint");
      client.subscribe("ecv/sensors/water_pressure_ch");
      client.subscribe("ecv/sensors/outside_temperature");
      client.subscribe("ecv/sensors/heater_flow_temperature");
      client.subscribe("ecv/sensors/return_water_temperature");
      client.subscribe("ecv/sensors/water_flow_dhw");
      client.subscribe("ecv/sensors/dhw_temperature");
      
      //TEST: Print the result
      if (strcmp(serial_mqtt, "1") == 0 ) {
        Serial.print("Publish message: ");
        Serial.println(msg);
      }

    } else {
      //Switch OFF the LED
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

      //Show failed with error code on serial terminal
      if (strcmp(serial_monitor, "1") == 0 ) {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
      }

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//FUNCTION: Print list of onewire device address if debug_onewire is enabled
void printAddress(DeviceAddress deviceAddress) { 
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.println("");
}

//FUNCTION: Read temperature sensors 
void read_temperature(){
  //Read sensors and save result in variable
  sensors.requestTemperatures();
  heater_temp  = sensors.getTempC(sensor1); // Gets the values of the temperature
  return_temp  = sensors.getTempC(sensor2); // Gets the values of the temperature

  //DEBUG_ONEWIRE: Print the temperature readings to the terminal
  if (strcmp(serial_onewire, "1") == 0 ) {
      Serial.print("Temperature heater is: ");
      Serial.print(heater_temp);
      Serial.print(" and return water is: ");
      Serial.print(return_temp);
      Serial.print(" celcius.");
      Serial.println();
  }
}

//OpenTherm process received data and send reply
void processRequest(unsigned long request, OpenThermResponseStatus status) {
//DECODE the MESSAGE_TYPE and formulate a response
  //Initialize variables
  unsigned long msg_rx_ts     = millis();
  const char* l2f_message     = "NO_VALID_INPUT";
  const char* f2l_message     = "NO_VALID_REPLY";
  const char* f2l_hex         = "0";
  const char* msg_description = "NO_VALID_DESCRIPTION";
  const char* msg_flag        = "";
  const char* pass            = "";
  const char* msg_rw          = "";
  String msg_heater           = "T-";
  String msg_thermostat       = "B-";
  String msg_id               = "";
  String msg_value            = "";
  String msg_value_hex        = "";
  String msg_value_leader     = "";
  String msg_value_follower   = "";
  String msg_received         = "";
  String msg_full             = "";
  String msg_pos[8]           = {""};

  double old_value            = 0;
  double range_low            = 0;
  double range_high           = 0;

  //Init variables
  f2l_parity                  = 0;
  msg_value_follower          = "";

  //Build incoming message into a String msg_heater
  msg_heater = String(request, HEX);
  while (msg_heater.length() < 8 ) {
    msg_heater = "0" + msg_heater;
  }

    //Build incoming message into msg_pos[i];
  for (int i = 0; i < 8; i++) {
    msg_pos[i] = msg_heater.substring(i,i+1); 
  }

 //DEBUG_DEBUG: Print the decoded message
  if (strcmp(serial_debug, "1") == 0 ) {
    Serial.print("Decoded message: ");
    Serial.print(msg_heater);
    Serial.println();
  }

  //Check the message type
  if (msg_pos[0] == "0" || msg_pos[0] == "8") {l2f_message = "READ-DATA     ";}
  if (msg_pos[0] == "1" || msg_pos[0] == "9") {l2f_message = "WRITE-DATA    ";}
  if (msg_pos[0] == "2" || msg_pos[0] == "A") {l2f_message = "INVALID-DATA  ";}
  if (msg_pos[0] == "3" || msg_pos[0] == "B") {l2f_message = "RESERVED      ";}

  //DECODE the MESSAGE_IS and formulate a response
  msg_id = msg_pos[2] + msg_pos[3];
  if (msg_id == "00") {msg_description = "Status flags: ";                                        msg_flag = "flag8"; msg_rw = "R"; f2l_parity = f2l_parity + 0;} //Decimal 0
  if (msg_id == "01") {msg_description = "Control setpoint CH water temperature (C): ";           msg_flag = "f8.8";  msg_rw = "W"; f2l_parity = f2l_parity + 1; range_low =   0; range_high = 100;} //Decimal 1
  if (msg_id == "03") {msg_description = "Follower config flags and Leader MemberID code: ";      msg_flag = "flag8"; msg_rw = "R"; f2l_parity = f2l_parity + 2;} //Decimal 3
  if (msg_id == "05") {msg_description = "Application-specific and OEM fault flags: ";            msg_flag = "u8"  ;  msg_rw = "R"; f2l_parity = f2l_parity + 2;} //Decimal 5
  if (msg_id == "0e") {msg_description = "Maximum relative modulation level setting (Percent): "; msg_flag = "f8.8";  msg_rw = "W"; f2l_parity = f2l_parity + 3; range_low =   0; range_high = 100;} //Decimal 14
  if (msg_id == "10") {msg_description = "Room setpoint: ";                                       msg_flag = "f8.8";  msg_rw = "W"; f2l_parity = f2l_parity + 1; range_low = -40; range_high = 127;} //Decimal 16
  if (msg_id == "11") {msg_description = "Relative modulation level (Percent): ";                 msg_flag = "f8.8";  msg_rw = "R"; f2l_parity = f2l_parity + 2; range_low =   0; range_high = 100;} //Decimal 17
  if (msg_id == "12") {msg_description = "Water pressure in CH circuit (bar): ";                  msg_flag = "f8.8";  msg_rw = "R"; f2l_parity = f2l_parity + 2; range_low =   0; range_high =   5;} //Decimal 18
  if (msg_id == "13") {msg_description = "Water flow rate in DHW circuit (litres/minute): ";      msg_flag = "f8.8";  msg_rw = "R"; f2l_parity = f2l_parity + 3; range_low =   0; range_high =  16;} //Decimal 19
  if (msg_id == "18") {msg_description = "Room temperature (C): ";                                msg_flag = "f8.8";  msg_rw = "W"; f2l_parity = f2l_parity + 2; range_low = -40; range_high = 127;} //Decimal 24
  if (msg_id == "19") {msg_description = "Boiler flow water temperature (C): ";                   msg_flag = "f8.8";  msg_rw = "R"; f2l_parity = f2l_parity + 3; range_low = -40; range_high = 127;} //Decimal 25
  if (msg_id == "1a") {msg_description = "DHW temperature (C): ";                                 msg_flag = "f8.8";  msg_rw = "R"; f2l_parity = f2l_parity + 3; range_low = -40; range_high = 127;} //Decimal 26
  if (msg_id == "1b") {msg_description = "Outside temperature (C): ";                             msg_flag = "f8.8";  msg_rw = "R"; f2l_parity = f2l_parity + 4; range_low = -40; range_high = 127;} //Decimal 27
  if (msg_id == "1c") {msg_description = "Return water temperature (C): ";                        msg_flag = "f8.8";  msg_rw = "R"; f2l_parity = f2l_parity + 3; range_low = -40; range_high = 127;} //Decimal 28
  if (msg_id == "38") {msg_description = "DHW setpoint (C): ";                                    msg_flag = "f8.8";  msg_rw = "R"; f2l_parity = f2l_parity + 3; range_low =   0; range_high = 127;} //Decimal 56
  if (msg_id == "39") {msg_description = "Maximum CH water setpoint (C): ";                       msg_flag = "f8.8";  msg_rw = "R"; f2l_parity = f2l_parity + 4; range_low =   0; range_high = 127;} //Decimal 57

  //Check the message type and set corresponding reply message type
  if(strcmp(msg_rw, "R") == 0) {
    f2l_message = "READ-ACK      "; f2l_hex = "4"; f2l_parity = f2l_parity + 1;
  } else {
    f2l_message = "WRITE-ACK     "; f2l_hex = "5"; f2l_parity = f2l_parity + 2;
  }

  //DEBUG_DEBUG: Print the received message ID and description to the serial monitor
  if (strcmp(serial_debug, "1") == 0 ) {
    Serial.print("Decoded message ID:");
    Serial.print(msg_id);
    Serial.print(" with description:");
    Serial.print(msg_description);
    Serial.print(" parity count:");
    Serial.print(f2l_parity);
    Serial.println();
  }

  //DECODE message flag flag8/flag8, publish result on topic "ecv/thermostat" and send
  if (strcmp(msg_flag, "flag8") == 0) {
    msg_value_leader = msg_pos[4] + msg_pos[5];
    msg_value_leader = decode_flag_flag8(msg_value_leader);

    //Publish the received OpenTherm message with flag flag8/flag8 to MQTT
    String msg_full = "T-" +msg_heater + " " + l2f_message + " " + msg_description + msg_value_leader;

    //DEBUG_MONITOR: Print the OpenTherm incoming message to the serial monitor
    if (strcmp(serial_monitor, "1") == 0 ) {
      Serial.print(msg_full);
      if (strcmp(serial_debug, "1") == 0 ) {
        Serial.print(" parity count:");
        Serial.print(f2l_parity);
      }
      Serial.println();
      //  Print message type 00 details
      if (msg_id == "00") {
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - CH  Enabled is: %d", leader_status[7] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - DHW Enabled is: %d", leader_status[6] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - Cooling enable: %d", leader_status[5] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - OTC active: %d", leader_status[4] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - CH2 enable: %d", leader_status[3] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - Reserved: %d", leader_status[2] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - Reserved: %d", leader_status[1] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - Reserved: %d", leader_status[0] ); Serial.print (msg); Serial.println();
      } 
      // Print message type 03 details 
      if (msg_id == "03") {
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - DHW present: %d", leader_status[7] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - Control type: %d", leader_status[6] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - Cooling config: %d", leader_status[5] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - DHW Config: %d", leader_status[4] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - Leader low-off & pump control function: %d", leader_status[3] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - CH2 present: %d", leader_status[2] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - Reserved: %d", leader_status[1] ); Serial.print (msg); Serial.println();
        snprintf (msg, MSG_BUFFER_SIZE, "                                  - Reserved: %d", leader_status[0] ); Serial.print (msg); Serial.println();
      }
    }

    //Set ch_enabled flag for MQTT modulation reporting
    ch_enabled = leader_status[7];

    //Publish the received message to MQTT "ecv/thermostat/rawdata/rx"
    snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
    client.publish("ecv/thermostat/rawdata/rx", msg);
  }

 //DECODE message flag flag8/u8, publish result on topic "ecv/thermostat/rawdata/rx" and send
  if (strcmp(msg_flag, "u8") == 0) {
    //Change the message type to DATA-INVALID and correct the parity
    f2l_message = "DATA-INVALID  "; f2l_hex = "6"; f2l_parity = f2l_parity + 1;
    //Set the leader status HB and LB to 0 and update parity
    leader_status[7] = 0; leader_status[6] = 0; msg_value = "00000000"; f2l_parity = f2l_parity + 0; msg_value_leader = "00000000";

    //Publish the received OpenTherm message with flag flag8/u8 to MQTT
    String msg_full = "T-" +msg_heater + " " + l2f_message + " " + msg_description + " " + msg_value_leader;

    //DEBUG_MONITOR: Print the OpenTherm incoming message to the serial monitor
    if (strcmp(serial_monitor, "1") == 0 ) {
      Serial.print(msg_full);
      if (strcmp(serial_debug, "1") == 0 ) {
        Serial.print(" parity count:");
        Serial.print(f2l_parity);
      }
      Serial.println();
    }

    //Publish the received message to MQTT "ecv/thermostat/rawdata/rx"
    snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
    client.publish("ecv/thermostat/rawdata/rx", msg);
  }

  //DECODE message flag f8.8, publish result on topic "ecv/thermostat/rawdata/rx" and send 
  if (strcmp(msg_flag, "f8.8") == 0) {
    msg_value = msg_pos[4] + msg_pos[5] + msg_pos[6] + msg_pos[7];
    msg_value = decode_flag_f8(msg_value);
    
    //Publish the received OpenTherm message with flag f8.8 to MQTT
    String msg_full = "T-" + msg_heater + " " + l2f_message + " " + msg_description + " " + msg_value;

    //DEBUG_MONITOR: Print the Opentherm received message to the serial monitor
    if (strcmp(serial_monitor, "1") == 0 ) {
      Serial.print(msg_full);
      Serial.println();
    }

    //Publish the received message to MQTT "ecv/thermostat/rawdata/rx"
    snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
    client.publish("ecv/thermostat/rawdata/rx", msg);
  }

  //ENCODE message flag flag8/flag8
  for (int i=0; i<8; i++) {
    msg_value_follower = msg_value_follower + follower_status[i];
  }

  //ENCODE if message ID is 00 the follower status to msg_pos[7]
  if (msg_id == "00") {
    //If no fault condition is present set the CH mode and flame status
    if (follower_status[7] == 0 ) {
      if (follower_status[6] == 0 && follower_status[4] == 0) {msg_pos[7] = "0"; f2l_parity = f2l_parity + 0;}
      if (follower_status[6] == 1 && follower_status[4] == 0) {msg_pos[7] = "2"; f2l_parity = f2l_parity + 1;}
      if (follower_status[6] == 0 && follower_status[4] == 1) {msg_pos[7] = "8"; f2l_parity = f2l_parity + 1;}
      if (follower_status[6] == 1 && follower_status[4] == 1) {msg_pos[7] = "A"; f2l_parity = f2l_parity + 2;}
    } else{
      //If fault condition is present set fault and switch off CH mode and flame status
      msg_pos[7] = "1";
    }
  }

  //ENCODE if message ID is 03 the follower status to msg_pos[4] and msg[5]
  if (msg_id == "03") {
    //Correct the parity by subtracting the old leader parity
    f2l_parity = f2l_parity - parity_correction;

    //Set HARD defaults
    leader_status[7] = 0; // DHW present
    leader_status[6] = 1; // Modulating on/off
    leader_status[5] = 0; // Cooling config
    leader_status[4] = 0; // Instantaneous or not-specified storage tank
    leader_status[3] = 0; // Leader low & pump control
    leader_status[2] = 0; // CH2 present
    leader_status[1] = 0; // Reserved
    leader_status[0] = 0; // Reserved

    // If DHW is present update bit 0 and set msg_pos[5] to the correct value
    if (strcmp(DHW_mode, "1") == 0 ) {
      leader_status[7] = 1; // DHW present
      msg_pos[5] = "3"; f2l_parity = f2l_parity + 2;
      msg_value_leader = "00000011";
    } else {
      msg_pos[5] = "2"; f2l_parity = f2l_parity + 1;
      msg_value_leader = "00000010";
    }

    // msg_pos[4] permanently set to 0 as the software does not support Leader low & pump and CH2 present
    msg_pos[4] = "0"; f2l_parity = f2l_parity + 0;
  }

  //CHECK if there are updated default or MQTT received values to report back to the ecv/thermostat/*
  if (msg_id == "01" || msg_id == "10" || msg_id == "11" || msg_id == "12" || msg_id == "13" || msg_id == "18" || msg_id == "19" || msg_id == "0e" || msg_id == "1a" || msg_id == "1b"|| msg_id == "1c" || msg_id == "38" || msg_id == "39") {

    //Check the ID 01 Control CH setpoint
    if (msg_id == "01") {
      old_value = msg_value.toDouble();
      control_ch_setpoint = msg_value.toDouble();

      //Calucluate modulation
      temp_difference = control_ch_setpoint - heater_temp;
      //Check if 60 seconds have passed since last update
      unsigned long now = millis();
      if (now - last_modulation_update > 60000 ) {
        //Check if difference is > upper limit
        if (temp_difference > upper_limit) { set_modulation = 100.00; }
        //Check if diffeence is < lower limit
        if (temp_difference < lower_limit ) { set_modulation =  0.00; }
        //Check if differnce is between lower and upper limit
        if (temp_difference >= lower_limit && temp_difference <= upper_limit ) {
          set_modulation = temp_difference / (upper_limit - lower_limit) * 100;
        }
        //Publish the received calculation to MQTT
        String msg_full = "Request: " + String(control_ch_setpoint,2) + " Heater flow: " + String(heater_temp,2) + " Difference:" + String(temp_difference,2) + " Set Modulation: " + String(set_modulation,2) ;
        snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
        client.publish("ecv/thermostat/rawdata/modulation", msg);
        //Reset timer
        last_modulation_update = millis();
      }
    }

    //Check the ID 16 Room setpoint
    if (msg_id == "10") {
      old_value = msg_value.toDouble();
    }  

    //Check the ID 17 Relative modulation level (Percent)
    if (msg_id == "11") {
      //Compare the received value with the default(MQTT update value)
      if (msg_value.toDouble() == set_modulation ) {
        old_value = msg_value.toDouble();
      } else {
        old_value = msg_value.toDouble();
        msg_value = String(set_modulation,2);
      }
    }   

   //Check the ID 18 Water pressure
    if (msg_id == "12"){
      //Compare the received value with the default(MQTT update value)
      if (msg_value.toDouble() == water_pressure_ch ) {
        old_value = msg_value.toDouble();
      } else {
        old_value = msg_value.toDouble();
        msg_value = String(water_pressure_ch,2);
      }
    }

   //Check the ID 19 Water flow DHW
    if (msg_id == "13"){
      //Compare the received value with the default(MQTT update value)
      if (msg_value.toDouble() == water_flow_dhw ) {
        old_value = msg_value.toDouble();
      } else {
        old_value = msg_value.toDouble();
        msg_value = String(water_flow_dhw,2);
      }
    }

 //Check the ID 25 Boiler flow temperature
    if (msg_id == "19"){
      //Check if a live temperature is available before using the MQTT provided temperature
      if (heater_temp == 0) {
        //Compare the received value with the default(MQTT update value)
        if (msg_value.toDouble() == heater_flow_temperature ) {
          old_value = msg_value.toDouble();
        } else {
          old_value = msg_value.toDouble();
          msg_value = heater_flow_temperature;
        }
      } else {
        old_value = msg_value.toDouble();
        msg_value = String(heater_temp);
      }
      //Publish the boiler temperature to MQTT [ecv/thermostat/boilertemp]
      msg_full = String(heater_temp);
      snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
      client.publish("ecv/thermostat/boilertemp", msg);
    }

       //Check the ID 14 Max relative modulation
    if (msg_id == "0E"){
      //Compare the received value with the default(MQTT update value)
      if (msg_value.toDouble() == max_rel_modulation) {
        old_value = msg_value.toDouble();
      } else {
        old_value = msg_value.toDouble();
        msg_value = max_rel_modulation;
      }
    }

    //Check the ID 24 Room temperature
    if (msg_id == "18") {
      old_value = msg_value.toDouble();
    }  

    //Check the ID 26 DHW Temperature
    if (msg_id == "1A"){
      //Compare the received value with the default(MQTT update value)
      if (msg_value.toDouble() == dhw_temperature ) {
        old_value = msg_value.toDouble();
      } else {
        old_value = msg_value.toDouble();
        msg_value = dhw_temperature;
      }
    }
 
   //Check the ID 27 Outside temperature
    if (msg_id == "1B"){
      //Compare the received value with the default(MQTT update value)
      if (msg_value.toDouble() == outside_temperature ) {
        old_value = msg_value.toDouble();
      } else {
        old_value = msg_value.toDouble();
        msg_value = outside_temperature;
      }
    }

 //Check the ID 28 Return water temperature
    if (msg_id == "1C"){
      //Check if a live temperature is available before using the MQTT provided temperature
      if (return_temp == 0) {
        //Compare the received value with the default(MQTT update value)
        if (msg_value.toDouble() == return_water_temperature ) {
          old_value = msg_value.toDouble();
        } else {
          old_value = msg_value.toDouble();
          msg_value = return_water_temperature;
        }
      } else {
          old_value = msg_value.toDouble();
          msg_value = String(return_temp);
      }
      //Publish the boiler returntemperature to MQTT [ecv/thermostat/returntemp]
        msg_full = String(return_temp);
        snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
        client.publish("ecv/thermostat/returntemp", msg);
    }
 
 //Check the ID 56 DHW setpoint
    if (msg_id == "38"){
      //Compare the received value with the default(MQTT update value)
      if (msg_value.toDouble() == dhw_setpoint) {
        old_value = msg_value.toDouble();
      } else {
        old_value = msg_value.toDouble();
        msg_value = String(dhw_setpoint,2);
      }
    }

    //Check the ID 57 Max CH Water setpoint
    if (msg_id == "39"){
      //Compare the received value with the default(MQTT update value)
      if (msg_value.toDouble() == max_ch_water_setpoint) {
        old_value = msg_value.toDouble();
      } else {
        old_value = msg_value.toDouble();
        msg_value = max_ch_water_setpoint;
      }
    }

    //Convert the measurement value to Hex
    msg_value_hex = encode_flag_f8(msg_value);

    //Load the Hex result into the OpenTherm return message
    msg_pos[4] = msg_value_hex.substring( 1, 0 );
    msg_pos[5] = msg_value_hex.substring( 2, 1 );
    msg_pos[6] = msg_value_hex.substring( 3, 2 );
    msg_pos[7] = msg_value_hex.substring( 4, 3 );

    //DEBUG_MONITOR: Result of value override
    if (strcmp(serial_update, "1") == 0 ) {
      if (old_value == msg_value.toDouble()) {
        Serial.print("Value of message type: ");
        Serial.print(msg_id);
        Serial.print(" did not change. The parity is: ");
        Serial.print(f2l_parity);
        Serial.println();
      } else {
        Serial.print("Value of message type: ");
        Serial.print(msg_id);
        Serial.print(" was changed from: ");
        Serial.print(old_value);
        Serial.print(" to: ");
        Serial.print(msg_value);
        Serial.print(" and the parity is: ");
        Serial.print(f2l_parity);
        Serial.println();
      }
    }
  }

  //CHECK if received measurement is within protocol range and update message type accordingly of other then message ID 0 & 5
  if (msg_id != "00" || msg_id != "03" || msg_id != "05" ) {
    //Convert string to double
    double range_test = msg_value.toDouble();
    //Check against the range
    if (range_test >= range_low && range_test <= range_high) {
      pass = "Valid";
    } else {
      //If invalid change the message type and return value
      pass = "Invalid"; f2l_message = "DATA-INVALID  "; f2l_hex = "6"; f2l_parity = f2l_parity + 1;
      //Set the follower byte 3 & 4 and update parity
      msg_value = "0"; msg_pos[7] = "0"; msg_pos[6] = "0"; msg_pos[5] = "0"; msg_pos[4] = "0";
    }

    //DEBUG_RANGE: Print the resutl of checking if the measurement is in the pre-defined range
    if (strcmp(serial_range, "1") == 0 ) {
      Serial.print("Current measurment: ");
      Serial.print(range_test);
      Serial.print(" is being checked for range: ");
      Serial.print(range_low);
      Serial.print(" to: ");
      Serial.print(range_high);
      Serial.print(" and the result is: ");
      Serial.print(pass);
      Serial.println();
    }
  }  

  //DEBUG_CONVERT: Print the Opentherm encoded value to the serial monitor
  if (strcmp(serial_convert, "1") == 0 ) {
    Serial.print("Encode measurement value: ");
    Serial.print(msg_value);
    Serial.print(" to Hex: ");
    Serial.print(msg_value_hex);
    Serial.print(" with total parity count: ");
    Serial.print(f2l_parity);
  }

  //Publish the response to the OpenTherm message to MQTT
  //Build the message type considering the parity 
  if ((f2l_parity & 1) == 0) {
    msg_pos[0] = f2l_hex;
    if (strcmp(serial_convert, "1") == 0 ) {
      Serial.print(" Parity is EVEN.");
      Serial.println();
    }
  } else {
    if (strcmp(f2l_hex, "4") == 0 ) {msg_pos[0] = "c";}
    if (strcmp(f2l_hex, "5") == 0 ) {msg_pos[0] = "d";}
    if (strcmp(f2l_hex, "6") == 0 ) {msg_pos[0] = "e";}
    if (strcmp(f2l_hex, "7") == 0 ) {msg_pos[0] = "f";}
    if (strcmp(serial_convert, "1") == 0 ) {
      Serial.print(" Parity is UN-EVEN.");
      Serial.println();
    }
  }

  //Build outgoing message type into a String msg_thermostat
  for (int i = 0; i < 8; i++) {msg_thermostat = msg_thermostat + msg_pos[i]; }

  //Build the string for message type 00 and 03 else build all other message type strings
  if (msg_id == "00" || msg_id == "03") {
    msg_full = msg_thermostat + " " + f2l_message + " " + msg_description + msg_value_leader + " " + msg_value_follower;
  } else {
    msg_full = msg_thermostat + " " + f2l_message + " " + msg_description + " " + msg_value;
  }

  //DEBUG_MONITOR: Print the OpenTherm response result to the serial monitor
  if (strcmp(serial_monitor, "1") == 0 ) {
    Serial.print(msg_full);
    if (strcmp(serial_debug, "1") == 0 ) {
      Serial.print(" parity count:");
      Serial.print(f2l_parity);
    }
    Serial.println();
    if (msg_id == "00") {
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - Fault indication is: %d", follower_status[7] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - CH Mode is: %d", follower_status[6] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - DHW Mode: %d", follower_status[5] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - Flame status is: %d", follower_status[4] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - Cooling status: %d", follower_status[3] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - CH2 mode: %d", follower_status[2] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - Diagnostics indication: %d", follower_status[1] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - Reserved: %d", follower_status[0] ); Serial.print (msg); Serial.println();
    } 
    if (msg_id == "03") {
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - DHW present: %d", leader_status[7] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - Control type: %d", leader_status[6] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - Cooling config: %d", leader_status[5] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - DHW Config: %d", leader_status[4] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - Leader low-off & pump control function: %d", leader_status[3] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - CH2 present: %d", leader_status[2] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - Reserved: %d", leader_status[1] ); Serial.print (msg); Serial.println();
      snprintf (msg, MSG_BUFFER_SIZE, "                                  - Reserved: %d", leader_status[0] ); Serial.print (msg); Serial.println();
    }
  }

  //Delay pre-set ms to meet protocol requirements
  unsigned long now = millis();
  while (now - msg_rx_ts < timing) {
    now = millis();
  }

  //Publish the received message to MQTT [ecv/thermostat/rawdata/tx]
  msg_full = msg_full + " Replied after: " + (now - msg_rx_ts) + "ms.";
  snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
  client.publish("ecv/thermostat/rawdata/tx", msg);

  //Publish CH requested to MQTT [ecv/thermostat/ch_requested]
  if ( ch_enabled != ch_enabled_history ) {
    ch_enabled_history = ch_enabled;
    if (ch_enabled == 1 ) {
      msg_full = "1";
      snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
      client.publish("ecv/thermostat/ch_requested", msg);
    } else {
      msg_full = "0";
      snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
      client.publish("ecv/thermostat/ch_requested", msg);
    }
  } else {
    //Send MQTT Message every 60 sec if no change
    unsigned long now = millis();
    if (now - last_ch_update > 60000) {
      if (ch_enabled == 1 ) {
        msg_full = "1";
        snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
        client.publish("ecv/thermostat/ch_requested", msg);
      } else {
        msg_full = "0";
        snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
        client.publish("ecv/thermostat/ch_requested", msg);
      }
      last_ch_update = millis();
    }
  }

  //Publish the CH Setpoint to MQTT [ecv/thermostat/ch_setpoint]
  if ( msg_id == "01" ) {
    msg_full = msg_value;
    snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
    client.publish("ecv/thermostat/ch_setpoint", msg);
  }

  //Publish the modulation level to MQTT [ecv/thermostat/modulation]
  if ( msg_id == "11" ) {
    msg_full = msg_value;
    snprintf (msg, MSG_BUFFER_SIZE, msg_full.c_str());
    client.publish("ecv/thermostat/modulation", msg);
  }


  //Initializing base value to 1, i.e 16^0, set the variable length fixed to 4 and the dec_val to 0
  int base = 1;  int len = 8; unsigned long dec_val = 0;

  //Convert response from Hex to Dec
  msg_value = msg_thermostat.substring(2,10);
  //Extracting characters as digits from last character 
  for (int i=len-1; i>=0; i--) {
    // if character lies in '0'-'9', converting it to integral 0-9 by subtracting 48 from ASCII value.
    if (msg_value[i]>='0' && msg_value[i]<='9') {
      dec_val += (msg_value[i] - 48)*base;

      // incrementing base by power
      base = base * 16;
    } 

    // if character lies in 'a'-'f' , converting it to integral 10 - 15 by subtracting 87 from ASCII value
    else if (msg_value[i]>='a' && msg_value[i]<='f') {
        dec_val += (msg_value[i] - 87)*base;

        // incrementing base by power
        base = base*16;
    }

    // if character lies in 'A'-'F' , converting it to integral 10 - 15 by subtracting 55 from ASCII value
    else if (msg_value[i]>='A' && msg_value[i]<='F') {
        dec_val += (msg_value[i] - 55)*base;

        // incrementing base by power
        base = base*16;
    }
  }

  //send response
  ot.sendResponse(dec_val);
}



// -----------------------------------------------------------SETUP--------------------------------------------------------------
//SETUP This code will run once and setup WiFi, set t(ime)s(tamp), call OT interrupt, setup MQTT server, client topic, payload and length.
void setup() {
  //Set serial speed for serial monitor
  Serial.begin(9600);
  Serial.println();
  Serial.println("Start program.");

  //Init OpenTerm interupt handler
  ot.begin(handleInterrupt, processRequest);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  //Setup WiFi by calling setup_wifi function
  setup_wifi();

  //Init MQTT Client server and port with static variables
  client.setServer(mqtt_server, mqtt_port);
  
  //Init MQTT Client topic, payload and length
  client.setCallback(callback);
  
  //Start onewire library
  sensors.begin();

  //OTA 
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am the E-CV running on a ESP8266 .");
  });

  ElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  // locate onewire devices on the bus and print to terminal
  if (strcmp(serial_onewire, "1") == 0 ) {
    //Locating onewire addresses
    Serial.println("Locating devices...");
    Serial.print("Found ");
    deviceCount = sensors.getDeviceCount();
    Serial.print(deviceCount, DEC);
    Serial.println(" devices.");
    Serial.println("");
    
    //Printing onewire addresses
    Serial.println("Printing addresses...");
    for (int i = 0;  i < deviceCount;  i++) {
      Serial.print("Sensor ");
      Serial.print(i+1);
      Serial.print(" : ");
      sensors.getAddress(Thermometer, i);
      printAddress(Thermometer);
    }
  }

  //Init follower status
  if (strcmp(fault_indication, "0" ) == 0 ) {follower_status[7] = 0;} else {follower_status[7] = 1;};
  if (strcmp(CH_mode, "0" ) == 0 )          {follower_status[6] = 0;} else {follower_status[6] = 1;};
  if (strcmp(flame_status, "0") == 0 )      {follower_status[4] = 0;} else {follower_status[4] = 1;};
}


//------------------------------------------------------------LOOP----------------------------------------------------------------
// LOOP Runs the main code.
void loop() {
  //Check if MQTT client is connected and reconnect if necessary
  if (!client.connected()) {
    //Switch OFF the LED
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    //Reconnect
    reconnect();
  } 

  //OpenTerm process
  ot.process();

  client.loop();

  //OTA process
  ElegantOTA.loop();
   
  //Read temperature every 5 seconds
  unsigned long now = millis();
  if (now - last_temp > 5000) {
    read_temperature();

    //Publish the boiler returntemperature to MQTT [ecv/thermostat/returntemp]
    String msg_with_return_temp = String(return_temp);
    snprintf (msg, MSG_BUFFER_SIZE, msg_with_return_temp.c_str());
    client.publish("ecv/thermostat/returntemp", msg);

    //Reset timer
    last_temp = millis();

  }

  void loop();
}
