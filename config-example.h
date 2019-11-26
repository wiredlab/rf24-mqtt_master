/*
 * configuration
 *
 * copy to config.h and modify parameters
 *  !!! do not check config.h in to git !!!
 */

// on-board LED blinks once per packet
// quiescent state (on or off) is set by "/control" topic reception
#define LED_PIN 2
#define BLINK_MS 50


// RF24 config
const int RF24_CE_PIN = 0;
const int RF24_CS_PIN = 15;
const int RF24_CHANNEL = 97;  // default for RF24Mesh


// WiFi config
const char *WLAN_SSID[] = {"valpo-net"};
const char *WLAN_PASS[] = {"brownandgold"};
const int NUM_WLANS = 1;


// SNTP time config
const int UTC_OFFSET = 0;
const char *NTP_SERVER = "pool.ntp.org";
const int NTP_UPDATE_INTERVAL = 600000;  // ms between NTP queries


// MQTT settings
const char *MQTT_PREFIX_TOPIC = "esp8266-rf24-mqtt/";
const char *MQTT_ANNOUNCE_TOPIC = "/status";
const char *MQTT_CONTROL_TOPIC = "/control";
const char *MQTT_MSG_TOPIC = "/msg";
const char *MQTT_SERVER = "mqtt.example.net";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "username";
const char *MQTT_PASS = "password";
