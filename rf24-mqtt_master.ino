/* 
 *  ESP8266-based rf24 gateway to MQTT
 *  
 *  board: NodeMCU 1.0
 */
 
#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include "PubSubClient.h"

#include <WiFiUdp.h>
#include <NTPClient.h>
#include "time.h"

#include <ArduinoJson.h>


/*
 * configuration includes passwords/etc
 * include separately to not leak private information
 */
#include "config.h"


/*
 * globals
 */
// LED blinking
int nBlinks = 0;
bool led_state = 1;  // active low

struct tm timeinfo;

String my_mac;
String msg_topic;



RF24 radio(RF24_CE_PIN, RF24_CS_PIN);
RF24Network network(radio);

WiFiClient wifi;
ESP8266WiFiMulti wifiMulti;  // use multiple wifi options

PubSubClient mqtt(wifi);

WiFiUDP udp;
NTPClient ntpClient(udp, NTP_SERVER, UTC_OFFSET, NTP_UPDATE_INTERVAL);




/*
 * Given a byte array of length (n), return the ASCII hex representation
 * and properly zero pad values less than 0x10.
 * (String(0x08, HEX) will yield '8' instead of the expected '08' string)
 */
String hexToStr(uint8_t* arr, int n)
{
  String result;
  for (int i = 0; i < n; ++i) {
    if (arr[i] < 0x10) {result += '0';}
    result += String(arr[i], HEX);
  }
  return result;
}



/*
 * Return a string of the current time in RFC3339 format.
 * Will return a placeholder if there is no network connection to an
 * NTP server.
 */
String getIsoTime()
{
  char timeStr[20+1] = {0};  // NOTE: change if strftime format changes

  time_t time_now = ntpClient.getEpochTime();
  localtime_r(&time_now, &timeinfo);

  if (timeinfo.tm_year <= (2016 - 1900)) {
    // Failed to obtain time
    return String("YYYY-MM-DDTHH:MM:SSZ");
  }

  strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(timeStr);
}



/*
 * Check WiFi connection, attempt to reconnect.
 * This blocks until a connection is (re)established.
 */
void check_wifi()
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("WiFi connection lost, reconnecting");
    while (wifiMulti.run() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  }
}




/*
 * Check the MQTT connection state and attempt to reconnect.
 * If we do reconnect, then subscribe to MQTT_CONTROL_TOPIC and
 * make an announcement to MQTT_ANNOUNCE_TOPIC with the WiFi SSID and
 * local IP address.
 */
void check_mqtt()
{
  // This tool helps compute the space to reserve
  // https://arduinojson.org/v6/assistant/
  //    --> don't forget to estimate the payload string lengths
  const size_t json_capacity = JSON_OBJECT_SIZE(3) + 128;

  if (mqtt.connected()) { return; }

  Serial.print("MQTT reconnect...");
  // Attempt to connect
  int connect_status = mqtt.connect(
                    my_mac.c_str(),
                    MQTT_USER,
                    MQTT_PASS,
                    (MQTT_PREFIX_TOPIC + my_mac + MQTT_ANNOUNCE_TOPIC).c_str(),
                    2,  // willQoS
                    1,  // willRetain
                    "{\"state\":\"disconnected\"}");

  if (connect_status) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    // JSON formatted payload
    StaticJsonDocument<json_capacity> json;
    json["state"] = "connected";
    json["ssid"] = WiFi.SSID();
    json["ip"] = WiFi.localIP().toString();

    String msg;
    serializeJson(json, msg);
    Serial.println(msg);
    mqtt.publish((MQTT_PREFIX_TOPIC + my_mac + MQTT_ANNOUNCE_TOPIC).c_str(),
                 msg.c_str(),
                 true // retain
                );

    // ... and resubscribe to the control topic
    mqtt.subscribe((MQTT_PREFIX_TOPIC + my_mac + MQTT_CONTROL_TOPIC).c_str());
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqtt.state());
  }
}



/*
 * Called whenever a payload is received from a subscribed MQTT topic
 */
void mqtt_receive_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT-receive [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    ledOn();
  } else {
    ledOff();
  }

  // this will effectively be a half-blink, forcing the LED to the
  // requested state
  nBlinks += 1;
}

/*
 * Abstract LED state
 * this changes depending if LED is active low or active high
 */
void ledOn(void) {
  led_state = 0;  //active low
}

void ledOff(void) {
  led_state = 1;  //active low
}


/*
 * Handle blinking without using delay()
 */
void do_blink(void) {
  static bool in_blink = false;
  static unsigned long last_blink = 0;

  unsigned long now = millis();

  // time to change LED state?
  if ((nBlinks > 0) && (now - last_blink >= BLINK_MS)) {
    last_blink = now;

    if (in_blink) {
      // finish a blink
      digitalWrite(LED_PIN, led_state);
      nBlinks--;
      in_blink = false;
    } else {
      // start a blink
      digitalWrite(LED_PIN, !led_state);
      in_blink = true;
    }
  }
}





void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, led_state);

  /*
   * setup WiFi
   */
  //WiFi.mode(WIFI_STA);
  for (int i=0; i<NUM_WLANS; i++) {
    wifiMulti.addAP(WLAN_SSID[i], WLAN_PASS[i]);
  }

  uint8_t mac[6];
  WiFi.macAddress(mac);
  my_mac = hexToStr(mac, 6);
  msg_topic = MQTT_PREFIX_TOPIC + my_mac + MQTT_MSG_TOPIC;

  Serial.print("MAC: ");
  Serial.println(my_mac);

  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  /*
   * setup MQTT
   */
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqtt_receive_callback);
  Serial.println("MQTT setup.");


  /*
   * setup NTP time sync
   */
  ntpClient.begin();
  Serial.println("NTP setup.");

  /*
   * setup RF24Network
   */
   SPI.begin();
   radio.begin();
   network.begin(RF24_CHANNEL, 00);  // master node is address zero
   Serial.println("RF24Network setup.");
   while (!radio.isChipConnected()) {
    Serial.println("rf24 not connected");
    delay(1000);
    radio.begin();
    delay(100);
    network.begin(RF24_CHANNEL, 00);
    delay(100);
   }
}



void loop() {    
  check_wifi();
  check_mqtt();

  mqtt.loop();
  
  ntpClient.update();
  
  // Keep the network updated
  network.update();
  
  // LED blinking without using delays
  do_blink();

  // Check for incoming data from the sensors
  if (network.available()) {
    RF24NetworkHeader header;
    uint8_t buf[MAX_PAYLOAD_SIZE];  // as configured in RF24Network_config.h
    uint16_t payload_len;

    Serial.print("message ");
    
    payload_len = network.read(header, &buf, sizeof(buf));

    String payload = hexToStr(buf, payload_len);

    DynamicJsonDocument json(JSON_OBJECT_SIZE(8) + 512);

    json["channel"] = RF24_CHANNEL;
    json["time"] = getIsoTime();
    json["from_addr"] = String(header.from_node, OCT);
    json["to_addr"] = String(header.to_node, OCT);
    json["type"] = header.type;
    json["payload"] = payload;
    
    // serializeJson needs a stream
    // but we ultimately need a buffer of bytes
    // is this way too slow?  hasn't been an issue yet
    String msg;
    msg.reserve(measureJson(json));
    serializeJson(json, msg);

    // Publish the string via MQTT
    mqtt.publish(msg_topic.c_str(), msg.c_str(), msg.length());

    // Blink for every received message
    nBlinks += 1;

    Serial.println(msg);
  }
}
