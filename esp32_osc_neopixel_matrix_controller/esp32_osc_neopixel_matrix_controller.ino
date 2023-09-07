/**
* esp32_osc_neopixel_matrix_controller.ino
*
* Changes a Neopixel matrix based on incoming
* OSC values.
*
* NOTE: Libraries Used
*  - OSC by Adrian Freed v1.3.5
*  - Adafruit NeoMatix v1.3.0
*
* Madeline Gannon | ATONATON
* April 2023
* https://atonaton.com
*/

#include "WiFi.h"
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 32
#define LED_COUNT 32

// #define HOME_NETWORK

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

WiFiUDP udp;
int port = 55555;

// const char* ssid = "WHH";
// const char* password = "benderisgreat";
// Set your Static IP address
// IPAddress static_IP(192, 168, 1, 10);
// IPAddress gateway(192, 168, 1, 1);
// IPAddress subnet(255, 255, 255, 0);

// home network 192.168.0.23
#ifdef HOME_NETWORK
const char* ssid = "NETGEAR94";
const char* password = "wideprairie401";
IPAddress static_IP(192, 168, 0, 10);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
#else
const char* ssid = "WHH";
const char* password = "benderisgreat";
// Set your Static IP address
IPAddress static_IP(192, 168, 1, 10);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
#endif

void setup() {
  Serial.begin(115200);

  // Configure a Static Address
  if (!WiFi.config(static_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  // Connect to the wifi router's network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Confirm the Static Address
  Serial.print("ESP32_0 IP Address: ");
  Serial.println(WiFi.localIP());

  // Begin listening on the UDP port
  udp.begin(port);

  // Intialize Neopixel Matrix
  pixels.begin();
  pixels.show();
  pixels.setBrightness(50);
}

void loop() {
  check_for_OSC_message();
  delay(5);
}


void on_message_received(OSCMessage& msg) {
  // Get the LED index from the message address
  // Get the message address
  char msg_addr[255];
  msg.getAddress(msg_addr);
  Serial.println(msg_addr);
  String addr = "";
  addr += msg_addr;

  // if (addr == "/brightness") {
  //   pixels.setBrightness(msg.getInt(0));  // value between 0-255
  // } else
  if (msg.isString(0)) {
    pixels.clear();
    pixels.show();
  }
  // // The address is the index of the LED and values are the RGB color
  else {
    // int index = addr.substring(addr.lastIndexOf("/") + 1, addr.length()).toInt();
    int i = msg.getInt(0);
    int r = msg.getInt(1);
    int g = msg.getInt(2);
    int b = msg.getInt(3);
    uint32_t color = pixels.Color(r, g, b);
    pixels.setPixelColor(i, color);
    pixels.show();
  }
}

void check_for_OSC_message() {
  OSCMessage msg;
  int size = udp.parsePacket();
  if (size > 0) {
    while (size--) {
      msg.fill(udp.read());
    }
    if (!msg.hasError()) {
      // Get the message address
      char msg_addr[255];
      msg.getAddress(msg_addr);
      Serial.print("MSG_ADDR: ");
      Serial.println(msg_addr);

      // if (!msg.isBundle()) {
      msg.dispatch(msg_addr, on_message_received);
      // } else {

      // Example of how to parse the message
      for (int i = 0; i < msg.size(); i++) {
        Serial.print("\tVALUE\t");
        Serial.print(msg.getType(i));
        Serial.print(": ");
        if (msg.isInt(i)) {
          Serial.println(msg.getInt(i));
        } else if (msg.isFloat(i)) {
          Serial.println(msg.getFloat(i));
        } else if (msg.isDouble(i)) {
          Serial.println(msg.getDouble(i));
        } else if (msg.isString(i)) {
          char my_string[255];
          msg.getString(i, my_string);
          Serial.println(my_string);
        } else if (msg.isBoolean(i)) {
          Serial.println(msg.getBoolean(i));
        } else {
          Serial.println("Unknown data type.");
        }
        // }
      }
    }
  }
}