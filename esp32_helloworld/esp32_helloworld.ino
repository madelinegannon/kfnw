/**
* esp32_helloworld.ino
*
* This sketch connects to a wifi router and then  
* listens for OSCmessages on port 55555.
*
* Madeline Gannon | ATONATON
* April 2023
* https://atonaton.com
*/

#include "WiFi.h"
#include <WiFiUdp.h>
#include <OSCMessage.h>

WiFiUDP udp;
int port = 55555;

const char* ssid = "WHH";
const char* password = "benderisgreat";
// Set your Static IP address
IPAddress static_IP(192, 168, 1, 10);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

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
}

void loop() {
  check_for_OSC_message();
  delay(5);
}

// Example of an OSC callback function
void on_message_received(OSCMessage& msg) {
  if (msg.isFloat(0) && msg.isFloat(1)) {
    float x = msg.getFloat(0);
    float y = msg.getFloat(1);
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(", y: ");
    Serial.println(y);
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

      // Route to a specific callback function
      if (msg.size() == 2) {
        msg.dispatch(msg_addr, on_message_received);
      }
      // Parse the message
      else {
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
        }
      }
    }
  }
}