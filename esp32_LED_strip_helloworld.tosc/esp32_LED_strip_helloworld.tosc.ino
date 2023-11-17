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

#define LED_PIN A3
#define LED_COUNT 30

// #define HOME_NETWORK

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Colors
uint32_t color_white;
float count = 0;

int r = 0;
int g = 0;
int b = 0;
float duration = 0;
bool enable = 0;
bool enable_pulse = 0;
bool BFB = 0;
float curr_pulse_val = 0;

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
  color_white = pixels.Color(200, 200, 200);
  pixels.fill(color_white);
  pixels.setBrightness(200);
  pixels.show();
}

void loop() {
  check_for_OSC_message();
  if (enable){
    if (enable_pulse) {
      curr_pulse_val = curr_pulse_val + duration;

      float val = sin(curr_pulse_val);
      val = (val + 1.0) * 128;

      uint32_t color = pixels.Color(map(r, 0, 255, 0, int(val)), map(g, 0, 255, 0, int(val)), map(b, 0, 255, 0, int(val)));
      pixels.fill(color);
      pixels.show();

    }
    else{
      uint32_t color = pixels.Color(r, g,b);
      pixels.fill(color);
      pixels.show();
    }
}
  else{
    uint32_t color = pixels.Color(0, 0,0);
    pixels.fill(color);
    pixels.show();
  }
}


void on_message_received(OSCMessage& msg) {
  // Get the LED index from the message address
  // Get the message address
  char msg_addr[255];
  msg.getAddress(msg_addr);
  Serial.println(msg_addr);
  String addr = "";
  addr += msg_addr;

  if (addr == "/color_r") {
    r = msg.getInt(0);
    Serial.println(r);
  } else if (addr == "/color_g") {
    g = msg.getInt(0);
    Serial.println(g);
  } else if (addr == "/color_b") {
    b = msg.getInt(0);
    Serial.println(b);
  }

  else if (addr == "/duration") {
    duration = msg.getFloat(0);
    Serial.println(duration);
  }

  else if (addr == "/enable") {
    enable = msg.getBoolean(0);
    Serial.println(enable);
  }
  else if (addr == "/enable_pulse") {
    enable_pulse = msg.getBoolean(0);
    Serial.println(enable_pulse);

  } 
  else if (addr == "/BFB") {
    BFB = msg.getInt(0);
    Serial.println(BFB);
  }

  else {
    uint32_t color = pixels.Color(0, 0, 0);
    pixels.fill(color);
    pixels.show();
  }
  if (addr == "/BFB") {
    Serial.println("Don't Press the BIG FUCKING BUTTON");
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

    }
  }
}
