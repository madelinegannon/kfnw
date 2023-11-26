/**
* nws_led_disc_1.ino
*
* Changes a Neopixel strip based on incoming
* OSC values over WiFi.
*
* IP Addresses:
*     IPAddress static_IP(192, 168, 1, 10); // Disc 0
*     IPAddress static_IP(192, 168, 1, 11); // Disc 1
*     IPAddress static_IP(192, 168, 1, 12); // Disc 2
*     IPAddress static_IP(192, 168, 1, 13); // Disc 3
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

// global color value
uint32_t color = pixels.Color(255, 0, 80);
int r = 255;
int g = 0;
int b = 80;
float brightness = 1.0;

bool enable = true;
bool enable_pulse = false;
float curr_pulse_val = 0;
float duration = 0.001;


WiFiUDP udp;
int port = 55555;

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
// IPAddress static_IP(192, 168, 1, 10);  // Disc 0
IPAddress static_IP(192, 168, 1, 11); // Disc 1
// IPAddress static_IP(192, 168, 1, 12); // Disc 2
// IPAddress static_IP(192, 168, 1, 13); // Disc 3
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
  pixels.Color(200, 200, 200);
  pixels.fill(color);
  pixels.setBrightness(200);
  pixels.show();
}

void loop() {
  check_for_OSC_message();
  
  
  // if (enable) {
    if (enable && enable_pulse) {
      curr_pulse_val = curr_pulse_val + duration;

      float val = sin(curr_pulse_val);
      val = (val + 1.0) * 128;

      color = pixels.Color(map(r, 0, 255, 0, int(val)), map(g, 0, 255, 0, int(val)), map(b, 0, 255, 0, int(val)));
      pixels.fill(color);
      pixels.show();

    } 
    // else {
  //     uint32_t color = pixels.Color(r, g, b);
  //     pixels.fill(color);
  //     pixels.show();
  //   }
  // } else if (color != pixels.Color(0, 0, 0)) {
  //     Serial.println("TURN OFF");
  //     color = pixels.Color(0, 0, 0);
  //     pixels.fill(color);
  //     pixels.show();
  //   }
  //   uint32_t color = pixels.Color(0, 0, 0);
  //   // pixels.fill(color);
  //   // pixels.show();
  // }
}


void on_message_received(OSCMessage& msg) {
  // Get the LED index from the message address
  // Get the message address
  char msg_addr[255];
  msg.getAddress(msg_addr);
  //Serial.println(msg_addr);
  String addr = "";
  addr += msg_addr;

  bool update_color = false;

  if (addr == "/enable") {

    // sending a string "0" or "1" because there's a weird 
    // bug with sending bools or ints (only 1 is received)
    char val [8];
    msg.getString(0, val);
    String data = (char*)val;
    // Serial.println(data);
    if (data == "0"){
      enable = false;
    }
    else{
      enable = true;
    }
    // Serial.print("Enable set to ");
    // Serial.println(enable);
    if (!enable){
      color = pixels.Color(0, 0, 0);
      curr_pulse_val = 0;
      // reset pulse sine wave parameters
    }
    else{
      color = pixels.Color(r, g, b);
    }
    update_color = true;
  } 
  else if (addr == "/enable_pulse") {
    // sending a string "0" or "1" because there's a weird 
    // bug with sending bools or ints (only 1 is received)
    char val [8];
    msg.getString(0, val);
    String data = (char*)val;
    //Serial.println(data);
    if (data == "0"){
      enable_pulse = false;
      curr_pulse_val = 0;
    }
    else{
      enable_pulse = true;
    }
    // Serial.print("Enable Pulse set to ");
    // Serial.println(enable_pulse);
  } 
  else if (addr == "/color_r") {
    r = msg.getInt(0);
    if (!enable_pulse){   // enable pulse updates the color in loop()
      update_color = true;
      color = pixels.Color(r, g, b);
    }
    //Serial.println(r);
  } else if (addr == "/color_g") {
    g = msg.getInt(0);
    if (!enable_pulse){
      update_color = true;
      color = pixels.Color(r, g, b);
    }
    //Serial.println(g);
  } else if (addr == "/color_b") {
    b = msg.getInt(0);
    if (!enable_pulse){
      update_color = true;
      color = pixels.Color(r, g, b);
    }
    //Serial.println(b);
  } else if (addr == "/color_brightness") {
      if (!enable_pulse){
        brightness = msg.getFloat(0);
        int _r = int(r * 1.0 * brightness);
        int _g = int(g * 1.0 * brightness);
        int _b = int(b * 1.0 * brightness);
      // Serial.print("brightness: ");
      // Serial.print(brightness);
      // Serial.print("\tr: ");
      // Serial.print(_r);
      // Serial.print(" g: ");
      // Serial.print(_g);
      // Serial.print(" b: ");
      // Serial.println(_b);
        update_color = true;        
        color = pixels.Color(_r, _g, _b);
    }
  }

  else if (addr == "/duration") {
      duration = msg.getFloat(0);
    //Serial.println(duration);
  }

  
  else{
    Serial.print("Unknown Msg Format:\n\t");
    Serial.println(addr);
  }

  if (update_color){
    //color = pixels.Color(r, g, b);
    pixels.fill(color);
    pixels.show();
    update_color = false;
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
      // Serial.print("MSG_ADDR: ");
      // Serial.println(msg_addr);

      // if (!msg.isBundle()) {
      msg.dispatch(msg_addr, on_message_received);
    }
  }
}
