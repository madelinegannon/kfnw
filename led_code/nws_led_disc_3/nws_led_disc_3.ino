/**
* nws_led_disc_3.ino
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
float r_lerp = 255.0;
int r_raw = 255;
int r_pulse = 255;

int g = 0;
float g_lerp = 0.0;
int g_raw = 0;
int g_pulse = 0;

int b = 80;
float b_lerp = 80.0;
int b_raw = 80;
int b_pulse = 80;

float lerp_increment = 0.5;
float brightness = 1.0;

bool enable = true;
bool enable_pulse = false;
float curr_pulse_val = 0;
float pulse_speed = 0.001;


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
// IPAddress static_IP(192, 168, 1, 11); // Disc 1
// IPAddress static_IP(192, 168, 1, 12); // Disc 2
IPAddress static_IP(192, 168, 1, 13);  // Disc 3
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
  if (enable) {

    if (do_update_color) {
      // transition (instead of jumping) to change of color
      lerp_to_color();

      //if (!enable_pulse) { // <--update twice to smooth out the pulsing
      pixels.fill(color);
      pixels.show();
      //}
    }

    if (enable_pulse) {
      curr_pulse_val = curr_pulse_val + pulse_speed;

      float val = sin(curr_pulse_val);
      val = (val + 1.0) * 128;
      // r_pulse = map(r_lerp, 0, 255, 0, int(val));
      // g_pulse = map(g_lerp, 0, 255, 0, int(val));
      // b_pulse = map(b_lerp, 0, 255, 0, int(val));
      // color = pixels.Color(r_pulse, g_pulse, b_pulse);
      color = pixels.Color(map(r_lerp, 0, 255, 0, int(val)), map(g_lerp, 0, 255, 0, int(val)), map(b_lerp, 0, 255, 0, int(val)));
      pixels.fill(color);
      pixels.show();
    }
  }
}

bool do_update_color() {
  return (r_lerp != r && g_lerp != g && b_lerp != b);
}

void lerp_to_color() {
  if (r_lerp < r) {
    r_lerp += lerp_increment;
  } else if (r_lerp > r) {
    r_lerp -= lerp_increment;
  }

  if (g_lerp < g) {
    g_lerp += lerp_increment;
  } else if (g_lerp > g) {
    g_lerp -= lerp_increment;
  }

  if (b_lerp < b) {
    b_lerp += lerp_increment;
  } else if (b_lerp > b) {
    b_lerp -= lerp_increment;
  }
  color = pixels.Color(r_lerp, g_lerp, b_lerp);
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
    char val[8];
    msg.getString(0, val);
    String data = (char*)val;
    // Serial.println(data);
    if (data == "0") {
      enable = false;
    } else {
      enable = true;
    }
    // Serial.print("Enable set to ");
    // Serial.println(enable);
    if (!enable) {
      color = pixels.Color(0, 0, 0);
      curr_pulse_val = 0;
      // reset pulse sine wave parameters
    } else {
      color = pixels.Color(r, g, b);
    }
    pixels.fill(color);
    pixels.show();
  } else if (addr == "/enable_pulse") {
    // sending a string "0" or "1" because there's a weird
    // bug with sending bools or ints (only 1 is received)
    char val[8];
    msg.getString(0, val);
    String data = (char*)val;
    if (data == "0") {
      enable_pulse = false;
      // reset the sine wave start at 0
      curr_pulse_val = -1 - pulse_speed;  
    } else {
      enable_pulse = true;
    }
    // Serial.print("Enable Pulse set to ");
    // Serial.println(enable_pulse);
  } else if (addr == "/color_r") {
    r = msg.getInt(0);
    r_raw = msg.getInt(0);
  } else if (addr == "/color_g") {
    g = msg.getInt(0);
    g_raw = msg.getInt(0);
  } else if (addr == "/color_b") {
    b = msg.getInt(0);
    b_raw = msg.getInt(0);
  } else if (addr == "/color_brightness") {
    if (!enable_pulse) {
      brightness = msg.getFloat(0);
      r = int(r_raw * 1.0 * brightness);
      g = int(g_raw * 1.0 * brightness);
      b = int(b_raw * 1.0 * brightness);
    }
  } else if (addr == "/pulse_speed") {
    pulse_speed = msg.getFloat(0);
  } else {
    Serial.print("Unknown Msg Format:\n\t");
    Serial.println(addr);
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
      msg.dispatch(msg_addr, on_message_received);
    }
  }
}
