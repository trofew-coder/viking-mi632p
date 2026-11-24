#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

// ---- WiFi ---- (значения в include/secrets.h, в git не попадает)
#include "secrets.h"
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASS;
const char* server   = ROBOT_URL; // приёмник на Flask

// ---- KY-040 (HW-040) ----
#define PIN_CLK  D5   // GPIO14
#define PIN_DT   D6   // GPIO12
#define PIN_SW   D7   // GPIO13

volatile int  rot = 0;
volatile bool clkLast;

const unsigned long LONG_PRESS_MS = 800;
bool         swWasPressed  = false;
bool         swLongFired   = false;
bool         swClickPending= false;
unsigned long swPressedAt  = 0;

int  lastSentRot = 0;
unsigned long lastRotSend = 0;

void ICACHE_RAM_ATTR encoderISR() {
  bool clk = digitalRead(PIN_CLK);
  bool dt  = digitalRead(PIN_DT);
  if (clk != clkLast) {
    if (dt != clk) rot++;
    else           rot--;
    clkLast = clk;
  }
}

void sendState(const char* event, int value) {
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClient client;
  HTTPClient http;
  char body[96];
  snprintf(body, sizeof(body), "{\"rot\":%d,\"event\":\"%s\"}", value, event);
  http.setTimeout(1500);
  http.begin(client, server);
  http.addHeader("Content-Type", "application/json");
  http.POST(body);
  http.end();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CLK, INPUT_PULLUP);
  pinMode(PIN_DT,  INPUT_PULLUP);
  pinMode(PIN_SW,  INPUT_PULLUP);
  clkLast = digitalRead(PIN_CLK);
  attachInterrupt(digitalPinToInterrupt(PIN_CLK), encoderISR, CHANGE);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print('.');
  }
  Serial.println("\n[ESP] WiFi OK");
  Serial.println(WiFi.localIP());
}

void loop() {
  bool swNow = (digitalRead(PIN_SW) == LOW);

  if (swNow && !swWasPressed) {
    swPressedAt = millis();
    swLongFired = false;
  }
  if (swNow && !swLongFired && (millis() - swPressedAt >= LONG_PRESS_MS)) {
    swLongFired = true;
    rot = 0;
    lastSentRot = 0;
    sendState("LONG_PRESS", 0);
  }
  if (!swNow && swWasPressed) {
    if (!swLongFired) swClickPending = true;
  }
  swWasPressed = swNow;

  if (swClickPending) {
    swClickPending = false;
    sendState("CLICK", rot);
  }

  if (rot != lastSentRot && (millis() - lastRotSend > 200)) {
    lastSentRot = rot;
    lastRotSend = millis();
    sendState("ROTATE", rot);
  }

  delay(5);
}
