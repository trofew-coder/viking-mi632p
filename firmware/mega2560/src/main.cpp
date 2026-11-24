// Viking MI 632 P: прошивка Arduino Mega 2560 (заглушка).
// Рабочая версия лежит на Pi: ~/viking_project/src/main.cpp. Переносим отдельным коммитом.
// Протокол: docs/PROTOCOL.md, пины: docs/PINOUT.md
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
}
