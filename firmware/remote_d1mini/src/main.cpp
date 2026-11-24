// Внешний пульт: D1 mini + энкодер HW-040 (заглушка).
// Рабочая версия: ~/viking_remote/src/main.cpp на Pi.
// Wi-Fi: скопировать include/secrets.h.example в include/secrets.h и заполнить. secrets.h в git не попадает.
#include <Arduino.h>
// #include "secrets.h"

void setup() {
  Serial.begin(115200);
}

void loop() {
}
