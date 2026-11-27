// firmware/mega2560/src/main.cpp
// BLDC ESC via Timer1 direct (no Servo lib), сервы через micros()
// От 27.11.2026: чистые C-строки для rxBuffer, без String fragmentation
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

// --- ESC через Timer1 (пины 11=OC1A, 12=OC1B) ---
const int PIN_ESC_LEFT  = 11;
const int PIN_ESC_RIGHT = 12;
// Сервы на Timer2 пинах (4, 5) — не конфликтуют с Timer1
const int PIN_SERVO1 = 5;
const int PIN_SERVO2 = 4;
const int PIN_RELAY1 = 47;
const int PIN_RELAY2 = 48;
const int PIN_BLADE  = 49;
const int PIN_BUMP_F = 30, PIN_BUMP_R = 31;
const int PIN_TILT_L = 32, PIN_TILT_R = 33;
const int PIN_BATT   = A0, PIN_LED_OK = 13;

const unsigned long HEARTBEAT_TIMEOUT_MS = 1000;
const unsigned long SENSORS_REPORT_MS    = 200;
const int BATT_DIV_RATIO = 11;

// ESC параметры (микросекунды)
const int ESC_NEUTRAL = 1500;
const int ESC_MIN     = 1000;
const int ESC_MAX     = 2000;

// Timer1 параметры: Fast PWM, prescaler 8, 20kHz servo signal
const unsigned long TIMER1_TOP = 20000 - 1;

// Софт-сервы: 50Hz (20ms период), 500-2300 мкс импульс
const unsigned long SERVO_PULSE_PERIOD = 20000;
const int SERVO_MIN_US = 500;  // 0°
const int SERVO_MAX_US = 2300; // 180°

// Буфер UART (char[], без String!)
char rxBuf[64];
uint8_t rxLen = 0;

// Состояние
unsigned long lastHB = 0, lastSR = 0, lastLCD = 0;
bool systemSafe = true;
int16_t speedL = 0, speedR = 0;
bool _bladeOn = false;
String safeReason = "";
int clockH = 0, clockM = 0;
unsigned long clockLastSync = 0;

// Пульт
long remoteRot = 0;
char remoteEvt = 'R';
unsigned long remoteEvtAt = 0;

// Софт-сервы
int servoAngle1 = 90, servoAngle2 = 90;
unsigned long servo1PulseStart = 0, servo2PulseStart = 0;
unsigned long servo1TargetUs = 1500, servo2TargetUs = 1500;

// --- ESC функции ---
void setESCLeft(int us) {
  us = constrain(us, ESC_MIN, ESC_MAX);
  OCR1A = (unsigned long)us * TIMER1_TOP / 10000UL;
}

void setESCRight(int us) {
  us = constrain(us, ESC_MIN, ESC_MAX);
  OCR1B = (unsigned long)us * TIMER1_TOP / 10000UL;
}

// Скорость (-255..255) → микросекунды ESC
int16_t speedToUS(int16_t speed) {
  if (speed == 0) return ESC_NEUTRAL;
  if (speed > 0) return ESC_NEUTRAL + (int16_t)((float)speed / 255.0 * (ESC_MAX - ESC_NEUTRAL));
  return ESC_NEUTRAL + (int16_t)((float)speed / 255.0 * (ESC_NEUTRAL - ESC_MIN));
}

void setMotors(int16_t left, int16_t right) {
  left  = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  speedL = left; speedR = right;
  setESCLeft(speedToUS(left));
  setESCRight(speedToUS(right));
}

void setBlade(bool on) { _bladeOn = on; digitalWrite(PIN_BLADE, on ? HIGH : LOW); }
void setRelay1(bool on) { digitalWrite(PIN_RELAY1, on ? HIGH : LOW); }
void setRelay2(bool on) { digitalWrite(PIN_RELAY2, on ? HIGH : LOW); }

void emergencyStop(const char* reason) {
  setMotors(0, 0); setBlade(false); setRelay1(false); setRelay2(false);
  setESCLeft(ESC_NEUTRAL); setESCRight(ESC_NEUTRAL);
  systemSafe = false; safeReason = String(reason);
  Serial.print("SAFE;"); Serial.println(reason);
}

String pad2(int v) { if (v < 10) return "0" + String(v); return String(v); }

// --- Часы ---
void tickClock() {
  if (clockLastSync == 0) return;
  unsigned long elapsedMin = (millis() - clockLastSync) / 60000;
  if (elapsedMin == 0) return;
  unsigned long total = (unsigned long)clockH * 60 + clockM + elapsedMin;
  clockH = (total / 60) % 24; clockM = total % 60;
  clockLastSync += elapsedMin * 60000UL;
}

// --- Софт-сервы ---
void updateServos() {
  unsigned long now = micros();
  int us1 = SERVO_MIN_US + (int)((float)servoAngle1 / 180.0 * (SERVO_MAX_US - SERVO_MIN_US));
  int us2 = SERVO_MIN_US + (int)((float)servoAngle2 / 180.0 * (SERVO_MAX_US - SERVO_MIN_US));
  servo1TargetUs = us1; servo2TargetUs = us2;
  if (now - servo1PulseStart >= SERVO_PULSE_PERIOD) servo1PulseStart = now;
  if (now - servo2PulseStart >= SERVO_PULSE_PERIOD) servo2PulseStart = now;
  digitalWrite(PIN_SERVO1, (now - servo1PulseStart < (unsigned long)us1) ? HIGH : LOW);
  digitalWrite(PIN_SERVO2, (now - servo2PulseStart < (unsigned long)us2) ? HIGH : LOW);
}

// --- LCD ---
void updateLcd() {
  tickClock();
  lcd.setCursor(0, 0);
  {
    String st = systemSafe ? F("STATUS: READY") : F("STATUS: SAFE ");
    char e = (millis() - remoteEvtAt < 1500UL) ? remoteEvt : 'R';
    String rs = String(e) + ":" + String(remoteRot);
    while (rs.length() < 6) rs += " ";
    if (rs.length() > 6) rs = rs.substring(0, 6);
    lcd.print(st + " " + rs);
  }
  long batt_mv = (long)analogRead(PIN_BATT) * 5000L / 1023L * BATT_DIV_RATIO;
  lcd.setCursor(0, 1);
  lcd.print("BAT:"); lcd.print(batt_mv / 1000); lcd.print("."); lcd.print((batt_mv % 1000) / 100); lcd.print("V  ");
  lcd.print(_bladeOn ? "BLD:ON " : "BLD:OFF"); lcd.print("    ");
  lcd.setCursor(0, 2);
  lcd.print("L:"); lcd.print(speedL); lcd.print(" R:"); lcd.print(speedR); lcd.print("  ");
  lcd.print(pad2(clockH) + ":" + pad2(clockM)); lcd.print("   ");
  lcd.setCursor(0, 3);
  if (!systemSafe) { lcd.print("! "); lcd.print(safeReason); lcd.print("          "); }
  else { lcd.print("HB OK  TILT:"); lcd.print(digitalRead(PIN_TILT_L) ? "0" : "1"); lcd.print(digitalRead(PIN_TILT_R) ? "0" : "1"); lcd.print("  "); }
}

// --- Парсер команд (чистые C-строки, без String) ---
void processCommand(const char* cmd) {
  if (cmd[0] == '\0') return;
  lastHB = millis();
  digitalWrite(PIN_LED_OK, !digitalRead(PIN_LED_OK));

  char cmdCopy[64];
  strncpy(cmdCopy, cmd, 63); cmdCopy[63] = '\0';
  char* semicolon = strchr(cmdCopy, ';');
  char* cmdType = cmdCopy;

  if (semicolon) {
    *semicolon = '\0';
    char* args = semicolon + 1;

    if (strcmp(cmdType, "DRV") == 0) {
      char* s2 = strchr(args, ';');
      if (!s2) { Serial.println("ERR;DRV_FORMAT"); return; }
      *s2 = '\0'; setMotors(atoi(args), atoi(s2 + 1)); Serial.println("ACK;DRV");
    } else if (strcmp(cmdType, "ESC") == 0) {
      char* s2 = strchr(args, ';');
      if (!s2) { Serial.println("ERR;ESC_FORMAT"); return; }
      *s2 = '\0'; setESCLeft(atoi(args)); setESCRight(atoi(s2 + 1));
      Serial.print("ACK;ESC;"); Serial.print(atoi(args)); Serial.print(";"); Serial.println(atoi(s2 + 1));
    } else if (strcmp(cmdType, "BLD") == 0) {
      setBlade(atoi(args) != 0); Serial.println("ACK;BLD");
    } else if (strcmp(cmdType, "HB") == 0) {
      if (!systemSafe) systemSafe = true;
      Serial.println("ACK;HB");
    } else if (strcmp(cmdType, "S1") == 0) {
      int a = constrain(atoi(args), 0, 180);
      servoAngle1 = a;
      servo1TargetUs = SERVO_MIN_US + (int)((float)a / 180.0 * (SERVO_MAX_US - SERVO_MIN_US));
      Serial.print("ACK;S1;"); Serial.println(a);
    } else if (strcmp(cmdType, "S2") == 0) {
      int a = constrain(atoi(args), 0, 180);
      servoAngle2 = a;
      servo2TargetUs = SERVO_MIN_US + (int)((float)a / 180.0 * (SERVO_MAX_US - SERVO_MIN_US));
      Serial.print("ACK;S2;"); Serial.println(a);
    } else if (strcmp(cmdType, "R1") == 0) {
      setRelay1(atoi(args) != 0);
      Serial.print("ACK;R1;"); Serial.println(atoi(args));
    } else if (strcmp(cmdType, "R2") == 0) {
      setRelay2(atoi(args) != 0);
      Serial.print("ACK;R2;"); Serial.println(atoi(args));
    } else if (strcmp(cmdType, "TIME") == 0) {
      char* i1 = strchr(args, ';');
      if (i1) {
        *i1 = '\0'; clockH = atoi(args); clockM = atoi(i1 + 1); clockLastSync = millis();
        Serial.print("ACK;TIME;"); Serial.print(pad2(clockH)); Serial.print(":"); Serial.println(pad2(clockM));
      } else { Serial.println("ERR;TIME_FORMAT"); }
    } else if (strcmp(cmdType, "ROT") == 0) {
      remoteRot = atol(args); lastLCD = 0;
    } else if (strcmp(cmdType, "BTN") == 0) {
      remoteEvt = 'C'; remoteEvtAt = millis(); lastLCD = 0;
    } else if (strcmp(cmdType, "RST") == 0) {
      remoteRot = 0; remoteEvt = 'Z'; remoteEvtAt = millis(); lastLCD = 0;
    } else {
      Serial.print("ERR;UNKNOWN_CMD:"); Serial.println(cmdType);
    }
  } else {
    if (strcmp(cmdType, "HB") == 0) {
      if (!systemSafe) systemSafe = true;
      Serial.println("ACK;HB");
    } else {
      Serial.print("ERR;UNKNOWN_CMD:"); Serial.println(cmdType);
    }
  }
}

// --- Датчики ---
void reportSensors() {
  int bumpF = !digitalRead(PIN_BUMP_F), bumpR = !digitalRead(PIN_BUMP_R);
  int tiltL = !digitalRead(PIN_TILT_L), tiltR = !digitalRead(PIN_TILT_R);
  long batt_mv = (long)analogRead(PIN_BATT) * 5000L / 1023L * BATT_DIV_RATIO;
  Serial.print("SENS;"); Serial.print(bumpF); Serial.print(";"); Serial.print(bumpR); Serial.print(";");
  Serial.print(tiltL); Serial.print(";"); Serial.print(tiltR); Serial.print(";"); Serial.println(batt_mv);
  if (tiltL || tiltR) emergencyStop("TILT");
}

// --- Init ---
void initESC() {
  // Timer1: Fast PWM 8-bit phase correct, prescaler 8 → 20kHz
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12);
  ICR1 = TIMER1_TOP;
  pinMode(PIN_ESC_LEFT, OUTPUT);
  pinMode(PIN_ESC_RIGHT, OUTPUT);
  setESCLeft(ESC_NEUTRAL);
  setESCRight(ESC_NEUTRAL);
}

void setup() {
  // Пины
  pinMode(PIN_BLADE, OUTPUT); pinMode(PIN_BUMP_F, INPUT_PULLUP); pinMode(PIN_BUMP_R, INPUT_PULLUP);
  pinMode(PIN_TILT_L, INPUT_PULLUP); pinMode(PIN_TILT_R, INPUT_PULLUP); pinMode(PIN_BATT, INPUT);
  pinMode(PIN_LED_OK, OUTPUT); pinMode(PIN_RELAY1, OUTPUT); pinMode(PIN_RELAY2, OUTPUT);
  pinMode(PIN_SERVO1, OUTPUT); pinMode(PIN_SERVO2, OUTPUT);

  // Безопасное состояние
  setBlade(false); setRelay1(false); setRelay2(false);
  digitalWrite(PIN_SERVO1, LOW); digitalWrite(PIN_SERVO2, LOW);

  // Инициализация ESC
  initESC();

  // LCD
  Wire.begin(); lcd.init(); lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Viking MI 632 P");
  lcd.setCursor(0, 1); lcd.print("Boot Timer1");

  // UART
  Serial.begin(115200);
  lastHB = millis();
  Serial.println("ACK;BOOT BLDC");

  // 2 сек для калибровки ESC
  delay(2000);
}

void loop() {
  unsigned long now = millis();

  // Чтение UART — БЕЗ String
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (rxLen > 0 && rxBuf[rxLen - 1] == '\r') rxBuf[--rxLen] = '\0';
      processCommand(rxBuf);
      rxLen = 0;
    } else {
      if (rxLen < 63) { rxBuf[rxLen++] = c; rxBuf[rxLen] = '\0'; }
      else { rxLen = 0; }
    }
  }

  // Heartbeat timeout
  if (now - lastHB > HEARTBEAT_TIMEOUT_MS) {
    if (systemSafe) emergencyStop("HEARTBEAT_LOST");
  } else {
    systemSafe = true;
  }

  // Датчики
  if (now - lastSR > SENSORS_REPORT_MS) {
    lastSR = now; reportSensors();
  }

  // LCD
  if (now - lastLCD > 500) {
    lastLCD = now; updateLcd();
  }

  // Софт-сервы
  updateServos();
}