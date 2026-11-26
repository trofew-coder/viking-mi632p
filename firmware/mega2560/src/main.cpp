#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

Servo servo1;
Servo servo2;
LiquidCrystal_I2C lcd(0x27, 20, 4);

const int PIN_ML_PWM = 2, PIN_ML_DIR = 22, PIN_MR_PWM = 3, PIN_MR_DIR = 24;
const int PIN_BLADE = 4, PIN_BUMP_F = 30, PIN_BUMP_R = 31;
const int PIN_TILT_L = 32, PIN_TILT_R = 33, PIN_BATT = A0, PIN_LED_OK = 13;
const int PIN_SERVO1 = 8, PIN_SERVO2 = 53;
const int PIN_RELAY1 = 9, PIN_RELAY2 = 10;

const unsigned long HEARTBEAT_TIMEOUT_MS = 300;
const unsigned long SENSORS_REPORT_MS = 200;
const int BATT_DIV_RATIO = 11;

String rxBuffer = "";
unsigned long lastHeartbeat = 0, lastSensorsReport = 0, lastLcdUpdate = 0;
bool systemSafe = true;
int currentSpeedL = 0, currentSpeedR = 0;
bool bladeOn = false;
String safeReason = "";
int clockH = 0, clockM = 0;
unsigned long clockLastSync = 0;
long remoteRot = 0;
char remoteEvt = 'R';
unsigned long remoteEvtAt = 0;

void setMotorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  currentSpeedL = speed;
  digitalWrite(PIN_ML_DIR, speed >= 0 ? HIGH : LOW);
  analogWrite(PIN_ML_PWM, abs(speed));
}
void setMotorRight(int speed) {
  speed = constrain(speed, -255, 255);
  currentSpeedR = speed;
  digitalWrite(PIN_MR_DIR, speed >= 0 ? HIGH : LOW);
  analogWrite(PIN_MR_PWM, abs(speed));
}
void setBlade(bool on) { bladeOn = on; digitalWrite(PIN_BLADE, on ? HIGH : LOW); }
void setRelay1(bool on) { digitalWrite(PIN_RELAY1, on ? HIGH : LOW); }
void setRelay2(bool on) { digitalWrite(PIN_RELAY2, on ? HIGH : LOW); }

void emergencyStop(const char* reason) {
  setMotorLeft(0); setMotorRight(0); setBlade(false);
  setRelay1(false); setRelay2(false);
  systemSafe = false;
  safeReason = String(reason);
  Serial.print("SAFE;"); Serial.println(reason);
}
void hardStop() {
  setMotorLeft(0); setMotorRight(0); setBlade(false);
  setRelay1(false); setRelay2(false);
}

void tickClock() {
  if (clockLastSync == 0) return;
  unsigned long elapsedMin = (millis() - clockLastSync) / 60000;
  if (elapsedMin == 0) return;
  unsigned long total = (unsigned long)clockH * 60 + clockM + elapsedMin;
  clockH = (total / 60) % 24;
  clockM = total % 60;
  clockLastSync += elapsedMin * 60000UL;
}

String pad2(int v) {
  if (v < 10) return "0" + String(v);
  return String(v);
}

void updateLcd() {
  tickClock();
  String timeStr = pad2(clockH) + ":" + pad2(clockM);
  
  lcd.setCursor(0, 0);
  {
    String st = systemSafe ? String("STATUS: READY") : String("STATUS: SAFE ");
    char e = (millis() - remoteEvtAt < 1500UL) ? remoteEvt : 'R';
    String rs = String(e) + ":" + String(remoteRot);
    while (rs.length() < 6) rs += " ";
    if (rs.length() > 6) rs = rs.substring(0, 6);
    lcd.print(st + " " + rs);
  }
  
  long batt_mv = (long)analogRead(PIN_BATT) * 5000L / 1023L * BATT_DIV_RATIO;
  lcd.setCursor(0, 1);
  lcd.print("BAT:");
  lcd.print(batt_mv / 1000);
  lcd.print(".");
  lcd.print((batt_mv % 1000) / 100);
  lcd.print("V  ");
  lcd.print(bladeOn ? "BLD:ON " : "BLD:OFF");
  lcd.print("    ");
  
  lcd.setCursor(0, 2);
  lcd.print("L:");
  lcd.print(currentSpeedL);
  lcd.print(" R:");
  lcd.print(currentSpeedR);
  lcd.print("  ");
  lcd.print(timeStr);
  lcd.print("   ");
  
  lcd.setCursor(0, 3);
  if (!systemSafe) {
    lcd.print("! ");
    lcd.print(safeReason);
    lcd.print("          ");
  } else {
    lcd.print("HB OK  TILT:");
    lcd.print(digitalRead(PIN_TILT_L) ? "0" : "1");
    lcd.print(digitalRead(PIN_TILT_R) ? "0" : "1");
    lcd.print("  ");
  }
}

void processCommand(String cmd) {
  if (cmd.length() == 0) return;
  lastHeartbeat = millis();
  digitalWrite(PIN_LED_OK, !digitalRead(PIN_LED_OK));
  int idx1 = cmd.indexOf(';');
  String type = (idx1 == -1) ? cmd : cmd.substring(0, idx1);
  String args = (idx1 == -1) ? "" : cmd.substring(idx1 + 1);
  
  if (type == "DRV") {
    int idx2 = args.indexOf(';');
    if (idx2 == -1) { Serial.println("ERR;DRV_FORMAT"); return; }
    setMotorLeft(args.substring(0, idx2).toInt());
    setMotorRight(args.substring(idx2 + 1).toInt());
    Serial.println("ACK;DRV");
  } else if (type == "BLD") {
    setBlade(args.toInt() != 0); Serial.println("ACK;BLD");
  } else if (type == "HB") {
    if (!systemSafe) systemSafe = true;
    Serial.println("ACK;HB");
  } else if (type == "S1") {
    servo1.write(constrain(args.toInt(), 0, 180));
    Serial.print("ACK;S1;"); Serial.println(args.toInt());
  } else if (type == "S2") {
    servo2.write(constrain(args.toInt(), 0, 180));
    Serial.print("ACK;S2;"); Serial.println(args.toInt());
  } else if (type == "R1") {
    setRelay1(args.toInt() != 0); Serial.print("ACK;R1;"); Serial.println(args.toInt());
  } else if (type == "R2") {
    setRelay2(args.toInt() != 0); Serial.print("ACK;R2;"); Serial.println(args.toInt());
  } else if (type == "TIME") {
    int i1 = args.indexOf(';');
    if (i1 != -1) {
      clockH = args.substring(0, i1).toInt();
      clockM = args.substring(i1 + 1).toInt();
      clockLastSync = millis();
      Serial.print("ACK;TIME;");
      Serial.print(pad2(clockH)); Serial.print(":");
      Serial.println(pad2(clockM));
    } else {
      Serial.println("ERR;TIME_FORMAT");
    }
  } else if (type == "ROT") {
    remoteRot = args.toInt(); lastLcdUpdate = 0;
  } else if (type == "BTN") {
    remoteEvt = 'C'; remoteEvtAt = millis(); lastLcdUpdate = 0;
  } else if (type == "RST") {
    remoteRot = 0; remoteEvt = 'Z'; remoteEvtAt = millis(); lastLcdUpdate = 0;
  } else {
    Serial.print("ERR;UNKNOWN_CMD:"); Serial.println(type);
  }
}

void reportSensors() {
  int bumpF = !digitalRead(PIN_BUMP_F), bumpR = !digitalRead(PIN_BUMP_R);
  int tiltL = !digitalRead(PIN_TILT_L), tiltR = !digitalRead(PIN_TILT_R);
  long batt_mv = (long)analogRead(PIN_BATT) * 5000L / 1023L * BATT_DIV_RATIO;
  Serial.print("SENS;"); Serial.print(bumpF); Serial.print(";");
  Serial.print(bumpR); Serial.print(";"); Serial.print(tiltL); Serial.print(";");
  Serial.print(tiltR); Serial.print(";"); Serial.println(batt_mv);
  if (tiltL || tiltR) emergencyStop("TILT");
}

void setup() {
  pinMode(PIN_ML_PWM, OUTPUT); pinMode(PIN_ML_DIR, OUTPUT);
  pinMode(PIN_MR_PWM, OUTPUT); pinMode(PIN_MR_DIR, OUTPUT);
  pinMode(PIN_BLADE, OUTPUT); pinMode(PIN_BUMP_F, INPUT_PULLUP);
  pinMode(PIN_BUMP_R, INPUT_PULLUP); pinMode(PIN_TILT_L, INPUT_PULLUP);
  pinMode(PIN_TILT_R, INPUT_PULLUP); pinMode(PIN_BATT, INPUT);
  pinMode(PIN_LED_OK, OUTPUT); pinMode(PIN_RELAY1, OUTPUT); pinMode(PIN_RELAY2, OUTPUT);
  servo1.attach(PIN_SERVO1); servo2.attach(PIN_SERVO2);
  servo1.write(90); servo2.write(90);
  hardStop();
  
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Viking MI 632 P");
  lcd.setCursor(0, 1);
  lcd.print("Booting...");
  
  Serial.begin(115200);
  lastHeartbeat = millis();
  Serial.println("ACK;BOOT");
}

void loop() {
  unsigned long now = millis();
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (rxBuffer.length() > 0 && rxBuffer[rxBuffer.length()-1] == '\r') rxBuffer.remove(rxBuffer.length()-1);
      processCommand(rxBuffer); rxBuffer = "";
    } else {
      if (rxBuffer.length() < 64) rxBuffer += c; else rxBuffer = "";
    }
  }
  if (now - lastHeartbeat > HEARTBEAT_TIMEOUT_MS) {
    if (systemSafe) emergencyStop("HEARTBEAT_LOST");
  } else { systemSafe = true; }
  if (now - lastSensorsReport > SENSORS_REPORT_MS) {
    lastSensorsReport = now; reportSensors();
  }
  if (now - lastLcdUpdate > 500) {
    lastLcdUpdate = now;
    updateLcd();
  }
}
