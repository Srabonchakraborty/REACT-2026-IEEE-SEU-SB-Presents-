#include <Wire.h>
#include <EEPROM.h>
#include <ezButton.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// --- Pin Definitions ---
#define AIN1 12 // Left Motors
#define AIN2 11
#define PWMA 10    

#define BIN1 7 // Right Motors
#define BIN2 8
#define PWMB 9     

#define BTN_UP A2
#define BTN_DOWN A1
#define BTN_SELECT A3
#define BTN_BACK 2

#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define MUX_SIG A0 

// --- Button Debouncing ---
ezButton btnUp(BTN_UP);
ezButton btnDown(BTN_DOWN);
ezButton btnSelect(BTN_SELECT);
ezButton btnBack(BTN_BACK);

// --- Display Setup ---
#define I2C_ADDR 0x3C
SSD1306AsciiWire oled;

// --- Enums ---
enum RunMode { MODE_NORMAL_PID, MODE_MAZE_DRY_RUN, MODE_MAZE_FAST_RUN };
enum JunctionType { JUNCT_NONE, JUNCT_LEFT_ONLY, JUNCT_RIGHT_ONLY, JUNCT_T, JUNCT_CROSS, JUNCT_DEAD_END, JUNCT_FINISH };

// --- Minimal EEPROM Structure ---
struct EMemory {
  int MIN[16];
  int MAX[16];
  float Kp, Ki, Kd;
  int BaseSpeed, MaxSpeed;
  char path[40];
  uint8_t pathLength;
  uint16_t key = 8802; 
};

const int EEPROM_ADDRESS = 0;

// --- Sensor Data ---
int arrayReading[16];
int minValue[16];
int maxValue[16];
int normalized[16];
const int edge = 800;

// Angled array weights (Exponential weighting for outer wings)
const int weight[16] = {
  -edge, -520, -360, -200,
   -100, -50, -25, -10,
    10, 25, 50, 100,
   200, 360, 520, edge
};

bool calibrated = false;

// --- Flash RAM Constants ---
const int CalibSpeed = 80;
const int MinSpeed = 20;
const float KpBoost = 0.65f; 
const float SpeedDrop = 0.08f; 

// --- User Parameters ---
int BaseSpeed = 160;
int MaxSpeed = 210;
float Kp = 0.85f;
float Ki = 0.00f;
float Kd = 0.35f;

// --- Runtime Variables ---
char path[40];
uint8_t pathLength = 0;
uint8_t pathIndex = 0;

RunMode currentRunMode = MODE_NORMAL_PID;

float Error = 0;
float PrevError = 0;
float Integral = 0;
unsigned long finishBoxTimer = 0;

// --- UI Navigation State Machine ---
enum MenuState {
  STATE_MAIN_MENU,
  STATE_RUN,
  STATE_CALIBRATE,
  STATE_PID_MENU,
  STATE_SPEED_MENU,
  STATE_ADV_MENU,
  STATE_SENSOR_DATA
};

MenuState currentState = STATE_MAIN_MENU;
bool needRedraw = true;

int mainMenuIndex = 0;
const int MAIN_MENU_COUNT = 6;
const char* mainMenuItems[] = {
  "1. Start Run",
  "2. Calibrate",
  "3. PID Tuning",
  "4. Speed Setup",
  "5. Mode Select",
  "6. Live Sensors"
};

int pidMenuIndex = 0;
int speedMenuIndex = 0;
unsigned long lastSensorUpdate = 0;

// --- Declarations ---
void readSensor();
void normalizeSensors();
void calculateDynamicPID(float &outSpeed, float &outPID);
JunctionType detectJunction();
void handleJunction(JunctionType junct);
void simplifyPath();
void executeTurn(char turn);

void stopMotors();
void leftMotorForward(int speed);
void rightMotorForward(int speed);
void leftMotorBackward(int speed);
void rightMotorBackward(int speed);

void loadEEPROM();
void saveEEPROM();

void handleMainMenu();
void handlePidMenu();
void handleSpeedMenu();
void handleAdvMenu();
void handleSensorData();
void handleRun();
void handleCalibrate();

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDR);
  oled.setFont(System5x7);
  oled.clear();

  btnUp.setDebounceTime(35);
  btnDown.setDebounceTime(35);
  btnSelect.setDebounceTime(35);
  btnBack.setDebounceTime(35);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(MUX_SIG, INPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  stopMotors();
  loadEEPROM();
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  btnUp.loop();
  btnDown.loop();
  btnSelect.loop();
  btnBack.loop();

  switch (currentState) {
    case STATE_MAIN_MENU: handleMainMenu(); break;
    case STATE_RUN: handleRun(); break;
    case STATE_CALIBRATE: handleCalibrate(); break;
    case STATE_PID_MENU: handlePidMenu(); break;
    case STATE_SPEED_MENU: handleSpeedMenu(); break;
    case STATE_ADV_MENU: handleAdvMenu(); break;
    case STATE_SENSOR_DATA: handleSensorData(); break;
  }
}

// ==========================================
// CORE CONTROL ENGINE
// ==========================================

void normalizeSensors() {
  for (int i = 0; i < 16; i++) {
    long diff = maxValue[i] - minValue[i];
    if (diff > 0) {
      float temp = (long)(arrayReading[i] - minValue[i]) * 100 / diff;
      int norm = 100 - (int)constrain(temp, 0, 100);
      normalized[i] = (norm < 12) ? 0 : norm; // Floor noise
    } else {
      normalized[i] = 0;
    }
  }
}

void calculateDynamicPID(float &activeBaseSpeed, float &PID_Value) {
  // --- LOOP & SPLIT PATH BIAS (For Circles, Diamonds, Hexagons) ---
  bool leftWingActive = (normalized[1] > 35 || normalized[2] > 35 || normalized[3] > 35);
  bool rightWingActive = (normalized[12] > 35 || normalized[13] > 35 || normalized[14] > 35);
  bool centerEmpty = (normalized[7] < 25 && normalized[8] < 25);

  // When approaching a loop/split: center drops to 0 while both sides pick up lines.
  // Lock onto the LEFT branch of the loop to guide the robot smoothly through.
  if (leftWingActive && rightWingActive && centerEmpty) {
    for (int i = 8; i < 16; i++) {
      normalized[i] = 0; // Ignore right side split to stay on left loop curve
    }
  }

  float sum = 0;
  float totalSensorValue = 0;

  for (int i = 0; i < 16; i++) {
    sum += ((float)weight[i] * (float)normalized[i]);
    totalSensorValue += (float)normalized[i];
  }

  if (totalSensorValue > 0) {
    Error = sum / totalSensorValue;
  } else {
    // Line loss recovery hysteresis
    Error = (PrevError < 0) ? -edge : edge;
  }

  Integral = constrain(Integral + Error, -800, 800);

  float absError = abs(Error);
  float dynamicKp = Kp + (KpBoost * (absError / (float)edge));
  float dynamicKd = Kd * (1.0f + (0.5f * (absError / (float)edge)));

  PID_Value = (Error * dynamicKp) + (Integral * Ki) + ((Error - PrevError) * dynamicKd);
  PrevError = Error;

  // Reduce speed dynamically on sharp curve entries
  float speedReduction = absError * SpeedDrop;
  activeBaseSpeed = constrain(BaseSpeed - speedReduction, MinSpeed, MaxSpeed);
}

JunctionType detectJunction() {
  int activeCount = 0;
  int leftActive = 0, centerActive = 0, rightActive = 0;

  for (int i = 0; i < 16; i++) {
    if (normalized[i] > 50) {
      activeCount++;
      if (i < 5) leftActive++;
      else if (i < 11) centerActive++;
      else rightActive++;
    }
  }

  // Terminal Finish Square (All or almost all sensors see black)
  if (activeCount >= 13) return JUNCT_FINISH;

  if (leftActive >= 3 && rightActive >= 3 && centerActive >= 3) return JUNCT_CROSS;
  if (leftActive >= 3 && rightActive >= 3) return JUNCT_T;
  if (leftActive >= 3) return JUNCT_LEFT_ONLY;
  if (rightActive >= 3) return JUNCT_RIGHT_ONLY;
  if (activeCount == 0) return JUNCT_DEAD_END;

  return JUNCT_NONE;
}

void handleRun() {
  if (btnBack.isPressed()) {
    stopMotors();
    currentState = STATE_MAIN_MENU;
    needRedraw = true;
    return;
  }

  if (!calibrated) {
    oled.clear(); oled.setCursor(0, 3); oled.print(" CALIBRATE FIRST!");
    delay(1200); currentState = STATE_MAIN_MENU; needRedraw = true;
    return;
  }

  if (needRedraw) {
    oled.clear();
    oled.setCursor(0, 1);
    oled.print("RUNNING - ");
    oled.print(currentRunMode == MODE_NORMAL_PID ? "OBSTACLE" : (currentRunMode == MODE_MAZE_DRY_RUN ? "DRY RUN" : "FAST RUN"));
    oled.setCursor(0, 4); oled.print("Press BACK to STOP");
    needRedraw = false;
    pathIndex = 0;
    finishBoxTimer = 0;
  }

  readSensor();
  normalizeSensors();

  JunctionType junct = detectJunction();

  // Handle Stop Terminal Box on Obstacle Track
  if (junct == JUNCT_FINISH) {
    if (finishBoxTimer == 0) finishBoxTimer = millis();
    if (millis() - finishBoxTimer > 120) { // Confirmed black square
      stopMotors();
      oled.clear(); oled.setCursor(0, 3); oled.print(" FINISH REACHED!");
      delay(1500);
      currentState = STATE_MAIN_MENU;
      needRedraw = true;
      return;
    }
  } else {
    finishBoxTimer = 0;
  }

  // Maze solving mode triggers hard turns; Obstacle mode relies purely on dynamic PID + loop bias
  if (junct != JUNCT_NONE && currentRunMode != MODE_NORMAL_PID) {
    handleJunction(junct);
  } else {
    float activeSpeed, pidVal;
    calculateDynamicPID(activeSpeed, pidVal);

    float leftMotor = constrain(activeSpeed - pidVal, -MaxSpeed, MaxSpeed);
    float rightMotor = constrain(activeSpeed + pidVal, -MaxSpeed, MaxSpeed);

    if (leftMotor < 0) leftMotorBackward(-leftMotor);
    else leftMotorForward(leftMotor);

    if (rightMotor < 0) rightMotorBackward(-rightMotor);
    else rightMotorForward(rightMotor);
  }
}

void handleJunction(JunctionType junct) {
  if (junct == JUNCT_NONE || junct == JUNCT_FINISH) return;

  stopMotors();
  delay(15);

  if (currentRunMode == MODE_MAZE_DRY_RUN) {
    char turnTaken = 'S';

    switch (junct) {
      case JUNCT_LEFT_ONLY:
      case JUNCT_T:
      case JUNCT_CROSS:
        turnTaken = 'L';
        break;
      case JUNCT_RIGHT_ONLY:
        readSensor(); normalizeSensors();
        if (detectJunction() == JUNCT_NONE) turnTaken = 'R';
        else turnTaken = 'S';
        break;
      case JUNCT_DEAD_END:
        turnTaken = 'B';
        break;
    }

    executeTurn(turnTaken);
    if (pathLength < 39) {
      path[pathLength++] = turnTaken;
      path[pathLength] = '\0';
    }
  } 
  else if (currentRunMode == MODE_MAZE_FAST_RUN) {
    if (pathIndex >= pathLength) {
      stopMotors();
      currentState = STATE_MAIN_MENU;
      needRedraw = true;
      return;
    }
    executeTurn(path[pathIndex++]);
  }
}

void executeTurn(char turn) {
  int turnSpeed = BaseSpeed * 0.75;
  
  switch (turn) {
    case 'L':
      leftMotorBackward(turnSpeed);
      rightMotorForward(turnSpeed);
      delay(170);
      while (analogRead(MUX_SIG) < 400) { readSensor(); }
      break;
    case 'R':
      leftMotorForward(turnSpeed);
      rightMotorBackward(turnSpeed);
      delay(170);
      while (analogRead(MUX_SIG) < 400) { readSensor(); }
      break;
    case 'B':
      leftMotorForward(turnSpeed);
      rightMotorBackward(turnSpeed);
      delay(300);
      while (analogRead(MUX_SIG) < 400) { readSensor(); }
      break;
    case 'S':
      leftMotorForward(BaseSpeed);
      rightMotorForward(BaseSpeed);
      delay(100);
      break;
  }
}

// ==========================================
// MENU SYSTEM
// ==========================================

void handleMainMenu() {
  if (btnUp.isPressed()) {
    mainMenuIndex = (mainMenuIndex - 1 + MAIN_MENU_COUNT) % MAIN_MENU_COUNT;
    needRedraw = true;
  }
  if (btnDown.isPressed()) {
    mainMenuIndex = (mainMenuIndex + 1) % MAIN_MENU_COUNT;
    needRedraw = true;
  }
  if (btnSelect.isPressed()) {
    switch (mainMenuIndex) {
      case 0: currentState = STATE_RUN; break;
      case 1: currentState = STATE_CALIBRATE; break;
      case 2: currentState = STATE_PID_MENU; pidMenuIndex = 0; break;
      case 3: currentState = STATE_SPEED_MENU; speedMenuIndex = 0; break;
      case 4: currentState = STATE_ADV_MENU; break;
      case 5: currentState = STATE_SENSOR_DATA; break;
    }
    needRedraw = true;
    return;
  }

  if (needRedraw) {
    oled.clear();
    oled.setCursor(0, 0); oled.print("--- LFR PRO ENGINE ---");
    for (int i = 0; i < MAIN_MENU_COUNT; i++) {
      oled.setCursor(0, i + 2);
      oled.print(i == mainMenuIndex ? "> " : " ");
      oled.print(mainMenuItems[i]);
    }
    needRedraw = false;
  }
}

void handlePidMenu() {
  if (btnUp.isPressed()) {
    if (pidMenuIndex == 0) Kp += 0.05;
    else if (pidMenuIndex == 1) Ki += 0.01;
    else if (pidMenuIndex == 2) Kd += 0.05;
    needRedraw = true;
  }
  if (btnDown.isPressed()) {
    if (pidMenuIndex == 0) Kp = max(0.00f, Kp - 0.05f);
    else if (pidMenuIndex == 1) Ki = max(0.00f, Ki - 0.01f);
    else if (pidMenuIndex == 2) Kd = max(0.00f, Kd - 0.05f);
    needRedraw = true;
  }
  if (btnSelect.isPressed()) {
    pidMenuIndex = (pidMenuIndex + 1) % 4;
    if (pidMenuIndex == 3) {
      saveEEPROM();
      oled.clear(); oled.setCursor(0, 3); oled.print(" PID Saved!");
      delay(800); pidMenuIndex = 0;
    }
    needRedraw = true;
  }
  if (btnBack.isPressed()) { currentState = STATE_MAIN_MENU; needRedraw = true; return; }

  if (needRedraw) {
    oled.clear();
    oled.setCursor(0, 0); oled.print("--- PID TUNING ---");
    oled.setCursor(0, 2); oled.print(pidMenuIndex == 0 ? "> " : " "); oled.print("Kp: "); oled.print(Kp, 2);
    oled.setCursor(0, 3); oled.print(pidMenuIndex == 1 ? "> " : " "); oled.print("Ki: "); oled.print(Ki, 2);
    oled.setCursor(0, 4); oled.print(pidMenuIndex == 2 ? "> " : " "); oled.print("Kd: "); oled.print(Kd, 2);
    oled.setCursor(0, 6); oled.print(pidMenuIndex == 3 ? "> [ SAVE & EXIT ]" : " [ SAVE & EXIT ]");
    needRedraw = false;
  }
}

void handleSpeedMenu() {
  if (btnUp.isPressed()) {
    if (speedMenuIndex == 0) BaseSpeed = min(255, BaseSpeed + 5);
    else if (speedMenuIndex == 1) MaxSpeed = min(255, MaxSpeed + 5);
    needRedraw = true;
  }
  if (btnDown.isPressed()) {
    if (speedMenuIndex == 0) BaseSpeed = max(10, BaseSpeed - 5);
    else if (speedMenuIndex == 1) MaxSpeed = max(10, MaxSpeed - 5);
    needRedraw = true;
  }
  if (btnSelect.isPressed()) {
    speedMenuIndex = (speedMenuIndex + 1) % 3;
    if (speedMenuIndex == 2) {
      saveEEPROM();
      oled.clear(); oled.setCursor(0, 3); oled.print(" Speeds Saved!");
      delay(800); speedMenuIndex = 0;
    }
    needRedraw = true;
  }
  if (btnBack.isPressed()) { currentState = STATE_MAIN_MENU; needRedraw = true; return; }

  if (needRedraw) {
    oled.clear();
    oled.setCursor(0, 0); oled.print("--- SPEED TUNING ---");
    oled.setCursor(0, 2); oled.print(speedMenuIndex == 0 ? "> " : " "); oled.print("Base Speed : "); oled.print(BaseSpeed);
    oled.setCursor(0, 4); oled.print(speedMenuIndex == 1 ? "> " : " "); oled.print("Max Speed : "); oled.print(MaxSpeed);
    oled.setCursor(0, 6); oled.print(speedMenuIndex == 2 ? "> [ SAVE & EXIT ]" : " [ SAVE & EXIT ]");
    needRedraw = false;
  }
}

void handleAdvMenu() {
  if (btnUp.isPressed() || btnDown.isPressed()) {
    int m = (int)currentRunMode;
    m = (m + 1) % 3;
    currentRunMode = (RunMode)m;
    needRedraw = true;
  }
  if (btnSelect.isPressed() || btnBack.isPressed()) {
    currentState = STATE_MAIN_MENU;
    needRedraw = true;
    return;
  }

  if (needRedraw) {
    oled.clear();
    oled.setCursor(0, 0); oled.print("-- COMPETITION MODE --");
    oled.setCursor(0, 2); oled.print("Mode: ");
    if (currentRunMode == MODE_NORMAL_PID) oled.print("Obstacle Track");
    else if (currentRunMode == MODE_MAZE_DRY_RUN) oled.print("Maze Dry Run");
    else oled.print("Maze Fast Run");

    oled.setCursor(0, 4); oled.print("Path: "); oled.print(pathLength > 0 ? path : "Empty");
    oled.setCursor(0, 6); oled.print("Press BACK to Return");
    needRedraw = false;
  }
}

void handleSensorData() {
  if (btnBack.isPressed()) { currentState = STATE_MAIN_MENU; needRedraw = true; return; }

  if (needRedraw) {
    oled.clear();
    oled.setCursor(0, 0); oled.print("SENSORS (0-9 Scale)");
    oled.setCursor(0, 2); oled.print("0-7 : ");
    oled.setCursor(0, 4); oled.print("8-15: ");
    oled.setCursor(0, 7); oled.print("Press BACK to return");
    needRedraw = false;
  }

  if (millis() - lastSensorUpdate > 90) {
    lastSensorUpdate = millis();
    readSensor();
    normalizeSensors();

    oled.setCursor(36, 2);
    for (int i = 0; i < 8; i++) { oled.print(normalized[i] / 10); oled.print(" "); }

    oled.setCursor(36, 4);
    for (int i = 8; i < 16; i++) { oled.print(normalized[i] / 10); oled.print(" "); }
  }
}

void handleCalibrate() {
  oled.clear();
  oled.setCursor(0, 2); oled.print(" CALIBRATING...");
  oled.setCursor(0, 4); oled.print(" Rotating Robot ");

  for (int i = 0; i < 16; i++) { minValue[i] = 1023; maxValue[i] = 0; }

  leftMotorForward(CalibSpeed);
  rightMotorBackward(CalibSpeed);

  unsigned long startTime = millis();
  while (millis() - startTime < 3500) {
    readSensor();
    for (int j = 0; j < 16; j++) {
      if (arrayReading[j] < minValue[j]) minValue[j] = arrayReading[j];
      if (arrayReading[j] > maxValue[j]) maxValue[j] = arrayReading[j];
    }
  }

  stopMotors();
  calibrated = true;
  saveEEPROM();

  oled.clear(); oled.setCursor(0, 3); oled.print(" Calibration Done!");
  delay(1200);
  currentState = STATE_MAIN_MENU;
  needRedraw = true;
}

void readSensor() {
  for (int i = 0; i < 16; i++) {
    digitalWrite(S0, i & 0x01);
    digitalWrite(S1, (i >> 1) & 0x01);
    digitalWrite(S2, (i >> 2) & 0x01);
    digitalWrite(S3, (i >> 3) & 0x01);
    arrayReading[i] = analogRead(MUX_SIG);
  }
}

void loadEEPROM() {
  EMemory data;
  EEPROM.get(EEPROM_ADDRESS, data);

  if (data.key == 8802) {
    for (int i = 0; i < 16; i++) {
      minValue[i] = data.MIN[i];
      maxValue[i] = data.MAX[i];
    }
    Kp = data.Kp; Ki = data.Ki; Kd = data.Kd;
    BaseSpeed = data.BaseSpeed; MaxSpeed = data.MaxSpeed;
    pathLength = data.pathLength;
    strcpy(path, data.path);
    calibrated = true;
  } else {
    calibrated = false;
  }
}

void saveEEPROM() {
  EMemory data;
  for (int i = 0; i < 16; i++) {
    data.MIN[i] = minValue[i];
    data.MAX[i] = maxValue[i];
  }
  data.Kp = Kp; data.Ki = Ki; data.Kd = Kd;
  data.BaseSpeed = BaseSpeed; data.MaxSpeed = MaxSpeed;
  data.pathLength = pathLength;
  strcpy(data.path, path);
  data.key = 8802;

  EEPROM.put(EEPROM_ADDRESS, data);
}

void leftMotorForward(int speed) {
  analogWrite(PWMA, speed);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}

void rightMotorForward(int speed) {
  analogWrite(PWMB, speed);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void leftMotorBackward(int speed) {
  analogWrite(PWMA, speed);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
}

void rightMotorBackward(int speed) {
  analogWrite(PWMB, speed);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void stopMotors() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
}