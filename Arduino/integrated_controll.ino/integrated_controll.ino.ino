#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include "MPU9250.h"
#include <Servo.h> 

// --- Pin Definitions ---
const int ESC1_PIN = 9;
const int ESC2_PIN = 10;
#define BUZZER_PIN 8
#define WATER_SENSOR_PIN A0
#define RELAY_PIN 7

#define RED_PIN 3
#define GREEN_PIN 6
#define BLUE_PIN 5

// state, mode
enum SystemMode { POWER_MONITOR, CONTROLLER };
SystemMode currentMode = POWER_MONITOR;
String currentState = "STANDBY";

// control, sensor
int pulse_esc1 = 1000;
int pulse_esc2 = 1000;
bool piPowerTriggered = false;

// timing, 임계
unsigned long lastBuzzerActionTime = 0;
int arrivedBeepCount = 0;
unsigned long lastPiHeartbeatTime = 0;
unsigned long lastPowerMonitorCheck = 0; // [ADDED] For non-blocking delay
const unsigned long PI_TIMEOUT = 20000; // 20 seconds
const int WATER_THRESHOLD = 450;
const int GYRO_THRESHOLD = 10;

// 하이트리거 릴레이
#define RELAY_ON  HIGH
#define RELAY_OFF LOW

// 
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
MPU9250 mpu;
Servo esc1;
Servo esc2;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  setLedColor(LOW, LOW, LOW); // Turn all off
  
  
  esc1.attach(ESC1_PIN, 1000, 2000);
  esc2.attach(ESC2_PIN, 1000, 2000);
  esc1.writeMicroseconds(1000);//바로꺼쭘
  esc2.writeMicroseconds(1000);
  
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF); 

  if (!tsl.begin()) { while (1); }
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  if (!mpu.setup(0x68)) { while (1); }
}

void loop() {
  if (currentMode == POWER_MONITOR) {
    powerMonitorTask();
  } else { 
    controllerTask();
  }
}

void setLedColor(int r, int g, int b) {
  digitalWrite(RED_PIN, r);
  digitalWrite(GREEN_PIN, g);
  digitalWrite(BLUE_PIN, b);
}


-
void powerMonitorTask() {
  if (millis() - lastPowerMonitorCheck < 500) {
    return;
  }
  lastPowerMonitorCheck = millis();

  bool gyroTriggeredNow = false;
  if (mpu.update()) {
    if (abs(mpu.getGyroX()) > GYRO_THRESHOLD || abs(mpu.getGyroY()) > GYRO_THRESHOLD) {
      gyroTriggeredNow = true;
    }
  }
  int waterValue = analogRead(WATER_SENSOR_PIN);

  if (gyroTriggeredNow && waterValue > WATER_THRESHOLD && !piPowerTriggered) {
    piPowerTriggered = true; .
    
    digitalWrite(RELAY_PIN, RELAY_ON);  
    delay(1500);                    
    digitalWrite(RELAY_PIN, RELAY_OFF); 
  }

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "START") {
      switchToControllerMode();
    }
  }
}

void controllerTask() {
  esc1.writeMicroseconds(pulse_esc1);
  esc2.writeMicroseconds(pulse_esc2);
  delay(20); 

  handleBuzzer();
  handleLed(); 
  if (Serial.available() > 0) {
    lastPiHeartbeatTime = millis(); 
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "?") {
      sensors_event_t event;
      tsl.getEvent(&event);
      Serial.println(event.light ? event.light : 0.0);
    }
    else {
      int firstComma = command.indexOf(',');
      int secondComma = command.indexOf(',', firstComma + 1);
      if (firstComma > 0 && secondComma > 0) {
        String stateStr = command.substring(0, firstComma);
        if (currentState != stateStr) {
          lastBuzzerActionTime = millis();
          arrivedBeepCount = 0;
          noTone(BUZZER_PIN);
        }
        currentState = stateStr;
        pulse_esc1 = constrain(command.substring(firstComma + 1, secondComma).toInt(), 1000, 2000);
        pulse_esc2 = constrain(command.substring(secondComma + 1).toInt(), 1000, 2000);
      }
    }
  }

  if (millis() - lastPiHeartbeatTime > PI_TIMEOUT) {
    // Pi seems to be off, switch back to power monitor mode
    switchToPowerMonitorMode();
  }
}

void switchToControllerMode() {
  currentMode = CONTROLLER;
  setLedColor(HIGH, HIGH, HIGH);  
  tone(BUZZER_PIN, 1000);
  delay(13000);
  noTone(BUZZER_PIN);
  setLedColor(LOW, LOW, LOW); 
    
  for (int i = 0; i < 150; i++) {
    esc1.writeMicroseconds(1000); 
    esc2.writeMicroseconds(1000); 
    delay(20);
  }
  Serial.println("READY");
  lastPiHeartbeatTime = millis();
}

void switchToPowerMonitorMode() {
  currentMode = POWER_MONITOR;
  digitalWrite(RELAY_PIN, RELAY_OFF);
  
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  pulse_esc1 = 1000;
  pulse_esc2 = 1000;

  currentState = "STANDBY";
  piPowerTriggered = false; 
  
  setLedColor(LOW, LOW, LOW); 
}

void handleBuzzer() {
  unsigned long currentTime = millis();

  if (currentState == "SEARCHING") {
    if (currentTime - lastBuzzerActionTime >= 5000) {
      lastBuzzerActionTime = currentTime;
      tone(BUZZER_PIN, 800, 3000);
    }
  }
  else if (currentState == "APPROACHING") {
    if (currentTime - lastBuzzerActionTime >= 2000) {
      lastBuzzerActionTime = currentTime;
      tone(BUZZER_PIN, 1200, 1000);
    }
  }
  else if (currentState == "ARRIVED") {
    if (arrivedBeepCount < 3 && currentTime - lastBuzzerActionTime >= 1000) {
      lastBuzzerActionTime = currentTime;
      tone(BUZZER_PIN, 1500, 200);
      arrivedBeepCount++;
    }
  }
}

void handleLed() {
  if (currentState == "SEARCHING") {
    setLedColor(LOW, HIGH, LOW); // 초
  } else if (currentState == "APPROACHING") {
    setLedColor(HIGH, LOW, LOW); // 빨
  } else if (currentState == "ARRIVED") {
    setLedColor(LOW, LOW, HIGH); // 파,노
  } else { // STANDBY or other
    setLedColor(LOW, LOW, LOW);
  }
}
