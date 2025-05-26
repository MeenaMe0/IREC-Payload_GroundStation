/*
 THIS CODE IS THE CODE THAT UPLOAD TO THE ARDUINO R4 for gimbal working IREC 2025

Simple version: pwm set as constant. and upload the data log to sd card
 - simple pwm
 - log sd
 - avoid g > 10
*/

#include <Wire.h>
#include <SD.h>
#include <SPI.h>

#define MPU_ADDR 0x68

#define IN1  2
#define IN2  3
#define ENA  9

#define SD_CS 10  // Changed for UNO R4 compatibility

float setpoint = 0.0;
float maxAngle = 80.0;
float currentAngle = 0.0;

const float shockThreshold = 10.0; // in Gs
bool launchDetected = false;
unsigned long launchTime = 0;

File logFile;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card failed!");
  } else {
    Serial.println("SD card initialized.");
    logFile = SD.open("log.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("Time(ms),Angle,Direction");
      logFile.close();
    }
  }

  Serial.println("Simple Gimbal with SD Logging started. Target: 0°");
}

void loop() {
  if (launchDetected && millis() - launchTime < 10000) {
    stopMotor();
    Serial.println("🚀 Shock detected! Pausing for 10 seconds...");
    delay(100);
    return;
  }

  // === Read MPU6050 Accel Data ===
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  // Convert to G-force
  float az_g = az / 16384.0;

  // Shock detection
  if (!launchDetected && abs(az_g) > shockThreshold) {
    launchDetected = true;
    launchTime = millis();
    stopMotor();
    Serial.println("🚀 Shock detected! Pausing for 10 seconds...");
    return;
  }

  // Compute angle
  currentAngle = atan2(ay, az) * 180.0 / PI;
  currentAngle = constrain(currentAngle, -maxAngle, maxAngle);

  float error = setpoint - currentAngle;
  String direction = driveMotor(error);

  // Debug
  Serial.print("Angle: ");
  Serial.print(currentAngle, 2);
  Serial.print(" | Dir: ");
  Serial.println(direction);

  // Log to SD card
  logFile = SD.open("log.csv", FILE_WRITE);
  if (logFile) {
    logFile.print(millis());
    logFile.print(",");
    logFile.print(currentAngle, 2);
    logFile.print(",");
    logFile.println(direction);
    logFile.close();
  }

  delay(10);
}

String driveMotor(float error) {
  const int fixedPWM = 20;
  const float deadzone = 2.0;

  if (error > deadzone && error < 86) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, fixedPWM);
    return "Right";
  } else if (error < -deadzone && error > -86) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, fixedPWM);
    return "Left";
  } else {
    stopMotor();
    return "Stop";
  }
}

void stopMotor() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
