/*
   Interesting code 1: 
   int pwm = abs(output);
  pwm = constrain(pwm, 0, 255);
  คำนวณ pwm จาก output

*/



#include <Wire.h>

#define MPU_ADDR 0x68



#define IN1  2
#define IN2  3
#define ENA  9  // PWM

// PID Settings
float Kp = 4.0;
float Ki = 0.1;
float Kd = 0.05;

float setpoint = 0.0;
float maxAngle = 80.0;
float maxPWM = 255.0;

float currentAngle = 0.0;
float lastError = 0.0;
float integral = 0.0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  lastTime = millis();
  Serial.println("Gimbal PID started. Target angle: 0°");
}

void loop() {
  // Read MPU6050 Accel Data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  // Compute Roll angle
  currentAngle = atan2(ay, az) * 180.0 / PI;
  currentAngle = constrain(currentAngle, -maxAngle, maxAngle);

  // PID
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  float error = setpoint - currentAngle;

  integral += error * dt;
  float derivative = (error - lastError) / dt;

  float output = Kp * error + Ki * integral + Kd * derivative;
  output = constrain(output, -maxPWM, maxPWM);

  driveMotor(output);

  // Debug
  Serial.print("Angle: ");
  Serial.print(currentAngle, 2);
  Serial.print(" | Output: ");
  Serial.print(output, 2);
  Serial.print(" | Dir: ");

  if (output > 5) Serial.println("→");
  else if (output < -5) Serial.println("←");
  else Serial.println("■");

  lastError = error;
  lastTime = now;

  delay(10);
}

void driveMotor(float output) {
  int pwm = abs(output);
  pwm = constrain(pwm, 0, 255);

  if(output <= 86 && output >= -86){ // if it between normal or tilt value then rotate
      if (output > 8) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, pwm);
    } else if (output < -8) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm);
    } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    }
  }else { // if the angle exceed the normal rotation (more than 86) at 2 sides, then stop to wait for normal angle, and return to 0. This hopes to avoid wire tornados.
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }


  
}
