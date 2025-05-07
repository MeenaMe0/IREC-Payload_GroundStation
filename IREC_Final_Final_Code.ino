//-- Library --//
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <HMC5883L.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SD.h>

// === Target Location (Ground Station) === if set
// const float lat_launch = 21.028511;
// const float lon_launch = 105.804817;
// const float alt_launch = 10.0; // Example altitude (meters)

//----- BME -------//
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
bool bme_ok = false;
float bme_h = 0, bme_t = 0, bme_p = 0, bme_alt = 0;

// === MPU6050 === //
float RateRoll1, RatePitch1, RateYaw1;
float RateRoll2, RatePitch2, RateYaw2;
float roll1, pitch1, yaw1, roll2, pitch2, yaw2;
float GyroX, GyroY, GyroZ, GyroX1, GyroY1, GyroZ1, GyroX2, GyroY2, GyroZ2;
float RateCalibrationRoll1 = 0, RateCalibrationPitch1 = 0, RateCalibrationYaw1 = 0;
float RateCalibrationRoll2 = 0, RateCalibrationPitch2 = 0, RateCalibrationYaw2 = 0;
int RateCalibrationNumber;

float AccX, AccY, AccZ, AccX_SD, AccY_SD, AccZ_SD;
float AngleRoll, AnglePitch, AngleYaw, AngleRoll_SD, AnglePitch_SD, AngleYaw_SD;
uint32_t LoopTimer;
bool mpu1_ok = true, mpu2_ok = true;

// === Complementary Filter Variables ===
float gyroYaw = 0, fusedYaw = 0;
float alpha = 0.98;
uint32_t lastTime_Gyro;

//----- GPS -------//
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
float lastAlt = 0;
uint32_t lastTime = 0;
float verticalSpeed_mps = 0;
float horizontalSpeed_mps = 0;
float lat_launch = 0, lon_launch = 0, alt_launch = 0;

// === Magnetometer === //
HMC5883L mag;
bool mag_ok = false;

// === Motor Pins ===
#define PITCH_ENA 5
#define PITCH_IN1 6
#define PITCH_IN2 7
#define YAW_ENA 9
#define YAW_IN1 10
#define YAW_IN2 11

// === PID Variables ===
float yawChange, pitchChange;
float yawPrevChange = 0, pitchPrevChange = 0;
float yawIntegral = 0, pitchIntegral = 0;
float Kp = 2.0, Ki = 0.0, Kd = 0.5; // Tuneable

//----- SD -------//
#define CSpin 10
#define FileName "data_log.csv"
File dataFile;

// === SoftwareSerial for TX-RX Communication === //
SoftwareSerial espSerial(A0, A1); // TX = A0, RX = A1

// === TCA9548A Multiplexer === //
void tca_select(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

//== gyro reading ==//
bool gyro_signals(uint8_t channel, uint8_t address, float &RateRoll, float &RatePitch, float &RateYaw, float &GyroX, float &GyroY, float &GyroZ) {
  tca_select(channel);
  Wire.beginTransmission(address);
  Wire.write(0x3B);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom(address, (uint8_t)6);
  if (Wire.available() < 6) return false;
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(address);
  Wire.write(0x43);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom(address, (uint8_t)6);
  if (Wire.available() < 6) return false;
  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096 - 0.05;
  AccY = (float)AccYLSB / 4096 + 0.01;
  AccZ = (float)AccZLSB / 4096 - 0.11;

  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI;
  AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;
  AngleYaw = atan2(AccZ, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;

  return true;
}

//--- GPS ---//
float calculateVerticalSpeed(float newAlt) {
  verticalSpeed_mps = (newAlt - lastAlt) * 1e6 / (micros() - lastTime);
  lastAlt = newAlt;
  lastTime = micros();
  return verticalSpeed_mps;
}

float calculateTotalVelocity() {
  return sqrt(horizontalSpeed_mps * horizontalSpeed_mps + verticalSpeed_mps * verticalSpeed_mps);
}

float calculateDistanceNorth(float lat_current, float lat_launch) {
  return (lat_current - lat_launch) * 111320.0;
}

float calculateDistanceEast(float lon_current, float lon_launch, float lat_launch) {
  return (lon_current - lon_launch) * (111320.0 * cos(radians(lat_launch)));
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371000; // Radius of Earth (meters)
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

//============= Motor Control ===============//
// === Bearing Calculation === //
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = radians(lon2 - lon1);
  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  float brng = atan2(y, x);
  brng = degrees(brng);
  if (brng < 0) brng += 360;
  return brng;
}

// === Motor Control ===
void controlMotor(float error, float &prevError, float &integral, int ENA, int IN1, int IN2) {
  integral += error;
  float derivative = error - prevError;
  float output = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;

  int pwm = constrain(abs(output), 0, 255);

  if (output > 2) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else if (output < -2) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, pwm);
}

// === Setup === //
void setup() {
  Serial.begin(57600);
  espSerial.begin(57600);
  Serial.println(F("[BOOT] Starting..."));
  Wire.begin();
  Wire.setClock(400000);

  //-- BME --//
  tca_select(1);
  bme_ok = bme.begin(0x76);
  Serial.println(F("[BME] Starting"));

  //-- GPS --//
  ss.begin(GPSBaud);
  Serial.println(F("[GPS] Starting"));

  //-- MAG --//
  tca_select(2);
  mag_ok = mag.begin();
  Serial.println(F("[MAG] Starting"));
  mag.setSamples(HMC5883L_SAMPLES_1);

  //-- IMU1 --//
  tca_select(6);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    if (!gyro_signals(6, 0x68, RateRoll1, RatePitch1, RateYaw1, GyroX1, GyroY1, GyroZ1)) { mpu1_ok = false; break; }
    RateCalibrationRoll1 += RateRoll1;
    RateCalibrationPitch1 += RatePitch1;
    RateCalibrationYaw1 += RateYaw1;
    delay(1);
  }
  if (mpu1_ok) {
    RateCalibrationRoll1 /= 2000;
    RateCalibrationPitch1 /= 2000;
    RateCalibrationYaw1 /= 2000;
  }

  //-- IMU2 --//
  tca_select(7);
  Wire.beginTransmission(0x69);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    if (!gyro_signals(7, 0x69, RateRoll2, RatePitch2, RateYaw2, GyroX2, GyroY2, GyroZ2)) { mpu2_ok = false; break; }
    RateCalibrationRoll2 += RateRoll2;
    RateCalibrationPitch2 += RatePitch2;
    RateCalibrationYaw2 += RateYaw2;
    delay(1);
  }
  if (mpu2_ok) {
    RateCalibrationRoll2 /= 2000;
    RateCalibrationPitch2 /= 2000;
    RateCalibrationYaw2 /= 2000;
  }

  pinMode(PITCH_ENA, OUTPUT);
  pinMode(PITCH_IN1, OUTPUT);
  pinMode(PITCH_IN2, OUTPUT);
  pinMode(YAW_ENA, OUTPUT);
  pinMode(YAW_IN1, OUTPUT);
  pinMode(YAW_IN2, OUTPUT);

  pinMode(CSpin, OUTPUT);

  // === SD Card ===
  if (!SD.begin(CSpin)) {
    Serial.println("[SD] fail");
  } else {
    dataFile = SD.open(FileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("lat,lon,altitude,verticalSpeed,horizontalSpeed_mps,totalVelocity,distance_north,distance_east,imu1_AccX,imu1_AccY,imu1_AccZ,imu1_AngleRoll,imu1_AnglePitch,imu2_AngleRoll,imu2_AnglePitch,bme_h,bme_t,bme_p,bme_alt");
      dataFile.close();
    }
    Serial.println("SD ready, header written.");
  }

  Serial.println(F("[SD] Starting"));

  LoopTimer = micros();
}

// === Loop === //
void loop() {
  uint32_t now_Gyro = millis();
  uint32_t dt = now_Gyro - lastTime_Gyro;
  lastTime_Gyro = now_Gyro;

  //-- BME --//
  if (bme_ok) {
    tca_select(1);
    bme_h = bme.readHumidity();
    bme_t = bme.readTemperature();
    bme_p = bme.readPressure() / 100.0F;
    bme_alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  }

  //--GPS--//
  while (ss.available()) gps.encode(ss.read());

  //-- IMU 1 --//
  if (mpu1_ok && gyro_signals(6, 0x68, RateRoll1, RatePitch1, RateYaw1, GyroX1, GyroY1, GyroZ1)) {
    RateRoll1 -= RateCalibrationRoll1;
    RatePitch1 -= RateCalibrationPitch1;
    RateYaw1 -= RateCalibrationYaw1;

    AccX_SD = AccX;
    AccY_SD = AccY;
    AccZ_SD = AccZ;

    AngleRoll_SD = AngleRoll; 
    AnglePitch_SD = AnglePitch;
    AngleYaw_SD = AngleYaw;
  }

  //-- IMU 2 --//
  if (mpu2_ok && gyro_signals(7, 0x69, RateRoll2, RatePitch2, RateYaw2, GyroX2, GyroY2, GyroZ2)) {
    RateRoll2 -= RateCalibrationRoll2;
    RatePitch2 -= RateCalibrationPitch2;
    RateYaw2 -= RateCalibrationYaw2;

    // === Read magnetometer ===
    tca_select(2);
    mag_ok = mag.begin();
    Vector norm = mag.readNormalize();
    float accYaw = atan2(norm.YAxis, norm.XAxis) * 180.0 / PI;
    if (accYaw < 0) accYaw += 360;


    fusedYaw = alpha * (fusedYaw + GyroZ2 * dt) + (1 - alpha) * accYaw;
  }

  //-- GPS --//
  float altitude = 0, verticalSpeed = 0;
  float lat = 0, lon = 0;
  float totalVelocity = 0;
  float distance_north = 0, distance_east = 0, distance_total = 0;

  //  GPS valid -> Update
  if (gps.location.isValid()) {
    altitude = gps.altitude.meters();
    verticalSpeed = calculateVerticalSpeed(altitude);
    lat = gps.location.lat();
    lon = gps.location.lng();
    horizontalSpeed_mps = gps.speed.mps();
    totalVelocity = calculateTotalVelocity();

    if (lat_launch == 0 && lon_launch == 0 && alt_launch == 0) {
      lat_launch = lat; 
      lon_launch = lon; 
      alt_launch = altitude; 
    }

    distance_north = calculateDistanceNorth(lat, lat_launch);
    distance_east = calculateDistanceEast(lon, lon_launch, lat_launch);
    distance_total = calculateDistance(lat, lon, lat_launch, lon_launch);

    // -- calculate GPS OK --
    float targetYaw = calculateBearing(lat, lon, lat_launch, lon_launch);
    yawChange = targetYaw - fusedYaw;
    if (yawChange > 180) yawChange -= 360;
    if (yawChange < -180) yawChange += 360;

    float dAlt = alt_launch - altitude;
    float pitchTarget = atan2(dAlt, distance_total) * 180.0 / PI;
    float currentPitch = AnglePitch;
    pitchChange = pitchTarget - currentPitch;
  } 
  
  // in case gps not ready, but i dont think the gps rate is that slow (5s?)
  else {
    if( micros() - lastTime > 5000){
      yawChange = 0;
      pitchChange = 0;
    }
  }

  controlMotor(yawChange, yawPrevChange, yawIntegral, YAW_ENA, YAW_IN1, YAW_IN2);
  controlMotor(pitchChange, pitchPrevChange, pitchIntegral, PITCH_ENA, PITCH_IN1, PITCH_IN2);


  //-- Log data to SD --//
  dataFile = SD.open(FileName, FILE_WRITE);
  if (dataFile) {
    dataFile.print(lat); dataFile.print(",");
    dataFile.print(lon); dataFile.print(",");
    dataFile.print(altitude); dataFile.print(",");
    dataFile.print(verticalSpeed); dataFile.print(",");
    dataFile.print(horizontalSpeed_mps); dataFile.print(",");
    dataFile.print(totalVelocity); dataFile.print(",");
    dataFile.print(distance_north); dataFile.print(",");
    dataFile.print(distance_east); dataFile.print(",");
    dataFile.print(distance_total); dataFile.print(",");
    dataFile.print(AccX_SD); dataFile.print(",");
    dataFile.print(AccY_SD); dataFile.print(",");
    dataFile.print(AccZ_SD); dataFile.print(",");
    dataFile.print(AngleRoll_SD); dataFile.print(",");
    dataFile.print(AnglePitch_SD); dataFile.print(",");
    dataFile.print(AngleYaw_SD); dataFile.print(",");
    dataFile.print(AngleRoll); dataFile.print(",");
    dataFile.print(AnglePitch); dataFile.print(",");
    dataFile.print(AngleYaw); dataFile.print(",");
    dataFile.print(bme_h); dataFile.print(",");
    dataFile.print(bme_t); dataFile.print(",");
    dataFile.print(bme_p); dataFile.print(",");
    dataFile.print(bme_alt);
    dataFile.println();
    dataFile.close();
  }

    //-- Send data to ESP32 and Serial Monitor --//
  String packet = "START|";
  packet += "lat:" + String(lat, 6) + "|";
  packet += "lon:" + String(lon, 6) + "|";
  packet += "altitude:" + String(altitude, 2) + "|";
  packet += "verticalSpeed:" + String(verticalSpeed, 2) + "|";
  packet += "horizontalSpeed:" + String(horizontalSpeed_mps, 2) + "|";
  packet += "totalVelocity:" + String(totalVelocity, 4) + "|";
  packet += "distance_north:" + String(distance_north, 4) + "|";
  packet += "distance_east:" + String(distance_east, 4) + "|";
  packet += "distance_total:" + String(distance_total, 4) + "|";
  packet += "AccX_SD:" + String(AccX_SD, 3) + "|";
  packet += "AccY_SD:" + String(AccY_SD, 3) + "|";
  packet += "AccZ_SD:" + String(AccZ_SD, 3) + "|";
  packet += "AngleRoll_SD:" + String(AngleRoll_SD, 2) + "|";
  packet += "AnglePitch_SD:" + String(AnglePitch_SD, 2) + "|";
  packet += "AngleYaw_SD:" + String(AngleYaw_SD, 2) + "|";
  packet += "AngleRoll:" + String(AngleRoll, 2) + "|";
  packet += "AnglePitch:" + String(AnglePitch, 2) + "|";
  packet += "AngleYaw:" + String(AngleYaw, 2) + "|";
  packet += "bme_h:" + String(bme_h, 2) + "|";
  packet += "bme_t:" + String(bme_t, 2) + "|";
  packet += "bme_p:" + String(bme_p, 2) + "|";
  packet += "bme_alt:" + String(bme_alt, 2) + "|END";

  Serial.println(packet);
  espSerial.println(packet);

  while (micros() - LoopTimer < 10000);
  LoopTimer = micros();
}
