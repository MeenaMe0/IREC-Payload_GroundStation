#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <TinyGPS++.h>

// ---------- BME280 ---------- //
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

// ---------- GPS ---------- //
HardwareSerial gpsSerial(2);  // Use UART2
TinyGPSPlus gps;
#define GPS_RX 16
#define GPS_TX 17

// ---------- MPU6050 ---------- //
float RateRoll, RatePitch;
float RateCalibrationRoll = 0, RateCalibrationPitch = 0;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;

//--- Filter for MPU ---//
void kalman_1d(float &state, float &uncertainty, float input, float measurement) {
  state += 0.004 * input;
  uncertainty += 0.004 * 0.004 * 16;
  float gain = uncertainty / (uncertainty + 9);
  state += gain * (measurement - state);
  uncertainty *= (1 - gain);
}

//-- Read the sensor directly due to the repetitive fuctions defined in Adafruits Sensor Library --//
void readMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  RateRoll = gx / 65.5 - RateCalibrationRoll;
  RatePitch = gy / 65.5 - RateCalibrationPitch;

  AccX = ax / 4096.0;
  AccY = ay / 4096.0;
  AccZ = az / 4096.0;

  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);
  AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
}

// ---------- SD + LoRa SPI Pins ---------- //
// --- Can't be use simultaneously due to the use of SPI --- //
#define HSPI_SCK 14
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_CS 15

#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DI0 26
#define LORA_BAND 868000000 //------- Band use -------//

SPIClass *hspi = NULL;
bool hspiInitialized = false;
bool useSD = true;
int counter = 0;
const int switchInterval = 30;

void setup() {
  Serial.begin(115200);
  delay(2000);

  // GPS Init
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Wire.begin();

  // MPU6050 Calibration
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (int i = 0; i < 1000; i++) {
    readMPU6050();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    delay(1);
  }
  RateCalibrationRoll /= 1000;
  RateCalibrationPitch /= 1000;

  // BME280 Init
  if (!bme.begin(0x76)) {
    Serial.println("BME280 not found!");
    while (1);
  }
}

void loop() {
  // Read all data
  float temp = bme.readTemperature();
  float press = bme.readPressure() / 100.0F;
  float alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humi = bme.readHumidity();

  readMPU6050();
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  float lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  float lon = gps.location.isValid() ? gps.location.lng() : 0.0;
  
  int hour = gps.time.isValid() ? gps.time.hour() : -1;
  int minute = gps.time.isValid() ? gps.time.minute() : -1;
  int second = gps.time.isValid() ? gps.time.second() : -1;

  // pack for LoRa
  char dataString[128];
  snprintf(dataString, sizeof(dataString),
         "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%02d:%02d:%02d",
         temp, humi, press, alt,
         KalmanAngleRoll, KalmanAnglePitch,
         lat, lon,
         hour + 7, minute, second);

  // In case switch to SD
  if (useSD) {
    Serial.println("Writing to SD...");
    LoRa.idle();  // Pause LoRa

    if (!hspi) hspi = new SPIClass(HSPI);
    if (!hspiInitialized) {
      hspi->begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
      hspiInitialized = true;
    }

    if (!SD.begin(HSPI_CS, *hspi)) {
      Serial.println("SD init failed!");
    } else {
      writeFile("/test_lora.csv", dataString);
      SD.end();
    }

    hspi->end();
    hspiInitialized = false;
  } else {
    Serial.println("Sending over LoRa...");
    // LoRa Init (VSPI)
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);
    if (!LoRa.begin(LORA_BAND)) {
      Serial.println("LoRa failed");
      // while (1)
      //   ;
    }
    LoRa.beginPacket();
    LoRa.print(dataString);
    if (LoRa.endPacket() == 1) {
      Serial.println("LoRa sent.");
    } else {
      Serial.println("LoRa failed to send!");
    }
    LoRa.end();
  }

  counter++;
  if (counter >= switchInterval) {
    counter = 0;
    useSD = !useSD;
    Serial.printf("Switching to %s mode\n", useSD ? "SD" : "LoRa");
  }

  delay(500);
}

//--- Function for Micro SD ---//
void writeFile(const char *path, const char *message) {
  bool newFile = !SD.exists(path);  // Checking 1st use

  File file = SD.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  if (newFile) {
    file.println("Temp,Humidity,Pressure,Altitude,Roll,Pitch,Lat,Lon,Time");  // write header for 1st use
  }

  if (file.println(message)) {
    Serial.printf("Wrote to file: %s\n", path);
  } else {
    Serial.println("Write failed");
  }

  file.close();
}

void readFile(const char *path) {
  Serial.printf("Reading file: %s\n", path);
  File file = SD.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}
