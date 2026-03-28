#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_BMP280.h>

#include "DataLogger.h"
#include "Vmath.h"

static const int PIN_SCK  = 5;
static const int PIN_MISO = 7;
static const int PIN_MOSI = 6;

static const int SD_CS  = 4;
static const int BMP_CS = 3;
static const int ICM_CS = 2;

SPIClass spi;
Adafruit_ICM20649 icm;
Adafruit_BMP280 bmp(BMP_CS, &spi);

SdFs sd;
FsFile logFile;
SdSpiConfig sdCfg(SD_CS, SHARED_SPI, SD_SCK_MHZ(4), &spi);

const unsigned long TEST_DURATION_MS = 5000;
const unsigned long SAMPLE_PERIOD_MS = 10;
const unsigned long FLUSH_PERIOD_MS  = 100; 

unsigned long startTime = 0;
unsigned long lastSample = 0;
unsigned long lastFlush = 0;

bool initSDLogFile() {
  digitalWrite(BMP_CS, HIGH);
  digitalWrite(ICM_CS, HIGH);
  digitalWrite(SD_CS, HIGH);

  if (!sd.begin(sdCfg)) {
    return false;
  }

  if (!logFile.open("/testfile.txt", O_WRONLY | O_CREAT | O_TRUNC)) {
    return false;
  }

  logFile.println("timestamp,x,y,z");
  logFile.flush();
  return true;
}

bool appendAccel(unsigned long t, const Vec3& v) {
  digitalWrite(BMP_CS, HIGH);
  digitalWrite(ICM_CS, HIGH);
  digitalWrite(SD_CS, HIGH);

  char buffer[96];
  int n = v.toCSV(buffer, sizeof(buffer));
  if (n <= 0 || n >= (int)sizeof(buffer)) {
    return false;
  }

  logFile.print(t);
  logFile.print(',');
  logFile.write((const uint8_t*)buffer, n);
  logFile.println();

  return !logFile.getWriteError();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(SD_CS, OUTPUT);
  pinMode(BMP_CS, OUTPUT);
  pinMode(ICM_CS, OUTPUT);

  digitalWrite(SD_CS, HIGH);
  digitalWrite(BMP_CS, HIGH);
  digitalWrite(ICM_CS, HIGH);

  spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  delay(20);

  if (!icm.begin_SPI(ICM_CS, &spi)) {
    Serial.println("ICM failed to init");
    while (true) delay(1000);
  }

  if (!bmp.begin()) {
    Serial.println("BMP failed to init");
    while (true) delay(1000);
  }

  if (!initSDLogFile()) {
    Serial.println("SD/log file init failed");
    while (true) delay(1000);
  }

  startTime = millis();
  lastSample = startTime;
  lastFlush = startTime;

  Serial.println("Logging accel for 5 seconds...");
}

void loop() {
  unsigned long now = millis();

  if (now - startTime >= TEST_DURATION_MS) {
    logFile.flush();
    logFile.close();
    sd.end();
    Serial.println("5 second logging test complete.");
    while (true) delay(1000);
  }

  if (now - lastSample < SAMPLE_PERIOD_MS) {
    return;
  }
  lastSample = now;

  sensors_event_t accel, gyro, temp;
  icm.getEvent(&accel, &gyro, &temp);

  Vec3 accelVec(
    accel.acceleration.x,
    accel.acceleration.y,
    accel.acceleration.z
  );

  if (!appendAccel(now, accelVec)) {
    Serial.println("Failed to log accelerometer data");
  }

  if (now - lastFlush >= FLUSH_PERIOD_MS) {
    logFile.flush();
    lastFlush = now;
  }
  
}

