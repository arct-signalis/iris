#ifndef SD_CARDS_H
#define SD_CARDS_H

#include <Arduino.h>
#include <SD.h>

#include "config.h"
#include "structs.h"

struct CSVRow
{
  uint32_t timestamp;
  bool hasAccel, hasGyro, hasMag, hasAttitude, hasBaro, hasGPS;
  AccelerometerData accel;
  GyroscopeData gyro;
  MagnetometerData mag;
  AttitudeData attitude;
  EnvironmentalData env;
  GPSData gps;

  CSVRow() : timestamp(0), hasAccel(false), hasGyro(false), hasMag(false),
             hasAttitude(false), hasBaro(false), hasGPS(false) {}
};

bool initSDCard(uint8_t chipSelect);
bool deleteFile(const char* filename, uint8_t chipSelect);
void initCSVFile(uint8_t chipSelect);
void testSDCard(uint8_t card);
bool writeBufferToSD(const uint8_t *buffer, uint16_t bufferSize, uint8_t chipSelect);
bool writeBufferToCSV(const uint8_t *buffer, uint16_t bufferSize, uint8_t chipSelect);
void readBufferFromSD(uint8_t chipSelect);
void readFile(const char* filename, uint8_t chipSelect);

#endif