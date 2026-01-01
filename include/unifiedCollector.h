#ifndef UNIFIED_COLLECTOR_H
#define UNIFIED_COLLECTOR_H

#include <Arduino.h>
#include "structs.h"
#include "settings.h"
#include "sensors.h"
#include "sdCards.h"

extern UnifiedCollectorBuffer collectorBuffer;
extern FlightData flightData;

void flushBuffer(uint8_t card = SD_CS_2) {
  if (!writeBufferToSD(collectorBuffer.buffer, collectorBuffer.bufferIndex, card)) {
    Serial.println("Warning: SD Card write failed");
  }
  
  collectorBuffer.bufferIndex = 0;
}

void writeToBuffer(uint8_t sensorType, const void* data, uint8_t dataSize, uint32_t timestamp) {
  // Check if we have enough space (1 byte type + 4 bytes timestamp + dataSize)
  if (collectorBuffer.bufferIndex + 1 + 4 + dataSize > UNIFIED_BUFFER_SIZE) {
    flushBuffer();  // Auto-flush when buffer is full
  }

  // sensorType
  collectorBuffer.buffer[collectorBuffer.bufferIndex++] = sensorType;

  // timestamp
  memcpy(&collectorBuffer.buffer[collectorBuffer.bufferIndex], &timestamp, 4);
  collectorBuffer.bufferIndex += 4;

  // data
  memcpy(&collectorBuffer.buffer[collectorBuffer.bufferIndex], data, dataSize);
  collectorBuffer.bufferIndex += dataSize;
}

void updateSensors() {
  uint32_t timestamp = millis();

  // Accelerometer and Gyroscope at 100Hz
  if (timestamp - flightData.lastAccelGyroTime >= ACCEL_GYRO_INTERVAL) {
    if (readAccelerometer(flightData.accel)) {
      writeToBuffer(SENSOR_ACCEL, &flightData.accel, AccelerometerData::SIZE, timestamp);
    }
    if (readGyroscope(flightData.gyro)) {
      writeToBuffer(SENSOR_GYRO, &flightData.gyro, GyroscopeData::SIZE, timestamp);
    }
    flightData.lastAccelGyroTime = timestamp;
  }

  // Magnetometer
  if (timestamp - flightData.lastMagTime >= MAG_INTERVAL) {
    if (readMagnetometer(flightData.mag)) {
      writeToBuffer(SENSOR_MAG, &flightData.mag, MagnetometerData::SIZE, timestamp);
    }
    flightData.lastMagTime = timestamp;
  }

  // Barometer
  if (timestamp - flightData.lastBaroTime >= BARO_INTERVAL) {
    if (readEnvironmental(flightData.env)) {
      writeToBuffer(SENSOR_BARO, &flightData.env, EnvironmentalData::SIZE, timestamp);
    }
    flightData.lastBaroTime = timestamp;
  }

  // GPS
  if (timestamp - flightData.lastGPSTime >= GPS_INTERVAL) {
    timestamp = millis();
    processGPSData();
    if (readGPS(flightData.gps)) {
      writeToBuffer(SENSOR_GPS, &flightData.gps, GPSData::SIZE, timestamp);
    }
    flightData.lastGPSTime = timestamp;
  }
}

#endif