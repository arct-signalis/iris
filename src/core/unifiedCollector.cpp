#include <string.h>
#include "unifiedCollector.h"
#include "structs.h"
#include "sensors.h"
#include "sdCards.h"
#include "states.h"
#include "utils.h"

void flushBuffer(uint8_t card)
{
  if (!writeBufferToSD(collectorBuffer.buffer, collectorBuffer.bufferIndex, card))
    Serial.println("Warning: SD Card write failed");

  collectorBuffer.bufferIndex = 0;
}

void writeToBuffer(uint8_t sensorType, const void *data, uint8_t dataSize, uint32_t timestamp)
{
  // Check if we have enough space
  if (collectorBuffer.bufferIndex + sizeof(sensorType) + sizeof(timestamp) + dataSize > UNIFIED_BUFFER_SIZE)
    flushBuffer(); // Auto-flush when buffer is full

  // sensorType
  collectorBuffer.buffer[collectorBuffer.bufferIndex++] = sensorType;

  // timestamp
  memcpy(&collectorBuffer.buffer[collectorBuffer.bufferIndex], &timestamp, sizeof(timestamp));
  collectorBuffer.bufferIndex += sizeof(timestamp);

  // data
  memcpy(&collectorBuffer.buffer[collectorBuffer.bufferIndex], data, dataSize);
  collectorBuffer.bufferIndex += dataSize;
}

void updateSensors()
{
  uint32_t timestamp = GET_TIME_MS();

  // Read all sensors - they self-regulate through availability checks
  if (readAccelerometer(&flightData.accel))
    writeToBuffer(SENSOR_ACCEL, &flightData.accel, sizeof(AccelerometerData), timestamp);

  if (readGyroscope(&flightData.gyro))
    writeToBuffer(SENSOR_GYRO, &flightData.gyro, sizeof(GyroscopeData), timestamp);

  if (readEnvironmental(&flightData.env))
  {
    calculateAttitude(&flightData.attitude, flightData.accel, flightData.mag);
    writeToBuffer(SENSOR_BARO, &flightData.env, sizeof(EnvironmentalData), timestamp);
    writeToBuffer(SENSOR_ATTITUDE, &flightData.attitude, sizeof(AttitudeData), timestamp);
  }

  if (readMagnetometer(&flightData.mag))
    writeToBuffer(SENSOR_MAG, &flightData.mag, sizeof(MagnetometerData), timestamp);

  processGPSData();
  if (readGPS(&flightData.gps))
    writeToBuffer(SENSOR_GPS, &flightData.gps, sizeof(GPSData), timestamp);
}