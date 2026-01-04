#ifndef STRUCTS_H
#define STRUCTS_H

#include <Arduino.h>
#include "config.h"

// ========== SENSOR DATA STRUCTURES ===================
// Accelerometer data structure (g) - BMI270 returns float
struct AccelerometerData
{
  float x, y, z;
  float totalAcceleration;
};

// Attitude data structure (degrees) - calculated from accel + mag
struct AttitudeData
{
  float pitch;
  float roll;
  float yaw;     // heading
  float offVert; // degrees off vertical
};

// Gyroscope data structure (°/s) - BMI270 returns float
struct GyroscopeData
{
  float x, y, z;
  float angularRate;
};

// Magnetometer data structure (µT) - BMM150 returns float
struct MagnetometerData
{
  float x, y, z;
  float magneticFieldStrength;
};

// Environmental sensor data structure - LPS22HB returns float
struct EnvironmentalData
{
  float temperature;            // °C
  float pressure;               // kPa
  float altitudeAboveLaunchPad; // m (relative to launch pad)
  float airDensity;             // kg/m³
};

// GPS data structure - TinyGPS++ returns double for lat/lng, others float
struct GPSData
{
  float latitude;     // degrees (converted from double)
  float longitude;    // degrees (converted from double)
  float hdop;         // horizontal dilution of precision
  float speed;        // km/h
  float course;       // degrees
  uint8_t satellites; // number of satellites
};

// ========== GLOBALS ====================================
struct FlightData
{
  AccelerometerData accel;
  GyroscopeData gyro;
  MagnetometerData mag;
  EnvironmentalData env;
  GPSData gps;
  AttitudeData attitude;

  // Altitude tracking
  float maxAltitude = 0;
  float launchAltitude = 0;
  float maxTotalAcceleration = 0;
};

struct UnifiedCollectorBuffer
{
  uint8_t buffer[UNIFIED_BUFFER_SIZE];
  uint16_t bufferIndex;
};

#endif