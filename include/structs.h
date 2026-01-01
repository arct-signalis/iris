#ifndef STRUCTS_H
#define STRUCTS_H

#include <Arduino.h>

// ========== DATA STRUCTURES ==========

// Accelerometer data structure (g) - BMI270 returns float
struct AccelerometerData {
  float x, y, z;
  static constexpr uint8_t SIZE = 12;  // 3 * sizeof(float) = 12 bytes
  // float gForce = totalAcceleration = sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z); // g
  // vx = ax * t; (you can find out position sx += vx * t)
};

// Gyroscope data structure (°/s) - BMI270 returns float  
struct GyroscopeData {
  float x, y, z;
  static constexpr uint8_t SIZE = 12;  // 3 * sizeof(float) = 12 bytes
  // float angularRate = sqrt(gyro.x*gyro.x + gyro.y*gyro.y + gyro.z*gyro.z); °/s
};

// Magnetometer data structure (µT) - BMM150 returns float
struct MagnetometerData {
  float x, y, z;
  static constexpr uint8_t SIZE = 12;  // 3 * sizeof(float) = 12 bytes
  // float magneticFieldStrength = sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z); // µT
};

// Environmental sensor data structure - LPS22HB returns float
struct EnvironmentalData {
  float temperature;    // °C
  float pressure;       // kPa  
  float pressureAltitude;       // m
  static constexpr uint8_t SIZE = 12;  // 3 * sizeof(float) = 12 bytes
  // float airDensity = (env.pressure * 100.0f) / (DRY_AIR_GAS_CONSTANT * (env.temperature + CELSIUS_TO_KELVIN)); // in kg/m³
};

// GPS data structure - TinyGPS++ returns double for lat/lng, others float
struct GPSData {
  double latitude;      // degrees - TinyGPS++ uses double for precision
  double longitude;     // degrees - TinyGPS++ uses double for precision  
  float hdop;           // horizontal dilution of precision
  uint16_t satellites;  // number of satellites (0-255 would fit in uint8_t but uint16_t for safety)
  float speed;          // km/h
  float course;         // degrees (0-360°)
  static constexpr uint8_t SIZE = 26;  // 2*8 + 3*4 + 2 = 26 bytes
};

// Complete flight data telemetry packet
struct FlightData {
  AccelerometerData accel;         // 12 bytes
  GyroscopeData gyro;              // 12 bytes  
  MagnetometerData mag;            // 12 bytes
  EnvironmentalData env;           // 12 bytes
  GPSData gps;                     // 26 bytes
  
  // Timing variables for collection intervals
  uint32_t lastAccelGyroTime = 0;  // 4 bytes
  uint32_t lastMagTime = 0;        // 4 bytes
  uint32_t lastBaroTime = 0;       // 4 bytes
  uint32_t lastGPSTime = 0;        // 4 bytes
  // Total FlightData: ~90 bytes
};

struct UnifiedCollectorBuffer {
  uint8_t buffer[UNIFIED_BUFFER_SIZE];  // 16384 bytes
  uint16_t bufferIndex = 0;             // 2 bytes
  // Total buffer struct: ~16386 bytes
};

/*
// Attitude data structure - calculated from accelerometer
struct AttitudeData {
  float pitch;    // degrees
  float roll;     // degrees
  float yaw;      // degrees (from magnetometer heading)
  static constexpr uint8_t SIZE = 12;  // 3 * sizeof(float) = 12 bytes
};
*/

#endif