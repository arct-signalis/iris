#ifndef STRUCTS_H
#define STRUCTS_H

#include <Arduino.h>

// ========== DATA STRUCTURES ==========

// Accelerometer data structure (m/s²)
// Axis orientation: Z-axis points UP (positive Z = upward, negative Z shows gravity ~-9.8 m/s²)
struct AccelerometerData {
  float x, y, z;
  // Acceleration magnitude can be calculated: sqrt(x*x + y*y + z*z)
};

// Gyroscope data structure (°/s)
// Axis orientation: Z-axis points UP (positive Z = counter-clockwise rotation when viewed from above)
struct GyroscopeData {
  float x, y, z;
  // Angular velocity magnitude can be calculated: sqrt(x*x + y*y + z*z)
};

// Magnetometer data structure (mG)
// Axis orientation: Z-axis points UP (positive Z = magnetic field component pointing up)
struct MagnetometerData {
  float x, y, z;
  float heading; // 0° = North, 90° = East, 180° = South, 270° = West
};

// Environmental sensor data structure
struct EnvironmentalData {
  float temperature;    // °C
  float pressure;       // Pa
  float altitude;       // m
};

// GPS data structure
struct GPSData {
  double latitude;      // degrees
  double longitude;     // degrees
  float hdop;           // horizontal dilution of precision
  uint8_t satellites;   // number of satellites
  float speed;          // km/h (ground speed - similar to magnitude of acceleration but for velocity)
  float course;         // degrees (0-360°, direction of travel relative to true north)
                        // Note: course is GPS movement direction, similar to magnetometer heading but based on actual travel
};

// Complete flight data telemetry packet
struct FlightData {
  uint32_t timestamp;          // milliseconds since boot
  uint16_t packetId;           // sequential packet counter
  AccelerometerData accel;     // acceleration data
  GyroscopeData gyro;          // angular velocity data
  MagnetometerData mag;        // magnetic field data
  EnvironmentalData env;       // environmental data
  GPSData gps;                 // GPS position data
};

#endif