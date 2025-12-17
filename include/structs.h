#ifndef STRUCTS_H
#define STRUCTS_H

#include <Arduino.h>

// ========== DATA STRUCTURES ==========

// Accelerometer data structure (m/s²)
// Axis orientation: Z-axis points UP (positive Z = upward, negative Z shows gravity ~-9.8 m/s²)
struct AccelerometerData {
  float x, y, z;
  float magnitude; // sqrt(x*x + y*y + z*z)
};

// Gyroscope data structure (°/s)
// Axis orientation: Z-axis points UP (positive Z = counter-clockwise rotation when viewed from above)
struct GyroscopeData {
  float x, y, z;
  float magnitude; // sqrt(x*x + y*y + z*z)
};

// Magnetometer data structure (µT - microTesla)
// Axis orientation: Z-axis points UP (positive Z = magnetic field component pointing up)
struct MagnetometerData {
  float x, y, z;
  float magnitude; // sqrt(x*x + y*y + z*z)
  float heading;   // 0° = North, 90° = East, 180° = South, 270° = West
};

// Environmental sensor data structure
struct EnvironmentalData {
  float temperature_lps;    // °C from LPS22HB (pressure sensor)
  float pressure;           // kPa from LPS22HB
  float altitude;           // m calculated from pressure
  float temperature_hs;     // °C from HS300x (humidity sensor) 
  float humidity;           // % relative humidity from HS300x
};

// GPS data structure
struct GPSData {
  double latitude;      // degrees
  double longitude;     // degrees
  float hdop;           // horizontal dilution of precision
  uint8_t satellites;   // number of satellites
  float speed;          // km/h (ground speed)
  float course;         // degrees (0-360°, direction of travel relative to true north)
  float altitude_gps;   // altitude from GPS (meters)
  bool fix_valid;       // GPS fix status
};

// Complete flight data telemetry packet (enhanced version)
struct FlightData {
  uint32_t timestamp;              // milliseconds since boot
  uint16_t packetId;               // sequential packet counter
  AccelerometerData accel;         // acceleration data
  GyroscopeData gyro;              // angular velocity data
  MagnetometerData mag;            // magnetic field data
  EnvironmentalData env;           // environmental data (temp, pressure, humidity)
  GPSData gps;                     // GPS position and navigation data
  float battery_voltage;           // estimated battery voltage (if available)
};

// Gesture enumeration for clarity
enum GestureType {
  GESTURE_NONE = -1,
  GESTURE_UP = 0,
  GESTURE_DOWN = 1,
  GESTURE_LEFT = 2,
  GESTURE_RIGHT = 3
};

#endif