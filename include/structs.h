#ifndef STRUCTS_H
#define STRUCTS_H

#include <Arduino.h>

#include "settings.h"

// ========== SENSOR DATA STRUCTURES ===================
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
  float temperature;            // °C
  float pressure;               // kPa  
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

/*
// Attitude data structure - calculated from accelerometer
struct AttitudeData {
  float pitch;    // degrees
  float roll;     // degrees
  float yaw;      // degrees (from magnetometer heading)
  static constexpr uint8_t SIZE = 12;  // 3 * sizeof(float) = 12 bytes
};
*/

// ========== Calibration ===============================
struct AccelerometerCalibration {
  float biasX, biasY, biasZ;              // Zero-g offset
  float scaleX, scaleY, scaleZ;           // Scale factors
  bool isCalibrated = false;
  static constexpr uint8_t SIZE = 25;     // 6*sizeof(float) + sizeof(bool) = 25 bytes
};

struct GyroscopeCalibration {
  float biasX, biasY, biasZ;              // Drift compensation
  bool isCalibrated = false;
  static constexpr uint8_t SIZE = 13;     // 3*sizeof(float) + sizeof(bool) = 13 bytes
};

struct MagnetometerCalibration {
  float hardIronX, hardIronY, hardIronZ;  // Hard iron distortion
  float softIronXX, softIronXY, softIronXZ; // Soft iron matrix
  float softIronYX, softIronYY, softIronYZ;
  float softIronZX, softIronZY, softIronZZ;
  bool isCalibrated = false;
  static constexpr uint8_t SIZE = 49;     // 12*sizeof(float) + sizeof(bool) = 49 bytes
};

struct EnvironmentalCalibration {
  float pressureOffset;                   // Sea level pressure adjustment
  float temperatureOffset;                // Temperature sensor offset
  bool isCalibrated = false;
  static constexpr uint8_t SIZE = 9;      // 2*sizeof(float) + sizeof(bool) = 9 bytes
};

struct SensorCalibration {
  AccelerometerCalibration accel;
  GyroscopeCalibration gyro;
  MagnetometerCalibration mag;
  EnvironmentalCalibration env;
  static constexpr uint8_t SIZE = 96;     // 25 + 13 + 49 + 9 = 96 bytes
};

// ========== Globals ====================================  
struct FlightData {
  AccelerometerData accel;         // 12 bytes
  GyroscopeData gyro;              // 12 bytes  
  MagnetometerData mag;            // 12 bytes
  EnvironmentalData env;           // 12 bytes
  GPSData gps;                     // 26 bytes
  
  // Altitude tracking
  float maxAltitude = 0;           // 4 bytes
  float launchAltitude = 0;        // 4 bytes
  
  // Timing variables for collection intervals
  uint32_t lastAccelGyroTime = 0;  // 4 bytes
  uint32_t lastMagTime = 0;        // 4 bytes
  uint32_t lastBaroTime = 0;       // 4 bytes
  uint32_t lastGPSTime = 0;        // 4 bytes
  // Total FlightData: ~98 bytes
};

struct UnifiedCollectorBuffer {
  uint8_t buffer[UNIFIED_BUFFER_SIZE];  // 16384 bytes
  uint16_t bufferIndex = 0;             // 2 bytes
  // Total buffer struct: ~16386 bytes
};

// ========== FLIGHT STATE MANAGEMENT ====================
enum FlightState {
  SENSORS_CALIBRATING,
  GROUND_IDLE,                      
  ASCENDING_NO_ENGINE,
  ASCENDING_ENGINE,
  DESCENDING,
  LANDED,
  DEBUG
};

struct FlightStateData {
  FlightState currentState;
  uint32_t startTime = 0;
#ifdef DEBUG_STATE_ENABLED
  uint32_t lastEchoTime = 0;
#endif
  unsigned long launchDetectionStart = 0;
  unsigned long engineCutoffStart = 0;
  float previousAltitude = 0;
  unsigned long lastAltitudeTime = 0;
  
  // Improved detection tracking
  uint8_t descendingCount = 0;     // Count consecutive descending readings
  uint32_t landingConfirmStart = 0; // Start time for landing confirmation
  float landingAltitude = 0;       // Altitude when landing detection started
  
  static constexpr uint8_t SIZE = sizeof(FlightState) + sizeof(uint32_t) 
#ifdef DEBUG_STATE_ENABLED
    + sizeof(uint32_t)
#endif
    + 4 * sizeof(unsigned long) + sizeof(float);  // ~29-33 bytes
};

// ========== TELEMETRY MANAGEMENT ====================
enum TelemetryType {
  ACCEL,
  GYRO, 
  MAG,
  ENV,
  GPS,
  ALL
};

#endif