#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "Arduino_BMI270_BMM150.h"  // IMU library
#include "Arduino_LPS22HB.h"        // Pressure sensor library
#include "Arduino_HS300x.h"         // Humidity sensor library
#include <TinyGPSPlus.h>            // GPS library
#include <SoftwareSerial.h>         // Software serial for GPS
#include "structs.h"
#include "settings.h"

// Global sensor instances
extern BoschSensorClass IMU;       // BMI270 + BMM150 IMU
extern bool imuInitialized;
extern bool pressureInitialized;
extern bool humidityInitialized;
extern bool gpsInitialized;

// GPS instances
extern TinyGPSPlus gps;
extern SoftwareSerial gpsSerial;

// ========== SENSOR INITIALIZATION ==========

bool initializeIMU() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    return false;
  }
  Serial.println("IMU (BMI270 + BMM150) initialized");
  return true;
}

bool initializePressureSensor() {
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    return false;
  }
  Serial.println("Pressure sensor (LPS22HB) initialized");
  return true;
}

bool initializeHumiditySensor() {
  if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity sensor!");
    return false;
  }
  Serial.println("Humidity sensor (HS300x) initialized");
  return true;
}

bool initializeGPS() {
#ifdef ENABLE_GPS
  pinMode(GPS_PPS_PIN, INPUT);
  gpsSerial.begin(GPS_BAUD_RATE);
  Serial.println("GPS module initialized");
  Serial.println("Waiting for GPS fix...");
  return true;
#else
  Serial.println("GPS disabled in settings");
  return false;
#endif
}

bool initializeSensors() {
  Serial.println("Initializing sensors...");
  
  imuInitialized = initializeIMU();
  pressureInitialized = initializePressureSensor();
  humidityInitialized = initializeHumiditySensor();
  gpsInitialized = initializeGPS();
  
  // At minimum, we need the IMU to work
  if (!imuInitialized) {
    Serial.println("Critical: IMU initialization failed!");
    return false;
  }
  
  Serial.println("Sensor initialization complete");
  return true;
}

// ========== SENSOR READING FUNCTIONS ==========

AccelerometerData readAccelerometer() {
  AccelerometerData data = {0};
  
  if (imuInitialized && IMU.accelerationAvailable()) {
    IMU.readAcceleration(data.x, data.y, data.z);
    
    // Convert from g to m/s²
    data.x *= EARTH_GRAVITY_MS2;
    data.y *= EARTH_GRAVITY_MS2;
    data.z *= EARTH_GRAVITY_MS2;
    
    // Calculate magnitude
    data.magnitude = sqrt(data.x * data.x + data.y * data.y + data.z * data.z);
  }
  
  return data;
}

GyroscopeData readGyroscope() {
  GyroscopeData data = {0};
  
  if (imuInitialized && IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(data.x, data.y, data.z);
    
    // Calculate magnitude
    data.magnitude = sqrt(data.x * data.x + data.y * data.y + data.z * data.z);
  }
  
  return data;
}

MagnetometerData readMagnetometer() {
  MagnetometerData data = {0};
  
  if (imuInitialized && IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(data.x, data.y, data.z);
    
    // Calculate magnitude
    data.magnitude = sqrt(data.x * data.x + data.y * data.y + data.z * data.z);
    
    // Calculate heading (simple 2D compass)
    // Note: This is a basic calculation - for better accuracy, use tilt compensation
    data.heading = atan2(data.y, data.x) * 180.0 / PI;
    if (data.heading < 0) {
      data.heading += 360.0;
    }
  }
  
  return data;
}

EnvironmentalData readEnvironmental() {
  EnvironmentalData data = {0};
  
  // Read pressure sensor data
  if (pressureInitialized) {
    data.temperature_lps = BARO.readTemperature();
    data.pressure = BARO.readPressure(); // Returns kPa
    
    // Calculate altitude from pressure (standard atmosphere model)
    data.altitude = 44330.0 * (1.0 - pow(data.pressure / STANDARD_SEA_LEVEL_PRESSURE_KPA, 0.1903));
  }
  
  // Read humidity sensor data  
  if (humidityInitialized) {
    data.temperature_hs = HS300x.readTemperature();
    data.humidity = HS300x.readHumidity();
  }
  
  return data;
}

GPSData readGPS() {
  GPSData data = {0};
  
  if (!gpsInitialized) {
    return data;
  }
  
  data.latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
  data.longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
  data.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0;
  data.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
  data.speed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  data.course = gps.course.isValid() ? gps.course.deg() : 0.0;
  data.altitude_gps = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  data.fix_valid = gps.location.isValid();
  
  return data;
}

// Process incoming GPS data
void processGPSData() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // GPS data successfully parsed - ready for use in telemetry
    }
  }
}

// ========== COMPLETE DATA COLLECTION ==========

FlightData collectTelemetryData() {
  FlightData data;
  
  data.timestamp = millis();
  data.packetId = 0; // Will be set by caller
  
  data.accel = readAccelerometer();
  data.gyro = readGyroscope();
  data.mag = readMagnetometer();
  data.env = readEnvironmental();
  data.gps = readGPS();
  
  // Estimate battery voltage (placeholder - Nano 33 BLE doesn't have built-in battery monitoring)
  data.battery_voltage = 3.3; // Assume 3.3V operation
  
  return data;
}

// ========== SENSOR STATUS FUNCTIONS ==========

void printSensorStatus() {
  Serial.println("=== Sensor Status ===");
  Serial.println("IMU (BMI270+BMM150): " + String(imuInitialized ? "OK" : "FAILED"));
  Serial.println("Pressure (LPS22HB): " + String(pressureInitialized ? "OK" : "FAILED"));
  Serial.println("Humidity (HS300x): " + String(humidityInitialized ? "OK" : "FAILED"));
  Serial.println("GPS Module: " + String(gpsInitialized ? "OK" : "FAILED"));
  if (gpsInitialized && gps.satellites.isValid()) {
    Serial.println("  GPS Fix: " + String(gps.location.isValid() ? "VALID" : "SEARCHING..."));
    Serial.println("  Satellites: " + String(gps.satellites.value()));
  }
  Serial.println("====================");
}

#endif