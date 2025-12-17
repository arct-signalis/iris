// IRIS2 Flight Controller - Enhanced Sensor Telemetry System
// Arduino Nano 33 BLE Sense Rev2 - All-in-One Sensor Suite

#include <Arduino.h>
#include "Arduino_BMI270_BMM150.h"
#include "Arduino_LPS22HB.h"
#include "Arduino_HS300x.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// Include modular headers
#include "settings.h"
#include "structs.h"
#include "sensors.h"
#include "telemetry.h"

// Global sensor instances (extern declarations in sensors.h)
BoschSensorClass IMU;
bool imuInitialized = false;
bool pressureInitialized = false;
bool humidityInitialized = false;
bool gpsInitialized = false;

// GPS instances
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

// Global telemetry variables
FlightData telemetryData;
uint16_t packetCounter = 0;
unsigned long lastSensorUpdate = 0;
unsigned long lastSlowSensorUpdate = 0;

// Telemetry mode (can be changed via serial commands)
TelemetryMode currentTelemetryMode = TELEMETRY_HUMAN_READABLE;

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {
    delay(10); // Wait for serial connection
  }
  
  Serial.println("IRIS2 Flight Controller Starting...");
  Serial.println("Arduino Nano 33 BLE Sense Rev2");
  
  // Initialize all sensors
  if (!initializeSensors()) {
    Serial.println("Critical sensor initialization failure!");
    Serial.println("System will continue with available sensors...");
  }
  
  // Print sensor status
  printSensorStatus();
  
#ifdef ENABLE_SENSOR_CALIBRATION
  Serial.println("Performing sensor calibration...");
  delay(2000); // Give user time to read message
  // Note: Individual sensor libraries handle their own calibration
  Serial.println("Calibration complete (handled by sensor libraries)");
#endif
  
  // Print CSV header if in CSV mode
  if (currentTelemetryMode == TELEMETRY_CSV) {
    printCSVHeader();
  }
  
  // System ready message
  printSystemReady();
  
  Serial.println("Starting telemetry in 3 seconds...");
  delay(3000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Process GPS data continuously
  processGPSData();
  
  // Process serial commands (non-blocking)
  processSerialCommands();
  
  // Update fast sensors at regular intervals
  if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL_MS) {
    updateFastSensors();
    lastSensorUpdate = currentTime;
  }
  
  // Update slow sensors less frequently
  if (currentTime - lastSlowSensorUpdate >= SLOW_SENSOR_INTERVAL_MS) {
    updateSlowSensors();
    lastSlowSensorUpdate = currentTime;
  }
  
  // Send telemetry data
  if (currentTime % TELEMETRY_LOOP_DELAY_MS == 0) {
    telemetryData = collectTelemetryData();
    telemetryData.packetId = packetCounter++;
    
#ifdef ENABLE_SERIAL_OUTPUT
    String telemetryOutput = formatTelemetryByMode(telemetryData, currentTelemetryMode);
    Serial.println(telemetryOutput);
#endif
  }
  
  delay(1); // Small delay to prevent overwhelming the system
}

// ========== SENSOR UPDATE FUNCTIONS ==========

void updateFastSensors() {
  // Update IMU data (fast sensors - 104Hz capable)
  if (imuInitialized) {
    // IMU data is read in collectTelemetryData when needed
    // This function could be used for filtering or preprocessing
  }
}

void updateSlowSensors() {
  // Update humidity sensor (slow - 1Hz)
  // Pressure sensor can also be updated here if needed
  // Data is read in collectTelemetryData when needed
}

// ========== SERIAL COMMAND PROCESSING ==========

void processSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "help" || command == "?") {
      printHelpMenu();
    }
    else if (command == "status") {
      printSensorStatus();
    }
    else if (command == "mode human" || command == "mode h") {
      currentTelemetryMode = TELEMETRY_HUMAN_READABLE;
      Serial.println("Telemetry mode: Human Readable");
    }
    else if (command == "mode csv" || command == "mode c") {
      currentTelemetryMode = TELEMETRY_CSV;
      Serial.println("Telemetry mode: CSV");
      printCSVHeader();
    }
    else if (command == "mode json" || command == "mode j") {
      currentTelemetryMode = TELEMETRY_JSON;
      Serial.println("Telemetry mode: JSON");
    }
    else if (command == "test") {
      runSensorTest();
    }
    else if (command == "info") {
      printSystemInfo();
    }
    else if (command != "") {
      Serial.println("Unknown command: " + command);
      Serial.println("Type 'help' for available commands");
    }
  }
}

void printHelpMenu() {
  Serial.println("\n=== IRIS2 Commands ===");
  Serial.println("help, ?      - Show this menu");
  Serial.println("status       - Show sensor status");
  Serial.println("info         - Show system information");
  Serial.println("test         - Run sensor test");
  Serial.println("mode human   - Human readable telemetry");
  Serial.println("mode csv     - CSV format telemetry");
  Serial.println("mode json    - JSON format telemetry");
  Serial.println("=====================\n");
}

void runSensorTest() {
  Serial.println("\n=== Sensor Test ===");
  
  // Test each sensor individually
  Serial.println("Testing IMU...");
  AccelerometerData accel = readAccelerometer();
  GyroscopeData gyro = readGyroscope();
  MagnetometerData mag = readMagnetometer();
  
  Serial.println("Accel: " + String(accel.magnitude, 2) + " m/s²");
  Serial.println("Gyro: " + String(gyro.magnitude, 2) + " °/s");
  Serial.println("Mag: " + String(mag.magnitude, 2) + " µT, Heading: " + String(mag.heading, 1) + "°");
  
  Serial.println("Testing Environmental sensors...");
  EnvironmentalData env = readEnvironmental();
  Serial.println("Temp (LPS): " + String(env.temperature_lps, 1) + "°C");
  Serial.println("Temp (HS): " + String(env.temperature_hs, 1) + "°C");
  Serial.println("Pressure: " + String(env.pressure, 1) + " kPa");
  Serial.println("Humidity: " + String(env.humidity, 1) + "%");
  
  Serial.println("Testing GPS...");
  GPSData gpsData = readGPS();
  if (gpsData.fix_valid) {
    Serial.println("GPS Fix: VALID");
    Serial.println("Location: " + String(gpsData.latitude, 6) + ", " + String(gpsData.longitude, 6));
    Serial.println("Altitude: " + String(gpsData.altitude_gps, 1) + "m");
    Serial.println("Speed: " + String(gpsData.speed, 1) + " km/h");
  } else {
    Serial.println("GPS Fix: SEARCHING...");
    Serial.println("Satellites: " + String(gpsData.satellites));
  }
  
  Serial.println("=== Test Complete ===\n");
}

void printSystemInfo() {
  Serial.println("\n=== System Information ===");
  Serial.println("Device: Arduino Nano 33 BLE Sense Rev2");
  Serial.println("Firmware: IRIS2 Flight Controller");
  Serial.println("Compile Date: " + String(__DATE__) + " " + String(__TIME__));
  Serial.println("Uptime: " + String(millis() / 1000) + " seconds");
  Serial.println("Packet Count: " + String(packetCounter));
  Serial.println("Telemetry Mode: " + String(currentTelemetryMode));
  Serial.println("Available Sensors:");
  Serial.println("  • 9-axis IMU (BMI270 + BMM150)");
  Serial.println("  • Barometric Pressure (LPS22HB)"); 
  Serial.println("  • Temperature & Humidity (HS300x)");
  Serial.println("  • GPS Module (External)");
  Serial.println("==========================\n");
}