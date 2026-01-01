#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include "structs.h"
#include "settings.h"

// ========== TELEMETRY FORMATTING FUNCTIONS ==========

String formatTelemetry(const FlightData& data) {
  String output = "";
  
  // Header with timestamp and packet info
  output += "=== IRIS Flight Telemetry ===\n";
  //output += "Time: " + String(data.timestamp) + "ms | Packet: #" + String(data.packetId) + "\n";
  
  // Accelerometer data (m/s2)
  output += "Accelerometer: ";
  output += "X=" + String(data.accel.x, 2) + " Y=" + String(data.accel.y, 2) + " Z=" + String(data.accel.z, 2) + " m/s2";
  output += "\n";
  
  // Gyroscope data (°/s)
  output += "Gyroscope: ";
  output += "X=" + String(data.gyro.x, 2) + " Y=" + String(data.gyro.y, 2) + " Z=" + String(data.gyro.z, 2) + " °/s";
  output += "\n";
  
  // Magnetometer data (mG) + Heading
  output += "Magnetometer: ";
  output += "X=" + String(data.mag.x, 1) + " Y=" + String(data.mag.y, 1) + " Z=" + String(data.mag.z, 1) + " mG";
  output += "\n";
  
  // Environmental data
  output += "Environment: ";
  output += "T=" + String(data.env.temperature, 1) + "C P=" + String(data.env.pressure, 0) + "Pa Alt=" + String(data.env.pressureAltitude, 1) + "m";
  output += "\n";
  
  // GPS data
  output += "GPS: ";
  if (data.gps.latitude != 0.0 || data.gps.longitude != 0.0) {
    output += "Lat=" + String(data.gps.latitude, 6) + " Lon=" + String(data.gps.longitude, 6);
    output += " Sat=" + String(data.gps.satellites) + " HDOP=" + String(data.gps.hdop, 2);
    if (data.gps.speed > 0.1) {
      output += " Spd=" + String(data.gps.speed, 1) + "km/h Course=" + String(data.gps.course, 1) + "°";
    }
  } else {
    output += "NO FIX | Satellites=" + String(data.gps.satellites);
  }
  output += "\n";
  
  output += "================================\n";
  
  return output;
}

enum TelemetryType {
  ACCEL,
  GYRO, 
  MAG,
  ENV,
  GPS,
  ALL
};

String formatTestTelemetry(const FlightData& data, TelemetryType type = ALL) {
  String output = "=== TEST TELEMETRY PACKET ===\n";
  
  if (type == ACCEL || type == ALL) {
    output += String(data.accel.x, 2) + "\n";
    output += String(data.accel.y, 2) + "\n"; 
    output += String(data.accel.z, 2) + "\n";
  }
  
  if (type == GYRO || type == ALL) {
    output += String(data.gyro.x, 2) + "\n";
    output += String(data.gyro.y, 2) + "\n";
    output += String(data.gyro.z, 2) + "\n";
  }
  
  if (type == MAG || type == ALL) {
    output += String(data.mag.x, 1) + "\n";
    output += String(data.mag.y, 1) + "\n";
    output += String(data.mag.z, 1) + "\n";
  }
  
  if (type == ENV || type == ALL) {
    output += String(data.env.temperature, 1) + "\n";
    output += String(data.env.pressure, 0) + "\n";
    output += String(data.env.pressureAltitude, 1) + "\n";
  }
  
  if (type == GPS || type == ALL) {
    output += String(data.gps.latitude, 6) + "\n";
    output += String(data.gps.longitude, 6) + "\n";
    output += String(data.gps.speed, 1) + "\n";
    output += String(data.gps.course, 1) + "\n";
  }
  
  output += "=============================\n";
  return output;
}

String formatCompactTelemetry(const FlightData& data) {
  String output = "";
  
  // CSV-style compact format: timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,heading,temp,pressure,lat,lon,satellites
  //output += String(data.timestamp) + ",";
  
  // Accelerometer
  output += String(data.accel.x, 2) + "," + String(data.accel.y, 2) + "," + String(data.accel.z, 2) + ",";
  
  // Gyroscope
  output += String(data.gyro.x, 2) + "," + String(data.gyro.y, 2) + "," + String(data.gyro.z, 2) + ",";
  
  // Magnetometer
  output += String(data.mag.x, 1) + "," + String(data.mag.y, 1) + "," + String(data.mag.z, 1) + ",";
  
  // Environmental
  output += String(data.env.temperature, 1) + "," + String(data.env.pressure, 1) + ",";
  
  // GPS
  if (data.gps.latitude != 0.0 || data.gps.longitude != 0.0) {
    output += String(data.gps.latitude, 6) + "," + String(data.gps.longitude, 6) + "," + String(data.gps.satellites);
  } else {
    output += "0.000000,0.000000," + String(data.gps.satellites);
  }
  
  return output;
}

#endif