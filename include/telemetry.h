#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include "structs.h"
#include "settings.h"

// ========== TELEMETRY FORMATTING FUNCTIONS ==========

String formatTelemetry(const FlightData& data) {
  String output = "";
  
  // Header with timestamp and packet info
  output += "=== IRIS2 Flight Telemetry ===\n";
  output += "Time: " + String(data.timestamp) + "ms | Packet: #" + String(data.packetId) + "\n";
  
  // Accelerometer data (m/s²)
  output += "Accelerometer: ";
  output += "X=" + String(data.accel.x, 2) + " Y=" + String(data.accel.y, 2) + " Z=" + String(data.accel.z, 2);
  output += " |Mag=" + String(data.accel.magnitude, 2) + " m/s²\n";
  
  // Gyroscope data (°/s)
  output += "Gyroscope: ";
  output += "X=" + String(data.gyro.x, 2) + " Y=" + String(data.gyro.y, 2) + " Z=" + String(data.gyro.z, 2);
  output += " |Mag=" + String(data.gyro.magnitude, 2) + " °/s\n";
  
  // Magnetometer data (µT) + Heading
  output += "Magnetometer: ";
  output += "X=" + String(data.mag.x, 1) + " Y=" + String(data.mag.y, 1) + " Z=" + String(data.mag.z, 1);
  output += " |Mag=" + String(data.mag.magnitude, 1) + " µT |Hdg=" + String(data.mag.heading, 1) + "°\n";
  
  // Environmental data (enhanced)
  output += "Environment: ";
  output += "T1=" + String(data.env.temperature_lps, 1) + "°C(LPS) T2=" + String(data.env.temperature_hs, 1) + "°C(HS)";
  output += " P=" + String(data.env.pressure, 1) + "kPa Alt=" + String(data.env.altitude, 1) + "m";
  output += " RH=" + String(data.env.humidity, 1) + "%\n";
  
  // GPS data
  output += "GPS: ";
  if (data.gps.fix_valid) {
    output += "Lat=" + String(data.gps.latitude, 6) + " Lon=" + String(data.gps.longitude, 6);
    output += " Alt=" + String(data.gps.altitude_gps, 1) + "m";
    output += " Sat=" + String(data.gps.satellites) + " HDOP=" + String(data.gps.hdop, 2);
    if (data.gps.speed > 0.1) {
      output += " Spd=" + String(data.gps.speed, 1) + "km/h Course=" + String(data.gps.course, 1) + "°";
    }
  } else {
    output += "NO FIX | Satellites=" + String(data.gps.satellites);
  }
  output += "\n";
  
  // System info
  output += "System: ";
  output += "Vbat=" + String(data.battery_voltage, 2) + "V\n";
  
  output += "===============================\n";
  
  return output;
}

String formatCompactTelemetry(const FlightData& data) {
  String output = "";
  
  // CSV-style compact format for data logging
  // timestamp,ax,ay,az,amag,gx,gy,gz,gmag,mx,my,mz,mmag,heading,temp_lps,temp_hs,pressure,altitude,humidity,proximity,red,green,blue,gesture,audio_rms,audio_peak,sound_detected,battery
  
  output += String(data.timestamp) + ",";
  
  // Accelerometer
  output += String(data.accel.x, 3) + "," + String(data.accel.y, 3) + "," + String(data.accel.z, 3) + "," + String(data.accel.magnitude, 3) + ",";
  
  // Gyroscope  
  output += String(data.gyro.x, 3) + "," + String(data.gyro.y, 3) + "," + String(data.gyro.z, 3) + "," + String(data.gyro.magnitude, 3) + ",";
  
  // Magnetometer
  output += String(data.mag.x, 2) + "," + String(data.mag.y, 2) + "," + String(data.mag.z, 2) + "," + String(data.mag.magnitude, 2) + "," + String(data.mag.heading, 2) + ",";
  
  // Environmental
  output += String(data.env.temperature_lps, 2) + "," + String(data.env.temperature_hs, 2) + ",";
  output += String(data.env.pressure, 2) + "," + String(data.env.altitude, 1) + "," + String(data.env.humidity, 1) + ",";
  
  // GPS
  if (data.gps.fix_valid) {
    output += String(data.gps.latitude, 6) + "," + String(data.gps.longitude, 6) + "," + String(data.gps.altitude_gps, 1) + "," + String(data.gps.speed, 1) + "," + String(data.gps.course, 1) + "," + String(data.gps.satellites) + "," + String(data.gps.hdop, 3) + ",";
  } else {
    output += "0.000000,0.000000,0.0,0.0,0.0," + String(data.gps.satellites) + ",0.000,";
  }
  
  // System
  output += String(data.battery_voltage, 2);
  
  return output;
}

String formatJSONTelemetry(const FlightData& data) {
  String output = "{";
  
  output += "\"timestamp\":" + String(data.timestamp) + ",";
  output += "\"packet_id\":" + String(data.packetId) + ",";
  
  // Accelerometer
  output += "\"accelerometer\":{";
  output += "\"x\":" + String(data.accel.x, 3) + ",";
  output += "\"y\":" + String(data.accel.y, 3) + ",";
  output += "\"z\":" + String(data.accel.z, 3) + ",";
  output += "\"magnitude\":" + String(data.accel.magnitude, 3);
  output += "},";
  
  // Gyroscope
  output += "\"gyroscope\":{";
  output += "\"x\":" + String(data.gyro.x, 3) + ",";
  output += "\"y\":" + String(data.gyro.y, 3) + ",";
  output += "\"z\":" + String(data.gyro.z, 3) + ",";
  output += "\"magnitude\":" + String(data.gyro.magnitude, 3);
  output += "},";
  
  // Magnetometer
  output += "\"magnetometer\":{";
  output += "\"x\":" + String(data.mag.x, 2) + ",";
  output += "\"y\":" + String(data.mag.y, 2) + ",";
  output += "\"z\":" + String(data.mag.z, 2) + ",";
  output += "\"magnitude\":" + String(data.mag.magnitude, 2) + ",";
  output += "\"heading\":" + String(data.mag.heading, 2);
  output += "},";
  
  // Environmental
  output += "\"environment\":{";
  output += "\"temperature_lps\":" + String(data.env.temperature_lps, 2) + ",";
  output += "\"temperature_hs\":" + String(data.env.temperature_hs, 2) + ",";
  output += "\"pressure\":" + String(data.env.pressure, 2) + ",";
  output += "\"altitude\":" + String(data.env.altitude, 1) + ",";
  output += "\"humidity\":" + String(data.env.humidity, 1);
  output += "},";
  
  // GPS
  output += "\"gps\":{";
  output += "\"latitude\":" + String(data.gps.latitude, 6) + ",";
  output += "\"longitude\":" + String(data.gps.longitude, 6) + ",";
  output += "\"altitude\":" + String(data.gps.altitude_gps, 2) + ",";
  output += "\"speed\":" + String(data.gps.speed, 2) + ",";
  output += "\"course\":" + String(data.gps.course, 2) + ",";
  output += "\"satellites\":" + String(data.gps.satellites) + ",";
  output += "\"hdop\":" + String(data.gps.hdop, 3) + ",";
  output += "\"fix_valid\":" + String(data.gps.fix_valid ? "true" : "false");
  output += "},";
  
  // System
  output += "\"system\":{";
  output += "\"battery_voltage\":" + String(data.battery_voltage, 2);
  output += "}";
  
  output += "}";
  
  return output;
}

void printSystemReady() {
  Serial.println("\n========================================");
  Serial.println("IRIS2 Flight Controller Ready");
  Serial.println("Arduino Nano 33 BLE Sense Rev2");
  Serial.println("Enhanced Multi-Sensor Telemetry System");
  Serial.println("========================================");
}

void printCSVHeader() {
  Serial.println("timestamp,accel_x,accel_y,accel_z,accel_mag,gyro_x,gyro_y,gyro_z,gyro_mag,mag_x,mag_y,mag_z,mag_mag,heading,temp_lps,temp_hs,pressure,altitude,humidity,gps_lat,gps_lon,gps_alt,gps_speed,gps_course,gps_satellites,gps_hdop,battery_voltage");
}

// ========== TELEMETRY OUTPUT MODES ==========

enum TelemetryMode {
  TELEMETRY_HUMAN_READABLE,
  TELEMETRY_CSV,
  TELEMETRY_JSON,
  TELEMETRY_COMPACT
};

String formatTelemetryByMode(const FlightData& data, TelemetryMode mode) {
  switch(mode) {
    case TELEMETRY_CSV:
      return formatCompactTelemetry(data);
    case TELEMETRY_JSON:
      return formatJSONTelemetry(data);
    case TELEMETRY_COMPACT:
      return formatCompactTelemetry(data);
    case TELEMETRY_HUMAN_READABLE:
    default:
      return formatTelemetry(data);
  }
}

#endif