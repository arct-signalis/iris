#include "telemetry.h"
#include "structs.h"
#include "config.h"
#include "utils.h"

String formatTestTelemetry(const FlightData *data, TelemetryType type)
{
  String output = "";

  // Return empty string for NONE type
  if (type == NONE)
    return output;

  // Header with timestamp and packet info
  output += "\n================================";
  output += "\nTime: " + String(GET_TIME_MS()) + "ms";

  if (type == ACCEL || type == ALL)
  {
    output += "\n";
    // Accelerometer data (g)
    output += "Accelerometer: ";
    output += "X=" + String(data->accel.x, 2) + " Y=" + String(data->accel.y, 2) + " Z=" + String(data->accel.z, 2) + " g";
    output += " Total=" + String(data->accel.totalAcceleration, 2) + "g Max=" + String(data->maxTotalAcceleration, 2) + "g";
  }

  if (type == GYRO || type == ALL)
  {
    output += "\n";
    // Gyroscope data (°/s)
    output += "Gyroscope: ";
    output += "X=" + String(data->gyro.x, 2) + " Y=" + String(data->gyro.y, 2) + " Z=" + String(data->gyro.z, 2) + " °/s";
    output += " Rate=" + String(data->gyro.angularRate, 2) + "°/s";
  }

  if (type == MAG || type == ALL)
  {
    output += "\n";
    // Magnetometer data (µT)
    output += "Magnetometer: ";
    output += "X=" + String(data->mag.x, 1) + " Y=" + String(data->mag.y, 1) + " Z=" + String(data->mag.z, 1) + " µT";
    output += " Strength=" + String(data->mag.magneticFieldStrength, 1) + "µT";
  }

  if (type == ENV || type == ALL)
  {
    output += "\n";
    // Environmental data
    output += "Environment: ";
    output += "T=" + String(data->env.temperature, 1) + "°C P=" + String(data->env.pressure, 0) + "kPa";
    output += " AirDensity=" + String(data->env.airDensity, 3) + "kg/m³";
    output += "\n";
    // Altitude tracking
    output += "Altitude Tracking: ";
    output += "Max=" + String(data->maxAltitude, 1) + "m Launch=" + String(data->launchAltitude, 1) + "m AGL=" + String(data->env.altitudeAboveLaunchPad, 1) + "m";
  }

  if (type == GPS_SENSOR || type == ALL)
  {
    output += "\n";
    // GPS data
    output += "GPS: ";
    if (data->gps.latitude != 0.0 || data->gps.longitude != 0.0)
    {
      output += "Lat=" + String(data->gps.latitude, 6) + " Lon=" + String(data->gps.longitude, 6);
      output += " Sat=" + String(data->gps.satellites) + " HDOP=" + String(data->gps.hdop, 2);
      if (data->gps.speed > 0.1)
      {
        output += " Spd=" + String(data->gps.speed, 1) + "km/h Course=" + String(data->gps.course, 1) + "°";
      }
    }
    else
    {
      output += "NO FIX | Satellites=" + String(data->gps.satellites);
    }
  }

  if (type == ATTITUDE || type == ALL)
  {
    output += "\n";
    // Attitude data
    output += "Attitude: ";
    output += "Roll=" + String(data->attitude.roll, 1) + "° Pitch=" + String(data->attitude.pitch, 1) + "° Yaw=" + String(data->attitude.yaw, 1) + "°";
    output += " OffVert=" + String(data->attitude.offVert, 1) + "°";
  }

  return output;
}