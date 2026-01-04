#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "config.h"
#include "structs.h"

struct AccelerometerCalibration
{
  float biasX = 0.0, biasY = 0.0, biasZ = 0.0;
  float scaleX = 1.0, scaleY = 1.0, scaleZ = 1.0;
};

struct GyroscopeCalibration
{
  float biasX = 0.0, biasY = 0.0, biasZ = 0.0;
};

struct MagnetometerCalibration
{
  float hardIronX = 0.0, hardIronY = 0.0, hardIronZ = 0.0;
  float softIronXX = 1.0, softIronXY = 0.0, softIronXZ = 0.0;
  float softIronYX = 0.0, softIronYY = 1.0, softIronYZ = 0.0;
  float softIronZX = 0.0, softIronZY = 0.0, softIronZZ = 1.0;
};


AccelerometerData applyCalibratedAccelerometer(AccelerometerData data);
GyroscopeData applyCalibratedGyroscope(GyroscopeData data);
MagnetometerData applyCalibratedMagnetometer(MagnetometerData data);

void calibrateMagnetometer();
void calibrateAccelerometer();
void calibrateGyroscope();
void calibrateBarometer();

void calibrateAllSensors();
#endif