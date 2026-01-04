#include "sensors.h"
#include "config.h"
#include "utils.h"

// ========== GLOBAL VARIABLES (DEFINITIONS) ==========
UART gpsSerial(digitalPinToPinName(GPS_RX_PIN), digitalPinToPinName(GPS_TX_PIN));
TinyGPSPlus gps;
extern FlightData flightData;

// ========== SENSOR READING FUNCTIONS ==========
// Read from built-in BMI270 accelerometer
bool readAccelerometer(AccelerometerData *data)
{
  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(data->x, data->y, data->z);
    *data = applyCalibratedAccelerometer(*data);
    // Calculate total acceleration magnitude
    data->totalAcceleration = VECTOR_LENGTH(data->x, data->y, data->z);

    // Track maximum total acceleration
    if (data->totalAcceleration > flightData.maxTotalAcceleration)
    {
      flightData.maxTotalAcceleration = data->totalAcceleration;
    }
    return true;
  }
  return false;
}

// Read from built-in BMI270 gyroscope
bool readGyroscope(GyroscopeData *data)
{
  if (IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(data->x, data->y, data->z);
    *data = applyCalibratedGyroscope(*data);
    // Calculate angular rate magnitude
    data->angularRate = VECTOR_LENGTH(data->x, data->y, data->z);
    return true;
  }
  return false;
}

// Read from built-in BMM150 magnetometer
bool readMagnetometer(MagnetometerData *data)
{
  if (IMU.magneticFieldAvailable())
  {
    IMU.readMagneticField(data->x, data->y, data->z);
    *data = applyCalibratedMagnetometer(*data);
    // Calculate magnetic field strength magnitude
    data->magneticFieldStrength = VECTOR_LENGTH(data->x, data->y, data->z);
    return true;
  }
  return false;
}

void calculateAttitude(AttitudeData *attitude, AccelerometerData accel, MagnetometerData mag)
{
  // Roll and Pitch from accelerometer
  attitude->roll = atan2f(accel.y, sqrtf(powf(accel.x, 2) + powf(accel.z, 2))) * 180.0f / PI;
  attitude->pitch = atan2f(-accel.x, sqrtf(powf(accel.y, 2) + powf(accel.z, 2))) * 180.0f / PI;

  // Convert to radians for tilt compensation
  float pitch_rad = attitude->pitch * PI / 180.0f;
  float roll_rad = attitude->roll * PI / 180.0f;

  // Tilt-compensated magnetometer readings
  float mag_x_comp = mag.x * cosf(pitch_rad) + mag.z * sinf(pitch_rad);
  float mag_y_comp = mag.x * sinf(roll_rad) * sinf(pitch_rad) +
                     mag.y * cosf(roll_rad) -
                     mag.z * sinf(roll_rad) * cosf(pitch_rad);

  // Yaw (heading) from tilt-compensated magnetometer
  attitude->yaw = atan2f(mag_y_comp, mag_x_comp) * 180.0f / PI;
  attitude->yaw = fmodf(attitude->yaw + 360.0f, 360.0f);
  // Calculate off-vertical angle
  attitude->offVert = sqrtf(powf(attitude->roll, 2) + powf(attitude->pitch, 2));
}

// Read from built-in LPS22HB barometric pressure sensor
bool readEnvironmental(EnvironmentalData *data)
{
  data->temperature = BARO.readTemperature();
  data->pressure = BARO.readPressure();

  // Check for valid readings first
  if (isnanf(data->temperature) || isnanf(data->pressure))
    return false;

  // Calculate current altitude using barometric formula
  float currentAltitude = SEA_LEVEL_ALTITUDE * (1.0f - powf(data->pressure / STANDARD_SEA_LEVEL_PRESSURE_KPA, BAROMETRIC_EXPONENT));

  // Calculate altitude above launch pad (always relative to ground)
  data->altitudeAboveLaunchPad = currentAltitude - flightData.launchAltitude > 0 ? currentAltitude - flightData.launchAltitude : 0;

  // Calculate air density using ideal gas law
  data->airDensity = (data->pressure * KPA_TO_PASCAL) / (DRY_AIR_GAS_CONSTANT * (data->temperature + CELSIUS_TO_KELVIN));

  // Track maximum altitude (relative to launch pad)
  if (data->altitudeAboveLaunchPad > flightData.maxAltitude)
    flightData.maxAltitude = data->altitudeAboveLaunchPad;

  return true;
}

bool readGPS(GPSData *data)
{
  // GPS is valid if we have at least location data
  if (gps.location.isValid())
  {
    data->latitude = (float)(gps.location.lat());
    data->longitude = (float)(gps.location.lng());
    data->hdop = gps.hdop.isValid() ? gps.hdop.hdop() : -1.0;
    data->satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
    data->speed = gps.speed.isValid() ? gps.speed.kmph() : -1.0;
    data->course = gps.course.isValid() ? gps.course.deg() : -1.0;
    return true;
  }
  return false;
}

void processGPSData()
{
  while (gpsSerial.available() > 0)
    gps.encode(gpsSerial.read());
}

// ========== SENSOR INITIALIZATION ==========

bool initializeSensors()
{
  // Initialize GPS
  gpsSerial.begin(GPS_BAUD_RATE);
#ifdef GPS_PPS_PIN
  pinMode(GPS_PPS_PIN, INPUT);
#endif
  Serial.println("GPS init");

#ifdef GPS_WAIT_FOR_SATELLITES
  Serial.println("Waiting for GPS...");
  while (gps.satellites.value() < 3)
    processGPSData();
  Serial.println("GPS ready");
#endif

  // Initialize built-in IMU (BMI270 + BMM150)
  if (!IMU.begin())
  {
    Serial.println("IMU init failed!");
    return false;
  }
  Serial.println("IMU init");

  Serial.println("IMU Sample Rates (Hz):");
  Serial.print("  Accel: ");
  Serial.println(IMU.accelerationSampleRate());
  Serial.print("  Mag: ");
  Serial.println(IMU.magneticFieldSampleRate());
  Serial.print("  Gyro: ");
  Serial.println(IMU.gyroscopeSampleRate());

  // Initialize built-in barometric pressure sensor (LPS22HB)
  if (!BARO.begin())
    Serial.println("Baro init failed!");
  else
    Serial.println("Baro init");

  Serial.println("Calibrating sensors...");
  calibrateAllSensors();

  return true;
}