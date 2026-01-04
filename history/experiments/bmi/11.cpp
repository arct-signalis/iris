#include <Wire.h>

#include "bosch_bmm150.h"
#include "SparkFun_BMI270_Arduino_Library.h"

BMI270 imu;
andrgrue::sensor::bosch_bmm150 Mag(Wire1, BMM150_DEFAULT_I2C_ADDRESS);

struct GyroscopeData {
  float x, y, z;
  static constexpr uint8_t SIZE = 12;  // 3 * sizeof(float) = 12 bytes
  // magnitude = sqrt(x*x + y*y + z*z) for total angular velocity
};

struct AccelerometerData {
  float x, y, z;
  static constexpr uint8_t SIZE = 12;  // 3 * sizeof(float) = 12 bytes
  // magnitude = sqrt(x*x + y*y + z*z) for total acceleration
};

struct MagnetometerData {
  float x, y, z;
  float heading; // 0° = North, 90° = East, 180° = South, 270° = West
  static constexpr uint8_t SIZE = 16;  // 4 * sizeof(float) = 16 bytes
  // magnitude = sqrt(x*x + y*y + z*z) for total field strength µT (Earth's field: ~25-65µT)
};

// Sensor data instances
GyroscopeData gyroData;
AccelerometerData accelData;
MagnetometerData magData;

#define PRINT_CALCULATED
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

// Simple moving average filter
#define FILTER_SIZE 5
float magX_buffer[FILTER_SIZE] = {0};
float magY_buffer[FILTER_SIZE] = {0};
float magZ_buffer[FILTER_SIZE] = {0};
int filter_index = 0;

// Calibration offsets (you'll need to determine these)
float magX_offset = 0.0;
float magY_offset = 0.0; 
float magZ_offset = 0.0;

void printMag();
void printHeading();
void printMagnitude();
void printGyroData();
void printAccelData();
void printMagData();
bool readAccelerometerAndGyroscope(AccelerometerData& accelData, GyroscopeData& gyroData);
bool readMagnetometer(MagnetometerData& data);
void initBMM150Mag(); 
void initBMM270AccelGyro();

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Wire1.begin();
  Wire1.setClock(400000);
  initBMM150Mag();
  initBMM270AccelGyro();
}

void loop()
{
  readAccelerometerAndGyroscope(accelData, gyroData);
  readMagnetometer(magData);
  
  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printAccelData();
    printGyroData();
    printMagData();
    Serial.println();

    lastPrint = millis();
  }
}

bool readAccelerometerAndGyroscope(AccelerometerData& accelerometerData, GyroscopeData& gyroscopeData) {
  if (imu.getSensorData() == BMI2_OK) {
    accelerometerData.x = imu.data.accelX;
    accelerometerData.y = imu.data.accelY;
    accelerometerData.z = imu.data.accelZ;
    gyroscopeData.x = imu.data.gyroX;
    gyroscopeData.y = imu.data.gyroY;
    gyroscopeData.z = imu.data.gyroZ; 
    return true;
  }
  return false;
}

bool readMagnetometer(MagnetometerData& magnetometerData) {
  if (Mag.magneticfieldAvailable()) {
    float rawX, rawY, rawZ;
    Mag.magneticfield(rawX, rawY, rawZ);

    // Apply simple moving average filter
    magX_buffer[filter_index] = rawX;
    magY_buffer[filter_index] = rawY;
    magZ_buffer[filter_index] = rawZ;
    filter_index = (filter_index + 1) % FILTER_SIZE;
    
    // Calculate filtered values
    float magX = 0, magY = 0, magZ = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
      magX += magX_buffer[i];
      magY += magY_buffer[i];
      magZ += magZ_buffer[i];
    }
    
    // Update magnetometer data struct
    magnetometerData.x = (magX / FILTER_SIZE) - magX_offset;
    magnetometerData.y = (magY / FILTER_SIZE) - magY_offset;
    magnetometerData.z = (magZ / FILTER_SIZE) - magZ_offset;
    
    // Calculate heading
    float heading = atan2(magnetometerData.y, magnetometerData.x);
    heading -= DECLINATION * PI / 180;
    
    if (heading > PI) {
      heading -= (2 * PI);
    }
    else if (heading < -PI) {
      heading += (2 * PI);
    }
    
    heading *= 180.0 / PI;
    
    if (heading < 0) {
      heading += 360;
    }
    
    magnetometerData.heading = heading;
    
    // BMM150 returns values in µT (micro-tesla)
    return true;
  }
  return false;
}

void printAccelData() {
  Serial.print("Accel (g): X:");
  Serial.print(accelData.x, 3);
  Serial.print(" Y:");
  Serial.print(accelData.y, 3);
  Serial.print(" Z:");
  Serial.println(accelData.z, 3);
}

void printGyroData() {
  Serial.print("Gyro (°/s): X:");
  Serial.print(gyroData.x, 3);
  Serial.print(" Y:");
  Serial.print(gyroData.y, 3);
  Serial.print(" Z:");
  Serial.println(gyroData.z, 3);
}

void printMagData() {
  Serial.print("Mag (µT): X:");
  Serial.print(magData.x, 2);
  Serial.print(" Y:");
  Serial.print(magData.y, 2);
  Serial.print(" Z:");
  Serial.print(magData.z, 2);
  Serial.print(" Heading:");
  Serial.print(magData.heading, 2);
  Serial.println("°");
}

void initBMM270AccelGyro() {
  while(imu.beginI2C(BMI2_I2C_PRIM_ADDR, Wire1) != BMI2_OK) {
    Serial.println("Failed to initialize BMI270 sensor!");
    delay(1000);
  }

  imu.setAccelODR(BMI2_ACC_ODR_200HZ);
  imu.setGyroODR(BMI2_GYR_ODR_200HZ);
  Serial.println("BMI270 connected!");
}

void initBMM150Mag() {
  while (!Mag.initialize(BMM150_DATA_RATE_30HZ)) {
    Serial.println("Failed to initialize BMM150 magnetometer!");
    delay(1000);
  }

  Serial.print("Magnetometer sample rate = ");
  Serial.print(Mag.magneticfieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
}
