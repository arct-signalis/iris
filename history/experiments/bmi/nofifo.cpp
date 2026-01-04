#include <Wire.h>

#include "SparkFun_BMI270_Arduino_Library.h"

BMI270 imu;
#include "DFRobot_BMM150.h"
// Use: DFRobot_BMM150_I2C bmm150(&Wire1, I2C_ADDRESS_1);

//When using I2C communication, use the following program to construct an object by DFRobot_BMM150_I2C
/*!
 * @brief Constructor 
 * @param pWire I2C controller
 * @param I2C address
 *        i2c Address select, that CS and SDO pin select 1 or 0 indicates the high or low respectively. There are 4 combinations: 
 *          I2C_ADDRESS_1 0x10  (CS:0 SDO:0)
 *          I2C_ADDRESS_2 0x11  (CS:0 SDO:1)
 *          I2C_ADDRESS_3 0x12  (CS:1 SDO:0)
 *          I2C_ADDRESS_4 0x13  (CS:1 SDO:1) default i2c address
 */
DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);

//When using SPI communication, use the following program to construct an object by DFRobot_BMM150_SPI
#if defined(ESP32) || defined(ESP8266)
  #define BMM150_CS D3
#elif defined(__AVR__) || defined(ARDUINO_SAM_ZERO)
  #define BMM150_CS 3
#elif (defined NRF5)
  #define BMM150_CS 2  //The corresponding silkscreen on the development board is the pin P2
#endif
/*!
 * @brief Constructor 
 * @param cs Chip selection pinChip selection pin
 *        spi Connection method
 *        (SDO<-->MISO)    (SDI<-->MOSI)
 *        (SCK<-->SCK)     (PS<--> GND)
 *        (CS<-->CS customize pin)
 */
//DFRobot_BMM150_SPI bmm150(/*cs = */BMM150_CS);


void initAccelGyro();
void initMagnetometer();
void readAccelGyro();
void scanForMagnetometer();

void setup()
{
  // Start serial
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("BMI270 Example 1 - Basic Readings I2C");
  initAccelGyro();
  scanForMagnetometer();
  //initMagnetometer();
}

void loop()
{   
  //readAccelGyro();
  Serial.println(bmm150.getOperationMode());
  sBmm150MagData_t magData = bmm150.getGeomagneticData(); 
  Serial.print("mag x = "); Serial.print(magData.xx); Serial.println(" uT");
  Serial.print("mag y = "); Serial.print(magData.yy); Serial.println(" uT");
  Serial.print("mag z = "); Serial.print(magData.zz); Serial.println(" uT");
  float compassDegree = bmm150.getCompassDegree();
  Serial.print("the angle between the pointing direction and north (counterclockwise) is:");
  Serial.println(compassDegree);
  Serial.println("--------------------------------");

  delay(1000);
}

void readAccelGyro() {
  // Get measurements from the sensor. This must be called before accessing
  // the sensor data, otherwise it will never update
  imu.getSensorData();

  // Print acceleration data
  Serial.print("Acceleration in g's");
  Serial.print("\t");
  Serial.print("X: ");
  Serial.print(imu.data.accelX, 3);
  Serial.print("\t");
  Serial.print("Y: ");
  Serial.print(imu.data.accelY, 3);
  Serial.print("\t");
  Serial.print("Z: ");
  Serial.print(imu.data.accelZ, 3);

  Serial.print("\t");

  // Print rotation data
  Serial.print("Rotation in deg/sec");
  Serial.print("\t");
  Serial.print("X: ");
  Serial.print(imu.data.gyroX, 3);
  Serial.print("\t");
  Serial.print("Y: ");
  Serial.print(imu.data.gyroY, 3);
  Serial.print("\t");
  Serial.print("Z: ");
  Serial.println(imu.data.gyroZ, 3);
}

void initAccelGyro() {
  // Initialize the I2C library
  Wire1.begin();
  Wire1.setClock(400000);

  // Check if sensor is connected and initialize
  // Address is optional (defaults to 0x68)
  while(imu.beginI2C(BMI2_I2C_PRIM_ADDR, Wire1) != BMI2_OK)
  {
      // Not connected, inform user
      Serial.println("Error: BMI270 not connected, check wiring and I2C address!");

      // Wait a bit to see if connection is established
      delay(1000);
  }

  imu.setAccelODR(BMI2_ACC_ODR_1600HZ);
  imu.setGyroODR(BMI2_GYR_ODR_1600HZ);
  Serial.println("BMI270 connected!");
}

void initMagnetometer() {
  while(bmm150.begin()){
    Serial.println("bmm150 init failed, Please try again!");
    delay(1000);
  } 
  Serial.println("bmm150 init success!");

  /**!
   * Sensor self test, the returned character string indicates the test result.
   * mode:
   *  BMM150_SELF_TEST_NORMAL              // Normal self test, test whether x-axis, y-axis and z-axis are connected or short-circuited
   *  BMM150_SELF_TEST_ADVANCED            // Advanced self test, test the data accuracy at z-axis
   */
  Serial.println(bmm150.selfTest(BMM150_SELF_TEST_NORMAL));

  /**!
   * Set sensor operation mode
   * opMode:
   *   BMM150_POWERMODE_NORMAL  // normal mode  Get geomagnetic data normally
   *   BMM150_POWERMODE_FORCED  // forced mode  Single measurement, the sensor restores to sleep mode when the measurement is done.
   *   BMM150_POWERMODE_SLEEP   // sleep mode   Users can visit all the registers, but can’t measure geomagnetic data
   *   BMM150_POWERMODE_SUSPEND // suspend mode At the time the sensor cpu doesn't work and can't implement any operation.
   *                                            Users can only visit the content of the control register BMM150_REG_POWER_CONTROL
   */
  bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);

  /**!
   * Set preset mode, make it easier for users to configure sensor to get geomagnetic data
   * presetMode:
   *   BMM150_PRESETMODE_LOWPOWER      // Low power mode, get a fraction of data and take the mean value.
   *   BMM150_PRESETMODE_REGULAR       // Regular mode, get a number of data and take the mean value.
   *   BMM150_PRESETMODE_ENHANCED      // Enhanced mode, get a plenty of data and take the mean value.
   *   BMM150_PRESETMODE_HIGHACCURACY  // High accuracy mode, get a huge number of take and draw the mean value.
   */
  bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);

  /**!
   * Set the rate of obtaining geomagnetic data, the higher, the faster(without delay function)
   * rate:
   *   BMM150_DATA_RATE_02HZ
   *   BMM150_DATA_RATE_06HZ
   *   BMM150_DATA_RATE_08HZ
   *   BMM150_DATA_RATE_10HZ   (default rate)
   *   BMM150_DATA_RATE_15HZ
   *   BMM150_DATA_RATE_20HZ
   *   BMM150_DATA_RATE_25HZ
   *   BMM150_DATA_RATE_30HZ
   */
  bmm150.setRate(BMM150_DATA_RATE_30HZ);

  /**!
   * Enable the measurement at x-axis, y-axis and z-axis, default to be enabled, no config required, the geomagnetic data at x, y and z will be inaccurate when disabled.
   * Refer to setMeasurementXYZ() function in the .h file if you want to configure more parameters.
   */
  bmm150.setMeasurementXYZ();

  /**!
   * Get the config data rate unit: HZ
   */
  uint8_t rate = bmm150.getRate();
  Serial.print("rate is "); Serial.print(rate); Serial.println(" HZ");
  
  /**!
   * Get the measurement status at x-axis, y-axis and z-axis, return the measurement status as character string
   */
  Serial.println(bmm150.getMeasurementStateXYZ());
  
  /**!
   * Get the sensor operation mode, return the sensor operation status as character string
   */
  Serial.println(bmm150.getOperationMode());
  
  /**!
   * Soft reset, restore to suspend mode after soft reset and then enter sleep mode, soft reset can't be implemented under suspend mode.
   */
  bmm150.softReset();    // After software reset, resume sleep mode (Suspend mode cannot be reset)
}

void scanForMagnetometer() {
  Serial.println("Scanning for BMM150 magnetometer...");
  
  // Try different I2C addresses with Wire1 (same as BMI270)
  uint8_t addresses[] = {0x10, 0x11, 0x12, 0x13}; // I2C_ADDRESS_1 through I2C_ADDRESS_4
  const char* addressNames[] = {"I2C_ADDRESS_1 (0x10)", "I2C_ADDRESS_2 (0x11)", "I2C_ADDRESS_3 (0x12)", "I2C_ADDRESS_4 (0x13)"};
  
  for(int i = 2; i < 4; i++) {
    Serial.print("Trying Wire1 with ");
    Serial.println(addressNames[i]);
    
    DFRobot_BMM150_I2C testBmm150(&Wire1, addresses[i]);
    
    if(!testBmm150.begin()) {
      Serial.println("✓ BMM150 found and initialized successfully!");
      Serial.print("Use: DFRobot_BMM150_I2C bmm150(&Wire1, ");
      switch(addresses[i]) {
        case 0x10: Serial.println("I2C_ADDRESS_1);"); break;
        case 0x11: Serial.println("I2C_ADDRESS_2);"); break;
        case 0x12: Serial.println("I2C_ADDRESS_3);"); break;
        case 0x13: Serial.println("I2C_ADDRESS_4);"); break;
      }
      return;
    } else {
      Serial.println("✗ Failed to initialize");
    }
    delay(100);
  }
  
  // If Wire1 didn't work, try Wire (original I2C bus)
  Serial.println("\nWire1 failed, trying Wire...");
  
  for(int i = 0; i < 4; i++) {
    Serial.print("Trying Wire with ");
    Serial.println(addressNames[i]);
    
    DFRobot_BMM150_I2C testBmm150(&Wire, addresses[i]);
    
    if(!testBmm150.begin()) {
      Serial.println("✓ BMM150 found and initialized successfully!");
      Serial.print("Use: DFRobot_BMM150_I2C bmm150(&Wire, ");
      switch(addresses[i]) {
        case 0x10: Serial.println("I2C_ADDRESS_1);"); break;
        case 0x11: Serial.println("I2C_ADDRESS_2);"); break;
        case 0x12: Serial.println("I2C_ADDRESS_3);"); break;
        case 0x13: Serial.println("I2C_ADDRESS_4);"); break;
      }
      return;
    } else {
      Serial.println("✗ Failed to initialize");
    }
    delay(100);
  }
  
  Serial.println("\n❌ BMM150 not found on any address or I2C bus!");
  Serial.println("Check wiring and connections.");
}
