#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"

BMI270 imu;

// I2C address selection
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; 
#define BMI_MAX_NUM_SAMPLES 150
#define BMI_INTERRUPT_THRESHOLD (BMI_MAX_NUM_SAMPLES - 25)
#define WATERMARK_LEVEL BMI_MAX_NUM_SAMPLES // 25
BMI270_SensorData fifoData[BMI_MAX_NUM_SAMPLES];

volatile bool interruptOccurred = false;
uint8_t previousFIFOLength = 0;

void connectBMI();
void handleBMIInterrupt();

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    Serial.println("BMI270 Example 5 - FIFO Buffer");

    Wire1.begin();

    while(imu.beginI2C(i2cAddress, Wire1) != BMI2_OK)
    {
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
        delay(1000);
    }
    connectBMI();
    Serial.println("BMI270 connected!");
}

void loop() 
{
    uint16_t currentFIFOLength = 0;
    imu.getFIFOLength(&currentFIFOLength);

    // if size of FIFO changed
    if(previousFIFOLength != currentFIFOLength)
    {
        // upgrade previous FIFO length
        previousFIFOLength = currentFIFOLength;
        // Print current FIFO length
        Serial.print("FIFO Length: ");
        Serial.print(currentFIFOLength);
        Serial.print("/");
        Serial.println(BMI_MAX_NUM_SAMPLES);

        // if we cross the interrupt threshold handle BMI FIFO
        if(currentFIFOLength > BMI_INTERRUPT_THRESHOLD)
        {
            handleBMIInterrupt();
        }

        // if the buffer length goes beyond the watermark level, flush
        if(currentFIFOLength > BMI_MAX_NUM_SAMPLES)
        {
            Serial.println("!!!Too many samples in FIFO buffer, flushing");
            imu.flushFIFO();
        }
    }
}

void handleBMIInterrupt()
{  
    Serial.println("Handling SW interrupt");
    for (uint8_t i = 0; i < 6; i++) 
    {
        uint16_t samplesRead = BMI_MAX_NUM_SAMPLES;
        imu.getFIFOData(fifoData, &samplesRead);

        Serial.print("Read");
        Serial.println(samplesRead);
        for(uint8_t j = 0; j < samplesRead; j++)
        {
            // Print acceleration data
            Serial.print("Acceleration in g's");
            Serial.print("\t");
            Serial.print("X: ");
            Serial.print(fifoData[j].accelX, 3);
            Serial.print("\t");
            Serial.print("Y: ");
            Serial.print(fifoData[j].accelY, 3);
            Serial.print("\t");
            Serial.print("Z: ");
            Serial.print(fifoData[j].accelZ, 3);
            Serial.print("\t");

            Serial.print("\t");

            // Print rotation data
            Serial.print("Rotation in deg/sec");
            Serial.print("\t");
            Serial.print("X: ");
            Serial.print(fifoData[j].gyroX, 3);
            Serial.print("\t"); 
            Serial.print("Y: ");
            Serial.print(fifoData[j].gyroY, 3);
            Serial.print("\t");
            Serial.print("Z: ");
            Serial.println(fifoData[j].gyroZ, 3);
        }
    }
}

void connectBMI()
{
    Wire1.begin();
    while(imu.beginI2C(i2cAddress, Wire1) != BMI2_OK)
    {
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
        delay(1000);
    }

    // Set to 1600Hz
    imu.setAccelODR(BMI2_ACC_ODR_1600HZ);
    imu.setGyroODR(BMI2_GYR_ODR_1600HZ);


    // Here we configure the FIFO buffer of the BMI270. Each of the config
    // parameters are described below:
    // 
    // .flags           - Which features of the FIFO to enable. Only the
    //                    accelerometer and gyroscope in headless mode is
    //                    currently supported
    // .watermark       - Number of data samples to store before triggering an
    //                    interrupt
    // .accelDownSample - Downsampling factor for the accelerometer, relative to
    //                    the sensor's ODR (output data rate)
    // .gyroDownSample  - Downsampling factor for the gyroscope, relative to the
    //                    sensor's ODR (output data rate)
    // .accelFilter     - Whether the accelerometer data is filtered
    // .gyroFilter      - Whether the gyroscope data is filtered
    // .selfWakeUp      - Whether FIFO can be read in low power mode
    BMI270_FIFOConfig config;
    config.flags = BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN | BMI2_FIFO_HEADER_EN | BMI2_FIFO_TIME_EN; 
    config.watermark = WATERMARK_LEVEL;
    config.accelDownSample = BMI2_FIFO_DOWN_SAMPLE_1;
    config.gyroDownSample = BMI2_FIFO_DOWN_SAMPLE_1;
    config.accelFilter = BMI2_ENABLE;
    config.gyroFilter = BMI2_ENABLE;
    config.selfWakeUp = BMI2_ENABLE;
    imu.setFIFOConfig(config);
}