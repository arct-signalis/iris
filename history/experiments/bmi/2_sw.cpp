#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"

#define BMI_INPUT_PIN 5
#define BMI_NUM_SAMPLES 150
#define BMI_INTERRUPT_THRESHOLD (BMI_NUM_SAMPLES - 25)

BMI270 imu;
uint16_t previousFIFOLength = 0;

void connectBMI();
void handleBMIInterrupt();

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMI270 Example 5 - FIFO Buffer");

    connectBMI();   
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

        // if we cross the interrupt threshold handle BMI FIFO
        if(currentFIFOLength > BMI_INTERRUPT_THRESHOLD)
        {
            handleBMIInterrupt();
        }

        // if the buffer length goes beyond the watermark level, flush
        if(currentFIFOLength > BMI_NUM_SAMPLES)
        {
            Serial.println("!!!Too many samples in FIFO buffer, flushing");
            imu.flushFIFO();
        }
    }
}

void handleBMIInterrupt()
{
    Serial.println("Processing software interrupt...");
    BMI270_SensorData fifoData[BMI_NUM_SAMPLES];    
    uint16_t samplesRead = BMI_NUM_SAMPLES;
    imu.getFIFOData(fifoData, &samplesRead);
    imu.flushFIFO();
    
    Serial.print("Read ");
    Serial.print(samplesRead);
    Serial.println(" samples from FIFO");

    // Print all samples in array format
    if(samplesRead > 0) {
        Serial.println("All samples:");
        for(uint16_t i = 0; i < samplesRead; i++) {
            Serial.print("[");
            Serial.print(i);
            Serial.print("] A[");
            Serial.print(fifoData[i].accelX, 2);
            Serial.print(",");
            Serial.print(fifoData[i].accelY, 2);
            Serial.print(",");
            Serial.print(fifoData[i].accelZ, 2);
            Serial.print("] G[");
            Serial.print(fifoData[i].gyroX, 0);
            Serial.print(",");
            Serial.print(fifoData[i].gyroY, 0);
            Serial.print(",");
            Serial.print(fifoData[i].gyroZ, 0);
            Serial.println("]");
        }
    }    
}

void connectBMI(){
    Wire1.begin();
    // Check if sensor is connected and initialize
    while(imu.beginI2C(BMI2_I2C_PRIM_ADDR, Wire1) != BMI2_OK)
    {
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
        delay(1000);
    }

    Serial.println("BMI270 connected!");

    // Set to 25Hz ODR
    imu.setAccelODR(BMI2_ACC_ODR_25HZ);
    imu.setGyroODR(BMI2_GYR_ODR_25HZ);

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
    config.flags = BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN;
    config.watermark = BMI_NUM_SAMPLES;
    config.accelDownSample = BMI2_FIFO_DOWN_SAMPLE_1;  // No downsampling for full rate
    config.gyroDownSample = BMI2_FIFO_DOWN_SAMPLE_1;   // No downsampling for full rate
    config.accelFilter = BMI2_ENABLE;   // Enable filtering for cleaner data
    config.gyroFilter = BMI2_ENABLE;    // Enable filtering for cleaner data
    config.selfWakeUp = BMI2_ENABLE;    
    imu.setFIFOConfig(config);
}