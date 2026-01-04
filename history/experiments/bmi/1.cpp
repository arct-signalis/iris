#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"

// Function declarations
void myInterruptHandler();

// Create a new sensor object
BMI270 imu;

// I2C address - Arduino Nano 33 BLE BMI270 uses 0x68
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68 - CONFIRMED

// Pin used for interrupt detection
int interruptPin = 5;

// Flag to know when interrupts occur
volatile bool interruptOccurred = false;

// Create a larger buffer for 1600Hz FIFO data
// At 1600Hz, we want to buffer more samples before interrupt
const uint16_t numSamples = 50; // Increased for high-frequency data
BMI270_SensorData fifoData[numSamples];

// Track FIFO length to give progress updates
uint8_t previousFIFOLength = 0;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMI270 Example 5 - FIFO Buffer");

    // Initialize the I2C library - Arduino Nano 33 BLE uses Wire1 for BMI270
    Wire1.begin();

    // Check if sensor is connected and initialize
    // Arduino Nano 33 BLE BMI270 is at address 0x68 on Wire1
    while(imu.beginI2C(i2cAddress, Wire1) != BMI2_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("BMI270 connected!");

    // Set maximum ODR (output data rate) - 1600Hz for both sensors
    // This will generate data every 0.625ms (1600 samples per second)
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
    config.flags = BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN;
    config.watermark = numSamples;
    config.accelDownSample = BMI2_FIFO_DOWN_SAMPLE_1; // No downsampling for max rate
    config.gyroDownSample = BMI2_FIFO_DOWN_SAMPLE_1;  // No downsampling for max rate
    config.accelFilter = BMI2_ENABLE;
    config.gyroFilter = BMI2_ENABLE;
    config.selfWakeUp = BMI2_ENABLE;
    imu.setFIFOConfig(config);

    // The BMI270 has 2 interrupt pins. All interrupt conditions can be mapped
    // to either pin, so we'll just choose the first one for this example. We're
    // using the FIFO watermark condition, which triggers whenever the FIFO
    // buffer reches the watermark level
    imu.mapInterruptToPin(BMI2_FWM_INT, BMI2_INT1);

    // Here we configure the interrupt pins using a bmi2_int_pin_config, which 
    // allows for both pins to be configured simultaneously if needed. Here's a
    // brief description of each value:
    // 
    // .pin_type  - Which pin(s) is being configured (INT1, INT2, or BOTH)
    // .int_latch - Latched or pulsed signal (applies to both pins)
    // .pin_cfg   - Array of settings for each pin (index 0/1 for INT1/2):
    //     .lvl       - Active high or low
    //     .od        - Push/pull or open drain output
    //     .output_en - Whether to enable outputs from this pin
    //     .input_en  - Whether to enable inputs to this pin (see datasheet)
    // 
    // In this case, we set INT1 to pulsed, active high, push/pull
    bmi2_int_pin_config intPinConfig;
    intPinConfig.pin_type = BMI2_INT1;
    intPinConfig.int_latch = BMI2_INT_NON_LATCH;
    intPinConfig.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    intPinConfig.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    intPinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    intPinConfig.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
    imu.setInterruptPinConfig(intPinConfig);
    
    // Setup interrupt handler
    attachInterrupt(digitalPinToInterrupt(interruptPin), myInterruptHandler, RISING);
}

void loop()
{
    // Get number of data samples currently stored in FIFO buffer
    uint16_t currentFIFOLength = 0;
    imu.getFIFOLength(&currentFIFOLength);
    
    // Check whether number of samples in FIFO buffer has changed
    if(previousFIFOLength != currentFIFOLength)
    {
        // Update FIFO length
        previousFIFOLength = currentFIFOLength;

        // Print current FIFO length
        Serial.print("FIFO Length: ");
        Serial.print(currentFIFOLength);
        Serial.print("/");
        Serial.println(numSamples);

        // If the buffer length goes beyond the watermark level, then an
        // interrupt was missed. This example will likely run into issues,
        // so we'll just clear the FIFO buffer
        if(currentFIFOLength > numSamples)
        {
            Serial.println("Too many samples in FIFO buffer, flushing...");
            imu.flushFIFO();
        }
    }

    // Wait for interrupt to occur
    if(interruptOccurred)
    {
        // Reset flag for next interrupt
        interruptOccurred = false;

        Serial.println("Interrupt occurred!");

        // Get the interrupt status to know which condition triggered
        uint16_t interruptStatus = 0;
        imu.getInterruptStatus(&interruptStatus);
        
        // Check if this is the FIFO watermerk interrupt condition
        if(interruptStatus & BMI2_FWM_INT_STATUS_MASK)
        {
            // Get FIFO data from the sensor
            uint16_t samplesRead = numSamples;
            imu.getFIFOData(fifoData, &samplesRead);
            
            // samplesRead will be changed to the number of data frames actually
            // read from the FIFO buffer. Check whether it's equal to numSamples
            if(samplesRead != numSamples)
            {
                // Most likely didn't have enough data frames in FIFO buffer.
                // This can happen if control frames are inserted into the FIFO
                // buffer, which occurs when certain configuration changes occur
                Serial.print("Unexpected number of samples read from FIFO: ");
                Serial.println(samplesRead);
            }

            // At 1600Hz, don't print every sample - too fast for Serial
            // Instead, print summary statistics or every Nth sample
            Serial.print("Got ");
            Serial.print(samplesRead);
            Serial.print(" samples. Latest: ");
            
            // Print only the last sample to avoid overwhelming serial output
            if(samplesRead > 0) {
                uint16_t last = samplesRead - 1;
                Serial.print("A[");
                Serial.print(fifoData[last].accelX, 2);
                Serial.print(",");
                Serial.print(fifoData[last].accelY, 2);
                Serial.print(",");
                Serial.print(fifoData[last].accelZ, 2);
                Serial.print("] G[");
                Serial.print(fifoData[last].gyroX, 2);
                Serial.print(",");
                Serial.print(fifoData[last].gyroY, 2);
                Serial.print(",");
                Serial.print(fifoData[last].gyroZ, 2);
                Serial.println("]");
            }
        }
        else
        {
            Serial.println("Unknown interrupt condition!");
        }
    }
}

void myInterruptHandler()
{
    interruptOccurred = true;
}