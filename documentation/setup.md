## Iris Wiring

```C
// GPS - hardware serial (existing, d0, d1)
// VCC 3.3V
// GND GND 
#define GPS_TX 8 // sensor - 10kohm - arduino - 20kohm - ground with voltage divider
#define GPS_RX 9

// LoRa - hardware serial (created, 3, 4)
#define LORA_M0 5
#define LORA_M1 6
#define LORA_RX 3
#define LORA_TX 4
#define LORA_AUX 7
// VCC 3.3V
// GND GND 

// SD Fixed
// VIN 3.3V 
// GND GND
// SCK  = 13 (automatic)
// MISO = 12 (automatic) 
// MOSI = 11 (automatic)
#define SD_CS 10

// SD Removable
#define SD_CS_2 2
// 3V3 3.3
// CS 2
// MOSI 11
// CLK Pin 13 
// MISO Pin 12
// GND GND
```

## Install PlatformIO IDE for VSCode
Quickstart [Documentation](https://docs.platformio.org/en/latest/integration/ide/vscode.html)


## Change Arduino_BMI270_BMM150 Library for higher

After you download the Arduino_BMI270_BMM150 library into `.pio/libdeps/nano33ble` make sure to enable higher sample rate.

`src/BMI270.cpp`

```c
sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;

settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
rslt = bmm150_set_presetmode(&settings, dev);
```

jump inside function  `bmm150_set_presetmode`
```c
settings->data_rate = BMM150_DATA_RATE_30HZ;
```