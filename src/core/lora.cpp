#include <EByte_LoRa_E220_library.h>
#include "config.h"
#include "lora.h"

UART loraSerial(digitalPinToPinName(LORA_RX), digitalPinToPinName(LORA_TX), NC, NC);
LoRa_E220 e220ttl(&loraSerial, LORA_AUX, LORA_M0, LORA_M1, UART_BPS_RATE_9600);

void loraSend() {
    FlightPacket packet;
    
    strcpy(packet.rocket_name, "IRIS_ROCKET");
    // Get altitude from flight data
    packet.altitude = flightData.env.altitudeAboveLaunchPad;
    
    // Get gyroscope data (x, y, z)
    packet.gyro[0] = flightData.gyro.x;
    packet.gyro[1] = flightData.gyro.y;
    packet.gyro[2] = flightData.gyro.z;
    
    // Get GPS coordinates
    packet.lat = flightData.gps.latitude;
    packet.lon = flightData.gps.longitude;
    
    // Environmental temperature
    packet.temperature = flightData.env.temperature;

    // Send struct via LoRa using public sendMessage method
    ResponseStatus rs = e220ttl.sendMessage(&packet, sizeof(packet));
    
    if (rs.code == E220_SUCCESS) {
        Serial.println("Flight packet sent successfully");
        Serial.println("Rocket Name: " + String(packet.rocket_name));
        Serial.print("Altitude: "); Serial.print(packet.altitude); Serial.println(" m");
        Serial.print("Gyro X: "); Serial.print(packet.gyro[0]); Serial.println(" °/s");
        Serial.print("Gyro Y: "); Serial.print(packet.gyro[1]); Serial.println(" °/s");
        Serial.print("Gyro Z: "); Serial.print(packet.gyro[2]); Serial.println(" °/s"); 
        Serial.print("Latitude: "); Serial.println(packet.lat, 6);
        Serial.print("Longitude: "); Serial.println(packet.lon, 6);
        Serial.print("Temperature: "); Serial.print(packet.temperature); Serial.println(" °C");
        Serial.print("Packet size: "); Serial.print(sizeof(FlightPacket)); Serial.println(" bytes");
    } else {
        Serial.println("Failed to send flight packet");
        Serial.println(rs.getResponseDescription());
    }
}

void loraInit() {
    e220ttl.begin();
    delay(1000);
 
    ResponseStructContainer c;
    c = e220ttl.getConfiguration();
    // It's important get configuration pointer before all other operation
    Configuration configuration = *(Configuration*) c.data;
    Serial.println(c.status.getResponseDescription());
    Serial.println(c.status.code);
 
    printParameters(configuration);
 
    ResponseStructContainer cMi;
    cMi = e220ttl.getModuleInformation();
    // It's important get information pointer before all other operation
    ModuleInformation mi = *(ModuleInformation*)cMi.data;
 
    Serial.println(cMi.status.getResponseDescription());
    Serial.println(cMi.status.code);
 
    printModuleInformation(mi);
    c.close();
}
void printParameters(struct Configuration configuration) {
    Serial.println("----------------------------------------");
 
    Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX);Serial.print(" ");Serial.print(configuration.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(configuration.LENGHT, HEX);
    Serial.println(F(" "));
    Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
    Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
    Serial.println(F(" "));
    Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
    Serial.println(F(" "));
    Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
    Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRateDescription());
    Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRateDescription());
    Serial.println(F(" "));
    Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getSubPacketSetting());
    Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
    Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
    Serial.println(F(" "));
    Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
    Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
    Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
    Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());
 
 
    Serial.println("----------------------------------------");
}
void printModuleInformation(struct ModuleInformation moduleInformation) {
    Serial.println("----------------------------------------");
    Serial.print(F("HEAD: "));  Serial.print(moduleInformation.COMMAND, HEX);Serial.print(" ");Serial.print(moduleInformation.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(moduleInformation.LENGHT, DEC);
 
    Serial.print(F("Model no.: "));  Serial.println(moduleInformation.model, HEX);
    Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, HEX);
    Serial.print(F("Features : "));  Serial.println(moduleInformation.features, HEX);
    Serial.println("----------------------------------------");
 
}