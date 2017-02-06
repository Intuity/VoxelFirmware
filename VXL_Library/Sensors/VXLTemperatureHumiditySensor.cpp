//
//  VXLTemperatureHumiditySensor.cpp
//  VXL_Library
//
//  Created by Peter Birch on 05/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#include "VXLTemperatureHumiditySensor.h"
#include "../VXLBoard.h"

void VXLTemperatureHumiditySensor::initPeripheral() {
    uint8_t command[2] = {
        0xE6,
        // (7,0) 12bit RH & 14bit Temp measurements, (2) Disable heater
        (0 << 7) | (0 << 2) | 0
    };
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_Si7020, command, 2, false);
}

void VXLTemperatureHumiditySensor::updateValue() {
    // Read back temperature
    uint8_t cmd = 0xE3;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_Si7020, &cmd, 1, true);
    VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_Si7020, 2, (uint8_t *)&temperature, false);
    
    // Read back humidity
    cmd = 0xE5;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_Si7020, &cmd, 1, true);
    VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_Si7020, 2, (uint8_t *)&humidity, false);
    
    // Byte flip results
    temperature = ((temperature & 255) << 8) | ((temperature >> 8) & 255);
    humidity = ((humidity & 255) << 8) | ((humidity >> 8) & 255);
}

uint16_t VXLTemperatureHumiditySensor::getRawTemperature() {
    return temperature;
}

uint16_t VXLTemperatureHumiditySensor::getRawHumidity() {
    return humidity;
}

uint8_t VXLTemperatureHumiditySensor::getPeripheralID() {
    uint8_t cmd[] = { 0xFA, 0x0F };
    uint8_t data_1[] = { 0, 0, 0, 0 };
    uint8_t data_2[] = { 0, 0, 0, 0 };
    
    // Read first section of serial back
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_Si7020, cmd, 2, true);
    VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_Si7020, 2, data_1, false);
    
    // Read second section of serial back
    cmd[0] = 0xFC; cmd[1] = 0xC9;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_Si7020, cmd, 2, true);
    VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_Si7020, 2, data_2, false);
    
    // The device type is at SNB_3 or data_2[0]
    return data_2[0];
}

bool VXLTemperatureHumiditySensor::hasCorrectPeripheralID() {
    return (getPeripheralID() == 0x14);
}

