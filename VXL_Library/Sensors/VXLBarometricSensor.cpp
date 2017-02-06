//
//  VXLBarometricSensor.cpp
//  VXL_Library
//
//  Created by Peter Birch on 05/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

/*
* Technical Note: Setting OST=1 gives a one shot measurement
* where it takes a measurement and then clears OST=0. If SBYB=1, setting OST=1 takes a
* measurement in line with CTRL_REG2 & then keeps OST=1 and refreshes data at periods
* defined by CTRL_REG2 - setting OST=0..1 will force another immediate measurement
*/

#include "VXLBarometricSensor.h"
#include "../VXLBoard.h"

void VXLBarometricSensor::initPeripheral() {
#ifndef VXL_DISABLE_BARO
    uint8_t command[] = {0,0};
    
    // Setup control registers
    // - Register 1: (7) Altimeter = 1 & Barometer = 0, (6) Raw data = 1, Adjusted = 0, (5,4,3) Oversample ratio, 2^OS,
    //               (2) Reset = 1, Normal = 0, (1) OST: If in standby OST = 1 -> one shot measurement, 
    //               (0) Active = 1, Standby = 0
    command[0] = 0x26; command[1] = (0 << 7) | (0 << 6) | (0 << 3) | (0 << 2) | (0 << 1) | 0;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_MPL3115A2, command, 2, false);
    // - Register 2: (5) Don't load values as interrupt targets, (4) Use target values for interrupts,
    //               (3,2,1) ST: The refresh interval from 0-15, used as 2^ST, ST=5 -> 32 seconds
    command[0] = 0x27; command[1] = (0 << 5) | (0 << 4) | 5;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_MPL3115A2, command, 2, false);
    // - PT_DATA_CFG: (2) Enable data ready flag, (1) Flag new pressure, (0) Flag new temperature
    command[0] = 0x13; command[1] = (1 << 2) | (1 << 1) | 1;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_MPL3115A2, command, 2, false);
#endif
}

void VXLBarometricSensor::updateValue() {
#ifndef VXL_DISABLE_BARO
    uint8_t cmd[2] = { 
        0x26, 
        // (1) OST: If SBYB=0, setting OST=1 gives a one shot measurement
        (0 << 7) | (0 << 6) | (0 << 3) | (0 << 2) | (1 << 1) | 0 
    };
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_MPL3115A2, cmd, 2, false); 
    
    // Now poll the I2C interface to pickup read status
    cmd[0] = 0x00; cmd[1] = 0;
    bool ready = false;
    for(uint8_t i = 0; i < 100; i++) {
        uint8_t read_reg = 0;
        VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_MPL3115A2, cmd, 1, true);
        VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_MPL3115A2, 1, &read_reg, false);
        bool pdr = (((read_reg >> 2) & 1) == 1); // Pressure data ready
        bool tdr = (((read_reg >> 1) & 1) == 1); // Temperature data ready
        ready = (pdr && tdr);
        if(ready) break;
        wait_ms(1);
    }
    
    if(!ready) return;
    
    // Now read back the latest pressure & temperature data
    uint8_t read_buffer[] = {0,0,0};
    cmd[0] = 0x01; cmd[1] = 0;
    
    // Read pressure
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_MPL3115A2, cmd, 1, true);
    VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_MPL3115A2, 3, read_buffer, false);
    pressure_value = (read_buffer[0] << 16) | (read_buffer[1] << 8) | (read_buffer[2]);

    // Clear buffer
    memset(read_buffer, 0, 3);
    
    // Read temperature
    cmd[0] = 0x04; cmd[1] = 0;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_MPL3115A2, cmd, 1, true);
    VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_MPL3115A2, 2, read_buffer, false);
    temperature_value = (read_buffer[0] << 8) | (read_buffer[1]);
#endif
}

uint32_t VXLBarometricSensor::getRawPressureValue() { 
    return pressure_value;
}

uint16_t VXLBarometricSensor::getRawTemperatureValue() {
    return temperature_value;
}

uint8_t VXLBarometricSensor::getPeripheralID() {
#ifndef VXL_DISABLE_BARO
    uint8_t cmd = 0x0C;
    uint8_t data = 0;
    
    // Read values back
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_MPL3115A2, &cmd, 1, true);
    VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_MPL3115A2, 1, &data, false);
    
    return data;
#else
    return 0xC4;
#endif
}

bool VXLBarometricSensor::hasCorrectPeripheralID() {
    return (getPeripheralID() == 0xC4);
}
