//
//  VXLGyroscopeSensor.cpp
//  VXL_Library
//
//  Created by Peter Birch on 05/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

/*
* Technical note: Gyroscope range register
* 000 -> ±2000 deg/s with resolution of 16.4 LSB/deg/s or 61 mdeg/s/LSB
* 001 -> ±1000 deg/s with resolution of 32.8 LSB/deg/s or 30.5 mdeg/s/LSB
* 010 -> ± 500 deg/s with resolution of 65.6 LSB/deg/s or 15.3 mdeg/s/LSB
* 011 -> ± 250 deg/s with resolution of 131.2 LSB/deg/s or 7.6 mdeg/s/LSB
* 100 -> ± 125 deg/s with resolution of 262.4 LSB/deg/s or 3.8 mdeg/s/LSB
* 101, 110, 111 are reserved...
*
* Technical note: Advanced power saving
* 000 -> 2ms, 001 -> 4ms, 010 -> 5ms, 011 -> 8ms, 100 -> 10ms, 101 -> 15ms, 
* 110 -> 18ms, 111 -> 20ms
* Suspend-Deep Suspend valid modes: 00 - Normal, 01 - DEEP_SUSPEND, 10 -> SUSPEND
*
* Technical note: External trigger select
* 00 -> None, 01 -> INT1, 10 -> INT2, 11 -> SDO pin (SPI3 Mode)
*
* Technical note: Auto-sleep duration
* 000 -> Illegal, 001 -> 4ms, 010 -> 5ms, 011 -> 8ms, 100 -> 10ms, 101 -> 15ms, 
* 110 -> 20ms, 111 -> 40ms
*/

#include "VXLGyroscopeSensor.h"
#include "../VXLBoard.h"

void VXLGyroscopeSensor::initPeripheral() {
#ifndef VXL_DISABLE_GYRO
    uint8_t command[] = { 0, 0 };
    
    // Setup registers
    // - Range: (7) Specified by data sheet, (2,1,0) Set the angular rate range
    command[0] = 0x0F; command[1] = (1 << 7) | 3;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_BMG160, command, 2, false);
    // - Power: (7) Suspended = 1 & Active = 0, (5) Deep suspend = 1, (3,2,1) Sleep duration
    command[0] = 0x11; command[1] = (0 << 7) | (0 << 5) | (0 << 1);
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_BMG160, command, 2, false);
    // - Fast Power-up & Triggers: (7) Drive switched off in suspend, (6) Power save mode disabled,
    //                             (5,4) External trigger select, (2,1,0) Autosleep duration
    command[0] = 0x12; command[1] = (0 << 7) | (0 << 6) | (0 << 4) | 7;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_BMG160, command, 2, false);
    // - Data rate: (7) Filtered data, (6) Shadow mechanism enabled
    //              Shadow mechanism locks the MSB register when LSB is read to ensure continuity, removed when MSB read
    command[0] = 0x13; command[1] = (0 << 7) | (0 << 6);
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_BMG160, command, 2, false);
#endif
}

void VXLGyroscopeSensor::updateValue() {
#ifndef VXL_DISABLE_GYRO
    // Read all of the data in one sweep
    uint8_t read_cmd[] = { 0x02 };
    uint8_t read_values[7] = {0};
    
    // Read values back
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_BMG160, read_cmd, 1, true);
    VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_BMG160, 7, read_values, false);
    
    gyroscope_values[0] = (read_values[1] << 8) | read_values[0];
    gyroscope_values[1] = (read_values[3] << 8) | read_values[2];
    gyroscope_values[2] = (read_values[5] << 8) | read_values[4];
    temperature_value = read_values[6];
#endif
}

int16_t * VXLGyroscopeSensor::getRawGyroscopeValues() {
    return gyroscope_values;
}

int8_t VXLGyroscopeSensor::getRawTemperatureValue() {
    return temperature_value;
}

static bool gyro_on = true;
void VXLGyroscopeSensor::powerOnOffDevice(bool on) {
#ifndef VXL_DISABLE_GYRO
    // Setup power mode
    uint8_t power_reg = (0 << 7);   // Suspend: 1 = Suspended, 0 = Not suspended
    if(on) {
        power_reg |= (0 << 5);      // Deep Suspend: 1 = Deep, 0 = Not deep
    } else {
        power_reg |= (1 << 5);
        // Only {0,0}, {0,1} and {1,0} are allowed - {1,1} is an illegal state
        // for {suspend,deep suspend}.
    }
    power_reg = power_reg | (0 << 1); // Sleep Duration - for advanced power saving
    uint8_t cmd[2] = { 0x11, power_reg };
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_BMG160, cmd, 2, false);
    
    gyro_on = on;
#endif
}

bool VXLGyroscopeSensor::getDevicePowerStatus() {
    return gyro_on;
}

uint8_t VXLGyroscopeSensor::getPeripheralID() {
#ifndef VXL_DISABLE_GYRO
    uint8_t cmd = 0x00;
    uint8_t data[] = { 0 };
    
    // Read values back
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_BMG160, &cmd, 1, true);
    VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_BMG160, 1, data, false);
    
    return data[0];
#else
    return 0x0F;
#endif
}

bool VXLGyroscopeSensor::hasCorrectPeripheralID() {
    return (getPeripheralID() == 0x0F);
}
