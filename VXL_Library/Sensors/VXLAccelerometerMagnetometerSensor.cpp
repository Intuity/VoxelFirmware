//
//  VXLAccelerometerMagnetometerSensor.cpp
//  VXL_Library
//
//  Created by Peter Birch on 05/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#include "VXLAccelerometerMagnetometerSensor.h"
#include <string.h>
#include "../VXLBoard.h"

void VXLAccelerometerMagnetometerSensor::initPeripheral() {
    
    uint8_t command[] = {0,0};
    
    // Setup control registers
    // - Register 0: Normal mode, no filters
    command[0] = 0x19; command[1] = 0;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, command, 2, false);
    // - Register 1: (7,6,5,4) Refresh rate @ 25Hz, (3) Disable blocking update, (2,1,0) Enable Z,Y,X axis
    command[0] = 0x20; command[1] = 1 | (1 << 1) | (1 << 2) | (0 << 3) | (4 << 4);
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, command, 2, false);
    // - Register 2: (7) ±2g, (6) No self-test, (0) 50Hz anti-alias
    command[0] = 0x21; command[1] = 0b00000001;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, command, 2, false);
    // - Register 3: No interrupt actions on INT1
    command[0] = 0x22; command[1] = 0;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, command, 2, false);
    // - Register 4: No interrupt actions on INT2
    command[0] = 0x23; command[1] = 0;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, command, 2, false);
    // - Register 5: Temperature enabled, hi-res magnetic, 25Hz, don't latch interrupts
    command[0] = 0x24; command[1] = 0 | (1 << 7) | (1 << 6) | (1 << 5) | (1 << 3) | (1 << 2);
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, command, 2, false);
    // - Register 6: ±2 gauss sensitivity
    command[0] = 0x25; command[1] = 0;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, command, 2, false);
    // - Register 7: (6) Normal high pass filter for acc, (5) Enable acc filter, (4) Disable temperature only mode
    //               (2) Disable low power mode, (0) Enable magnetometer continuous conversion
    command[0] = 0x26; command[1] = (0 << 6) | (1 << 5) | (0 << 4) | (0 << 2) | 0;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, command, 2, false);
    
}

void VXLAccelerometerMagnetometerSensor::updateValue() {
    uint8_t tri_axis_buffer[] = {0,0,0,0,0,0};
    uint8_t command = 0;
    
    // Get latest accelerometer data
    for(uint8_t i = 0; i < 6; i++) {
        command = (0x28 + i);
        VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, &command, 1, true);
        VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_LSM303D, 1, &tri_axis_buffer[i], false);
    }
    memcpy(accelerometer_values, tri_axis_buffer, 6);
    
    // Get latest magnetometer data
    memset(tri_axis_buffer, 6, 0);
    for(uint8_t i = 0; i < 6; i++) {
        command = (0x08 + i);
        VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, &command, 1, true);
        VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_LSM303D, 1, &tri_axis_buffer[i], false);
    }
    memcpy(magnetometer_values, tri_axis_buffer, 6);
    
    // Get latest temperature data
    memset(tri_axis_buffer, 6, 0);
    for(uint8_t i = 0; i < 2; i++) {
        command = (0x05 + i);
        VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, &command, 1, true);
        VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_LSM303D, 1, &tri_axis_buffer[i], false);
    }
    memcpy(&temperature_value, tri_axis_buffer, 2);
}

int16_t * VXLAccelerometerMagnetometerSensor::getRawAccelerometerValues() {
    return accelerometer_values;
}

int16_t * VXLAccelerometerMagnetometerSensor::getRawMagnetometerValues() {
    return magnetometer_values;
}

int16_t VXLAccelerometerMagnetometerSensor::getRawTemperatureValue() {
    return temperature_value;
}

static bool acc_on = true;
static bool mag_on = true;
void VXLAccelerometerMagnetometerSensor::powerOnOffDevice(bool on) {
    powerOnOffAccelerometer(on);
    powerOnOffMagnetometer(on);
}

bool VXLAccelerometerMagnetometerSensor::getDevicePowerStatus() {
    return (acc_on && mag_on); // Placeholder
}

void VXLAccelerometerMagnetometerSensor::powerOnOffAccelerometer(bool on) {
    // Placeholder
    uint8_t ctrl_1_reg = 1; // Enable X axis
    ctrl_1_reg |= (1 << 1); // Enable Y axis
    ctrl_1_reg |= (1 << 2); // Enable Z axis
    ctrl_1_reg |= (0 << 3); // Disable block data update for acc/mag
    if(on) { 
        ctrl_1_reg |= (4 << 4); // Refresh at 25 Hz
    } else {
        ctrl_1_reg |= (0 << 4); // Power down the device
    }
    uint8_t commands[2];
    // Write out register 1
    commands[0] = 0x20; commands[1] = ctrl_1_reg;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, commands, 2, false);
    acc_on = on;
}

bool VXLAccelerometerMagnetometerSensor::getAccelerometerPowerStatus() {
    return acc_on; // Placeholder
}

void VXLAccelerometerMagnetometerSensor::powerOnOffMagnetometer(bool on) {
    uint8_t ctrl_7_reg = 0; // Enable magnetometer continuous-conversion mode
    if(!on) {
        ctrl_7_reg = 3; // Power-down mode (MD1,MD0=1,1)
    }
    ctrl_7_reg |= (0 << 2); // Disable low power mode
    ctrl_7_reg |= (0 << 4); // Disable temperature sensor only mode
    ctrl_7_reg |= (1 << 5); // Enable internal acc. data filter
    ctrl_7_reg |= (0 << 6); // Normal high pass filter mode for acc. data
    uint8_t commands[2];
    // Write out register 1
    commands[0] = 0x26; commands[1] = ctrl_7_reg;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, commands, 2, false);
    mag_on = on;
}

bool VXLAccelerometerMagnetometerSensor::getMagnetometerPowerStatus() {
    return mag_on;
}

uint8_t VXLAccelerometerMagnetometerSensor::getPeripheralID() {
    uint8_t cmd = 0x0F;
    uint8_t data[] = { 0 };
    
    // Read values back
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_LSM303D, &cmd, 1, true);
    VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_LSM303D, 1, data, false);
    
    return data[0];
}

bool VXLAccelerometerMagnetometerSensor::hasCorrectPeripheralID() {
    return (getPeripheralID() == 0x49);
}
