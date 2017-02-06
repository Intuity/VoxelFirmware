//
//  VXLLightSensor.cpp
//  VXL_Library
//
//  Created by Peter Birch on 05/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#include "VXLLightSensor.h"
#include "../VXLBoard.h"

/*
* Technical note: I2C interface
* In this case all commands must be prefixed with 0x80 to work
*
* Technical note: Integrator mode options
* Value 00 (0): Integrator is free running, uses preset integration time
* Value 01 (1): Manual start/stop of integration via ADC_EN field in Control Register
* Value 10 (2): Synchronise exactly one internally timed integration cycle, initiated by the SYNC pin
* Value 11 (3): Integrate over a specified number of pulses of the SYNC pin
*
* Technical note: SYNC pulse counting
* 4 bits, specifies the number of SYNC IN pulses to count when in INTEG_MODE 11 or if
* in INTEG_MODE 00 then this specifies the length of integration time: 0000 -> 12 ms, 0001 -> 100 ms, 0010 -> 400 ms
*
* Technical note: Gain control
* Analog gain control, 00 -> 1x, 01 -> 4x, 10 -> 16x, 11 -> 64x
*/

void VXLLightSensor::initPeripheral() {
#ifndef VXL_DISABLE_LIGHT
    uint8_t command[] = {0,0};
    
    // Setup control register
    // - Control: (1) Enable the ADC channels, (0) Turn the power ON
    command[0] = 0x80 | 0x00; command[1] = (1 << 1) | 1;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_TCS3414, command, 2, false);
    // - Timings: (6) Falling SYNC edge stops integration, (5,4) Preset integrator mode, 
    //            (3,2,1,0) Number of SYNC pulses to integrate over
    command[0] = 0x80 | 0x01; command[1] = (0 << 6) | (0 << 4) | 2;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_TCS3414, command, 2, false);
    // - Gain: (5,4) Analog gain, (3) Reserved, (2,1,0) Prescaler, divides by 2^Prescaler
    command[0] = 0x80 | 0x07; command[1] = (0 << 4) | (0 << 3) | 0;
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_TCS3414, command, 2, false);
#endif
}

void VXLLightSensor::updateValue() {
#ifndef VXL_DISABLE_LIGHT
    uint8_t read_cmd[] = { 0x80 | 0x40 | 0x10 }; // 0x80 instructs command, 0x40 instructs block protocol
    uint8_t * read_values = new uint8_t[8];
    memset(read_values, 0, 8);
    
    // Read values back
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_TCS3414, read_cmd, 1, true);
    VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_TCS3414, 8, read_values, false);
    
    // Construct 16-bit values
    channel_values[1] = (read_values[1] << 8) | read_values[0];
    channel_values[0] = (read_values[3] << 8) | read_values[2];
    channel_values[2] = (read_values[5] << 8) | read_values[4];
    channel_values[3] = (read_values[7] << 8) | read_values[6];

    delete read_values;
#endif
}

uint16_t * VXLLightSensor::getRawIntensityValues() {
    return channel_values;
}

static bool light_on = true;
void VXLLightSensor::powerOnOffDevice(bool on) {
#ifndef VXL_DISABLE_LIGHT
    // Setup control register
    uint8_t ctrl_reg = 0;
    if(on) {
        ctrl_reg = ctrl_reg | (1 << 1); // ADC_EN: 1 = Activate ADC channels, 0 = Deactivate
        ctrl_reg = ctrl_reg | 1;        // POWER: 1 = Power ON, 0 = Power OFF
    } else {
        ctrl_reg = ctrl_reg | (0 << 1); // ADC_EN: 1 = Activate ADC channels, 0 = Deactivate
        ctrl_reg = ctrl_reg | 0;        // POWER: 1 = Power ON, 0 = Power OFF
    }
    uint8_t cmd[] = { 0x80 | 0x00, ctrl_reg }; // The 0x80 instructs device this is a command
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_TCS3414, cmd, 2, false);
    light_on = on;
#endif
}

bool VXLLightSensor::getDevicePowerStatus() {
    return light_on;
}

uint8_t VXLLightSensor::getPeripheralID() {
#ifndef VXL_DISABLE_LIGHT
    uint8_t cmd[] = { 0x80 | 0x04 }; // The 0x80 instructs device this is a command
    uint8_t data[] = { 0 };
    
    // Read values back
    VXLBoard::getBoard()->sendDataToI2C(I2C_ADDR_TCS3414, cmd, 1, true);
    VXLBoard::getBoard()->readBackDataFromI2C(I2C_ADDR_TCS3414, 1, data, false);
    
    return data[0];
#else
    return (1 << 4);
#endif
}

bool VXLLightSensor::hasCorrectPeripheralID() {
    uint8_t data = getPeripheralID();
    uint8_t partno = (data >> 4) & 15;
    //uint8_t revno = data & 15;
    return (partno == 1);
}

