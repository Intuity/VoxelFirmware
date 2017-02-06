//
//  VXLGyroscopeSensor.h
//  VXL_Library
//
//  Created by Peter Birch on 05/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#ifndef __VXL_Library__VXLGyroscopeSensor__
#define __VXL_Library__VXLGyroscopeSensor__

#include <stdio.h>
#include "mbed-drivers/mbed.h"

#define I2C_ADDR_BMG160  0x68

class VXLGyroscopeSensor {
    
public:
    VXLGyroscopeSensor() {
        gyroscope_values = new int16_t[3];
        temperature_value = 0;
        initPeripheral();
    };
    
    /* Class methods */
    int16_t * getRawGyroscopeValues();
    int8_t getRawTemperatureValue();
    
    void updateValue();
    
    // Device management
    void initPeripheral();
    uint8_t getPeripheralID();
    bool hasCorrectPeripheralID();
    void powerOnOffDevice(bool on);
    bool getDevicePowerStatus();
    
private:
    // '_zero' values allow for calibration points to be set
    int16_t * gyroscope_values;
    int8_t temperature_value;
};

#endif /* defined(__VXL_Library__VXLGyroscopeSensor__) */
