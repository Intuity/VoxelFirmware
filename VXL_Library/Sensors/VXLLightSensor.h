//
//  VXLLightSensor.h
//  VXL_Library
//
//  Created by Peter Birch on 05/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#ifndef __VXL_Library__VXLLightSensor__
#define __VXL_Library__VXLLightSensor__

#include <stdio.h>

#include "mbed-drivers/mbed.h"

#define I2C_ADDR_TCS3414 0x39

class VXLLightSensor {
    
public:
    VXLLightSensor() {
        // Create and initialise
        channel_values = new uint16_t[4];
        initPeripheral();
    };
    
    /* Class methods */
    uint16_t * getRawIntensityValues();
    
    void updateValue();
    
    // Device management
    void initPeripheral();
    uint8_t getPeripheralID();
    bool hasCorrectPeripheralID();
    void powerOnOffDevice(bool on);
    bool getDevicePowerStatus();
    
private:
    // '_zero' values allow for calibration points to be set
    uint16_t * channel_values; // R, G, B, Clear
    
};

#endif /* defined(__VXL_Library__VXLLightSensor__) */
