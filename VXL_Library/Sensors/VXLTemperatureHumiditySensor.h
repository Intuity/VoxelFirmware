//
//  VXLTemperatureHumiditySensor.h
//  VXL_Library
//
//  Created by Peter Birch on 05/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#ifndef __VXL_Library__VXLTemperatureHumiditySensor__
#define __VXL_Library__VXLTemperatureHumiditySensor__

#include <stdio.h>

#include "mbed-drivers/mbed.h"

#define I2C_ADDR_Si7020 0x40

class VXLTemperatureHumiditySensor  {
    
public:
    VXLTemperatureHumiditySensor() {
        temperature = 0;
        humidity = 0;
        initPeripheral();
    };
    
    /* Class methods */
    uint16_t getRawTemperature();
    uint16_t getRawHumidity();
    
    void updateValue();
    
    // Device management
    void initPeripheral();
    uint8_t getPeripheralID();
    bool hasCorrectPeripheralID();
    
private:
    // '_zero' values allow for calibration points to be set
    uint16_t temperature;
    uint16_t humidity;
    
};

#endif /* defined(__VXL_Library__VXLTemperatureHumiditySensor__) */
