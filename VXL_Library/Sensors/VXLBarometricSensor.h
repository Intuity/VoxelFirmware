//
//  VXLBarometricSensor.h
//  VXL_Library
//
//  Created by Peter Birch on 05/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#ifndef __VXL_Library__VXLBarometricSensor__
#define __VXL_Library__VXLBarometricSensor__

#include <stdio.h>
#include "mbed-drivers/mbed.h"

#define I2C_ADDR_MPL3115A2 0x60

class VXLBarometricSensor {
    
public:
    VXLBarometricSensor() {
        pressure_value = 0;
        temperature_value = 0;
        initPeripheral();
    };
    
    /* Class methods */
    uint32_t getRawPressureValue();
    uint16_t getRawTemperatureValue();
    
    /* Inherited methods from VXLSensor */
    void updateValue();
    
    // Device management
    void initPeripheral();
    uint8_t getPeripheralID();
    bool hasCorrectPeripheralID();
    
private:
    // '_zero' values allow for calibration points to be set
    uint32_t pressure_value;
    uint16_t temperature_value;
    
};

#endif /* defined(__VXL_Library__VXLBarometricSensor__) */
