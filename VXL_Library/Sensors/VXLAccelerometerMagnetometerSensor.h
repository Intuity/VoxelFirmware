//
//  VXLAccelerometerMagnetometerSensor.h
//  VXL_Library
//
//  Created by Peter Birch on 05/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#ifndef __VXL_Library__VXLAccelerometerMagnetometerSensor__
#define __VXL_Library__VXLAccelerometerMagnetometerSensor__

#include <stdio.h>

#define I2C_ADDR_LSM303D 0x1D

class VXLAccelerometerMagnetometerSensor {
    
public:
    VXLAccelerometerMagnetometerSensor() {
        accelerometer_values = new int16_t[3];
        magnetometer_values = new int16_t[3];        
        initPeripheral();
    };
    
    /* Class methods */
    int16_t * getRawAccelerometerValues();
    int16_t * getRawMagnetometerValues();
    int16_t getRawTemperatureValue();
    
    void powerOnOffAccelerometer(bool on);
    bool getAccelerometerPowerStatus();
    void powerOnOffMagnetometer(bool on);
    bool getMagnetometerPowerStatus();
    
    void updateValue();
    
    // Device management
    void initPeripheral();
    uint8_t getPeripheralID();
    bool hasCorrectPeripheralID();
    void powerOnOffDevice(bool on);
    bool getDevicePowerStatus();
    
private:
    // '_zero' values allow for calibration points to be set
    int16_t * accelerometer_values;
    int16_t * magnetometer_values;
    int16_t temperature_value;
    
};

#endif /* defined(__VXL_Library__VXLAccelerometerMagnetometerSensor__) */
