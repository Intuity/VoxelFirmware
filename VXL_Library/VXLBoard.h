//
//  VXLBoard.h
//  VXL_Library
//
//  Created by Peter Birch on 04/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#include "mbed-drivers/mbed.h"

#ifndef __VXL_Library__VXLBoard__
#define __VXL_Library__VXLBoard__

#include <stdio.h>

// Voxel Constants
#define VXL_VERSION 1
#define VXLLED_PIN  P0_0
#define I2C_ADDR_PCF85063TP 0x51

// Disable controls
// #define VXL_DISABLE_BARO
// #define VXL_DISABLE_CLOCK
// #define VXL_DISABLE_GYRO
// #define VXL_DISABLE_LIGHT
// #define VXL_DISABLE_STORAGE

// Bluetooth interface
#include "VXLBLE.h"

// Memory interface
#include "VXLSPIStorage.h"

// All sensors
#include "Sensors/VXLTemperatureHumiditySensor.h"
#include "Sensors/VXLAccelerometerMagnetometerSensor.h"
#include "Sensors/VXLGyroscopeSensor.h"
#include "Sensors/VXLBarometricSensor.h"
#include "Sensors/VXLLightSensor.h"

/* 
 Singleton pattern taken from:
 http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
 Credits to: Crappy Experience Bye
*/

class VXLBoard {
public:
    static void setupBoard(); // Can only be called once
    static VXLBoard * getBoard();
    
    // Setup functions
    void loadPreferences();
    void savePreferences();
    
    // Preference access functions
    bool isMemoryInitialised();
    void initialiseMemory();
    void factoryResetDevice();
    
    uint8_t * getMACAddress();
    void setMACAddress(uint8_t * address);  // Address must be 6 bytes
    uint8_t * getDeviceID();
    void setDeviceID(uint8_t * identifier); // Device ID must be 8 bytes
    
    uint16_t getCaptureInterval();
    void setCaptureInterval(uint16_t interval);
    uint8_t getSamplesPerCapture();
    void setSamplesPerCapture(uint8_t samples);
    uint16_t getSampleInterval();
    void setSampleInterval(uint16_t interval);
    uint8_t getCaptureStoragePrefences();
    void setCaptureStoragePreferences(uint8_t storage);
    uint8_t getEnabledSensors();
    void setEnabledSensors(uint8_t enabled);
    void configureEnabledDevices();
    
    // Access sensor instances
    VXLTemperatureHumiditySensor * getTemperatureHumiditySensor() {
        return &temp_humid;
    };
    VXLAccelerometerMagnetometerSensor * getAccelerometerMagnetometerSensor() {
        return &accel_mag;
    };
    VXLGyroscopeSensor * getGyroscopicSensor() {
        return &gyro;
    };
    VXLBarometricSensor * getBarometricSensor() {
        return &barometer;
    };
    VXLLightSensor * getLightSensor() {
        return &light;
    };

    // Access Bluetooth interface
    VXLBLE * getBLEInterface() {
        return &ble;
    };
    
    // Access memory interface
    VXLSPIStorage * getStorage() {
        return &storage;
    };
    
    // Manage I2C
    // - Send data to an I2C device
    void sendDataToI2C(uint8_t address, uint8_t *data, int send_length, bool repeated);
    // - Receive data from an I2C device
    void readBackDataFromI2C(uint8_t address, uint8_t read_length, uint8_t * read_buffer, bool repeated);
    
    // Control the LED
    // - Set an RGB colour onto the LED
    void setLEDColour(uint8_t red, uint8_t green, uint8_t blue);
    // - Turn off the LED
    void turnOffLED();
    // - The colour to pulse, how long to pulse for (in ms), and whether to turn off afterwards
    void pulseLED(uint8_t red, uint8_t green, uint8_t blue, uint16_t pulse_length);
    
    // Manage the clock
    // - Set the clock to a particular time - years are relative to the year 2000
    void setClock(uint8_t years, uint8_t months, uint8_t days, uint8_t hours, uint8_t minutes, uint8_t seconds);
    // - Fetch the latest clock data from the device (updates stored variables)
    //   Returns an array in the order 0: years, 1: months, 2: days, 3: hours, 4: minutes, 5: seconds
    uint8_t * fetchTime();
    
    void logString(char * to_log);

private:
    VXLBoard(); // Constructor
    
    // Disable copy constructors
    VXLBoard(VXLBoard const&)            {}
    //VXLBoard &operator=(VXLBoard const&) {}
    
    // Sensor instances
    VXLTemperatureHumiditySensor temp_humid;
    VXLAccelerometerMagnetometerSensor accel_mag;
    VXLGyroscopeSensor gyro;
    VXLBarometricSensor barometer;
    VXLLightSensor light;
    
    // Bluetooth instance
    VXLBLE ble;
    
    // Storage instance
    VXLSPIStorage storage;
    
    // Clock values
    uint8_t * clk_values;
    
    // Control the LED
    void shiftLEDValue(uint8_t shift_byte);
    
};

#endif /* defined(__VXL_Library__VXLBoard__) */
