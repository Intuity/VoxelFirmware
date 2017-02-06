//
//  VXLBLE.cpp
//  VXL_Library
//
//  Created by Peter Birch on 15/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#include "VXLBoard.h"

static VXLBoard *singleton;
/*static I2C i2c(P0_22, P0_21);*/

static bool created_board = false;
void VXLBoard::setupBoard() {
    if(created_board) return;
    created_board = true;
    singleton = new VXLBoard();
};

VXLBoard * VXLBoard::getBoard() {
    return singleton;
};

VXLBoard::VXLBoard() {
    // Create led instances
    turnOffLED(); // Turn it off!
    
    // Create storage instance
    storage.configureSPI();
    
    // Load up preferences
    loadPreferences();
    // Check if memory initialised
    if(!isMemoryInitialised()) {
        initialiseMemory();
        return;
    }
    
    // Configure the clock
    // - Create and set the clock values to zero
#ifndef VXL_DISABLE_CLOCK
    clk_values = new uint8_t[6];
    memset(clk_values, 0, 6);
    // - Configure the I2C peripheral
    uint8_t reg_1_cmd[2] = { 
        0x00, 
        // 7: EXT_TEST (0 -> no test), 5: STOP (1 -> RTC disabled), 4: SR (1 -> reset), 2: CIE (1 -> generate interrupts),
        // 1: 12_24 (1 -> use 12hr mode), 0: CAP_SEL (1 -> 12.5 pF, 0 -> 7 pF), 6 & 3 are unused
        ((0 << 7) | (0 << 5) | (0 << 4) | (0 << 2) | (0 << 1) | 0)
    };
    sendDataToI2C(I2C_ADDR_PCF85063TP, reg_1_cmd, 2, false);
#endif
    
    // Bring up Bluetooth - do this last so it can
    // read parameters from elsewhere (i.e. SPI flash)
    ble.initBluetooth(VXLBLE_DEFAULT_INTERVAL);

    // Enable & disable sensors depending on requirements
    configureEnabledDevices();
    
    // Put everything into a low power state
    accel_mag.powerOnOffAccelerometer(false);
    accel_mag.powerOnOffMagnetometer(false);
    gyro.powerOnOffDevice(false);
    light.powerOnOffDevice(false);
    storage.powerOnOffDevice(false);
};

// I2C Management

// - Send data to an I2C device
void VXLBoard::sendDataToI2C(uint8_t address, uint8_t *data, int send_length, bool repeated) {
    I2C i2c(P0_22, P0_21);
    uint8_t shifted_address = address << 1;
    int write_result = i2c.write(shifted_address, (const char *)data, send_length, repeated);
}
// - Receive data from an I2C device
void VXLBoard::readBackDataFromI2C(uint8_t address, uint8_t read_length, uint8_t * read_buffer, bool repeated) {
    I2C i2c(P0_22, P0_21);
    uint8_t shifted_address = address << 1;
    i2c.read(shifted_address, (char *)read_buffer, read_length, repeated);
}

// Control the LED

DigitalOut led_ctl(VXLLED_PIN);
// - Shift a value out onto the LED control lines
void VXLBoard::shiftLEDValue(uint8_t shift_byte) {
    for(uint8_t i = 0; i < 8; i++) {
        led_ctl = 1;
        led_ctl = 1;
        led_ctl = 0;
        led_ctl = 0;
        led_ctl = ((shift_byte >> (7 - i)) & 1);
        led_ctl = ((shift_byte >> (7 - i)) & 1);
        for(uint8_t j = 0; j < 10; j++) {
            led_ctl = 0;
        }
    }
}
// - Controls the Texas Instruments (TI) TLC59731
void VXLBoard::setLEDColour(uint8_t red, uint8_t green, uint8_t blue) {
    // Signify start of packet
    shiftLEDValue(0x3A);
    // Now shift through the colours
    shiftLEDValue(red);
    shiftLEDValue(green);
    shiftLEDValue(blue);
    // Write the end of the sequence
    for(uint8_t i = 0; i < 160; i++) {
        led_ctl = 0;
    }
}
// - Turn off the LED
void VXLBoard::turnOffLED() {
    setLEDColour(0, 0, 0);
}
// - The colour to pulse, how long to pulse for (in ms), and whether to turn off afterwards
void VXLBoard::pulseLED(uint8_t red, uint8_t green, uint8_t blue, uint16_t pulse_length) {
    setLEDColour(red, green, blue);
    wait_ms(pulse_length);
    turnOffLED();
}

// Manage the clock
// - Set the clock to a specific date
void VXLBoard::setClock(uint8_t years, uint8_t months, uint8_t days, uint8_t hours, uint8_t minutes, uint8_t seconds) {
#ifndef VXL_DISABLE_CLOCK
    // Convert to BCD and write to device
    uint8_t tens = 0;
    uint8_t units = 0;
    uint8_t cmd[2] = { 0,0 };
    // - Seconds
    tens = (seconds / 10) & 7;
    units = (seconds % 10) & 15;
    cmd[0] = 0x04; cmd[1] = (tens << 4) | units;
    sendDataToI2C(I2C_ADDR_PCF85063TP, cmd, 2, false);
    // - Minutes
    tens = (minutes / 10) & 7;
    units = (minutes % 10) & 15;
    cmd[0] = 0x05; cmd[1] = (tens << 4) | units;
    sendDataToI2C(I2C_ADDR_PCF85063TP, cmd, 2, false);
    // - Hours
    tens = (hours / 10) & 3;
    units = (hours % 10) & 15;
    cmd[0] = 0x06; cmd[1] = (tens << 4) | units;
    sendDataToI2C(I2C_ADDR_PCF85063TP, cmd, 2, false);
    // - Days
    tens = (days / 10) & 3;
    units = (days % 10) & 15;
    cmd[0] = 0x07; cmd[1] = (tens << 4) | units;
    sendDataToI2C(I2C_ADDR_PCF85063TP, cmd, 2, false);
    // - Months
    tens = (months / 10) & 1;
    units = (months % 10) & 15;
    cmd[0] = 0x09; cmd[1] = (tens << 4) | units;
    sendDataToI2C(I2C_ADDR_PCF85063TP, cmd, 2, false);
    // - Years
    tens = (years / 10) & 15;
    units = (years % 10) & 15;
    cmd[0] = 0x0A; cmd[1] = (tens << 4) | units;
    sendDataToI2C(I2C_ADDR_PCF85063TP, cmd, 2, false);
#endif
}

// - Read the time back from the device
uint8_t * VXLBoard::fetchTime() {
#ifndef VXL_DISABLE_CLOCK
    // Read back the date
    // - Seconds
    uint8_t cmd = 0x04;
    sendDataToI2C(I2C_ADDR_PCF85063TP, &cmd, 1, true);
    readBackDataFromI2C(I2C_ADDR_PCF85063TP, 1, &clk_values[5], false);
    // - Minutes
    cmd = 0x05;
    sendDataToI2C(I2C_ADDR_PCF85063TP, &cmd, 1, true);
    readBackDataFromI2C(I2C_ADDR_PCF85063TP, 1, &clk_values[4], false);
    // - Hours
    cmd = 0x06;
    sendDataToI2C(I2C_ADDR_PCF85063TP, &cmd, 1, true);
    readBackDataFromI2C(I2C_ADDR_PCF85063TP, 1, &clk_values[3], false);
    // - Days
    cmd = 0x07;
    sendDataToI2C(I2C_ADDR_PCF85063TP, &cmd, 1, true);
    readBackDataFromI2C(I2C_ADDR_PCF85063TP, 1, &clk_values[2], false);
    // - Months
    cmd = 0x09;
    sendDataToI2C(I2C_ADDR_PCF85063TP, &cmd, 1, true);
    readBackDataFromI2C(I2C_ADDR_PCF85063TP, 1, &clk_values[1], false);
    // - Years
    cmd = 0x0A;
    sendDataToI2C(I2C_ADDR_PCF85063TP, &cmd, 1, true);
    readBackDataFromI2C(I2C_ADDR_PCF85063TP, 1, &clk_values[0], false);
    
    // Convert BCD to 1-byte integers
    for(uint8_t i = 0; i < 6; i++) {
        clk_values[i] = ((clk_values[i] >> 4) & 15) * 10 + (clk_values[i] & 15);
    }
    
    return clk_values;
#else
    return clk_values;
#endif
}

// Logging function
void VXLBoard::logString(char * to_log) {
    printf("%s\n\r", to_log);
}

// Data registers
static uint8_t identity_register[35] = {0};
static uint8_t preference_register[42] = {0};
static uint8_t ident_string[] = "VoxelV5Hardware";

void VXLBoard::loadPreferences() {
#ifndef VXL_DISABLE_STORAGE
    // Load identities from the identities register
    storage.readBytesIntoBuffer(
        VXL_MEM_PAGE_IDENTITIES,
        35,
        identity_register
    );
    
    uint8_t device_id[8] = {0};
    uint8_t mac_address[6] = {0};
    memcpy(device_id, identity_register, 8);
    memcpy(mac_address, &identity_register[8], 6);
    
    // Load preferences from storage
    storage.readBytesIntoBuffer(
        VXL_MEM_PAGE_PREFERENCES,
        42,         // Only using 42 bytes for preferences at present
        preference_register
    );
#else
    // Setup a dummy set of identities & preferences
    memset(identity_register, 0, 35);
    // - Device ID
    identity_register[0] = 'V'; identity_register[1] = 'o'; identity_register[2] = 'x'; identity_register[3] = 'e';
    identity_register[4] = 'l'; identity_register[5] = '_'; identity_register[6] = 'V'; identity_register[7] = '5';
    // - MAC Address
    identity_register[8] = 0x56; identity_register[9] = 0x58; identity_register[10] = 0x4C;
    identity_register[11] = 0x00; identity_register[12] = 0x00; identity_register[13] = 0x00;
    // - The initiation marker string
    memcpy(&identity_register[14], ident_string, 15);

    // Initialise the preferences register
    memset(preference_register, 0, 42);
    // - Setup the capture interval (default 10 seconds)
    uint16_t capture_interval = 10;
    memcpy(&preference_register[32], &capture_interval, 2);
    // - Setup the number of samples per capture (default 3)
    uint8_t capture_samples = 3;
    preference_register[34] = capture_samples;
    // - Setup the sample interval (default 100 milliseconds)
    uint16_t sample_interval = 100;
    memcpy(&preference_register[35], &sample_interval, 2);
    // - Setup the capture preferences
    preference_register[37] = 7;
    // - Setup the enabled sensors (all enabled, bit 7 is unused)
    preference_register[38] = 127;
#endif
}

void VXLBoard::savePreferences() {
#ifndef VXL_DISABLE_STORAGE
    // WARNING! We take the current values in memory to be the values 
    // to save, anything in flash is about to be overwritten
    
    // First, erase the entire first sector (pages 0-15)
    storage.eraseSector(0);
    // Now, reprogramme all of the preference options
    storage.programBytesFromBuffer(
        VXL_MEM_PAGE_IDENTITIES,
        35,
        identity_register
    );
    storage.programBytesFromBuffer(
        VXL_MEM_PAGE_PREFERENCES,
        42,
        preference_register
    );
#endif
}

// Identity functions
bool VXLBoard::isMemoryInitialised() {
#ifndef VXL_DISABLE_STORAGE
    for(uint8_t i = 0; i < 15; i++) {
        if(ident_string[i] != identity_register[14 + i]) {
            return false;
        }
    }
#endif
    return true;
}

void VXLBoard::initialiseMemory() {
#ifndef VXL_DISABLE_STORAGE
    setLEDColour(5,5,0);
    //storage.eraseChip();
    // Just erase the configuration sector, leave everything else in place (just in case we had
    // a power failure and we are coming back up with a corrupt 0 sector - this allows us to
    // still retrieve the data)
    storage.eraseSector(0);

    // Initialise the identities register
    memset(identity_register, 0, 35);
    // - Device ID
    identity_register[0] = 'V'; identity_register[1] = 'o'; identity_register[2] = 'x'; identity_register[3] = 'e';
    identity_register[4] = 'l'; identity_register[5] = '_'; identity_register[6] = 'V'; identity_register[7] = '5';
    // - MAC Address
    identity_register[8] = 0x56; identity_register[9] = 0x58; identity_register[10] = 0x4C;
    identity_register[11] = 0x00; identity_register[12] = 0x00; identity_register[13] = 0x00;
    // - The initiation marker string
    memcpy(&identity_register[14], ident_string, 15);
    // - Write to the device
    storage.programBytesFromBuffer(VXL_MEM_PAGE_IDENTITIES, 35, identity_register);

    // Initialise the preferences register
    memset(preference_register, 0, 42);
    // - Setup the capture interval (default 10 seconds)
    uint16_t capture_interval = 10;
    memcpy(&preference_register[32], &capture_interval, 2);
    // - Setup the number of samples per capture (default 3)
    uint8_t capture_samples = 3;
    preference_register[34] = capture_samples;
    // - Setup the sample interval (default 100 milliseconds)
    uint16_t sample_interval = 100;
    memcpy(&preference_register[35], &sample_interval, 2);
    // - Setup the capture preferences
    preference_register[37] = 7;
    // - Setup the enabled sensors (all enabled, bit 7 is unused)
    preference_register[38] = 127;
    // - Write to the device
    storage.programBytesFromBuffer(VXL_MEM_PAGE_PREFERENCES, 42, preference_register);

    // Reset the device for safety
    turnOffLED();
    NVIC_SystemReset();
#endif
}

void VXLBoard::factoryResetDevice() {
    setLEDColour(5,5,0);
    storage.eraseChip();
    initialiseMemory();
}

uint8_t * VXLBoard::getMACAddress() {
    return &identity_register[8];
}

void VXLBoard::setMACAddress(uint8_t * address) {
    memcpy(&identity_register[8], address, 6);
    savePreferences();
}

uint8_t * VXLBoard::getDeviceID() {
    return identity_register;
}

void VXLBoard::setDeviceID(uint8_t * identifier) {
    memcpy(identity_register, identifier, 8);
    savePreferences();
}

// Preference functions

uint16_t VXLBoard::getCaptureInterval() {
    uint16_t interval = 0;
    memcpy(&interval, &preference_register[32], 2);
    return interval;
}

void VXLBoard::setCaptureInterval(uint16_t interval) {
    if(interval < 10) interval = 10; // Minimum interval of 10 seconds
    memcpy(&preference_register[32], &interval, 2);
    savePreferences();
}

uint8_t VXLBoard::getSamplesPerCapture() {
    return preference_register[34];
}

void VXLBoard::setSamplesPerCapture(uint8_t samples) {
    preference_register[34] = samples;
    savePreferences();
}

uint16_t VXLBoard::getSampleInterval() {
    uint16_t interval = 0;
    memcpy(&interval, &preference_register[35], 2);
    return interval;
}

void VXLBoard::setSampleInterval(uint16_t interval) {
    memcpy(&preference_register[35], &interval, 2);
    savePreferences();
}

uint8_t VXLBoard::getCaptureStoragePrefences() {
    /*
    * 8-bit register, controlling:
    * [0] Capture enabled - set to '1' to start capture
    * [1] Store to flash - set to '1' to store to flash
    * [2] Wrap flash storage - set to '1' to allow wrap
    */
    return preference_register[37];
}

void VXLBoard::setCaptureStoragePreferences(uint8_t storage) {
    preference_register[37] = storage;
    savePreferences();
}

uint8_t VXLBoard::getEnabledSensors() {
    /*
    * 8-bit register, controlling:
    * [0] Clock enabled
    * [1] Accelerometer enabled
    * [2] Magnetometer enabled
    * [3] Gyroscope enabled
    * [4] Barometer enabled
    * [5] Temperature/humidity enabled
    * [6] Light enabled
    * [7] Unused
    */
    return preference_register[38];
}

void VXLBoard::setEnabledSensors(uint8_t enabled) {
    preference_register[38] = enabled;
    savePreferences();
    configureEnabledDevices();
}

void VXLBoard::configureEnabledDevices() {
    accel_mag.powerOnOffAccelerometer((((preference_register[38] >> 1) & 1) == 1) ? true : false);
    accel_mag.powerOnOffMagnetometer((((preference_register[38] >> 2) & 1) == 1) ? true : false);
    gyro.powerOnOffDevice((((preference_register[38] >> 3) & 1) == 1) ? true : false);
    light.powerOnOffDevice((((preference_register[38] >> 6) & 1) == 1) ? true : false);
}
