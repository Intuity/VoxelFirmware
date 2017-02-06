//
//  VXLSPIStorage.cpp
//  VXL_Library
//
//  Created by Peter Birch on 05/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#include "VXLSPIStorage.h"
#include "VXLBoard.h"
#include <stdio.h>

SPI spi(P0_23, P0_28, P0_24); // MOSI, MISO, SCLK
DigitalOut cs(P0_25);

static uint16_t current_frame_page = 0;
static uint8_t current_page_slot = 0;
static bool pause_storage = false;

void VXLSPIStorage::configureSPI() {
    // Force deselect of the chip
    cs = 1;
    
    // Setup SPI for 8-bit data in mode 3
    spi.format(8, 3);
    // Set clock speed to 1 MHz
    spi.frequency(1000000);
    
    // Enable the device
    cs = 0;
    
    // Read back the manufacturer, memory type & capacity
    // - First write in the command
    spi.write(0x9F);
    // - Read back the three items using dummy commands
    uint8_t manufacturer = spi.write(0x00);
    uint8_t mem_type = spi.write(0x00);
    uint8_t capacity = spi.write(0x00);
    
    // Cycle to clear the previous transaction
    cs = 1;
    cs = 0;
    
    // Find the latest page & slot stored being used in memory
    findCurrentPageAndSlotIndices();
    
    // Disable the device
    cs = 1;
    
    // Read back the lock status of the Security Registers
    /*
    cs = 0;
    spi.write(0x35);
    uint8_t sr_2 = spi.write(0x00);
    
    bool lb_3 = ((sr_2 >> 5) & 1) == 1;
    bool lb_2 = ((sr_2 >> 4) & 1) == 1;
    bool lb_1 = ((sr_2 >> 3) & 1) == 1;
    bool lb_0 = ((sr_2 >> 2) & 1) == 1; // SFDP register (pre-locked)
    pc_spi.printf("Memory: LB0: %i LB1: %i LB2: %i LB3: %i Reg: 0x%02x\n\r", lb_0, lb_1, lb_2, lb_3, sr_2);
    cs = 1;
    */
}

void VXLSPIStorage::setWriteEnabled(bool enable) {
    cs = 0;
    if(enable) {
        spi.write(0x06);
    } else {
        spi.write(0x04);
    }
    cs = 1;
}

bool VXLSPIStorage::isStorageBusy() {
    cs = 0;
    // Read status register 1
    spi.write(0x05);
    uint8_t result = spi.write(0x00);
    cs = 1;
    // Work out the device state
    return (result & 1); // Busy indicated by lowest bit
}

bool VXLSPIStorage::eraseChip() {
    if(isStorageBusy()) return false;
    
    setWriteEnabled(true);
    cs = 0;
    spi.write(0x60);
    cs = 1;
    
    while(isStorageBusy()) wait_ms(1);
    
    return true;    
}

bool VXLSPIStorage::eraseFrameStore() {
    if(isStorageBusy()) return false;
    
    pause_storage = true;
    
    // (current_frame_page + 1) - To ensure that all initialised pages are erased
    // ((current_frame_page + 1) / 16) + 1 - To compensate for the zeroeth sector being used for config
    uint16_t up_to_sector = ((current_frame_page + 1) / 16) + 1;
    
    for(uint16_t i = 1; i <= up_to_sector; i++) {
        eraseSector(i);
    }
    
    current_frame_page = 0;
    current_page_slot = 0;
    pause_storage = false;
    
    return true;
};

// Erase a 4 kB sector
bool VXLSPIStorage::eraseSector(int sector) {
    if(sector < 0 || sector > 1023) return false;
    
    // Calculate the 24-bit sector start address
    int start_addr = sector * 4096;
    uint8_t a23_a16 = (start_addr >> 16) & 255;
    uint8_t a15_a8 = (start_addr >> 8) & 255;
    uint8_t a7_a0 = start_addr & 255;
    
    setWriteEnabled(true);
    cs = 0;
    // Erase sector command
    spi.write(0x20);
    // 24-bit sector address
    spi.write(a23_a16);
    spi.write(a15_a8);
    spi.write(a7_a0);
    cs = 1;
    
    while(isStorageBusy()) wait_ms(1);
    
    return true;
}

// Erase a 64 kB block
bool VXLSPIStorage::eraseBlock(int block) {
    if(block < 0 || block > 63) return false;
    
    // Calculate the 24-bit sector start address
    int start_addr = block * 65536;
    uint8_t a23_a16 = (start_addr >> 16) & 255;
    uint8_t a15_a8 = (start_addr >> 8) & 255;
    uint8_t a7_a0 = start_addr & 255;
    
    setWriteEnabled(true);
    cs = 0;
    // Erase block command
    spi.write(0xD8);
    // 24-bit sector address
    spi.write(a23_a16);
    spi.write(a15_a8);
    spi.write(a7_a0);
    cs = 1;
    
    while(isStorageBusy()) wait_ms(1);
    
    return true;
}

void VXLSPIStorage::readBytesIntoBuffer(int start_addr, int num_bytes, uint8_t *buffer) {
    if(start_addr < 0 || start_addr > 4194304 || num_bytes < 0 || num_bytes > (4194304 - start_addr)) return;
    else if(isStorageBusy()) return;
    memset(buffer, 0, num_bytes);
    
    // Calculate the 24-bit sector start address
    uint8_t a23_a16 = (start_addr >> 16) & 255;
    uint8_t a15_a8 = (start_addr >> 8) & 255;
    uint8_t a7_a0 = start_addr & 255;
    
    // Issue the read command
    cs = 0;
    spi.write(0x03);
    // 24-bit sector address
    spi.write(a23_a16);
    spi.write(a15_a8);
    spi.write(a7_a0);
    // Now start reading back
    for(int i = 0; i < num_bytes; i++) {
        buffer[i] = spi.write(0x00);
    }
    // Stop reading
    cs = 1;
}

bool VXLSPIStorage::readFrameAtFrameIndex(uint32_t frame_index, uint8_t *buffer) {
    if(isStorageBusy()) return false;
    
    uint16_t page_index = (frame_index / 6) + VXL_MEM_FRAMES_BASE_PAGE;
    uint8_t slot_index = (frame_index % 6);
    // First get to the start of the right page, compensating for the preferences offset...
    uint32_t byte_index = (page_index * VXL_MEM_PAGE_SIZE);
    // ...now compensate for the 4 blank bytes and the offset of the frame within the page
    byte_index += 4 + (slot_index * VXL_MEM_FRAME_SIZE);
    
    readBytesIntoBuffer(
        byte_index,
        VXL_MEM_FRAME_SIZE,
        buffer
    );
    
    return true;
}

// Page start is in bytes - needs to be multiple of 256 bytes,
// can only program up to 256 bytes at a time
bool VXLSPIStorage::programBytesFromBuffer(int start_addr, int num_bytes, uint8_t *data) {
    if(start_addr < 0 || start_addr > 4194048 || num_bytes < 1 || num_bytes > VXL_MEM_PAGE_SIZE) {
        // This would write outside the valid range
        return false;
    } else if(((start_addr % VXL_MEM_PAGE_SIZE) + num_bytes) > VXL_MEM_PAGE_SIZE) {
        // This would wrap back on the page and starting re-writing the start, so refuse
        return false;
    } else if(isStorageBusy()) {
       return false;
    }
    
    // Calculate the 24-bit sector start address
    uint8_t a23_a16 = (start_addr >> 16) & 255;
    uint8_t a15_a8 = (start_addr >> 8) & 255;
    uint8_t a7_a0 = start_addr & 255;
    
    // Run page write
    setWriteEnabled(true);
    cs = 0;
    spi.write(0x02);
    spi.write(a23_a16);
    spi.write(a15_a8);
    spi.write(a7_a0);
    for(int i = 0; i < num_bytes; i++) {
        spi.write(data[i]);
    }
    cs = 1;
    
    while(isStorageBusy()) wait_ms(1);
    
    return true;
}

static uint8_t latest_frame[42];
uint8_t * VXLSPIStorage::generateDataFrame() {
    // Get the enabled devices
    uint8_t enabled = VXLBoard::getBoard()->getEnabledSensors();
    bool en_clk = ((enabled & 1) == 1) ? true : false;
    bool en_acc = (((enabled >> 1) & 1) == 1) ? true : false;
    bool en_mag = (((enabled >> 2) & 1) == 1) ? true : false;
    bool en_gyr = (((enabled >> 3) & 1) == 1) ? true : false;
    bool en_bar = (((enabled >> 4) & 1) == 1) ? true : false;
    bool en_t_h = (((enabled >> 5) & 1) == 1) ? true : false;
    bool en_lit = (((enabled >> 6) & 1) == 1) ? true : false;
    
    // Start up all of the required devices
    // - Note barometer & TH sensor have no power control
    VXLBoard::getBoard()->getAccelerometerMagnetometerSensor()->powerOnOffAccelerometer(en_acc);
    VXLBoard::getBoard()->getAccelerometerMagnetometerSensor()->powerOnOffMagnetometer(en_mag);
    VXLBoard::getBoard()->getGyroscopicSensor()->powerOnOffDevice(en_gyr);
    VXLBoard::getBoard()->getLightSensor()->powerOnOffDevice(en_lit);
    
    // Wait for everything to come up
    wait_ms(250);
    
    // Get the sampling parameters w/ some sanity checks
    uint16_t capture_interval = VXLBoard::getBoard()->getCaptureInterval();     // Seconds
    if(capture_interval < 1) capture_interval = 1;
    uint16_t sample_interval = VXLBoard::getBoard()->getSampleInterval();       // Milliseconds
    if(sample_interval < 10) sample_interval = 10;
    uint8_t capture_samples = VXLBoard::getBoard()->getSamplesPerCapture();     // Number
    if(capture_samples < 1) capture_samples = 1;
    
    // Verify that we can actually run this configuration
    if((((capture_samples - 1) * sample_interval) / 1000.0f) >= capture_interval) {
        // Force capture samples to 1
        capture_samples = 1;
    }
    
    int16_t acc_avgs[3] = {0,0,0};
    int16_t mag_avgs[3] = {0,0,0};
    uint16_t acc_mag_temp_avg = 0;
    int16_t gyr_avgs[3] = {0,0,0};
    uint8_t gyr_temp_avg = 0;
    uint32_t baro_pressure_avg = 0;
    uint16_t baro_temp_avg = 0;
    uint16_t th_temp_avg = 0;
    uint16_t th_humd_avg = 0;
    uint16_t light_avgs[4] = {0,0,0,0};
    
    for(uint8_t i = 0; i < capture_samples; i++) {
        // - Refresh the values of all of the sensors
        if(en_acc || en_mag) VXLBoard::getBoard()->getAccelerometerMagnetometerSensor()->updateValue();
        if(en_gyr) VXLBoard::getBoard()->getGyroscopicSensor()->updateValue();
        if(en_bar) VXLBoard::getBoard()->getBarometricSensor()->updateValue();
        if(en_t_h) VXLBoard::getBoard()->getTemperatureHumiditySensor()->updateValue();
        if(en_lit) VXLBoard::getBoard()->getLightSensor()->updateValue();
        
        // - Prepare and store the accelerometer data (w/ magnetometer 108 bits, uses 14 bytes)
        if(en_acc) {
            int16_t * acc_values = VXLBoard::getBoard()->getAccelerometerMagnetometerSensor()->getRawAccelerometerValues();
            acc_avgs[0] = ((acc_avgs[0] * i) + acc_values[0]) / (i + 1);
            acc_avgs[1] = ((acc_avgs[1] * i) + acc_values[1]) / (i + 1);
            acc_avgs[2] = ((acc_avgs[2] * i) + acc_values[2]) / (i + 1);
        }
        
        // - Prepare and store the magnetometer data
        if(en_mag) {
            uint16_t acc_mag_temperature = VXLBoard::getBoard()->getAccelerometerMagnetometerSensor()->getRawTemperatureValue(); // 12-bit value
            acc_mag_temp_avg = ((acc_mag_temp_avg * i) + acc_mag_temperature) / (i + 1);
            int16_t * mag_values = VXLBoard::getBoard()->getAccelerometerMagnetometerSensor()->getRawMagnetometerValues();
            mag_avgs[0] = ((mag_avgs[0] * i) + mag_values[0]) / (i + 1);
            mag_avgs[1] = ((mag_avgs[1] * i) + mag_values[1]) / (i + 1);
            mag_avgs[2] = ((mag_avgs[2] * i) + mag_values[2]) / (i + 1);
        }
        
        // - Prepare and store the gyroscope data (56 bits, uses 7 bytes)
        if(en_gyr) {
            int16_t * gyro_values = VXLBoard::getBoard()->getGyroscopicSensor()->getRawGyroscopeValues();
            gyr_avgs[0] = ((gyr_avgs[0] * i) + gyro_values[0]) / (i + 1);
            gyr_avgs[1] = ((gyr_avgs[1] * i) + gyro_values[1]) / (i + 1);
            gyr_avgs[2] = ((gyr_avgs[2] * i) + gyro_values[2]) / (i + 1);
            uint8_t gyro_temperature = VXLBoard::getBoard()->getGyroscopicSensor()->getRawTemperatureValue(); // 8-bit value
            gyr_temp_avg = ((gyr_temp_avg * i) + gyro_temperature) / (i + 1);
        }
        
        // - Prepare and store the barometer data (40 bits, uses 5 bytes)
        if(en_bar) {
            uint32_t baro_pressure_value = VXLBoard::getBoard()->getBarometricSensor()->getRawPressureValue();
            uint16_t baro_temperature_value = VXLBoard::getBoard()->getBarometricSensor()->getRawTemperatureValue();
            baro_pressure_avg = ((baro_pressure_avg * i) + baro_pressure_value) / (i + 1);
            baro_temp_avg = ((baro_temp_avg * i) + baro_temperature_value) / (i + 1);
        }
        
        // - Prepare and store the temp/humd data (26 bits, uses 4 bytes)
        if(en_t_h) {
            uint16_t th_temperature_value = VXLBoard::getBoard()->getTemperatureHumiditySensor()->getRawTemperature();
            uint16_t th_humidity_value = VXLBoard::getBoard()->getTemperatureHumiditySensor()->getRawHumidity();
            th_temp_avg = ((th_temp_avg * i) + th_temperature_value) / (i + 1);
            th_humd_avg = ((th_humd_avg * i) + th_humidity_value) / (i + 1);
        }
        
        // - Prepare and store the light channel values (64 bits, uses 8 bytes)
        if(en_lit) {
            uint16_t * light_levels = VXLBoard::getBoard()->getLightSensor()->getRawIntensityValues();
            light_avgs[0] = ((light_avgs[0] * i) + light_levels[0]) / (i + 1);
            light_avgs[1] = ((light_avgs[1] * i) + light_levels[1]) / (i + 1);
            light_avgs[2] = ((light_avgs[2] * i) + light_levels[2]) / (i + 1);
            light_avgs[3] = ((light_avgs[3] * i) + light_levels[3]) / (i + 1);
        }
        // Don't wait on the last sample, also ensures single sampling works fine
        if(i < (capture_samples - 1)) wait_ms(sample_interval);
    }
    
    // Now prepare the data frame
    memset(latest_frame, 0, 42);
    
    if(en_clk) {
        // - Prepare and store the compact timestamp (31 bits, uses 4 bytes)
        uint8_t * time_parts = VXLBoard::getBoard()->fetchTime();
        // Years stored as 5 bits, months stored as 4 bits, days stored as 5 bits
        uint32_t compact_time = (time_parts[0] & 31) | ((time_parts[1] & 15) << 5) | ((time_parts[2] & 31) << 9);
        // Hours stored as 5 bits, minutes stored as 6 bits, seconds stored as 6 bits
        compact_time |= ((time_parts[3] & 31) << 14) | ((time_parts[4] & 63) << 19) | ((time_parts[5] & 63) << 25);
        memcpy(latest_frame, &compact_time, 4);
    }
        
    memcpy(&latest_frame[4], acc_avgs, 6);
    memcpy(&latest_frame[10], &acc_mag_temp_avg, 2);
    memcpy(&latest_frame[12], mag_avgs, 6);
    memcpy(&latest_frame[18], gyr_avgs, 6);
    latest_frame[24] = gyr_temp_avg; // Only 1 byte - so no need for memcpy
    memcpy(&latest_frame[25], &baro_pressure_avg, 3);
    memcpy(&latest_frame[28], &baro_temp_avg, 2);
    memcpy(&latest_frame[30], &th_temp_avg, 2);
    memcpy(&latest_frame[32], &th_humd_avg, 2);
    memcpy(&latest_frame[34], light_avgs, 8);
    
    // Shut down all of the devices
    // - Note barometer & TH sensor have no power control
    VXLBoard::getBoard()->getAccelerometerMagnetometerSensor()->powerOnOffAccelerometer(false);
    VXLBoard::getBoard()->getAccelerometerMagnetometerSensor()->powerOnOffMagnetometer(false);
    VXLBoard::getBoard()->getGyroscopicSensor()->powerOnOffDevice(false);
    VXLBoard::getBoard()->getLightSensor()->powerOnOffDevice(false);
    
    return latest_frame;
}

void VXLSPIStorage::storeDataFrame() {
    // Check if we should be running - allows erase to run un-interrupted
    if(pause_storage) return;
    
    // Power up storage - wait for device to be ready
    powerOnOffDevice(true);
    wait_ms(500);
    
    // If this is a new page in memory, we need to store the index
    if(current_page_slot == 0) {
        uint8_t page_index[2] = {0};
        memcpy(page_index, &current_frame_page, 2);
        programBytesFromBuffer(
            (current_frame_page + VXL_MEM_FRAMES_BASE_PAGE) * VXL_MEM_PAGE_SIZE,
            2,
            page_index
        );
    }
    
    // Now, generate the frame
    uint8_t * frame = generateDataFrame();
    
    // Establish the byte index, firstly the root of the page we are working in...
    uint32_t byte_index = (current_frame_page + VXL_MEM_FRAMES_BASE_PAGE) * VXL_MEM_PAGE_SIZE;
    // ...now skip over the 2 bytes of page index & 2 blank bytes...
    byte_index += 4;
    // ...finally add on the offset from the current slot index
    byte_index += (current_page_slot * VXL_MEM_FRAME_SIZE);
    
    //pc_spi.printf("Programming bytes into page: %i slot: %i with byte index: %i\n\r", current_frame_page, current_page_slot, byte_index);
    
    // Write the frame to memory
    programBytesFromBuffer(
        byte_index,
        VXL_MEM_FRAME_SIZE,
        frame
    );
    
    // Update the current page & slot
    current_page_slot += 1;
    if(current_page_slot >= 6) {
        current_page_slot = 0;
        current_frame_page += 1;
        // Repeat on the last page if necessary!
        if(current_frame_page > (16000 - VXL_MEM_FRAMES_BASE_PAGE)) {
            current_frame_page -= 1;
        }
    }
    
    powerOnOffDevice(false);
}

void VXLSPIStorage::findCurrentPageAndSlotIndices() {
    // 16000 is the number of 256 byte pages we have (4 MB)
    uint8_t buffer[256] = {0};
    int16_t first_empty = -1;
    for(uint16_t i = VXL_MEM_FRAMES_BASE_PAGE; i < 16000; i++) {
        // Read in the page
        readBytesIntoBuffer(
            i * 256,
            4,
            buffer
        );
        // Get the index from the front of the buffer
        uint16_t page_index = 0;
        memcpy(&page_index, buffer, 2);
        if(page_index == 65535) {
            // Found an empty page!
            first_empty = i;
            break;
        }
        // Clear up
        memset(buffer, 0, 4);
    }
    // If first_empty = the base page index, then we haven't yet stored any data
    if(first_empty == VXL_MEM_FRAMES_BASE_PAGE) {
        current_frame_page = 0;
        current_page_slot = 0;
    
    // No free memory was detected
    } else if(first_empty < 0) {
        //pc_spi.printf("No free memory available on device\n\r");
        
    // Else we need to look at the previous page, and determine if all slots are full?
    } else {
        memset(buffer, 0, 256);
        readBytesIntoBuffer(
            (first_empty - 1) * 256,
            256,
            buffer
        );
        
        int8_t first_empty_slot = -1;
        for(uint8_t i = 0; i < 6; i++) { // Assuming a 42-byte frame
            uint8_t first_date_byte = buffer[4 + i*42];
            if(first_date_byte == 255) {
                // Uninitialised byte found
                first_empty_slot = i;
                break;
            }
        }
        
        if(first_empty_slot < 0 || first_empty_slot > 5) {
            // All slots in this page are full
            current_frame_page = (first_empty - VXL_MEM_FRAMES_BASE_PAGE);
            current_page_slot = 0;
        } else {
            current_frame_page = (first_empty - VXL_MEM_FRAMES_BASE_PAGE - 1);
            current_page_slot = first_empty_slot;
        }
    }
}

// Get the page in memory that we are currently storing into
uint16_t VXLSPIStorage::getCurrentPage() {
    // Load memory status from storage - page 5 holds the current page index for storage
    // all pages are adjusted by 20 to remove the data storage pages at the bottom
    return current_frame_page;
}

// Get the number of filled slots for the current page
uint8_t VXLSPIStorage::getFilledSlotsForCurrentPage() {
    return current_page_slot;
}

static bool storage_on = true;
void VXLSPIStorage::powerOnOffDevice(bool on) {
    /*
    // Enable the device
    cs = 0;
    
    // Write the power down or power up command
    if(on) {
        spi.write(0xAB);
    } else {
        spi.write(0xB9);
    }
    
    // Cycle to clear the previous transaction
    cs = 1;
    storage_on = on;   
    */
}

bool VXLSPIStorage::getDevicePowerStatus() {
    return storage_on;
}
