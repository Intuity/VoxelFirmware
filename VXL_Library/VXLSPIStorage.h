//
//  VXLSPIStorage.h
//  VXL_Library
//
//  Created by Peter Birch on 05/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#ifndef __VXL_Library__VXLSPIStorage__
#define __VXL_Library__VXLSPIStorage__

#include <stdio.h>
#include "mbed-drivers/mbed.h"

// Constants
// - Specific registers
#define VXL_MEM_PAGE_IDENTITIES     256
#define VXL_MEM_PAGE_PREFERENCES    512

// - Generic constants
#define VXL_MEM_PAGE_SIZE           256
#define VXL_MEM_FRAMES_BASE_PAGE    16
#define VXL_MEM_FRAME_SIZE          42

class VXLSPIStorage {
    
public:

    // Configure the SPI interface to the storage module
    void configureSPI();
    // Send the write enable or disable sequence to the device
    void setWriteEnabled(bool enable);
    // Check if the storage is busy performing an operation - true if busy
    bool isStorageBusy();
    // Erase the contents of the entire chip - set all to FF
    bool eraseChip();
    // Erase only the data frame store of the chip - set all to FF
    bool eraseFrameStore();
    // Erase a particular sector of the chip (4kB) to all FF
    bool eraseSector(int sector);
    // Erase a particular block of the chip (64kB) to all FF
    bool eraseBlock(int block);
    // Read bytes from the chip into a passed buffer
    void readBytesIntoBuffer(int start_addr, int num_bytes, uint8_t *buffer);
    // Read a stored frame from the memory at a provided frame index
    bool readFrameAtFrameIndex(uint32_t frame_index, uint8_t *buffer);
    // Programme up to 256 bytes of memory in one go from an arbitrary start
    // (byte) address in memory
    bool programBytesFromBuffer(int start_addr, int num_bytes, uint8_t *data);
    // Generate a data frame from all of the available sensors
    uint8_t * generateDataFrame();
    // Store a data frame from all of the available sensors
    void storeDataFrame();
    // Get the page in memory that we are currently storing into
    uint16_t getCurrentPage();
    // Get the number of filled slots for the current page
    uint8_t getFilledSlotsForCurrentPage();
    // Control the power status of the device (true -> on, false -> off)
    void powerOnOffDevice(bool on);
    // Read back the state of the device power
    bool getDevicePowerStatus();
    
private:

    // Search through memory to find the last edited page and page slot
    void findCurrentPageAndSlotIndices();
    
};

#endif /* defined(__VXL_Library__VXLSPIStorage__) */
