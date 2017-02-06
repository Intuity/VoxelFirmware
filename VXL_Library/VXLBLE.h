//
//  VXLBLE.h
//  VXL_Library
//
//  Created by Peter Birch on 15/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#ifndef __VXL_Library__VXLBLE__
#define __VXL_Library__VXLBLE__

#include <stdio.h>
#include <stdint.h>
#include "mbed-drivers/mbed.h"
#include "ble/BLE.h"

#define VXLBLE_DEFAULT_INTERVAL         2000

// Service UUIDs
// UUID-128 is 'PROJECTVOXELSTAT' -> 0x50 52 4f 4a 45 43 54 56 4f 58 45 4c 53 54 41 54
static const uint8_t VXLBLE_BASE_SERVICE[] = { 0x50, 0x52, 0x4F, 0x4A, 0x45, 0x43, 0x54, 0x56, 0x4F, 0x58, 0x45, 0x4C, 0x53, 0x54, 0x41, 0x54 };

// Characteristic UUIDs
#define VXLBLE_LIVE_DATA_CHAR           0xA001
#define VXLBLE_CONFIG_CHAR              0xA002
#define VXLBLE_STORED_DATA_CHAR         0xA003

#define MIN_CONN_INTERVAL   16  // 16*1.25 = 20 milliseconds
#define MAX_CONN_INTERVAL   40  // 40*1.25 = 50 milliseconds
#define SLAVE_LATENCY       0
#define SUPERVISION_TIMEOUT 400 // 400*1.25 = 500 ms

// Config function list
typedef enum {
    VXLBLE_CONFIG_NONE = 0,         // 0  (0x0)
    VXLBLE_READ_STATUS,             // 1  (0x1)
    VXLBLE_FACTORY_RESET,           // 2  (0x2)
    VXLBLE_CONFIG_LED,              // 3  (0x3)
    VXLBLE_CONFIG_DATE,             // 4  (0x4)
    VXLBLE_CONFIG_MAC_ADDRESS,      // 5  (0x5)
    VXLBLE_CONFIG_DEVICE_ID,        // 6  (0x6)
    VXLBLE_CONFIG_LOCK_IDS,         // 7  (0x7)
    VXLBLE_CONFIG_CAPTURE_TIMINGS,  // 8  (0x8)
    VXLBLE_CONFIG_ENABLED_SENSORS,  // 9  (0x9)
    VXLBLE_CLEAR_FRAME_STORE,       // 10 (0xA)
    VXLBLE_FORCE_SENSOR_UPDATE,     // 11 (0xB)
    VXLBLE_FORCE_DEVICE_RESET,      // 12 (0xC)
    VXLBLE_STREAM_STORED_PAGES      // 13 (0xD)
} VXLBLE_CONFIG_FUNCTION;

class VXLBLE {
public:
    // Initiate the BLE device an advertisement interval
    bool initBluetooth(uint16_t interval);
    // Callback function to wait for events on BLE connection
    void bleEventWait();
    // Function for managing the streaming of pages
    void streamPageCheck();
private:
    // Callbacks
    // - Callback whenever data is written to a register
    void onDataWrittenCallback(const GattWriteCallbackParams *params);
    // - Callback whenever data is sent to the central
    void onDataSentCallback(unsigned count);
};

#endif /* defined(__VXL_Library__VXLBLE__) */
