//
//  VXLBLE.cpp
//  VXL_Library
//
//  Created by Peter Birch on 15/02/2015.
//  Copyright (c) 2015 Peter Birch. All rights reserved.
//

#include "VXLBLE.h"
#include "VXLBoard.h"
#include "ble/Gap.h"

static bool initiated = false;

/*
* Status register breakdown:
* [0]    : Version (1-byte)
* [1]    : Operation status (1-byte) - '0' means ready, any other value means busy
* [2-3]  : Current memory page (2-bytes)
* [4]    : Current page slot index (1-byte)
* [5]    : Enabled sensors (1-byte)
* [6-7]  : Capture interval (2-bytes)
* [8]    : Samples per capture (1-byte)
* [9-10] : Sample interval (2-bytes)
* [11-18]: Device identifier (8-bytes)
* [19-24]: MAC address (6-bytes)
* [25-30]: Current date (6-bytes)
*/

// Base service definition
// - Define characteristics
static uint8_t live_data[43];
ReadOnlyArrayGattCharacteristic<uint8_t, sizeof(live_data)> live_data_char(VXLBLE_LIVE_DATA_CHAR, live_data, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);

static uint8_t config_data[10]; // Large enough to accept a command with variables
WriteOnlyArrayGattCharacteristic<uint8_t, sizeof(config_data)> config_char(VXLBLE_CONFIG_CHAR, config_data, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE);

static uint8_t stored_data[20];
ReadOnlyArrayGattCharacteristic<uint8_t, sizeof(stored_data)> stored_data_char(VXLBLE_STORED_DATA_CHAR, stored_data, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);

GattCharacteristic * base_chars[] = { &live_data_char, &config_char, &stored_data_char };

// - Define service
GattService base_service(VXLBLE_BASE_SERVICE, base_chars, sizeof(base_chars) / sizeof(GattCharacteristic *));

void onConnectCallback(const Gap::ConnectionCallbackParams_t *params) {
    // Something...
}

//void onDisconnectCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason) {
void onDisconnectCallback(const Gap::DisconnectionCallbackParams_t *params) {
    BLE::Instance().gap().startAdvertising();
}

static bool     stream_pages = false;
static uint16_t stream_page_current = 0;
static uint16_t stream_page_end = 0;
static uint8_t  cached_page[256] = {0};
static uint8_t  cache_progress = 0;

void VXLBLE::onDataSentCallback(unsigned count) {
    if(stream_pages) {
        streamPageCheck();
    }
}

void VXLBLE::streamPageCheck() {
    ble_error_t senderror = BLE_ERROR_NONE;
    // Queue up as many packets to be sent as possible
    while(senderror == BLE_ERROR_NONE) {
        if(cache_progress == 0) {
            if(stream_page_current > stream_page_end) {
                stream_pages = false;
                stream_page_current = stream_page_end = cache_progress = 0;
                break; // Don't read beyond requested length
            }
            VXLBoard::getBoard()->getStorage()->readBytesIntoBuffer(
                (stream_page_current + VXL_MEM_FRAMES_BASE_PAGE) * VXL_MEM_PAGE_SIZE,
                VXL_MEM_PAGE_SIZE,
                cached_page
            );
        }
    
        // Setup the send buffer
        uint8_t fragment[20];
        memcpy(&fragment[0], &stream_page_current, 2);
    
        // Can send 20 bytes total per MTU, with page number on front of notification
        // this leaves 18 bytes for data. As we already have the page number in every
        // notification we can skip the first 4 bytes of the page which are the page
        // number (and 2 blank bytes)
        memcpy(&fragment[2], &cached_page[18 * cache_progress + 4], 18);
        senderror = BLE::Instance().updateCharacteristicValue(stored_data_char.getValueHandle(), (uint8_t *)fragment, sizeof(fragment));
        cache_progress += 1;
        if(cache_progress >= 14) {
            // With 252 bytes to send per page, it takes us 14 sends per page to finish
            cache_progress = 0;
            stream_page_current++;
        }
    }
    if(senderror != BLE_ERROR_NONE) {
        // Will need to resend the previous fragment
        if(cache_progress == 0) {
            cache_progress = 13;
            stream_page_current -= 1;
        } else {
            cache_progress -= 1;
        }
    }
}

void VXLBLE::onDataWrittenCallback(const GattWriteCallbackParams *params) {
    if(params->handle == config_char.getValueHandle()) {
        // First byte is a command
        switch(params->data[0]) {
            case VXLBLE_CONFIG_NONE:
                break;
            case VXLBLE_READ_STATUS: {
                memset(live_data, 0, 43);
                live_data[0] = params->data[0]; // Reflect back the command
                live_data[1] = VXL_VERSION;
                uint8_t * device_name = VXLBoard::getBoard()->getDeviceID();
                uint8_t * mac_address = VXLBoard::getBoard()->getMACAddress();
                uint8_t * date = VXLBoard::getBoard()->fetchTime();
                uint16_t current_page = VXLBoard::getBoard()->getStorage()->getCurrentPage();
                uint16_t capture_interval = VXLBoard::getBoard()->getCaptureInterval();
                uint16_t sample_interval = VXLBoard::getBoard()->getSampleInterval();
                memcpy(&live_data[3], &current_page, 2);
                live_data[5] = VXLBoard::getBoard()->getStorage()->getFilledSlotsForCurrentPage();
                live_data[6] = VXLBoard::getBoard()->getEnabledSensors();
                memcpy(&live_data[7], &capture_interval, 2);
                live_data[9] = VXLBoard::getBoard()->getSamplesPerCapture();
                memcpy(&live_data[10], &sample_interval, 2);
                memcpy(&live_data[12], device_name, 8);
                memcpy(&live_data[20], mac_address, 6);
                memcpy(&live_data[26], date, 6);
                // Update the data characteristic with any updated value
                BLE::Instance().updateCharacteristicValue(live_data_char.getValueHandle(), (uint8_t *)live_data, sizeof(live_data));
                break;
            }
            case VXLBLE_FACTORY_RESET: {
                VXLBoard::getBoard()->factoryResetDevice();
                break;
            }
            case VXLBLE_CONFIG_LED: {
                if(params->len != 5) break;
                VXLBoard::getBoard()->pulseLED(params->data[1], params->data[2], params->data[3], params->data[4] * 100);
                break;
            }
            case VXLBLE_CONFIG_DATE: {
                if(params->len != 7) break;
                VXLBoard::getBoard()->setClock(
                    params->data[1], // Years
                    params->data[2], // Months
                    params->data[3], // Days
                    params->data[4], // Hours
                    params->data[5], // Minutes
                    params->data[6]  // Seconds
                );
                // Flash LED green as confirmation
                VXLBoard::getBoard()->pulseLED(0, 20, 0, 100);
                VXLBoard::getBoard()->turnOffLED();
                break;
            }
            case VXLBLE_CONFIG_MAC_ADDRESS: {
                if(params->len != 7) break;
                uint8_t mac_addr[6] = {0};
                memcpy(mac_addr, &params->data[1], 6);
                VXLBoard::getBoard()->setMACAddress(mac_addr);
                VXLBoard::getBoard()->pulseLED(0, 20, 0, 100);
                VXLBoard::getBoard()->turnOffLED();
                break;
            }
            case VXLBLE_CONFIG_DEVICE_ID: {
                if(params->len != 9) break;
                uint8_t device_id[8] = {0};
                memcpy(device_id, &params->data[1], 8);
                VXLBoard::getBoard()->setDeviceID(device_id);
                VXLBoard::getBoard()->pulseLED(0, 20, 0, 100);
                VXLBoard::getBoard()->turnOffLED();
                break;
            }
            case VXLBLE_CONFIG_CAPTURE_TIMINGS: {
                if(params->len != 6) break;
                uint16_t capture_interval = 0;
                uint8_t capture_samples = params->data[3];
                uint16_t sample_interval = 0;
                memcpy(&capture_interval, &params->data[1], 2);
                memcpy(&sample_interval, &params->data[4], 2);
                VXLBoard::getBoard()->setCaptureInterval(capture_interval);
                VXLBoard::getBoard()->setSamplesPerCapture(capture_samples);
                VXLBoard::getBoard()->setSampleInterval(sample_interval);
                VXLBoard::getBoard()->pulseLED(0, 20, 0, 100);
                VXLBoard::getBoard()->turnOffLED();
                // Reset device to use new timings
                NVIC_SystemReset();
                break;
            }
            case VXLBLE_CONFIG_ENABLED_SENSORS: {
                if(params->len != 2) break;
                uint8_t enabled = params->data[1];
                VXLBoard::getBoard()->setEnabledSensors(enabled);
                VXLBoard::getBoard()->pulseLED(0, 20, 0, 100);
                VXLBoard::getBoard()->turnOffLED();
                break;
            }
            case VXLBLE_CLEAR_FRAME_STORE: {
                VXLBoard::getBoard()->getStorage()->eraseFrameStore();
                // Flash LED green as confirmation
                VXLBoard::getBoard()->pulseLED(0, 20, 0, 100);
                VXLBoard::getBoard()->turnOffLED();
                break;
            }
            case VXLBLE_FORCE_SENSOR_UPDATE: {
                if(params->len != 1) break;
                memset(live_data, 0, 43);
                live_data[0] = params->data[0]; // Reflect back the command
                uint8_t * latest_data = VXLBoard::getBoard()->getStorage()->generateDataFrame();
                memcpy(&live_data[1], latest_data, 42);
                // Update the data characteristic with any updated value
                BLE::Instance().updateCharacteristicValue(live_data_char.getValueHandle(), (uint8_t *)live_data, sizeof(live_data));
                break;
            }
            case VXLBLE_FORCE_DEVICE_RESET: {
                if(params->len != 1) break;
                NVIC_SystemReset();
                break;
            }
            case VXLBLE_STREAM_STORED_PAGES: {
                if(params->len != 5) break;
                // Where to start from
                memcpy(&stream_page_current, &params->data[1], 2);
                // Where to read up to
                memcpy(&stream_page_end, &params->data[3], 2);
                // Setup for streaming
                cache_progress = 0;
                stream_pages = true;
                // Start streaming
                streamPageCheck();
                break;
            }
            default:
                break;
        }
    }
}

bool VXLBLE::initBluetooth(uint16_t interval) {
    if(initiated) return false;
    
    // Initiate the BLE device
    BLE &ble = BLE::Instance();
    if(ble.init() != BLE_ERROR_NONE) return false;
    
    // - Get the device name & mac address from the data storage
    uint8_t device_name[13] = {'V','X','L','_','V','O','X','E','L','_','V','5',0};
#ifndef VXL_DISABLE_STORAGE
    uint8_t * device_name_raw = VXLBoard::getBoard()->getDeviceID();
    memcpy(&device_name[4], device_name_raw, 8);
    uint8_t * mac_address = VXLBoard::getBoard()->getMACAddress();
#else
    uint8_t mac_address[6] = {0x58, 0x5C, 0x49, 0x00, 0x00, 0x00};
#endif
    
    // Setup MAC address
    ble.setAddress(Gap::ADDR_TYPE_PUBLIC, mac_address);
    
    // Set Tx power to +4 dBm
    ble.setTxPower(4);
    
    // Setup connection parameters
    Gap::ConnectionParams_t fast;
    ble.getPreferredConnectionParams(&fast);
    fast.minConnectionInterval = MIN_CONN_INTERVAL;
    fast.maxConnectionInterval = MAX_CONN_INTERVAL;
    fast.slaveLatency = SLAVE_LATENCY;
    fast.connectionSupervisionTimeout = SUPERVISION_TIMEOUT;
    ble.setPreferredConnectionParams(&fast);
    
    // Register callbacks
    ble.gap().onConnection(onConnectCallback);
    
    ble.gap().onDisconnection(onDisconnectCallback);
    
    ble.gattServer().onDataWritten(this, &VXLBLE::onDataWrittenCallback);
    ble.gattServer().onDataSent(this, &VXLBLE::onDataSentCallback);
    
    // - UUID is reversed in the advertisement frame
    uint8_t reversedServiceUUID[sizeof(VXLBLE_BASE_SERVICE)];
    for (unsigned int i = 0; i < sizeof(VXLBLE_BASE_SERVICE); i++) {
        reversedServiceUUID[i] =
            VXLBLE_BASE_SERVICE[sizeof(VXLBLE_BASE_SERVICE) - i - 1];
    }
    
    // - Ensure that the present services are advertised
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE); // BLE only, no classic BT
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED); // advertising type
    //ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME)); 
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, device_name, 8);
    ble.gap().accumulateAdvertisingPayload(
        GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
        (uint8_t *)reversedServiceUUID,
        sizeof(reversedServiceUUID)
    );
    
    // Set advertising interval
    if(interval == 0) interval = VXLBLE_DEFAULT_INTERVAL;
    ble.gap().setAdvertisingInterval(interval);
    
    // Start advertising
    ble.gap().startAdvertising();
    
    // - Register the services onto the BLE handler
    ble.addService(base_service);
    
    // Store that we have setup BLE
    initiated = true;
    
    return true;
}
