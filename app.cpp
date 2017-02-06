/*
Author:     Peter Birch
Team:       Project Voxel
Website:    www.projectvoxel.com

Copyright Project Voxel 2015, redistribution is prohibited without direct permission
from Project Voxel.
*/

#include "mbed-drivers/mbed.h"
#include "VXL_Library/VXLBoard.h"

void storeFrame() {
    VXLBoard::getBoard()->getStorage()->storeDataFrame();
}

void app_start(int, char**) {
    VXLBoard::getBoard()->pulseLED(5, 5, 0, 500);
    VXLBoard::setupBoard();
    // All ready to go
    VXLBoard::getBoard()->pulseLED(0, 5, 0, 500);
    VXLBoard::getBoard()->turnOffLED();
    
    // Schedule a periodic callback to store a data frame
    uint16_t capture_interval = VXLBoard::getBoard()->getCaptureInterval();
    // - Default to 1 second
    if(capture_interval < 10 || capture_interval >= 65535) capture_interval = 10;
    minar::Scheduler::postCallback(storeFrame).period(minar::milliseconds(capture_interval * 1000));
}