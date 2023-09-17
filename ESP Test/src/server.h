#pragma once
#include "BluetoothSerial.h"

class server
{
public:
    void recSetup();
    void recLoop();

private:
    uint8_t emitAdd[6] = {0x84, 0x0D, 0x8E, 0xE6, 0x91, 0x58}; // 84:0D:8E:E6:91:58
    BluetoothSerial SerialBTServer;
};