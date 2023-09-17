#pragma once
#include "BluetoothSerial.h"

class client
{
public:
    void emitSetup();
    void emitLoop();
    void printMac();

private:
    BluetoothSerial SerialBTClient;
};