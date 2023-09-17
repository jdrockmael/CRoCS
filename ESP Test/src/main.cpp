#include "client.h"
#include "server.h"

bool isEmitter = true;

client emit;
server rec;

void setup()
{
    if (isEmitter)
    {
        emit.printMac();
        emit.emitSetup();
    }
    else
    {
        rec.recSetup();
    }
}

void loop()
{
    if (isEmitter)
    {
        emit.emitLoop();
    }
    else
    {
        rec.recLoop();
    }
}