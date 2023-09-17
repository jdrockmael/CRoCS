#include "server.h"
bool SlaveConnected;

void recCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    if (event == ESP_SPP_OPEN_EVT)
    {
        Serial.println("Client Connected");
        SlaveConnected = true;
    }
    else if (event == ESP_SPP_CLOSE_EVT)
    {
        Serial.println("client disconnected");
        SlaveConnected = false;
    }
}

void server::recSetup()
{
    SlaveConnected = false;
    Serial.begin(9600);

    SerialBTServer.register_callback(recCallback);
    SerialBTServer.begin("receiver", true);
    SerialBTServer.connect(emitAdd);
}

void server::recLoop()
{

    if (SerialBTServer.available() && SlaveConnected)
    {
        Serial.println(SerialBTServer.read());
    }
}