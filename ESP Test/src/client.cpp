#include <WiFi.h>
#include "client.h"
bool MasterConnected;

void emitCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    if (event == ESP_SPP_SRV_OPEN_EVT)
    {
        Serial.println("Client Connected");
        MasterConnected = true;
    }
    else if (event == ESP_SPP_CLOSE_EVT)
    {
        Serial.println("client disconnected");
        MasterConnected = false;
    }
}

void client::printMac()
{
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
}

void client::emitSetup()
{
    Serial.begin(9600); 

    SerialBTClient.register_callback(emitCallback);
    SerialBTClient.begin("emitter");
}

void client::emitLoop()
{
    if (MasterConnected)
    {
        SerialBTClient.println("hello world");
    }
}