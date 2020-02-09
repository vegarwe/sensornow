#include <Arduino.h>

#include "ArduinoJson.h"
#include "wifi_mqtt.h"

static HardwareSerial* debugger = NULL;


//--------------------------------------------------------------------------------
static void mqtt_on_message(String &topic, String &payload)
{
    if (debugger) {
        debugger->print("mqtt_on_message: ");
        debugger->print("[");
        debugger->print(topic);
        debugger->print("] ");
        debugger->println(payload);
    }
}


//--------------------------------------------------------------------------------
void setup() {
    debugger = &Serial;

    if (debugger)
    {
        debugger->begin(115200);
        delay(100);
        debugger->println("");
    }

    wifi_mqtt_setup(debugger, mqtt_on_message);
}

//--------------------------------------------------------------------------------
void loop()
{
    wifi_mqtt_loop();
}
