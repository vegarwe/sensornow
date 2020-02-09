#include <HardwareSerial.h>
#include <MQTT.h>
#include <MQTTClient.h>
#include <WiFi.h>

#include "config.h"


static MQTTClient mqtt(384);
static HardwareSerial* debugger = NULL;


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


static void mqtt_connect()
{
    Serial.print("Connecting MQTT to ");
    Serial.println(MQTT_HOST);

    String mqttClientID(WiFi.macAddress());

    // Wait for the MQTT connection to complete
    while (!mqtt.connected())
    {
        if (! mqtt.connect(mqttClientID.c_str(), MQTT_USER, MQTT_PASS))
        {
            if (debugger) debugger->println("MQTT connect failed, trying again in 5 seconds");

            // Wait 2 seconds before retrying
            mqtt.disconnect();
            delay(1000);
            continue;
        }

        // Allow some resources for other threads
        yield();

        mqtt.subscribe("/raiomremote/cmd/#");
        mqtt.subscribe("/raiomremote/api/#");
    }

    Serial.println("Successfully connected to MQTT!");
}


void mqtt_setup(HardwareSerial* dbg)
{
    debugger = dbg;

    // Setup MQTT
    mqtt.begin(MQTT_HOST, client);
    mqtt.onMessage(mqtt_on_message);
    mqtt_connect();
}

void mqtt_loop()
{
    mqtt.loop();
    delay(10); // <- fixes some issues with stability

    // Reconnect to MQTT as needed
    if (!mqtt.connected()) {
        mqtt_connect();
    }
}
