#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>

#include "config.h"

#define WIFI_CONNECTION_TIMEOUT 30000;


static WiFiClient client;
static HardwareSerial* debugger = NULL;


void wifi_loop()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        return;
    }

    // Connect to WiFi access point.
    Serial.print("Connecting to WiFi network: ");
    Serial.println(WIFI_SSID);

    // Make one first attempt at connect, this seems to considerably speed up the first connection
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(1000);

    // Loop (forever...), waiting for the WiFi connection to complete
    long vTimeout = millis() + WIFI_CONNECTION_TIMEOUT;
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        if (debugger) debugger->print(".");

        // If we timed out, disconnect and try again
        if (vTimeout < millis())
        {
            if (debugger)
            {
                debugger->print("Timout during connect. WiFi status is: ");
                debugger->println(WiFi.status());
            }
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            vTimeout = millis() + WIFI_CONNECTION_TIMEOUT;
        }
        yield();
    }

    Serial.println("");
    Serial.println("WiFi connected");

    if (debugger) {
        debugger->print("IP address: ");
        debugger->println(WiFi.localIP());
    }
}


void wifi_setup(HardwareSerial* dbg)
{
    debugger = dbg;

    // Setup Wifi
    WiFi.enableAP(false);
    WiFi.mode(WIFI_STA);
    wifi_loop();
}

