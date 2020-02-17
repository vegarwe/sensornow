#include <HardwareSerial.h>
#include <MQTT.h>
#include <WiFiClientSecure.h>

#include "config.h"

#define WIFI_CONNECTION_TIMEOUT 30000;
#define AWS_MAX_RECONNECT_TRIES 5


// WiFi and MQTT client
static WiFiClientSecure net;
static MQTTClient mqtt(384);

// Object used for debug output
static HardwareSerial* debugger = NULL;


void wifi_loop()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        return;
    }

    // Connect to WiFi access point.
    if (debugger) debugger->print("Connecting to WiFi network: ");
    if (debugger) debugger->println(WIFI_SSID);

    // Make one first attempt at connect, this seems to considerably speed up the first connection
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(100);

    // Loop (forever...), waiting for the WiFi connection to complete
    long vTimeout = millis() + WIFI_CONNECTION_TIMEOUT;
    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
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

    //// If we still couldn't connect to the WiFi, go to deep sleep for a minute and try again.
    //if(WiFi.status() != WL_CONNECTED){
    //  esp_sleep_enable_timer_wakeup(1 * 60L * 1000000L);
    //  esp_deep_sleep_start();
    //}

    if (debugger) {
        debugger->println();
        debugger->print("WiFi connected, IP address: ");
        debugger->println(WiFi.localIP());
    }
}


void connectToAWS()
{
    // Try to connect to AWS and count how many times we retried.
    int retries = 0;
    if (debugger) debugger->println("Connecting to AWS IOT");

    while (!mqtt.connect(MQTT_NAME) && retries < AWS_MAX_RECONNECT_TRIES)
    {
        if (debugger) debugger->print(".");
        delay(200);
        retries++;
    }

    // Make sure that we did indeed successfully connect to the MQTT broker
    // If not we just end the function and wait for the next loop.
    if (! mqtt.connected())
    {
        if (debugger) debugger->println(" Timeout!");
        return;
    }

    // If we land here, we have successfully connected to AWS!
    // And we can subscribe to topics and send messages.
    if (debugger) debugger->println();
    if (debugger) debugger->println("MQTT Connected");

    // Allow some resources for the WiFi connection
    yield();

    // TODO: Subscribe to services?
    //mqtt.subscribe("fiskensubject");
}


void wifi_mqtt_setup(HardwareSerial* dbg, MQTTClientCallbackSimple messageCb)
{
    debugger = dbg;

    // Setup Wifi
    WiFi.enableAP(false);
    WiFi.mode(WIFI_STA);
    wifi_loop();

    // Configure WiFiClientSecure to use the AWS certificates we generated
    net.setCACert(aws_root_ca_pem);
    net.setCertificate(certificate_pem_crt);
    net.setPrivateKey(private_pem_key);

    // Setup MQTT
    mqtt.begin(MQTT_HOST, MQTT_PORT, net);
    mqtt.onMessage(messageCb);
    connectToAWS();
}


void wifi_mqtt_loop()
{
    wifi_loop();
    mqtt.loop();
    delay(10); // <- fixes some issues with WiFi stability

    // Reconnect to WiFi and MQTT as needed
    if (!mqtt.connected()) {
        connectToAWS();
    }
}


void wifi_mqtt_publish(String topic, String payload)
{
    mqtt.publish(topic, payload);
}