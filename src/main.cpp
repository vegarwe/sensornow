#include <Arduino.h>
#include <esp_now.h>
#include <Wire.h>
#include <WiFi.h>


#if defined(BME280_SENSOR)
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define POWPIN 14               // what pin is providing 3.3V power
static Adafruit_BME280 bme;
#endif

#if defined(DHT22_SENSOR)
#include <DHT.h>

#define POWPIN 14               // what pin is providing 3.3V power
#define DHTPIN 21               // what pin we're connected to
static DHT dht(DHTPIN, DHT22);  // Initialize DHT sensor for normal 16mhz Arduino
#endif


#if defined(SIMULATOR)
    #define SLEEP_TIME       20 * 1000 * 1000L
#else
    #define SLEEP_TIME  20 * 60 * 1000 * 1000L
#endif


static HardwareSerial*  debugger            = NULL;
static uint8_t          broadcastAddress[]  = {0x24, 0x6F, 0x28, 0x60, 0x37, 0x29}; // AMS Display
static uint8_t          broadcastChannel    = 1;


void print_wakeup_reason()
{
    if (! debugger) return;

    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch(wakeup_reason)
    {
        case ESP_SLEEP_WAKEUP_EXT0:     debugger->println("Wakeup caused by external signal using RTC_IO"); break;
        case ESP_SLEEP_WAKEUP_EXT1:     debugger->println("Wakeup caused by external signal using RTC_CNTL"); break;
        case ESP_SLEEP_WAKEUP_TIMER:    debugger->println("Wakeup caused by timer"); break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD: debugger->println("Wakeup caused by touchpad"); break;
        case ESP_SLEEP_WAKEUP_ULP:      debugger->println("Wakeup caused by ULP program"); break;
        default:                        debugger->printf( "Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
    }
}


void goto_sleep(const char* reason)
{
    if (debugger)
    {
        debugger->print(reason);
        debugger->print(" ");
        debugger->println(millis());
        debugger->flush();
    }

    esp_sleep_enable_timer_wakeup(SLEEP_TIME);
    esp_deep_sleep_start();
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    goto_sleep(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void vTaskFunction(void * pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t x20SecondsInTicks = (15000) / portTICK_RATE_MS;

    vTaskDelayUntil(&xLastWakeTime, x20SecondsInTicks);

    goto_sleep("giving up");

    vTaskDelete(NULL);
}


void setup()
{
#if defined(SIMULATOR)
    debugger = &Serial;
#endif

    if (debugger)
    {
        debugger->begin(115200);
        delay(100);
        debugger->println("");
        debugger->println("Starting");
    }

    uint16_t battery_value = 0;
#if defined(BATT_VOLT_DIV)
    battery_value = analogRead(35);
#if defined(BATT_PROTECTION)
    if (battery_value < BATT_PROTECTION)
    {
        // No wake-up just shutdown and protect the battery
        if (debugger) debugger->printf("Battery too low(%d), going to sleep\n", battery_value);
        esp_deep_sleep_start();
    }
#endif
#endif

    if (debugger && battery_value != 0)
    {
        debugger->print("battery measurement: ");
        debugger->println(battery_value);
    }

    print_wakeup_reason();

#if   defined(BME280_SENSOR)
    pinMode(POWPIN, OUTPUT);
    digitalWrite(POWPIN, HIGH);
#elif defined(DHT22_SENSOR)
    pinMode(POWPIN, OUTPUT);
    digitalWrite(POWPIN, HIGH);

    dht.begin();
#endif

    // Start background timeout
    xTaskCreate(vTaskFunction, "vTaskFunction", 10000, (void *)1, tskIDLE_PRIORITY, NULL);

    // Connect to gateway
    WiFi.mode(WIFI_STA);
    esp_now_init();
    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo = {0};
    memcpy(peerInfo.peer_addr, broadcastAddress, sizeof(broadcastAddress));
    peerInfo.channel = broadcastChannel;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        goto_sleep("Failed to add peer");
    }


    static char payload[512];
#if   defined(BME280_SENSOR)
    // status = bme.begin(0x76, &Wire2)
    if (! bme.begin(BME280_ADDRESS_ALTERNATE)) {
        if (debugger)
        {
            debugger->print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
            debugger->print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
            debugger->print("   ID of 0x56-0x58 represents a BMP 280,\n");
            debugger->print("        ID of 0x60 represents a BME 280.\n");
            debugger->print("        ID of 0x61 represents a BME 680.\n");
        }
        goto_sleep("ERR: Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    }
    float temperature   = bme.readTemperature();
    float pressure      = bme.readPressure() / 100.0F;
    float humidity      = bme.readHumidity();

    digitalWrite(POWPIN, LOW);

    sprintf(payload,"/sensor_now/data { \"temperature\": %0.2f, \"pressure\": %0.2f, \"humidity\": %.2f, \"battery_voltage\": %u, \"millis\": %lu }",
            temperature,
            pressure,
            humidity,
            battery_value,
            millis());
#elif defined(DHT22_SENSOR)
    float temperature   = dht.readTemperature();
    float humidity      = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity))
    {
        if (debugger) debugger->println("Give sensor extra time");
        delay(2000);
        temperature     = dht.readTemperature();
        humidity        = dht.readHumidity();
    }

    digitalWrite(POWPIN, LOW);

    sprintf(payload,"/sensor_now/data { \"temperature\": %0.2f, \"humidity\": %.2f, \"battery_voltage\": %u, \"millis\": %lu }",
            temperature,
            humidity,
            battery_value,
            millis());
#else
    sprintf(payload,"/sensor_now/data { \"battery_voltage\": %u, \"millis\": %lu }",
            battery_value,
            millis());
#endif

    if (debugger)
    {
        debugger->printf("%d\n", strnlen(payload, sizeof(payload)));
        debugger->print(payload);
        debugger->println();
    }

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)payload, strnlen(payload, sizeof(payload)));
    if (result != ESP_OK && debugger)
    {
        debugger->println("Error sending the data");
    }
}

void loop()
{
    delay(10); // Wait for message sent callback
}

