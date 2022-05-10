#include <Arduino.h>
//#include <adc.h>
//#include <bt/esp_bt.h>
//#include <esp_wifi.h>
#include <esp_now.h>
//#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include <Wire.h>
#include <WiFi.h>


#if defined(BME280_SENSOR)
    #include <Adafruit_Sensor.h>
    #include <Adafruit_BME280.h>

    #define POWPIN GPIO_NUM_14      // what pin is providing 3.3V power
    static Adafruit_BME280 bme;
#elif defined(DHT22_SENSOR)
    #include <DHT.h>

    #define POWPIN GPIO_NUM_14      // what pin is providing 3.3V power
    #define GNDPIN GPIO_NUM_15      // what pin is providing 3.3V power
    #define DHTPIN GPIO_NUM_21      // what pin we're connected to
    static DHT dht(DHTPIN, DHT22);  // Initialize DHT sensor for normal 16mhz Arduino
#endif


#if defined(SIMULATOR)
    #define SLEEP_TIME_S    (     10UL)
#else
    #define SLEEP_TIME_S    ( 5 * 60UL)
#endif
#define     SLEEP_TIME      ( SLEEP_TIME_S * 1000 * 1000UL)


typedef struct {
    float temperature;
    float humidity;
    float pressure;
} reading;


static HardwareSerial*          debugger            = NULL;
static uint8_t                  broadcastAddress[]  = {0x24, 0x6F, 0x28, 0x60, 0x37, 0x29}; // AMS Display
static uint8_t                  broadcastChannel    = 1;
static RTC_DATA_ATTR bool       very_first_reading  = true;
static RTC_DATA_ATTR uint8_t    reading_count       = 0;
static RTC_DATA_ATTR reading    readings[10]        = {0};
static RTC_DATA_ATTR uint32_t   success_count       = 0;


void print_wakeup_reason()
{
    if (! debugger) return;

    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    // ~/.platformio/packages/framework-arduinoespressif32@3.10004.201016/tools/sdk/include/esp32/rom/rtc.h
    //RESET_REASON rtc_get_reset_reason(int cpu_no);
    //RTCWDT_BROWN_OUT_RESET = 15,    /**<15, Reset when the vdd voltage is not stable*/

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

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();

    //adc_power_off();
    //esp_wifi_stop();
    //esp_bt_controller_disable();

    esp_sleep_enable_timer_wakeup(SLEEP_TIME);
    esp_deep_sleep_start();
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    success_count++;
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

#if defined(FEATHER_EZSBC)
double EzSBCReadVoltage(byte pin)
{
    double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
    if (reading < 1 || reading > 4095)
    {
        return 0;
    }

    // Return the voltage after fixin the ADC non-linearity
    return
        - 0.000000000000016 * pow(reading, 4)
        + 0.000000000118171 * pow(reading, 3)
        - 0.000000301211691 * pow(reading, 2)
        + 0.001109019271794 * reading
        + 0.034143524634089;
}
#endif


void setup()
{
#if defined(SIMULATOR)
    debugger = &Serial;
#endif
    //debugger = &Serial1;

    if (debugger)
    {
        debugger->begin(115200);
        debugger->println("");
        debugger->print("reading_count ");
        debugger->println(reading_count);
        debugger->print("success_count ");
        debugger->println(success_count);
        debugger->println("Starting");
    }
    else
    {
        Serial.end();
    }

#if defined(FEATHER_EZSBC)
    pinMode(GPIO_NUM_13, OUTPUT);
    digitalWrite(GPIO_NUM_13, HIGH);    // Turn off the blased red LED...!
#endif

    uint16_t battery_value = 0;
#if defined(BATT_VOLT_DIV)
#if defined(FEATHER_EZSBC)
    pinMode(GPIO_NUM_2, OUTPUT);        // Turn on voltage divider curcuit
    digitalWrite(GPIO_NUM_2, HIGH);
    delay(10);                          // Allow voltage to settle
    //battery_value = EzSBCReadVoltage(35);
#endif
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

    // Start background timeout
    xTaskCreate(vTaskFunction, "vTaskFunction", 10000, (void *)1, tskIDLE_PRIORITY, NULL);

#if   defined(BME280_SENSOR)
    pinMode(POWPIN, OUTPUT);
    digitalWrite(POWPIN, HIGH);

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

    readings[reading_count].temperature = bme.readTemperature();
    readings[reading_count].pressure    = bme.readPressure() / 100.0F;
    readings[reading_count].humidity    = bme.readHumidity();
    reading_count++;

    digitalWrite(POWPIN, LOW);
#elif defined(DHT22_SENSOR)
    pinMode(POWPIN, OUTPUT);
    digitalWrite(POWPIN, HIGH);
    pinMode(GNDPIN, OUTPUT);
    digitalWrite(GNDPIN, LOW);

    dht.begin();

    if (debugger) debugger->flush();
    rtc_gpio_hold_en(GNDPIN);
    rtc_gpio_hold_en(DHTPIN);
    rtc_gpio_hold_en(POWPIN);
    esp_sleep_enable_timer_wakeup(2100 * 1000);
    esp_light_sleep_start();

    readings[reading_count].temperature = dht.readTemperature();
    readings[reading_count].humidity    = dht.readHumidity();
    readings[reading_count].pressure    = NAN;

    if (isnan(readings[reading_count].temperature) || isnan(readings[reading_count].humidity))
    {
        goto_sleep("Did not get a valid reading");
    }
    reading_count++;

    digitalWrite(POWPIN, LOW);
    pinMode(DHTPIN, OUTPUT);
    digitalWrite(DHTPIN, LOW);
#else
    readings[reading_count].temperature = NAN;
    readings[reading_count].humidity    = NAN;
    readings[reading_count].pressure    = NAN;
    reading_count++;
#endif

    auto temp_diff = abs(readings[0].temperature - readings[reading_count - 1].temperature);
    if ((! very_first_reading)
        &&
        (temp_diff < 1.0f)
        &&
        (reading_count < (sizeof(readings) / sizeof(readings[0])))
        )
    {
        goto_sleep("caching");
    }
    very_first_reading = false;

    // Connect to gateway, send data
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

    static char payload[255]; // Max payload for ESP NOW
    int idx = 0;
    idx += sprintf(&payload[idx], "{\"b\":%u,\"s\":%lu,\"d\":[", battery_value, SLEEP_TIME_S);
    for (uint8_t i = 0; i < reading_count; i++)
    {
        idx += sprintf(&payload[idx], "[%.2f,%.2f],",
                readings[i].temperature,
                //readings[i].pressure,
                readings[i].humidity);
    }
    idx--; // Ignore superfluous comma
    idx += sprintf(&payload[idx], "]}");

    if (debugger)
    {
        debugger->printf("payload size: %d: ", strnlen(payload, sizeof(payload)));
        debugger->print(payload);
        debugger->println();
    }

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)payload, strnlen(payload, sizeof(payload)));
    if (result != ESP_OK && debugger)
    {
        debugger->println("Error sending the data");
    }

    reading_count = 0;
}

void loop()
{
    delay(10); // Wait for message sent callback
}

