#include <Arduino.h>
#include <MQTT.h>
#include <WiFiClientSecure.h>
#include <Wire.h>

#include "config.h"

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


// WiFi and MQTT client
static WiFiClientSecure net;
static MQTTClient mqtt(384);


#define SLEEP_TIME                          20 * 60 * 1000 * 1000L
//#define SLEEP_TIME                           1 * 60 * 1000 * 1000L
//#define SLEEP_TIME                           1 * 20 * 1000 * 1000L

static HardwareSerial* debugger = NULL;

static RTC_DATA_ATTR int boot_count = 0;
static RTC_DATA_ATTR int success_count = 0;

static RTC_DATA_ATTR struct {
    byte mac [ 6 ];
    byte mode;
    byte chl;
    uint32_t ip;
    uint32_t gw;
    uint32_t msk;
    uint32_t dns;
    uint32_t seq;
    uint32_t chk;
} cfgbuf;


static bool checkCfg()
{
    // See https://github.com/tve/low-power-wifi/blob/master/esp32-deep-sleep-mqtts/src/main.ino
    uint32_t x = 0;
    uint32_t *p = (uint32_t *)cfgbuf.mac;
    for (uint32_t i = 0; i < sizeof(cfgbuf)/4; i++) x += p[i];
    if (debugger) debugger->printf("RTC read: chk=%x x=%x ip=%08x mode=%d %s\n",
            cfgbuf.chk, x, cfgbuf.ip, cfgbuf.mode, x==0?"OK":"FAIL");
    if (x == 0 && cfgbuf.ip != 0) return true;
    if (debugger) debugger->println("NVRAM cfg init");
    // bad checksum, init data
    for (uint32_t i = 0; i < 6; i++) cfgbuf.mac[i] = 0xff;
    cfgbuf.mode = 0; // chk err, reconfig
    cfgbuf.chl = 0;
    cfgbuf.ip = IPAddress(0, 0, 0, 0);
    cfgbuf.gw = IPAddress(0, 0, 0, 0);
    cfgbuf.msk = IPAddress(255, 255, 255, 0);
    cfgbuf.dns = IPAddress(0, 0, 0, 0);
    cfgbuf.seq = 100;
    return false;
}


static void writecfg()
{
    // save new info
    uint8_t *bssid = WiFi.BSSID();
    for (int i=0; i<sizeof(cfgbuf.mac); i++) cfgbuf.mac[i] = bssid[i];
    cfgbuf.chl = WiFi.channel();
    cfgbuf.ip = WiFi.localIP();
    cfgbuf.gw = WiFi.gatewayIP();
    cfgbuf.msk = WiFi.subnetMask();
    cfgbuf.dns = WiFi.dnsIP();
    // recalculate checksum
    uint32_t x = 0;
    uint32_t *p = (uint32_t *)cfgbuf.mac;
    for (uint32_t i = 0; i < sizeof(cfgbuf)/4-1; i++) x += p[i];
    cfgbuf.chk = -x;
    if (debugger) debugger->printf("RTC write: chk=%x x=%x ip=%08x mode=%d\n", cfgbuf.chk, x, cfgbuf.ip, cfgbuf.mode);
}


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

#if defined(CHIRP_SENSOR)
//Default I2C Address of the sensor
#define SOILMOISTURESENSOR_DEFAULT_ADDR     0x20

//Soil Moisture Sensor Register Addresses
#define SOILMOISTURESENSOR_GET_CAPACITANCE  0x00 // (r) 2 bytes
#define SOILMOISTURESENSOR_SET_ADDRESS      0x01 // (w) 1 byte
#define SOILMOISTURESENSOR_GET_ADDRESS      0x02 // (r) 1 byte
#define SOILMOISTURESENSOR_MEASURE_LIGHT    0x03 // (w) n/a
#define SOILMOISTURESENSOR_GET_LIGHT        0x04 // (r) 2 bytes
#define SOILMOISTURESENSOR_GET_TEMPERATURE  0x05 // (r) 2 bytes
#define SOILMOISTURESENSOR_RESET            0x06 // (w) n/a
#define SOILMOISTURESENSOR_GET_VERSION      0x07 // (r) 1 bytes
#define SOILMOISTURESENSOR_SLEEP            0x08 // (w) n/a
#define SOILMOISTURESENSOR_GET_BUSY         0x09 // (r) 1 bytes


unsigned int readI2CRegister16bit(int addr, int reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
    delay(20); // TODO: Enough time? I2CSoilMoistureSensor.cpp seems to think so
    Wire.requestFrom(addr, 2);
    unsigned int t = Wire.read() << 8;
    t = t | Wire.read();
    return t;
}

#endif


void print_wakeup_reason() {
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
        default:                        debugger->printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
    }
}


void vTaskFunction(void * pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t x20SecondsInTicks = (15000) / portTICK_RATE_MS;

    if (cfgbuf.ip == 0) {
        x20SecondsInTicks = (25500) / portTICK_RATE_MS;
        if (debugger) debugger->println("Grant extra time");
    }

    // Wait for the next cycle.
    vTaskDelayUntil(&xLastWakeTime, x20SecondsInTicks);

    success_count = 0;

    Serial.print("giving up after ");
    Serial.println(millis());
    Serial.flush();

    cfgbuf.ip = 0; // Invalidate stored WIFI settings
    esp_sleep_enable_timer_wakeup(SLEEP_TIME);
    esp_deep_sleep_start();

    vTaskDelete(NULL);
}


//--------------------------------------------------------------------------------
void setup() {
    //debugger = &Serial;

    if (debugger)
    {
        debugger->begin(115200);
        delay(100);
        debugger->println("");
        debugger->println("Starting");
    }

    uint16_t analog_value = 0;
#if defined(BATT_VOLT_DIV)
    analog_value = analogRead(35);
#if defined(BATT_PROTECTION)
    if (analog_value < BATT_PROTECTION)
    {
        // No wake-up just shutdown and protect the battery
        if (debugger) debugger->printf("Battery too low(%d), going to sleep\n", analog_value);
        esp_deep_sleep_start();
    }
#endif
#endif

    if (debugger && analog_value != 0)
    {
        debugger->print("battery measurement: ");
        debugger->println(analog_value);
        // (2112 * 2 * 3.3 / 4095) + 0.366 ~= 3.77
        // (2444 * 2 * 3.3 / 4095) + 0.366 ~= 4.2 -- 4.3
    }

    ++boot_count;
    if (debugger) debugger->println("Boot number: " + String(boot_count));
    print_wakeup_reason();

#if defined(CHIRP_SENSOR)
    pinMode(23, OUTPUT);
    digitalWrite(23, HIGH);

    Wire.begin();
#elif defined(BME280_SENSOR)
    pinMode(POWPIN, OUTPUT);
    digitalWrite(POWPIN, HIGH);
#elif defined(DHT22_SENSOR)
    pinMode(POWPIN, OUTPUT);
    digitalWrite(POWPIN, HIGH);

    dht.begin();
#endif

    checkCfg();

    // Start background timeout
    xTaskCreate(vTaskFunction, "vTaskFunction", 10000, (void *)1, tskIDLE_PRIORITY, NULL);

    //if (WiFi.getMode() != WIFI_OFF)
    //{
    //    if (debugger) debugger->println("Wifi wasn't off!");
    //    WiFi.persistent(true);
    //    WiFi.mode(WIFI_OFF);
    //}

    //WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    bool ok;

    unsigned long fisken = millis();
    if (cfgbuf.mode == 2)
    {
        ok = WiFi.config(cfgbuf.ip, cfgbuf.gw, cfgbuf.msk, cfgbuf.dns);
        if (!ok && debugger) debugger->println("WiFi.config failed");
        ok = WiFi.begin(WIFI_SSID, WIFI_PASS);
        //ok = WiFi.begin(WIFI_SSID, WIFI_PASS, cfgbuf.chl, cfgbuf.mac);
        if (debugger) debugger->println("Using mode 2");
    }
    else
    {
        ok = WiFi.begin(WIFI_SSID, WIFI_PASS);
    }

    while (WiFi.status() != WL_CONNECTED) delay(1);
    if (debugger) debugger->printf("Connected  WiFi %lu\n", millis() - fisken);

    // Configure WiFiClientSecure to use the AWS certificates we generated
    //net.setPreSharedKey(MQTT_IDNT, MQTT__PSK);
    net.setCACert(root_ca_pem);

    // Setup MQTT
    fisken = millis();
    mqtt.begin(MQTT_HOST, MQTT_PORT, net);
    mqtt.onMessage(mqtt_on_message);

    //if (! mqtt.connect(MQTT_NAME))
    if (! mqtt.connect(MQTT_NAME, MQTT_USER, MQTT_PASS))
    {
        if (debugger) debugger->println("Unable to connect");
    }
    if (debugger) debugger->printf("Connected  MQTT %lu\n", millis() - fisken);

    cfgbuf.mode = 2;
    writecfg();

    unsigned long mqtt_connected = millis();
    success_count++;

    static char payload[512];
#if defined(CHIRP_SENSOR)
    //unsigned int fw_version         = readI2CRegister16bit(SOILMOISTURESENSOR_DEFAULT_ADDR, SOILMOISTURESENSOR_GET_VERSION);
    //unsigned int is_busy            = readI2CRegister16bit(SOILMOISTURESENSOR_DEFAULT_ADDR, SOILMOISTURESENSOR_GET_BUSY)
    unsigned int soil_capacitance   = readI2CRegister16bit(SOILMOISTURESENSOR_DEFAULT_ADDR, SOILMOISTURESENSOR_GET_CAPACITANCE);
    unsigned int temperature        = readI2CRegister16bit(SOILMOISTURESENSOR_DEFAULT_ADDR, SOILMOISTURESENSOR_GET_TEMPERATURE);

    if (debugger)
    {
        debugger->print("chirp!");
        //debugger->print(" sensor fw version: ");
        //debugger->print(fw_version, HEX);
        //debugger->print(", busy ");
        //debugger->print(is_busy, HEX);
        debugger->print(", soil moisture capacitance: ");
        debugger->print(soil_capacitance);
        debugger->print(", temperature: ");
        debugger->print(temperature / 10.0f);
        debugger->println();
    }

    digitalWrite(23, LOW);

    sprintf(payload,
            "{ \"temperature\": % 3d,"
             " \"soil_capacitance\": % 3d,"
             " \"battery_voltage\": %d,"
             " \"mqtt_connected\": %lu,"
             " \"success_count\": %u,"
             " \"boot_count\": %u,"
             " \"millis\": %lu"
            "}",
            temperature,
            soil_capacitance,
            analog_value,
            mqtt_connected,
            success_count,
            boot_count,
            millis());
#elif defined(BME280_SENSOR)
    // status = bme.begin(0x76, &Wire2)
    if (! bme.begin(BME280_ADDRESS_ALTERNATE)) {
        Serial.println("ERR: Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10); // TODO: Go back to sleep!
    }
    float temperature   = bme.readTemperature();
    float pressure      = bme.readPressure() / 100.0F;
    float humidity      = bme.readHumidity();

    digitalWrite(POWPIN, LOW);

    sprintf(payload,"{ \"temperature\": %0.2f, \"pressure\": %0.2f, \"humidity\": %.2f, \"battery_voltage\": %u, \"mqtt_connected\": %lu, \"success_count\": %u, \"boot_count\": %u, \"millis\": %lu }",
            temperature,
            pressure,
            humidity,
            analog_value,
            mqtt_connected,
            success_count,
            boot_count,
            millis());

    if (debugger)
    {
        debugger->print(payload);
        debugger->println();
    }
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

    sprintf(payload,"{ \"temperature\": %0.2f, \"humidity\": %.2f, \"battery_voltage\": %u, \"mqtt_connected\": %lu, \"success_count\": %u, \"boot_count\": %u, \"millis\": %lu }",
            temperature,
            humidity,
            analog_value,
            mqtt_connected,
            success_count,
            boot_count,
            millis());

    if (debugger)
    {
        debugger->print(payload);
        debugger->println();
    }
#else
    sprintf(payload,"{ \"mqtt_connected\": %lu, \"millis\": %lu }",
            mqtt_connected,
            millis());
#endif

    mqtt.loop();
    mqtt.publish("chirp/sonsor", payload);
    mqtt.loop();
    mqtt.disconnect();
    mqtt.loop();
    mqtt.loop();
    if (debugger) debugger->println("disconnecting");

    esp_sleep_enable_timer_wakeup(SLEEP_TIME);
    if (debugger)
    {
        debugger->print("sleeping after ");
        debugger->println(millis());
        debugger->flush();
    }
    esp_deep_sleep_start();
}

//--------------------------------------------------------------------------------
void loop()
{
}

