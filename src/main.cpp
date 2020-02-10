#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>

#include "wifi_mqtt.h"


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

RTC_DATA_ATTR int bootCount = 0;


//--------------------------------------------------------------------------------
void setup() {
    //debugger = &Serial1;

    if (debugger)
    {
        debugger->begin(115200);
        delay(100);
        debugger->println("");
        debugger->println("Starting");
    }

    uint16_t analog_value = analogRead(35);

    if (debugger)
    {
        debugger->print("battery measurement: ");
        debugger->println(analog_value);
        // (2112 * 2 * 3.3 / 4095) + 0.366 ~= 3.77
        // (2444 * 2 * 3.3 / 4095) + 0.366 ~= 4.2 -- 4.3
    }

    pinMode(14, OUTPUT);
    digitalWrite(14, HIGH);

    Wire.begin();

    wifi_mqtt_setup(debugger, mqtt_on_message);

    ++bootCount;
    if (debugger) debugger->println("Boot number: " + String(bootCount));
    print_wakeup_reason();

    //delay(1000); // give some time to boot up // TODO: Adaptive based on wifi_setup time

    //unsigned int fw_version         = readI2CRegister16bit(SOILMOISTURESENSOR_DEFAULT_ADDR, SOILMOISTURESENSOR_GET_VERSION);
    //unsigned int is_busy            = readI2CRegister16bit(SOILMOISTURESENSOR_DEFAULT_ADDR, SOILMOISTURESENSOR_GET_BUSY)
    unsigned int soil_capacitance  = readI2CRegister16bit(SOILMOISTURESENSOR_DEFAULT_ADDR, SOILMOISTURESENSOR_GET_CAPACITANCE);
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

    digitalWrite(14, LOW);

    wifi_mqtt_loop();
    static char payload[512];
    sprintf(payload,"{ \"temperature\": %d, \"soil_capacitance\": %d , \"battery_voltage\": %u}",
            temperature,
            soil_capacitance,
            analog_value);
    wifi_mqtt_publish("chirp/sensor", payload);
    wifi_mqtt_loop();

    esp_sleep_enable_timer_wakeup(20 * 60 * 1000 * 1000);
    //esp_sleep_enable_timer_wakeup(     10 * 1000 * 1000);
    if (debugger) debugger->println("sleeping");
    if (debugger) debugger->flush();
    esp_deep_sleep_start();
}

//--------------------------------------------------------------------------------
void loop()
{
    //uint16_t analog_value = analogRead(35);

    //if (debugger)
    //{
    //    debugger->print("battery measurement: ");
    //    debugger->println(analog_value);
    //}

    //static char payload[512];
    //sprintf(payload,"{ \"battery_voltage\": %u}", analog_value);
    //wifi_mqtt_publish("chirp/sensor", payload);
    //wifi_mqtt_loop();
    //delay(3000);                   //this can take a while
}

//void writeI2CRegister8bit(int addr, int value) {
//  Wire.beginTransmission(addr);
//  Wire.write(value);
//  Wire.endTransmission();
//}
//    writeI2CRegister8bit(SOILMOISTURESENSOR_DEFAULT_ADDR, SOILMOISTURESENSOR_MEASURE_LIGHT);
//    delay(3000);                   //this can take a while
//    Serial1.println(readI2CRegister16bit(SOILMOISTURESENSOR_DEFAULT_ADDR, SOILMOISTURESENSOR_GET_LIGHT));


