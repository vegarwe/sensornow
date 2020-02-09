#ifndef _MQTT_H
#define _MQTT_H

#include <HardwareSerial.h>

void mqtt_setup(HardwareSerial* dbg = NULL);
void mqtt_loop();

#endif//_MQTT_H

