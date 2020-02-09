#ifndef _WIFI_H
#define _WIFI_H

#include <HardwareSerial.h>

void wifi_setup(HardwareSerial* dbg = NULL);
void wifi_loop();

#endif//_WIFI_H

