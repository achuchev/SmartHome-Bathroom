#ifndef SETTING_H
#define SETTING_H

#include "DHT.h"

#define DEVICE_NAME "Bathroom"

#define MQTT_TOPIC_GET "get/apartment/bathroom/fan"
#define MQTT_TOPIC_SET "set/apartment/bathroom/fan"
#define MQTT_PUBLISH_STATUS_INTERVAL 10000

#define PIN_TEMP D5            // DHT 22 PIN
#define PIN_RELAY D2           // SSR
#define PIN_WALL_SWITCH_FAN A0 // Photoresistor

#define AUTOONOFF_HUMIDITY_TURN_ON 80
#define AUTOONOFF_HUMIDITY_TURN_OFF 70
#define AUTOONOFF_HUMIDITY_CHECK_INTERVAL 10000

#define WALL_SWITCH_ON_LEVEL_FAN 300
#define MANUALONOFF_CHECK_INTERVAL 2000

#define TEMP_MAX_POWER_ON_TIME 300000 // 5 min

#define TEMP_SENSOR_TYPE DHT22
#define TEMP_SENSOR_CORRECTION 0      // The correction in degrees

#define MQTT_TOPIC_TEMPERATURE_GET "get/temperature/apartment/bathroom/fan"

#endif // ifndef SETTING_H
