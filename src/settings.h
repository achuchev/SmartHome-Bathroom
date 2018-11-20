#ifndef SETTING_H
#define SETTING_H

#include "DHT.h"

#undef MAX_TIME_INACTIVE
#define MAX_TIME_INACTIVE ULONG_MAX

#define DEVICE_NAME "Bathroom"

#define MQTT_TOPIC_FAN_GET "get/apartment/bathroom/fan"
#define MQTT_TOPIC_FAN_SET "set/apartment/bathroom/fan"
#define MQTT_TOPIC_LAMP_GET "get/apartment/bathroom/lamp"
#define MQTT_TOPIC_LAMP_SET "set/apartment/bathroom/lamp"
#define MQTT_PUBLISH_STATUS_INTERVAL 60000

#define MQTT_TOPIC_TEMPERATURE_GET "get/temperature/apartment/bathroom/fan"

#define PIN_RELAY_LAMP D1      // SSR of Lamp
#define PIN_RELAY_FAN D2       // SSR of FAN
#define PIN_TEMP D5            // DHT 22 PIN
#define PIN_MOTION_SENSOR D6   // Microwave motion sensor
#define PIN_WALL_SWITCH_FAN A0 // Photoresistor

#define AUTOONOFF_HUMIDITY_TURN_ON 99
#define AUTOONOFF_HUMIDITY_TURN_OFF 97
#define AUTOONOFF_HUMIDITY_CHECK_INTERVAL 10000
#define AUTOONOFF_QUIET_PERIOD 900000 // 15 min

#define WALL_SWITCH_ON_LEVEL_FAN 100
#define MANUALONOFF_CHECK_INTERVAL 1000

#define TEMP_MAX_POWER_ON_TIME 900000 // 15 min
#define TEMP_SENSOR_TYPE DHT22
#define TEMP_SENSOR_CORRECTION 0      // The correction in degrees

#define LAMP_CHECK_INTERVAL 1000
#define LAMP_MAX_POWER_ON_TIME 180000 // 1,5 min

#endif // ifndef SETTING_H
