#include <Arduino.h>
#include <ArduinoJson.h>
#include <MqttClient.h>
#include <FotaClient.h>
#include <ESPWifiClient.h>
#include <RemotePrint.h>
#include <TemperatureClient.h>
#include "settings.h"

MqttClient *mqttClient               = NULL;
TemperatureClient *temperatureClient = new TemperatureClient();
FotaClient *fotaClient               = new FotaClient(DEVICE_NAME);
ESPWifiClient *wifiClient            = new ESPWifiClient(WIFI_SSID, WIFI_PASS);

// Fan variables
unsigned long fanLastStatusMsgSentAt           = 0;
unsigned long lastHumidityCheckedAt            = 0;
bool isFanPoweredOn                            = false;
unsigned long fanPoweredOnAt                   = 0;
bool isFanPoweredOnManually                    = false;
bool lastFanWallSwitchStatus                   = false;
unsigned long lastFanWallSwitchStatusCheckedAt = 0;
unsigned long fanQuietPeriodStopAt             = 0;
unsigned long fanQuietPeriodStartAt            = 0;

// Lamp variables
unsigned long lampLastStatusMsgSentAt    = 0;
unsigned long lastMotionDetectedStatusAt = 0;
bool isLampPoweredOn                     = false;

String topics[2]   = { MQTT_TOPIC_LAMP_SET, MQTT_TOPIC_FAN_SET };
size_t topicsCount = 2;

unsigned long publishStatus(const char   *topic,
                            bool          isPoweredOn,
                            unsigned long lastStatusMsgSentAt,
                            bool          forcePublish = false,
                            const char   *messageId    = NULL) {
  unsigned long now = millis();

  if ((!forcePublish) and (now - lastStatusMsgSentAt < MQTT_PUBLISH_STATUS_INTERVAL)) {
    return lastStatusMsgSentAt;
  }
  const size_t capacity = JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(2);
  DynamicJsonDocument root(capacity);
  JsonObject status = root.createNestedObject("status");

  if (messageId != NULL) {
    root["messageId"] = messageId;
  }

  status["powerOn"] = isPoweredOn;

  // convert to String
  String outString;
  serializeJson(root, outString);

  // publish the message
  mqttClient->publish(topic, outString, true);
  return now;
}

void fanPublishStatus(bool        forcePublish = false,
                      const char *messageId    = NULL) {
  fanLastStatusMsgSentAt = publishStatus(MQTT_TOPIC_FAN_GET,
                                         isFanPoweredOn,
                                         fanLastStatusMsgSentAt,
                                         forcePublish,
                                         messageId);
}

void lampPublishStatus(bool        forcePublish = false,
                       const char *messageId    = NULL) {
  lampLastStatusMsgSentAt = publishStatus(MQTT_TOPIC_LAMP_GET,
                                          isLampPoweredOn,
                                          lampLastStatusMsgSentAt,
                                          forcePublish,
                                          messageId);
}

void setFanPowerStatus(bool isOn) {
  PRINT("FAN: Setting fan to: ");

  if (isOn) {
    digitalWrite(PIN_RELAY_FAN, HIGH);
    fanPoweredOnAt = millis();
    PRINTLN("ON");
  } else {
    digitalWrite(PIN_RELAY_FAN, LOW);
    PRINTLN("OFF");
  }
  isFanPoweredOn = isOn;
  fanPublishStatus(true);
}

void setLampPowerStatus(bool isOn) {
  PRINT("LAMP: Setting lamp to: ");

  if (isOn) {
    digitalWrite(PIN_RELAY_LAMP, HIGH);
    lastMotionDetectedStatusAt = millis();
    PRINTLN("ON");
  } else {
    digitalWrite(PIN_RELAY_LAMP, LOW);
    PRINTLN("OFF");
  }
  isLampPoweredOn = isOn;
  lampPublishStatus(true);
}

void fanHandleRequest(String payload) {
  // deserialize the payload to JSON
  const size_t capacity = JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(2) + 70;
  DynamicJsonDocument  jsonDoc(capacity);
  DeserializationError error = deserializeJson(jsonDoc, payload);

  if (error) {
    PRINT_E("Failed to deserialize the received payload. Error: ");
    PRINTLN_E(error.c_str());
    PRINTLN_E("The payload is: ");
    PRINTLN_E(payload)
    return;
  }
  JsonObject root   = jsonDoc.as<JsonObject>();
  JsonObject status = root["status"];

  if (status.isNull()) {
    PRINTLN_E(
      "FAN: The received payload is valid JSON, but \"status\" key is not found.");
    PRINTLN_E("The payload is: ");
    PRINTLN_E(payload)
    return;
  }

  JsonVariant powerOnJV = status["powerOn"];

  if (!powerOnJV.isNull()) {
    bool isFanPoweredOnNew = powerOnJV.as<bool>();

    if (isFanPoweredOnNew == isFanPoweredOn) {
      PRINTLN("FAN: No need to set the state, as it is already set.");
      return;
    }
    setFanPowerStatus(isFanPoweredOnNew);
  }
  const char *messageId = root["messageId"];
  fanPublishStatus(true, messageId);
}

void lampHandleRequest(String payload) {
  // deserialize the payload to JSON
  const size_t capacity = JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(2) + 70;
  DynamicJsonDocument  jsonDoc(capacity);
  DeserializationError error = deserializeJson(jsonDoc, payload);

  if (error) {
    PRINT_E("Failed to deserialize the received payload. Error: ");
    PRINTLN_E(error.c_str());
    PRINTLN_E("The payload is: ");
    PRINTLN_E(payload)
    return;
  }
  JsonObject root   = jsonDoc.as<JsonObject>();
  JsonObject status = root["status"];

  if (status.isNull()) {
    PRINTLN_E(
      "LAMP: The received payload is valid JSON, but \"status\" key is not found.");
    PRINTLN_E("The payload is: ");
    PRINTLN_E(payload)
    return;
  }

  JsonVariant powerOnJV = status["powerOn"];

  if (!powerOnJV.isNull()) {
    bool isLampPoweredOnNew = powerOnJV.as<bool>();

    if (isLampPoweredOnNew == isLampPoweredOn) {
      PRINTLN("LAMP: No need to set the state, as it is already set.");
      return;
    }
    setLampPowerStatus(isLampPoweredOnNew);
  }
  const char *messageId = root["messageId"];
  lampPublishStatus(true, messageId);
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  PRINT("MQTT Message arrived [");
  PRINT(topic);
  PRINTLN("] ");

  // Convert the payload to string
  char spayload[length + 1];
  memcpy(spayload, payload, length);
  spayload[length] = '\0';
  String payloadString = String(spayload);

  // Do something according the topic
  if (strcmp(topic, MQTT_TOPIC_FAN_SET) == 0) {
    fanHandleRequest(payloadString);
  } else if (strcmp(topic, MQTT_TOPIC_LAMP_SET) == 0) {
    lampHandleRequest(payloadString);
  } else {
    PRINT("MQTT: Warning: Unknown topic: ");
    PRINTLN(topic);
  }
}

bool isMotionDetected() {
  int value = digitalRead(PIN_MOTION_SENSOR);

  // No logging here or it will flood the log
  if (value == HIGH) {
    return true;
  }
  return false;
}

void setup() {
  // fan
  pinMode(PIN_RELAY_FAN, OUTPUT);
  digitalWrite(PIN_RELAY_FAN, LOW);

  // lamp
  pinMode(PIN_RELAY_LAMP, OUTPUT);
  digitalWrite(PIN_RELAY_LAMP, LOW);

  // wall switch
  pinMode(PIN_WALL_SWITCH_FAN, INPUT);

  // motion sensor
  pinMode(PIN_MOTION_SENSOR,   INPUT);

  wifiClient->init();
  mqttClient = new MqttClient(MQTT_SERVER,
                              MQTT_SERVER_PORT,
                              DEVICE_NAME,
                              MQTT_USERNAME,
                              MQTT_PASS,
                              topics,
                              topicsCount,
                              MQTT_SERVER_FINGERPRINT,
                              mqttCallback);
  temperatureClient->init(DEVICE_NAME,
                          PIN_TEMP,
                          TEMP_SENSOR_TYPE,
                          mqttClient,
                          MQTT_TOPIC_TEMPERATURE_GET,
                          TEMP_SENSOR_CORRECTION);
  fotaClient->init();
}

void fanAutoOn() {
  // FIXME: Remove the "return" once I replace the humidity sensor
  return;
  unsigned long now = millis();

  if ((!isFanPoweredOn) && (now - lastHumidityCheckedAt > AUTOONOFF_HUMIDITY_CHECK_INTERVAL)) {
    lastHumidityCheckedAt = now;
    float humidity = temperatureClient->getHumidity();
    PRINT_D("FAN AutoOFF: The quiet window is between ");
    PRINT_D(fanQuietPeriodStartAt / 1000);
    PRINT_D(" and ");
    PRINT_D(fanQuietPeriodStopAt / 1000);
    PRINT_D(". Currently it is ");
    PRINTLN_D(now / 1000);

    PRINT("FAN AutoOn: Humidity is ");
    PRINT(humidity);
    PRINT(", limit is ");
    PRINT(AUTOONOFF_HUMIDITY_TURN_ON)
    PRINT(". ");


    if (humidity < AUTOONOFF_HUMIDITY_TURN_ON) {
      PRINTLN("No need to start the fan.");
      return;
    }

    if ((fanQuietPeriodStartAt <= now) && (now <= fanQuietPeriodStopAt)) {
      PRINT(
        "The humidity is above the limit, but the quiet period is still active. If needed, the fan will start in ");
      PRINT((fanQuietPeriodStopAt - now) / 1000);
      PRINT(" seconds. The quiet period is between ");
      PRINT(fanQuietPeriodStartAt / 1000);
      PRINT(" and ");
      PRINT(fanQuietPeriodStopAt / 1000);
      PRINT(". Currently it is ");
      PRINTLN(now / 1000);
      return;
    }
    fanQuietPeriodStartAt = now + FAN_MAX_POWER_ON_TIME;
    fanQuietPeriodStopAt  = fanQuietPeriodStartAt + AUTOONOFF_QUIET_PERIOD;

    PRINT("We need to start the fan. The quiet period is between ");
    PRINT(fanQuietPeriodStartAt / 1000);
    PRINT(" and ");
    PRINT(fanQuietPeriodStopAt / 1000);
    PRINT(". Currently it is ");
    PRINTLN(now / 1000);

    setFanPowerStatus(true);
    isFanPoweredOnManually = false;
  }
}

void fanAutoOff() {
  unsigned long now = millis();

  if (isFanPoweredOn && (now - lastHumidityCheckedAt > AUTOONOFF_HUMIDITY_CHECK_INTERVAL)) {
    float humidity = temperatureClient->getHumidity();
    lastHumidityCheckedAt = now;
    PRINT("FAN AutoOFF: Humidity is ");
    PRINT(humidity);
    PRINT(", we will turn it off once it is below ");
    PRINTLN(AUTOONOFF_HUMIDITY_TURN_OFF);

    bool turnOff = false;

    if ((isFanPoweredOnManually) && (now - fanPoweredOnAt > FAN_MAX_POWER_ON_TIME)) {
      // The fan was manually started and the time is up
      turnOff = true;
    }

    if ((!isFanPoweredOnManually) && (humidity <= AUTOONOFF_HUMIDITY_TURN_OFF)) {
      // The fan was automatically started and the humidity is lower than expected
      turnOff               = true;
      fanQuietPeriodStartAt = now;
      fanQuietPeriodStopAt  = now + AUTOONOFF_QUIET_PERIOD;
      PRINTLN("FAN AutoOFF: The quiet period is now set.");
    }

    if (turnOff) {
      PRINT("FAN AutoOFF: It's time to turn off the fan. Humidity is ");
      PRINTLN(humidity);
      setFanPowerStatus(false);
      fanPublishStatus(true);
    }
  }
}

bool isFanWallSwitchOn() {
  int value = analogRead(PIN_WALL_SWITCH_FAN);

  PRINT_D("FAN: Wall switch sensor value: ")
  PRINTLN_D(value);

  if (value > WALL_SWITCH_ON_LEVEL_FAN) {
    return true;
  }
  return false;
}

void fanManualOnOff() {
  unsigned long now = millis();

  if (now - lastFanWallSwitchStatusCheckedAt < MANUALONOFF_CHECK_INTERVAL) {
    // Still not the time
    return;
  }
  lastFanWallSwitchStatusCheckedAt = now;
  bool isSwitchOn = isFanWallSwitchOn();

  if (lastFanWallSwitchStatus != isSwitchOn) {
    // reset the quite period
    fanQuietPeriodStartAt = 0;
    fanQuietPeriodStopAt  = 0;

    setFanPowerStatus(isSwitchOn);
    lastFanWallSwitchStatus = isSwitchOn;
    isFanPoweredOnManually  = true;
    PRINT("FAN: Wall switch is ");

    if (isSwitchOn) {
      PRINTLN("ON");
    } else {
      PRINTLN("OFF");
    }
  }
}

void lampAutoOnOff() {
  unsigned long now                 =  millis();
  bool isMotionDetectedCurrentState = isMotionDetected();

  if (isMotionDetectedCurrentState) {
    // Auto ON
    lastMotionDetectedStatusAt = now;

    if (!isLampPoweredOn) {
      // Turn the lamp
      setLampPowerStatus(true);
    }
  } else {
    // Auto OFF
    if (isLampPoweredOn && (now - lastMotionDetectedStatusAt > LAMP_MAX_POWER_ON_TIME)) {
      // Time to turn off the lamp
      setLampPowerStatus(false);
    }
  }
}

void loop() {
  wifiClient->reconnectIfNeeded();
  RemotePrint::instance()->handle();
  fotaClient->loop();
  mqttClient->loop();
  temperatureClient->loop();
  fanPublishStatus();
  lampPublishStatus();
  fanManualOnOff();
  fanAutoOn();
  fanAutoOff();
  lampAutoOnOff();
}
