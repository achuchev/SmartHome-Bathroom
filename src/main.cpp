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

// Lamp variables
unsigned long lampLastStatusMsgSentAt           = 0;
unsigned long lastMotionDetectedStatusCheckedAt = 0;
unsigned long lastMotionDetectedStatusAt        = 0;
bool isLampPoweredOn                            = false;

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
  const size_t bufferSize = JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(3);
  DynamicJsonBuffer jsonBuffer(bufferSize);
  JsonObject& root   = jsonBuffer.createObject();
  JsonObject& status = root.createNestedObject("status");

  if (messageId != NULL) {
    root["messageId"] = messageId;
  }

  status["powerOn"] = isPoweredOn;

  // convert to String
  String outString;
  root.printTo(outString);

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
  const size_t bufferSize = 2 * JSON_OBJECT_SIZE(1) + 30;
  DynamicJsonBuffer jsonBuffer(bufferSize);
  JsonObject& root = jsonBuffer.parseObject(payload);

  if (!root.success()) {
    PRINTLN_E("FAN: JSON with \"root\" key not received.");
    PRINTLN_E(payload);
    return;
  }
  JsonObject& status = root.get<JsonObject&>("status");

  if (!status.success()) {
    PRINTLN_E("FAN: JSON with \"status\" key not received.");
    PRINTLN_E(payload);
    return;
  }
  const char *powerOnChar = status.get<const char *>("powerOn");

  if (powerOnChar) {
    bool isFanPoweredOnNew = (strcasecmp(powerOnChar, "true") == 0);

    if (isFanPoweredOnNew == isFanPoweredOn) {
      PRINTLN("FAN: No need to set the state, as it is already set.");
      return;
    }
    setFanPowerStatus(isFanPoweredOnNew);
  }
  const char *messageId = root.get<const char *>("messageId");
  fanPublishStatus(true, messageId);
}

void lampHandleRequest(String payload) {
  const size_t bufferSize = 2 * JSON_OBJECT_SIZE(1) + 30;
  DynamicJsonBuffer jsonBuffer(bufferSize);
  JsonObject& root = jsonBuffer.parseObject(payload);

  if (!root.success()) {
    PRINTLN_E("LAMP: JSON with \"root\" key not received.");
    PRINTLN_E(payload);
    return;
  }
  JsonObject& status = root.get<JsonObject&>("status");

  if (!status.success()) {
    PRINTLN_E("LAMP: JSON with \"status\" key not received.");
    PRINTLN_E(payload);
    return;
  }
  const char *powerOnChar = status.get<const char *>("powerOn");

  if (powerOnChar) {
    bool isLampPoweredOnNew = (strcasecmp(powerOnChar, "true") == 0);

    if (isLampPoweredOnNew == isLampPoweredOn) {
      PRINTLN("LAMP: No need to set the state, as it is already set.");
      return;
    }
    setLampPowerStatus(isLampPoweredOnNew);
  }
  const char *messageId = root.get<const char *>("messageId");
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

  PRINT_D("MOTION SENSOR: Motion ")

  if (value == HIGH) {
    PRINTLN_D("DETECTED.");
    return true;
  }
  PRINTLN_D("not detected.");
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
  unsigned long now = millis();

  if ((!isFanPoweredOn) && (now - lastHumidityCheckedAt > AUTOONOFF_HUMIDITY_CHECK_INTERVAL)) {
    lastHumidityCheckedAt = now;
    float humidity = temperatureClient->getHumidity();
    PRINT("FAN AutoOn: Humidity is ");
    PRINT(humidity);
    PRINT(", limit is ");
    PRINT(AUTOONOFF_HUMIDITY_TURN_ON)

    if (humidity > AUTOONOFF_HUMIDITY_TURN_ON) {
      PRINTLN(". We need to start the fan.");
      setFanPowerStatus(true);
      isFanPoweredOnManually = false;
    } else {
      PRINTLN(". We don't need to start the fan.");
    }
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

    if (((!isFanPoweredOnManually) && (humidity < AUTOONOFF_HUMIDITY_TURN_OFF)) ||
        ((isFanPoweredOnManually) && (now - fanPoweredOnAt > TEMP_MAX_POWER_ON_TIME))) {
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
  unsigned long now =  millis();

  if (now - lastMotionDetectedStatusCheckedAt < LAMP_CHECK_INTERVAL) {
    // Still not the time
    return;
  }

  lastMotionDetectedStatusCheckedAt = now;
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
