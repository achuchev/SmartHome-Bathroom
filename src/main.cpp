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

long lastStatusMsgSentAt        = 0;
long lastHumidityCheckedAt      = 0;
bool isFanPowerOn               = false;
long fanPoweredOnAt             = 0;
bool lastFanWallSwitchStatus    = false;
long lastFanWallSwitchCheckedAt = 0;

void setFanPowerStatus(bool isOn) {
  PRINT("FAN: Setting fan to: ");

  if (isOn) {
    digitalWrite(PIN_RELAY, HIGH);
    fanPoweredOnAt = millis();
    PRINTLN("ON");
  } else {
    digitalWrite(PIN_RELAY, LOW);
    PRINTLN("OFF");
  }
  isFanPowerOn = isOn;
}

void fanPublishStatus(bool        forcePublish = false,
                      const char *messageId    = NULL) {
  long now = millis();

  if ((forcePublish) or (now - lastStatusMsgSentAt >
                         MQTT_PUBLISH_STATUS_INTERVAL)) {
    lastStatusMsgSentAt = now;

    const size_t bufferSize = JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(3);
    DynamicJsonBuffer jsonBuffer(bufferSize);
    JsonObject& root   = jsonBuffer.createObject();
    JsonObject& status = root.createNestedObject("status");

    if (messageId != NULL) {
      root["messageId"] = messageId;
    }

    status["powerOn"] = isFanPowerOn;

    // convert to String
    String outString;
    root.printTo(outString);

    // publish the message
    mqttClient->publish(MQTT_TOPIC_GET, outString);
  }
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
    bool isFanPowerOnNew = (strcasecmp(powerOnChar, "true") == 0);

    if (isFanPowerOnNew == isFanPowerOn) {
      PRINTLN("FAN: No need to set the state, as it is already set.");
      return;
    }
    setFanPowerStatus(isFanPowerOnNew);
    const char *messageId = root.get<const char *>("messageId");
    fanPublishStatus(true, messageId);
  }
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
  if (strcmp(topic, MQTT_TOPIC_SET) == 0) {
    fanHandleRequest(payloadString);
  }
  else {
    PRINT("MQTT: Warning: Unknown topic: ");
    PRINTLN(topic);
  }
}

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  wifiClient->init();
  mqttClient = new MqttClient(MQTT_SERVER,
                              MQTT_SERVER_PORT,
                              DEVICE_NAME,
                              MQTT_USERNAME,
                              MQTT_PASS,
                              MQTT_TOPIC_SET,
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

void fanAutoOnOff() {
  long now = millis();

  if ((isFanPowerOn == false) && (now - lastHumidityCheckedAt > AUTOONOFF_HUMIDITY_CHECK_INTERVAL)) {
    lastHumidityCheckedAt = now;
    float humidity = temperatureClient->getHumidity();
    PRINT("FAN AutoOnOFF: Humidity is ");
    PRINT(humidity);
    PRINT(", limit is ");
    PRINT(AUTOONOFF_HUMIDITY_TURN_ON)

    if (humidity > AUTOONOFF_HUMIDITY_TURN_ON) {
      PRINTLN(". We need to start the fan.");
      setFanPowerStatus(true);
    } else {
      PRINTLN(". We don't need to start the fan.");
    }
  }

  if ((isFanPowerOn == true) && (now - fanPoweredOnAt > TEMP_MAX_POWER_ON_TIME)) {
    float humidity = temperatureClient->getHumidity();
    PRINT("FAN AutoOnOFF: Humidity is ");
    PRINT(humidity);
    PRINT(", we will turn it off once it is below ");
    PRINTLN(AUTOONOFF_HUMIDITY_TURN_OFF);

    if (humidity < AUTOONOFF_HUMIDITY_TURN_OFF) {
      PRINT("FAN AutoOnOFF: It's time to turning off the fan. The humidity is ");
      PRINTLN(humidity);
      setFanPowerStatus(false);
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
  long now = millis();

  if (now - lastFanWallSwitchCheckedAt > MANUALONOFF_CHECK_INTERVAL) {
    lastFanWallSwitchCheckedAt = now;
    bool isSwitchOn = isFanWallSwitchOn();

    if (lastFanWallSwitchStatus != isSwitchOn) {
      lastFanWallSwitchStatus = isSwitchOn;
      setFanPowerStatus(isSwitchOn);
      PRINT("FAN: Wall switch is ");

      if (isSwitchOn) {
        PRINTLN("ON");
      } else {
        PRINTLN("OFF");
      }
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
  fanManualOnOff();
  fanAutoOnOff();
}
