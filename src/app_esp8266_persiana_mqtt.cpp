#ifdef USE_ESP8266_PERSIANA_MQTT

/*
 * Firmware ESP8266 para persiana com MQTT.
 *
 * Mantem a logica do firmware simple:
 * - D5/D6: ponte H direcao
 * - D7: PWM enable
 * - D1/D2: encoder quadrature
 * - 0% = aberta, 100% = fechada
 * - EEPROM: zeroRaw + maxSteps
 * - stall/timeout de motor
 * - ArduinoOTA
 *
 * MQTT:
 * - Estado JSON:              <base>/<deviceId>/state
 * - Comando JSON:             <base>/<deviceId>/command
 * - Cover state HA:           <base>/<deviceId>/cover/state
 * - Cover position HA:        <base>/<deviceId>/cover/position
 * - Cover command HA:         <base>/<deviceId>/cover/set
 * - Cover set position HA:    <base>/<deviceId>/cover/position/set
 * - Availability:             <base>/<deviceId>/availability
 *
 * Comandos aceitos em <base>/<deviceId>/command:
 * {"desiredPosition": 50}
 * {"position": 50}
 * {"command": "open" | "close" | "stop" | "set_position", "position": 50}
 * {"manualControl": {"sequence": 1, "direction": 1}}  // 1 abre, -1 fecha, 0 para
 * {"calibration": {"encoderTicksClosed": 0, "encoderTicksOpen": 8000, "rebaseEncoderToCurrent": true}}
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "local_secrets_esp8266.h"

#ifndef MQTT_HOST
#define MQTT_HOST "192.168.1.136"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif

#ifndef MQTT_USER
#define MQTT_USER ""
#endif

#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD ""
#endif

#ifndef MQTT_BASE_TOPIC
#define MQTT_BASE_TOPIC "mini-has/devices"
#endif

namespace {

constexpr uint8_t kMotorIn1Pin = D5;
constexpr uint8_t kMotorIn2Pin = D6;
constexpr uint8_t kMotorEnablePin = D7;
constexpr uint8_t kEncoderAPin = D1;
constexpr uint8_t kEncoderBPin = D2;

constexpr int kMotorPwmMax = 1023;
constexpr int kMotorPwmSlow = 650;
constexpr long kTickTolerance = 48;
constexpr unsigned long kStatePublishIdleMs = 1000;
constexpr unsigned long kStatePublishMoveMs = 250;
constexpr unsigned long kMqttReconnectMs = 3000;
constexpr unsigned long kMotorMaxRunMs = 45000;
constexpr unsigned long kStallStopMs = 4000;
constexpr long kMinCalibrationSpan = 512;
constexpr long kMaxCalibrationSpan = 120000;
constexpr uint32_t kEepromMagic = 0x50523153UL;

constexpr char kMqttHost[] = MQTT_HOST;
constexpr uint16_t kMqttPort = MQTT_PORT;
constexpr char kMqttUser[] = MQTT_USER;
constexpr char kMqttPassword[] = MQTT_PASSWORD;
constexpr char kMqttBaseTopic[] = MQTT_BASE_TOPIC;

struct EepromStore {
  uint32_t magic;
  int32_t zeroRaw;
  int32_t maxSteps;
};

volatile long g_rawTicks = 0;
volatile uint8_t g_encPrev = 0;
int32_t g_zeroRaw = 0;
int32_t g_maxSteps = 8000;
int g_dirSign = 1;

int g_desiredPercent = 0;
bool g_motorOn = false;
int g_motorDir = 0;
unsigned long g_motionStartMs = 0;
unsigned long g_lastStatePublishMs = 0;
unsigned long g_lastEncoderChangeMs = 0;
unsigned long g_lastMqttAttemptMs = 0;
unsigned long g_lastManualSequence = 0;
long g_lastRawObserved = 0;
String g_resetReason;
bool g_otaReady = false;
int g_pwm = kMotorPwmMax;

WiFiClient g_wifiClient;
PubSubClient g_mqtt(g_wifiClient);

char g_topicState[160];
char g_topicCommand[160];
char g_topicAvailability[160];
char g_topicCoverState[180];
char g_topicCoverSet[180];
char g_topicCoverPosition[180];
char g_topicCoverPositionSet[200];
char g_topicDiscovery[220];

void motorStop();
void publishState(bool force = false);

void setPwm(int v) {
  g_pwm = constrain(v, 0, kMotorPwmMax);
  analogWrite(kMotorEnablePin, g_pwm);
}

void motorOpen() {
  digitalWrite(kMotorIn1Pin, HIGH);
  digitalWrite(kMotorIn2Pin, LOW);
  setPwm(kMotorPwmMax);
  g_motorOn = true;
  g_motorDir = 1;
  g_motionStartMs = millis();
}

void motorClose() {
  digitalWrite(kMotorIn1Pin, LOW);
  digitalWrite(kMotorIn2Pin, HIGH);
  setPwm(kMotorPwmMax);
  g_motorOn = true;
  g_motorDir = -1;
  g_motionStartMs = millis();
}

void motorStop() {
  digitalWrite(kMotorIn1Pin, LOW);
  digitalWrite(kMotorIn2Pin, LOW);
  setPwm(kMotorPwmMax);
  g_motorOn = false;
  g_motorDir = 0;
}

uint8_t readEncoderQuadratureState() {
  return (digitalRead(kEncoderAPin) ? 1U : 0U) | (digitalRead(kEncoderBPin) ? 2U : 0U);
}

IRAM_ATTR void onEncoderIsr() {
  static const int8_t kTable[16] = {
      0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  const uint8_t next = readEncoderQuadratureState();
  g_rawTicks += kTable[(g_encPrev << 2U) | next];
  g_encPrev = next;
}

long snapshotRaw() {
  noInterrupts();
  const long t = g_rawTicks;
  interrupts();
  return t;
}

long effectiveTicks() {
  const long e = (snapshotRaw() - g_zeroRaw) * static_cast<long>(g_dirSign);
  if (g_maxSteps <= 0) {
    return 0;
  }
  if (e < 0) {
    return 0;
  }
  if (e > g_maxSteps) {
    return g_maxSteps;
  }
  return e;
}

float currentPercent() {
  if (g_maxSteps <= 0) {
    return 0.0f;
  }
  return (100.0f * static_cast<float>(effectiveTicks())) / static_cast<float>(g_maxSteps);
}

int roundedCurrentPercent() {
  return constrain(static_cast<int>(roundf(currentPercent())), 0, 100);
}

const char *coverState() {
  if (g_motorOn && g_motorDir > 0) {
    return "opening";
  }
  if (g_motorOn && g_motorDir < 0) {
    return "closing";
  }
  const int p = roundedCurrentPercent();
  if (p <= 2) {
    return "open";
  }
  if (p >= 98) {
    return "closed";
  }
  return "stopped";
}

void loadEeprom() {
  EepromStore s{};
  EEPROM.begin(sizeof(EepromStore));
  EEPROM.get(0, s);
  EEPROM.end();
  if (s.magic == kEepromMagic && s.maxSteps >= kMinCalibrationSpan && s.maxSteps <= kMaxCalibrationSpan) {
    g_zeroRaw = s.zeroRaw;
    g_maxSteps = s.maxSteps;
    Serial.printf("[eeprom] zero=%ld maxSteps=%ld\n", static_cast<long>(g_zeroRaw), static_cast<long>(g_maxSteps));
  }
}

void saveEeprom() {
  EepromStore s{};
  s.magic = kEepromMagic;
  s.zeroRaw = g_zeroRaw;
  s.maxSteps = g_maxSteps;
  EEPROM.begin(sizeof(EepromStore));
  EEPROM.put(0, s);
  EEPROM.commit();
  EEPROM.end();
  Serial.println("[eeprom] salvo");
}

void zeroOriginHere(const char *reason) {
  g_zeroRaw = static_cast<int32_t>(snapshotRaw());
  saveEeprom();
  Serial.printf("[origem] zero raw=%ld (%s)\n", static_cast<long>(g_zeroRaw), reason);
}

bool calibrationSpanOk(long closedAbs, long openAbs) {
  const long span = openAbs - closedAbs;
  return span >= kMinCalibrationSpan && span <= kMaxCalibrationSpan;
}

void applyCalibrationFromCommand(JsonVariantConst cal) {
  if (cal.isNull()) {
    return;
  }

  const long c = cal["encoderTicksClosed"] | 0L;
  const long o = cal["encoderTicksOpen"] | 0L;
  if (calibrationSpanOk(c, o)) {
    const long span = o - c;
    if (span != g_maxSteps) {
      g_maxSteps = static_cast<int32_t>(span);
      saveEeprom();
      Serial.printf("[cal] span=%ld passos\n", span);
    }
  }

  if (cal["rebaseEncoderToCurrent"].is<bool>() && cal["rebaseEncoderToCurrent"].as<bool>()) {
    zeroOriginHere("rebaseEncoderToCurrent");
  }
}

bool ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(kWifiSsid, kWifiPassword);
  Serial.printf("[wifi] conectando %s", kWifiSsid);
  const unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000UL) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[wifi] IP %s\n", WiFi.localIP().toString().c_str());
    return true;
  }

  Serial.printf("[wifi] falha status=%d\n", WiFi.status());
  return false;
}

void ensureOta() {
  if (g_otaReady || WiFi.status() != WL_CONNECTED) {
    return;
  }

  ArduinoOTA.setHostname(kDeviceId);
  ArduinoOTA.onStart([]() { Serial.println("[ota] inicio"); });
  ArduinoOTA.onEnd([]() { Serial.println("[ota] fim"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    if (t) {
      Serial.printf("[ota] %u%%\r", (p * 100U) / t);
    }
  });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("[ota] erro %u\n", e); });
  ArduinoOTA.begin();
  g_otaReady = true;
  Serial.printf("[ota] pronto %s:8266\n", WiFi.localIP().toString().c_str());
}

void setupTopics() {
  snprintf(g_topicState, sizeof(g_topicState), "%s/%s/state", kMqttBaseTopic, kDeviceId);
  snprintf(g_topicCommand, sizeof(g_topicCommand), "%s/%s/command", kMqttBaseTopic, kDeviceId);
  snprintf(g_topicAvailability, sizeof(g_topicAvailability), "%s/%s/availability", kMqttBaseTopic, kDeviceId);
  snprintf(g_topicCoverState, sizeof(g_topicCoverState), "%s/%s/cover/state", kMqttBaseTopic, kDeviceId);
  snprintf(g_topicCoverSet, sizeof(g_topicCoverSet), "%s/%s/cover/set", kMqttBaseTopic, kDeviceId);
  snprintf(g_topicCoverPosition, sizeof(g_topicCoverPosition), "%s/%s/cover/position", kMqttBaseTopic, kDeviceId);
  snprintf(g_topicCoverPositionSet, sizeof(g_topicCoverPositionSet), "%s/%s/cover/position/set", kMqttBaseTopic, kDeviceId);
  snprintf(g_topicDiscovery, sizeof(g_topicDiscovery), "homeassistant/cover/%s/config", kDeviceId);
}

void publishHomeAssistantDiscovery() {
  JsonDocument doc;
  doc["name"] = "Persiana Sala";
  doc["unique_id"] = kDeviceId;
  doc["object_id"] = kDeviceId;
  doc["device_class"] = "shade";
  doc["command_topic"] = g_topicCoverSet;
  doc["state_topic"] = g_topicCoverState;
  doc["position_topic"] = g_topicCoverPosition;
  doc["set_position_topic"] = g_topicCoverPositionSet;
  doc["availability_topic"] = g_topicAvailability;
  doc["payload_open"] = "OPEN";
  doc["payload_close"] = "CLOSE";
  doc["payload_stop"] = "STOP";
  doc["position_open"] = 0;
  doc["position_closed"] = 100;
  doc["retain"] = true;

  JsonObject device = doc["device"].to<JsonObject>();
  device["identifiers"][0] = kDeviceId;
  device["name"] = "Persiana Sala";
  device["manufacturer"] = "Mini HAS";
  device["model"] = "ESP8266 Persiana MQTT";

  String payload;
  serializeJson(doc, payload);
  g_mqtt.publish(g_topicDiscovery, payload.c_str(), true);
}

void subscribeTopics() {
  g_mqtt.subscribe(g_topicCommand);
  g_mqtt.subscribe(g_topicCoverSet);
  g_mqtt.subscribe(g_topicCoverPositionSet);
  Serial.printf("[mqtt] subscribe %s\n", g_topicCommand);
  Serial.printf("[mqtt] subscribe %s\n", g_topicCoverSet);
  Serial.printf("[mqtt] subscribe %s\n", g_topicCoverPositionSet);
}

void setDesiredPercent(int percent, const char *reason) {
  g_desiredPercent = constrain(percent, 0, 100);
  Serial.printf("[target] %d%% (%s)\n", g_desiredPercent, reason);
  publishState(true);
}

void handleManual(JsonVariantConst manual) {
  if (manual.isNull()) {
    return;
  }

  const unsigned long seq = manual["sequence"] | g_lastManualSequence;
  const int dir = manual["direction"] | 0;
  if (seq == g_lastManualSequence) {
    return;
  }

  g_lastManualSequence = seq;
  if (dir > 0) {
    setDesiredPercent(0, "manual-open");
  } else if (dir < 0) {
    setDesiredPercent(100, "manual-close");
  } else {
    motorStop();
    publishState(true);
  }
}

void handleJsonCommand(const JsonDocument &doc) {
  applyCalibrationFromCommand(doc["calibration"]);
  handleManual(doc["manualControl"]);

  if (doc["desiredPosition"].is<int>()) {
    setDesiredPercent(doc["desiredPosition"].as<int>(), "desiredPosition");
    return;
  }

  if (doc["position"].is<int>()) {
    setDesiredPercent(doc["position"].as<int>(), "position");
    return;
  }

  const String command = doc["command"] | "";
  if (command == "open") {
    setDesiredPercent(0, "open");
  } else if (command == "close") {
    setDesiredPercent(100, "close");
  } else if (command == "stop") {
    motorStop();
    publishState(true);
  } else if (command == "set_position" || command == "setPosition") {
    setDesiredPercent(doc["position"] | g_desiredPercent, command.c_str());
  }
}

void handlePlainCoverCommand(const String &payload) {
  if (payload == "OPEN") {
    setDesiredPercent(0, "ha-open");
  } else if (payload == "CLOSE") {
    setDesiredPercent(100, "ha-close");
  } else if (payload == "STOP") {
    motorStop();
    publishState(true);
  }
}

void onMqttMessage(char *topic, byte *payload, unsigned int length) {
  String message;
  message.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) {
    message += static_cast<char>(payload[i]);
  }

  const String topicString(topic);
  Serial.printf("[mqtt] %s -> %s\n", topic, message.c_str());

  if (topicString == g_topicCoverSet) {
    handlePlainCoverCommand(message);
    return;
  }

  if (topicString == g_topicCoverPositionSet) {
    setDesiredPercent(message.toInt(), "ha-position");
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.printf("[mqtt] json invalido: %s\n", error.c_str());
    return;
  }

  handleJsonCommand(doc);
}

bool connectMqtt() {
  if (g_mqtt.connected()) {
    return true;
  }

  if (millis() - g_lastMqttAttemptMs < kMqttReconnectMs) {
    return false;
  }
  g_lastMqttAttemptMs = millis();

  Serial.printf("[mqtt] conectando %s:%u\n", kMqttHost, kMqttPort);

  bool ok = false;
  if (strlen(kMqttUser) > 0) {
    ok = g_mqtt.connect(kDeviceId, kMqttUser, kMqttPassword, g_topicAvailability, 1, true, "offline");
  } else {
    ok = g_mqtt.connect(kDeviceId, g_topicAvailability, 1, true, "offline");
  }

  if (!ok) {
    Serial.printf("[mqtt] falha rc=%d\n", g_mqtt.state());
    return false;
  }

  Serial.println("[mqtt] conectado");
  g_mqtt.publish(g_topicAvailability, "online", true);
  subscribeTopics();
  publishHomeAssistantDiscovery();
  publishState(true);
  return true;
}

void publishState(bool force) {
  if (!g_mqtt.connected()) {
    return;
  }

  const unsigned long now = millis();
  const unsigned long interval = g_motorOn ? kStatePublishMoveMs : kStatePublishIdleMs;
  if (!force && now - g_lastStatePublishMs < interval) {
    return;
  }
  g_lastStatePublishMs = now;

  const float position = roundf(currentPercent() * 10.0f) / 10.0f;
  const long raw = snapshotRaw();
  const long norm = effectiveTicks();

  JsonDocument doc;
  doc["deviceId"] = kDeviceId;
  JsonObject st = doc["state"].to<JsonObject>();
  st["position"] = position;
  st["targetPosition"] = g_desiredPercent;
  st["moving"] = g_motorOn;
  st["reachable"] = true;
  st["ip"] = WiFi.localIP().toString();
  st["uptimeMs"] = millis();
  st["wifiRssi"] = WiFi.RSSI();
  st["resetReason"] = g_resetReason;
  st["rawEncoderTicks"] = raw;
  st["normalizedEncoderTicks"] = norm;
  st["encoderState"] = readEncoderQuadratureState();
  st["encoderDirectionSign"] = g_dirSign;
  st["encoderTicksClosedApplied"] = 0;
  st["encoderTicksOpenApplied"] = g_maxSteps;

  String payload;
  serializeJson(doc, payload);
  g_mqtt.publish(g_topicState, payload.c_str(), true);

  char positionText[8];
  snprintf(positionText, sizeof(positionText), "%d", roundedCurrentPercent());
  g_mqtt.publish(g_topicCoverPosition, positionText, true);
  g_mqtt.publish(g_topicCoverState, coverState(), true);
}

void tickMotorTowardTarget() {
  const unsigned long now = millis();
  const long ticks = effectiveTicks();
  const long targetTicks = (static_cast<long>(g_desiredPercent) * g_maxSteps) / 100L;
  const long err = targetTicks - ticks;

  if (g_motorOn) {
    if (now - g_lastEncoderChangeMs >= kStallStopMs) {
      Serial.println("[motor] parada por stall");
      motorStop();
      publishState(true);
      return;
    }
    if (now - g_motionStartMs >= kMotorMaxRunMs) {
      Serial.println("[motor] parada por tempo maximo");
      motorStop();
      publishState(true);
      return;
    }
  }

  if (labs(err) <= kTickTolerance) {
    if (g_motorOn) {
      motorStop();
      publishState(true);
    }
    return;
  }

  const long halfSpan = static_cast<long>(g_maxSteps) / 8L;
  const long slowZone = 400L > halfSpan ? 400L : halfSpan;
  int pwm = kMotorPwmMax;
  if (labs(err) < slowZone) {
    pwm = kMotorPwmSlow;
  }

  if (err > 0) {
    motorClose();
    setPwm(pwm);
  } else {
    motorOpen();
    setPwm(pwm);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(120);
  g_resetReason = ESP.getResetReason();

  pinMode(kMotorIn1Pin, OUTPUT);
  pinMode(kMotorIn2Pin, OUTPUT);
  pinMode(kMotorEnablePin, OUTPUT);
  analogWriteRange(kMotorPwmMax);
  analogWriteFreq(1000);
  motorStop();

  pinMode(kEncoderAPin, INPUT_PULLUP);
  pinMode(kEncoderBPin, INPUT_PULLUP);
  g_encPrev = readEncoderQuadratureState();
  attachInterrupt(digitalPinToInterrupt(kEncoderAPin), onEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kEncoderBPin), onEncoderIsr, CHANGE);

  loadEeprom();
  g_lastRawObserved = snapshotRaw();
  g_lastEncoderChangeMs = millis();

  setupTopics();
  g_mqtt.setServer(kMqttHost, kMqttPort);
  g_mqtt.setCallback(onMqttMessage);
  g_mqtt.setBufferSize(4096);

  Serial.println("[boot] persiana-mqtt");
}

void loop() {
  const long r = snapshotRaw();
  if (r != g_lastRawObserved) {
    g_lastRawObserved = r;
    g_lastEncoderChangeMs = millis();
  }

  tickMotorTowardTarget();

  if (ensureWifi()) {
    ensureOta();
    connectMqtt();
  }

  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }

  if (g_mqtt.connected()) {
    g_mqtt.loop();
    publishState(false);
  }

  delay(10);
}

#endif
