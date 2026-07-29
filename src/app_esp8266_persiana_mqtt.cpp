#ifdef USE_ESP8266_PERSIANA_MQTT

/*
 * Firmware ESP8266 para persiana com MQTT.
 *
 * Lógica:
 * - D5/D6: ponte H direção
 * - D7: PWM enable
 * - D1/D2: encoder quadrature
 * - 0% = aberta
 * - 100% = fechada
 * - EEPROM v3: curso, sinal, posição normalizada e confiança
 * - reboot durante movimento invalida a posição até nova calibração
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
 * Comandos:
 * {"command":"open"}
 * {"command":"close"}
 * {"command":"stop"}
 * {"command":"set_position","position":50}
 * {"desiredPosition":50}
 * {"position":50}
 *
 * Jog/manual direto:
 * {"jog":"open"}
 * {"jog":"close"}
 * {"jog":"stop"}
 *
 * Calibração segura: aberto -> jog fechar -> fechado.
 * {"calibration":{"setOpenHere":true}}
 * {"calibration":{"setClosedHere":true}}
 *
 * Avançado: zeroHere invalida a calibração completa; maxSteps não calibra.
 * {"calibration":{"zeroHere":true}}
 * {"calibration":{"maxSteps":32000}}
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "local_secrets_esp8266.h"

#ifndef MQTT_HOST
#define MQTT_HOST "192.168.1.57"
#endif

#ifndef MQTT_HOST_FALLBACK
#define MQTT_HOST_FALLBACK MQTT_HOST
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

namespace
{
  constexpr uint8_t kMotorIn1Pin = D5;
  constexpr uint8_t kMotorIn2Pin = D6;
  constexpr uint8_t kMotorEnablePin = D7;
  constexpr uint8_t kEncoderAPin = D1;
  constexpr uint8_t kEncoderBPin = D2;

  constexpr int kMotorPwmMax = 1023;
  constexpr int kMotorPwmSlow = 900;
  constexpr int kMotorPwmEndpoint = 760;
  constexpr long kTickTolerance = 48;

  constexpr unsigned long kStatePublishIdleMs = 1000;
  constexpr unsigned long kStatePublishMoveMs = 250;
  constexpr unsigned long kMqttReconnectMs = 3000;
  constexpr unsigned long kMqttPrimaryProbeInitialMs = 30000;
  constexpr unsigned long kMqttPrimaryProbeMaxMs = 300000;
  constexpr unsigned long kWifiReconnectMs = 5000;
  constexpr unsigned long kMotorMaxRunMs = 45000;
  constexpr unsigned long kStallStopMs = 1200;
  constexpr uint16_t kMqttKeepAliveSeconds = 15;
  constexpr uint16_t kMqttSocketTimeoutSeconds = 3;

  constexpr long kMinCalibrationSpan = 512;
  constexpr long kMaxCalibrationSpan = 120000;

  // Versão 3: persiste apenas posição normalizada confirmada com o motor parado.
  constexpr uint32_t kEepromMagic = 0x50523356UL;

  constexpr const char *kMqttHosts[] = {MQTT_HOST, MQTT_HOST_FALLBACK};
  constexpr size_t kMqttHostCount = sizeof(kMqttHosts) / sizeof(kMqttHosts[0]);
  constexpr uint16_t kMqttPort = MQTT_PORT;
  constexpr char kMqttUser[] = MQTT_USER;
  constexpr char kMqttPassword[] = MQTT_PASSWORD;
  constexpr char kMqttBaseTopic[] = MQTT_BASE_TOPIC;

  enum class CalibrationState : uint8_t
  {
    None = 0,
    OpenReferenceSet = 1,
    Valid = 2,
  };

  struct EepromStore
  {
    uint32_t magic;
    int32_t maxSteps;
    int32_t dirSign;
    int32_t normalizedTicks;
    uint8_t calibrationState;
    uint8_t positionTrusted;
    uint8_t reserved[2];
  };

  volatile long g_rawTicks = 0;
  volatile uint8_t g_encPrev = 0;

  int32_t g_zeroRaw = 0;
  int32_t g_maxSteps = 8000;
  int g_dirSign = 1;

  int g_desiredPercent = 0;

  bool g_motorOn = false;
  bool g_jogMode = false;
  bool g_calibrated = false;
  bool g_positionKnown = false;
  bool g_positionTrusted = false;
  CalibrationState g_calibrationState = CalibrationState::None;
  int32_t g_persistedNormalizedTicks = 0;
  int g_motorDir = 0;

  unsigned long g_motionStartMs = 0;
  unsigned long g_lastStatePublishMs = 0;
  unsigned long g_lastEncoderChangeMs = 0;
  unsigned long g_lastMqttAttemptMs = 0;
  unsigned long g_lastWifiAttemptMs = 0;
  unsigned long g_nextMqttPrimaryProbeMs = 0;
  unsigned long g_mqttPrimaryProbeIntervalMs = kMqttPrimaryProbeInitialMs;
  size_t g_mqttHostIndex = 0;

  long g_lastRawObserved = 0;

  String g_resetReason;
  bool g_otaReady = false;
  int g_pwm = 0;

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
  void saveEeprom();

  void markPositionUntrustedForMotion()
  {
    if (!g_positionTrusted)
    {
      return;
    }

    g_positionTrusted = false;
    saveEeprom();
    Serial.println("[eeprom] posicao marcada como nao confiavel antes do movimento");
  }

  void setPwm(int value)
  {
    g_pwm = constrain(value, 0, kMotorPwmMax);
    analogWrite(kMotorEnablePin, g_pwm);
  }

  void motorOpen()
  {
    markPositionUntrustedForMotion();

    digitalWrite(kMotorIn1Pin, LOW);
    digitalWrite(kMotorIn2Pin, HIGH);
    setPwm(kMotorPwmMax);

    g_motorOn = true;
    g_motorDir = 1;
    g_motionStartMs = millis();
    g_lastEncoderChangeMs = millis();
  }

  void motorClose()
  {
    markPositionUntrustedForMotion();

    digitalWrite(kMotorIn1Pin, HIGH);
    digitalWrite(kMotorIn2Pin, LOW);
    setPwm(kMotorPwmMax);

    g_motorOn = true;
    g_motorDir = -1;
    g_motionStartMs = millis();
    g_lastEncoderChangeMs = millis();
  }

  void motorStop()
  {
    analogWrite(kMotorEnablePin, 0);
    digitalWrite(kMotorEnablePin, LOW);
    digitalWrite(kMotorIn1Pin, LOW);
    digitalWrite(kMotorIn2Pin, LOW);
    g_pwm = 0;
    g_motorOn = false;
    g_motorDir = 0;
  }

  uint8_t readEncoderQuadratureState()
  {
    return (digitalRead(kEncoderAPin) ? 1U : 0U) |
           (digitalRead(kEncoderBPin) ? 2U : 0U);
  }

  IRAM_ATTR void onEncoderIsr()
  {
    // digitalRead() e tabelas const podem acessar flash com o cache indisponível
    // durante a ISR no ESP8266. Leia o registrador GPIO e use apenas comparações.
    const uint32_t gpioLevels = GPI;
    const uint8_t next =
        ((gpioLevels >> kEncoderAPin) & 1U) |
        (((gpioLevels >> kEncoderBPin) & 1U) << 1U);
    const uint8_t transition = (g_encPrev << 2U) | next;

    if (
        transition == 2U ||
        transition == 4U ||
        transition == 11U ||
        transition == 13U)
    {
      g_rawTicks++;
    }
    else if (
        transition == 1U ||
        transition == 7U ||
        transition == 8U ||
        transition == 14U)
    {
      g_rawTicks--;
    }

    g_encPrev = next;
  }

  long snapshotRaw()
  {
    noInterrupts();
    const long ticks = g_rawTicks;
    interrupts();
    return ticks;
  }

  long effectiveTicks()
  {
    if (g_maxSteps <= 0)
    {
      return 0;
    }

    const long raw = snapshotRaw();
    const long effective = (raw - g_zeroRaw) * static_cast<long>(g_dirSign);

    if (effective < 0)
    {
      return 0;
    }

    if (effective > g_maxSteps)
    {
      return g_maxSteps;
    }

    return effective;
  }

  float currentPercent()
  {
    if (g_maxSteps <= 0)
    {
      return 0.0f;
    }

    return (100.0f * static_cast<float>(effectiveTicks())) /
           static_cast<float>(g_maxSteps);
  }

  int roundedCurrentPercent()
  {
    return constrain(static_cast<int>(roundf(currentPercent())), 0, 100);
  }

  const char *calibrationStateName()
  {
    switch (g_calibrationState)
    {
    case CalibrationState::OpenReferenceSet:
      return "open_reference_set";
    case CalibrationState::Valid:
      return "valid";
    default:
      return "none";
    }
  }

  void refreshCalibrationStatus()
  {
    g_calibrated =
        g_calibrationState == CalibrationState::Valid &&
        g_positionKnown;
  }

  void persistTrustedPositionAtRest()
  {
    if (
        !g_positionKnown ||
        g_calibrationState != CalibrationState::Valid)
    {
      return;
    }

    const int32_t normalized =
        static_cast<int32_t>(effectiveTicks());

    if (
        g_positionTrusted &&
        g_persistedNormalizedTicks == normalized)
    {
      return;
    }

    g_persistedNormalizedTicks = normalized;
    g_positionTrusted = true;
    saveEeprom();
    Serial.printf(
        "[eeprom] posicao parada confirmada normalizedTicks=%ld\n",
        static_cast<long>(g_persistedNormalizedTicks));
  }

  void stopAndHoldCurrentPosition()
  {
    g_jogMode = false;
    motorStop();

    if (g_positionKnown)
    {
      g_desiredPercent = roundedCurrentPercent();
    }
    else
    {
      g_desiredPercent = 0;
    }

    persistTrustedPositionAtRest();
    publishState(true);
  }

  const char *coverState()
  {
    if (g_motorOn && g_motorDir > 0)
    {
      return "opening";
    }

    if (g_motorOn && g_motorDir < 0)
    {
      return "closing";
    }

    if (!g_positionKnown)
    {
      return "unknown";
    }

    const int position = roundedCurrentPercent();

    if (position <= 2)
    {
      return "open";
    }

    if (position >= 98)
    {
      return "closed";
    }

    return "stopped";
  }

  void loadEeprom()
  {
    EepromStore store{};

    EEPROM.begin(sizeof(EepromStore));
    EEPROM.get(0, store);
    EEPROM.end();

    const bool stateValid =
        store.calibrationState <= static_cast<uint8_t>(CalibrationState::Valid);
    const bool spanValid =
        store.maxSteps >= kMinCalibrationSpan &&
        store.maxSteps <= kMaxCalibrationSpan;
    const CalibrationState storedState =
        stateValid
            ? static_cast<CalibrationState>(store.calibrationState)
            : CalibrationState::None;
    const bool normalizedValid =
        store.normalizedTicks >= 0 &&
        store.normalizedTicks <= store.maxSteps;
    const bool trustedPositionValid =
        store.positionTrusted == 1U &&
        normalizedValid &&
        (storedState == CalibrationState::Valid ||
         (storedState == CalibrationState::OpenReferenceSet &&
          store.normalizedTicks == 0));

    if (store.magic == kEepromMagic && stateValid && spanValid)
    {
      g_maxSteps = store.maxSteps;
      g_dirSign = store.dirSign == -1 ? -1 : 1;
      g_calibrationState = storedState;
      g_persistedNormalizedTicks =
          normalizedValid ? store.normalizedTicks : 0;
      g_positionTrusted = trustedPositionValid;
      g_positionKnown = trustedPositionValid;

      // O contador incremental volta a zero no boot. Reconstrua uma origem
      // relativa somente quando a última gravação ocorreu com o motor parado.
      g_zeroRaw = trustedPositionValid
                      ? -g_persistedNormalizedTicks * g_dirSign
                      : 0;
      refreshCalibrationStatus();

      Serial.printf(
          "[eeprom] maxSteps=%ld dirSign=%d calibrationState=%s positionTrusted=%s normalizedTicks=%ld\n",
          static_cast<long>(g_maxSteps),
          g_dirSign,
          calibrationStateName(),
          g_positionTrusted ? "true" : "false",
          static_cast<long>(g_persistedNormalizedTicks));
    }
    else
    {
      g_zeroRaw = 0;
      g_maxSteps = 8000;
      g_dirSign = 1;
      g_calibrationState = CalibrationState::None;
      g_positionKnown = false;
      g_positionTrusted = false;
      g_persistedNormalizedTicks = 0;
      refreshCalibrationStatus();
      Serial.println("[eeprom] sem parametros de calibracao validos, usando defaults");
    }
  }

  void saveEeprom()
  {
    EepromStore store{};

    store.magic = kEepromMagic;
    store.maxSteps = g_maxSteps;
    store.dirSign = g_dirSign;
    store.normalizedTicks = g_persistedNormalizedTicks;
    store.calibrationState = static_cast<uint8_t>(g_calibrationState);
    store.positionTrusted = g_positionTrusted ? 1U : 0U;

    EEPROM.begin(sizeof(EepromStore));
    EEPROM.put(0, store);
    EEPROM.commit();
    EEPROM.end();

    Serial.println("[eeprom] salvo");
  }

  void applyCalibrationFromCommand(JsonVariantConst calibration)
  {
    if (calibration.isNull())
    {
      return;
    }

    if (calibration["setOpenHere"].is<bool>() && calibration["setOpenHere"].as<bool>())
    {
      g_jogMode = false;
      motorStop();

      g_zeroRaw = static_cast<int32_t>(snapshotRaw());
      g_desiredPercent = 0;
      g_dirSign = 1;
      g_calibrationState = CalibrationState::OpenReferenceSet;
      g_positionKnown = true;
      g_positionTrusted = true;
      g_persistedNormalizedTicks = 0;
      refreshCalibrationStatus();

      saveEeprom();

      Serial.printf(
          "[cal] aberto definido aqui zeroRaw=%ld dirSign=%d\n",
          static_cast<long>(g_zeroRaw),
          g_dirSign);

      publishState(true);
      return;
    }

    if (calibration["setClosedHere"].is<bool>() && calibration["setClosedHere"].as<bool>())
    {
      g_jogMode = false;
      motorStop();

      const long raw = snapshotRaw();
      const long span = labs(raw - g_zeroRaw);

      if (
          g_calibrationState != CalibrationState::OpenReferenceSet ||
          !g_positionKnown)
      {
        Serial.println("[cal] fechado ignorado: defina primeiro uma referencia aberta confiavel");
        publishState(true);
        return;
      }

      if (span >= kMinCalibrationSpan && span <= kMaxCalibrationSpan)
      {
        g_maxSteps = static_cast<int32_t>(span);
        g_dirSign = raw >= g_zeroRaw ? 1 : -1;
        g_desiredPercent = 100;
        g_calibrationState = CalibrationState::Valid;
        g_positionKnown = true;
        g_positionTrusted = true;
        g_persistedNormalizedTicks = g_maxSteps;
        refreshCalibrationStatus();

        saveEeprom();

        Serial.printf(
            "[cal] fechado definido aqui raw=%ld zero=%ld maxSteps=%ld dirSign=%d\n",
            raw,
            static_cast<long>(g_zeroRaw),
            static_cast<long>(g_maxSteps),
            g_dirSign);

        publishState(true);
      }
      else
      {
        Serial.printf(
            "[cal] span invalido: %ld; calibracao permanece incompleta\n",
            span);
        publishState(true);
      }

      return;
    }

    if (calibration["maxSteps"].is<long>())
    {
      const long maxSteps = calibration["maxSteps"].as<long>();

      if (maxSteps >= kMinCalibrationSpan && maxSteps <= kMaxCalibrationSpan)
      {
        g_jogMode = false;
        motorStop();

        g_maxSteps = static_cast<int32_t>(maxSteps);
        g_desiredPercent = g_positionKnown ? roundedCurrentPercent() : 0;

        if (
            g_positionKnown &&
            g_calibrationState == CalibrationState::Valid)
        {
          g_persistedNormalizedTicks =
              static_cast<int32_t>(effectiveTicks());
          g_positionTrusted = true;
        }

        refreshCalibrationStatus();

        saveEeprom();

        Serial.printf(
            "[cal] maxSteps definido manualmente: %ld calibrationState=%s\n",
            maxSteps,
            calibrationStateName());
        publishState(true);
      }
      else
      {
        Serial.printf("[cal] maxSteps invalido: %ld\n", maxSteps);
        publishState(true);
      }

      return;
    }

    if (calibration["zeroHere"].is<bool>() && calibration["zeroHere"].as<bool>())
    {
      g_jogMode = false;
      motorStop();

      g_zeroRaw = static_cast<int32_t>(snapshotRaw());
      g_desiredPercent = 0;
      g_dirSign = 1;
      g_calibrationState = CalibrationState::OpenReferenceSet;
      g_positionKnown = true;
      g_positionTrusted = true;
      g_persistedNormalizedTicks = 0;
      refreshCalibrationStatus();

      saveEeprom();

      Serial.printf(
          "[cal] zero definido aqui zeroRaw=%ld calibrationState=%s calibrated=%s\n",
          static_cast<long>(g_zeroRaw),
          calibrationStateName(),
          g_calibrated ? "true" : "false");

      publishState(true);
      return;
    }
  }

  bool ensureWifi()
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      return true;
    }

    const unsigned long now = millis();

    if (now - g_lastWifiAttemptMs < kWifiReconnectMs)
    {
      return false;
    }

    g_lastWifiAttemptMs = now;

    WiFi.mode(WIFI_STA);
    WiFi.begin(kWifiSsid, kWifiPassword);

    Serial.printf("[wifi] tentando conectar em %s\n", kWifiSsid);

    return false;
  }

  void ensureOta()
  {
    if (g_otaReady || WiFi.status() != WL_CONNECTED)
    {
      return;
    }

    ArduinoOTA.setHostname(kDeviceId);

    ArduinoOTA.onStart([]()
                       {
                         g_jogMode = false;
                         stopAndHoldCurrentPosition();
                         Serial.println("[ota] inicio"); });

    ArduinoOTA.onEnd([]()
                     { Serial.println("[ota] fim"); });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          {
                            if (total)
                            {
                              Serial.printf("[ota] %u%%\r", (progress * 100U) / total);
                            } });

    ArduinoOTA.onError([](ota_error_t error)
                       { Serial.printf("[ota] erro %u\n", error); });

    ArduinoOTA.begin();

    g_otaReady = true;

    Serial.printf("[ota] pronto %s:8266\n", WiFi.localIP().toString().c_str());
  }

  void setupTopics()
  {
    snprintf(g_topicState, sizeof(g_topicState), "%s/%s/state", kMqttBaseTopic, kDeviceId);
    snprintf(g_topicCommand, sizeof(g_topicCommand), "%s/%s/command", kMqttBaseTopic, kDeviceId);
    snprintf(g_topicAvailability, sizeof(g_topicAvailability), "%s/%s/availability", kMqttBaseTopic, kDeviceId);
    snprintf(g_topicCoverState, sizeof(g_topicCoverState), "%s/%s/cover/state", kMqttBaseTopic, kDeviceId);
    snprintf(g_topicCoverSet, sizeof(g_topicCoverSet), "%s/%s/cover/set", kMqttBaseTopic, kDeviceId);
    snprintf(g_topicCoverPosition, sizeof(g_topicCoverPosition), "%s/%s/cover/position", kMqttBaseTopic, kDeviceId);
    snprintf(g_topicCoverPositionSet, sizeof(g_topicCoverPositionSet), "%s/%s/cover/position/set", kMqttBaseTopic, kDeviceId);
    snprintf(g_topicDiscovery, sizeof(g_topicDiscovery), "homeassistant/cover/%s/config", kDeviceId);
  }

  void publishHomeAssistantDiscovery()
  {
    JsonDocument doc;

    doc["name"] = "Persiana Sala";
    doc["unique_id"] = kDeviceId;
    doc["object_id"] = kDeviceId;
    doc["device_class"] = "shade";

    doc["command_topic"] = g_topicCoverSet;
    doc["json_command_topic"] = g_topicCommand;
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

  void subscribeTopics()
  {
    g_mqtt.subscribe(g_topicCommand);
    g_mqtt.subscribe(g_topicCoverSet);
    g_mqtt.subscribe(g_topicCoverPositionSet);

    Serial.printf("[mqtt] subscribe %s\n", g_topicCommand);
    Serial.printf("[mqtt] subscribe %s\n", g_topicCoverSet);
    Serial.printf("[mqtt] subscribe %s\n", g_topicCoverPositionSet);
  }

  void setDesiredPercent(int percent, const char *reason)
  {

    if (!g_calibrated)
    {
      Serial.printf("[target] ignorado sem calibracao: %d motivo=%s\n", percent, reason);
      publishState(true);
      return;
    }

    g_jogMode = false;
    g_desiredPercent = constrain(percent, 0, 100);

    Serial.printf("[target] %d%% motivo=%s\n", g_desiredPercent, reason);

    publishState(true);
  }

  void handleJsonCommand(const JsonDocument &doc)
  {
    applyCalibrationFromCommand(doc["calibration"]);

    const String jog = doc["jog"] | "";

    if (jog == "open")
    {
      g_jogMode = true;
      motorOpen();
      g_desiredPercent = roundedCurrentPercent();

      Serial.println("[jog] open");

      publishState(true);
      return;
    }

    if (jog == "close")
    {
      g_jogMode = true;
      motorClose();
      g_desiredPercent = roundedCurrentPercent();

      Serial.println("[jog] close");

      publishState(true);
      return;
    }

    if (jog == "stop")
    {
      stopAndHoldCurrentPosition();

      Serial.println("[jog] stop");

      return;
    }

    if (doc["desiredPosition"].is<int>())
    {
      setDesiredPercent(doc["desiredPosition"].as<int>(), "desiredPosition");
      return;
    }

    if (doc["position"].is<int>())
    {
      setDesiredPercent(doc["position"].as<int>(), "position");
      return;
    }

    const String command = doc["command"] | "";

    if (command == "open")
    {
      setDesiredPercent(0, "open");
      return;
    }

    if (command == "close")
    {
      setDesiredPercent(100, "close");
      return;
    }

    if (command == "stop")
    {
      stopAndHoldCurrentPosition();
      return;
    }

    if (command == "set_position" || command == "setPosition")
    {
      setDesiredPercent(doc["position"] | g_desiredPercent, command.c_str());
      return;
    }
  }

  void handlePlainCoverCommand(const String &payload)
  {
    if (payload == "OPEN")
    {
      setDesiredPercent(0, "ha-open");
      return;
    }

    if (payload == "CLOSE")
    {
      setDesiredPercent(100, "ha-close");
      return;
    }

    if (payload == "STOP")
    {
      stopAndHoldCurrentPosition();
      return;
    }
  }

  void onMqttMessage(char *topic, byte *payload, unsigned int length)
  {
    String message;
    message.reserve(length + 1);

    for (unsigned int i = 0; i < length; i++)
    {
      message += static_cast<char>(payload[i]);
    }

    const String topicString(topic);

    Serial.printf("[mqtt] %s -> %s\n", topic, message.c_str());

    if (topicString == g_topicCoverSet)
    {
      handlePlainCoverCommand(message);
      return;
    }

    if (topicString == g_topicCoverPositionSet)
    {
      setDesiredPercent(message.toInt(), "ha-position");
      return;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);

    if (error)
    {
      Serial.printf("[mqtt] json invalido: %s\n", error.c_str());
      return;
    }

    handleJsonCommand(doc);
  }

  bool connectMqtt()
  {
    if (g_mqtt.connected())
    {
      return true;
    }

    if (g_motorOn)
    {
      return false;
    }

    if (millis() - g_lastMqttAttemptMs < kMqttReconnectMs)
    {
      return false;
    }

    g_lastMqttAttemptMs = millis();

    const char *mqttHost = kMqttHosts[g_mqttHostIndex];
    g_mqtt.setServer(mqttHost, kMqttPort);

    Serial.printf("[mqtt] conectando %s:%u\n", mqttHost, kMqttPort);

    bool connected = false;

    if (strlen(kMqttUser) > 0)
    {
      connected = g_mqtt.connect(
          kDeviceId,
          kMqttUser,
          kMqttPassword,
          g_topicAvailability,
          1,
          true,
          "offline");
    }
    else
    {
      connected = g_mqtt.connect(
          kDeviceId,
          g_topicAvailability,
          1,
          true,
          "offline");
    }

    if (!connected)
    {
      Serial.printf("[mqtt] falha rc=%d\n", g_mqtt.state());

      if (g_mqttHostIndex == 0 && kMqttHostCount > 1)
      {
        g_mqttPrimaryProbeIntervalMs =
            min(g_mqttPrimaryProbeIntervalMs * 2UL, kMqttPrimaryProbeMaxMs);
      }

      g_mqttHostIndex = (g_mqttHostIndex + 1) % kMqttHostCount;
      return false;
    }

    Serial.println("[mqtt] conectado");

    if (g_mqttHostIndex == 0)
    {
      g_nextMqttPrimaryProbeMs = 0;
      g_mqttPrimaryProbeIntervalMs = kMqttPrimaryProbeInitialMs;
    }
    else
    {
      g_nextMqttPrimaryProbeMs = millis() + g_mqttPrimaryProbeIntervalMs;
    }

    g_mqtt.publish(g_topicAvailability, "online", true);

    subscribeTopics();
    publishHomeAssistantDiscovery();
    publishState(true);

    return true;
  }

  void preferPrimaryMqttHost()
  {
    if (!g_mqtt.connected() || g_mqttHostIndex == 0 || g_motorOn)
    {
      return;
    }

    const unsigned long now = millis();

    if (g_nextMqttPrimaryProbeMs == 0 ||
        static_cast<long>(now - g_nextMqttPrimaryProbeMs) < 0)
    {
      return;
    }

    Serial.printf("[mqtt] testando servidor principal %s:%u\n", kMqttHosts[0], kMqttPort);

    g_mqtt.publish(g_topicAvailability, "offline", true);
    g_mqtt.disconnect();
    g_mqttHostIndex = 0;
    g_lastMqttAttemptMs = now - kMqttReconnectMs;
    g_nextMqttPrimaryProbeMs = 0;
  }

  void publishState(bool force)
  {
    if (!g_mqtt.connected())
    {
      return;
    }

    const unsigned long now = millis();
    const unsigned long interval = g_motorOn ? kStatePublishMoveMs : kStatePublishIdleMs;

    if (!force && now - g_lastStatePublishMs < interval)
    {
      return;
    }

    g_lastStatePublishMs = now;

    const long raw = snapshotRaw();

    JsonDocument doc;

    doc["deviceId"] = kDeviceId;

    JsonObject state = doc["state"].to<JsonObject>();

    if (g_positionKnown)
    {
      state["position"] = roundf(currentPercent() * 10.0f) / 10.0f;
      state["targetPosition"] = g_desiredPercent;
      state["normalizedEncoderTicks"] = effectiveTicks();
    }
    else
    {
      state["position"] = nullptr;
      state["targetPosition"] = nullptr;
      state["normalizedEncoderTicks"] = nullptr;
    }

    state["moving"] = g_motorOn;
    state["jogMode"] = g_jogMode;
    state["motionDirection"] =
        g_motorOn
            ? (g_motorDir > 0 ? "opening" : "closing")
            : "stopped";
    state["calibrated"] = g_calibrated;
    state["calibrationState"] = calibrationStateName();
    state["positionKnown"] = g_positionKnown;
    state["positionTrusted"] = g_positionTrusted;
    state["reachable"] = true;
    state["ip"] = WiFi.localIP().toString();
    state["uptimeMs"] = millis();
    state["wifiRssi"] = WiFi.RSSI();
    state["resetReason"] = g_resetReason;
    state["rawEncoderTicks"] = raw;
    state["encoderState"] = readEncoderQuadratureState();
    state["encoderDirectionSign"] = g_dirSign;
    state["encoderTicksOpenApplied"] = 0;
    state["encoderTicksClosedApplied"] = g_maxSteps;
    state["pwm"] = g_pwm;

    String payload;
    serializeJson(doc, payload);

    g_mqtt.publish(g_topicState, payload.c_str(), true);

    if (g_positionKnown)
    {
      char positionText[8];
      snprintf(positionText, sizeof(positionText), "%d", roundedCurrentPercent());

      g_mqtt.publish(g_topicCoverPosition, positionText, true);
    }
    else
    {
      // Remove uma posição retida de antes do reboot.
      g_mqtt.publish(g_topicCoverPosition, "", true);
    }

    g_mqtt.publish(g_topicCoverState, coverState(), true);
  }

  void tickMotorTowardTarget()
  {
    const unsigned long now = millis();

    if (g_motorOn)
    {
      if (now - g_lastEncoderChangeMs >= kStallStopMs)
      {
        Serial.println("[motor] parada por stall");
        stopAndHoldCurrentPosition();
        return;
      }

      if (now - g_motionStartMs >= kMotorMaxRunMs)
      {
        Serial.println("[motor] parada por tempo maximo");
        stopAndHoldCurrentPosition();
        return;
      }
    }

    if (g_jogMode)
    {
      return;
    }

    if (!g_calibrated)
    {
      if (g_motorOn)
      {
        stopAndHoldCurrentPosition();
      }

      return;
    }

    const long currentTicks = effectiveTicks();
    const long targetTicks = (static_cast<long>(g_desiredPercent) * g_maxSteps) / 100L;
    const long error = targetTicks - currentTicks;

    if (labs(error) <= kTickTolerance)
    {
      if (g_motorOn)
      {
        stopAndHoldCurrentPosition();
      }

      return;
    }

    const long halfSpan = static_cast<long>(g_maxSteps) / 8L;
    const long slowZone = 400L > halfSpan ? 400L : halfSpan;

    int pwm = kMotorPwmMax;

    if (labs(error) < slowZone)
    {
      pwm = kMotorPwmSlow;
    }

    if (
        (g_desiredPercent == 0 || g_desiredPercent == 100) &&
        labs(error) < slowZone * 2L &&
        pwm > kMotorPwmEndpoint)
    {
      pwm = kMotorPwmEndpoint;
    }

    if (error > 0)
    {
      if (!g_motorOn || g_motorDir != -1)
      {
        motorClose();
      }

      setPwm(pwm);
    }
    else
    {
      if (!g_motorOn || g_motorDir != 1)
      {
        motorOpen();
      }

      setPwm(pwm);
    }
  }

} // namespace

void setup()
{
  Serial.begin(115200);
  delay(120);

  g_resetReason = ESP.getResetReason();

  pinMode(kMotorIn1Pin, OUTPUT);
  pinMode(kMotorIn2Pin, OUTPUT);
  pinMode(kMotorEnablePin, OUTPUT);

  analogWriteRange(kMotorPwmMax);
  analogWriteFreq(20000);

  motorStop();

  pinMode(kEncoderAPin, INPUT_PULLUP);
  pinMode(kEncoderBPin, INPUT_PULLUP);

  g_encPrev = readEncoderQuadratureState();

  attachInterrupt(digitalPinToInterrupt(kEncoderAPin), onEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kEncoderBPin), onEncoderIsr, CHANGE);

  loadEeprom();

  g_lastRawObserved = snapshotRaw();
  g_lastEncoderChangeMs = millis();

  // Evita movimento automático ao reiniciar.
  g_desiredPercent = roundedCurrentPercent();

  setupTopics();

  g_mqtt.setServer(kMqttHosts[g_mqttHostIndex], kMqttPort);
  g_mqtt.setCallback(onMqttMessage);
  g_mqtt.setBufferSize(4096);
  g_mqtt.setKeepAlive(kMqttKeepAliveSeconds);
  g_mqtt.setSocketTimeout(kMqttSocketTimeoutSeconds);

  Serial.println("[boot] persiana-mqtt");
}

void loop()
{
  const long raw = snapshotRaw();

  if (raw != g_lastRawObserved)
  {
    g_lastRawObserved = raw;
    g_lastEncoderChangeMs = millis();
  }

  tickMotorTowardTarget();

  if (ensureWifi())
  {
    ensureOta();
    preferPrimaryMqttHost();
    connectMqtt();
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    ArduinoOTA.handle();
  }

  if (g_mqtt.connected())
  {
    g_mqtt.loop();
    publishState(false);
  }

  delay(10);
}

#endif
