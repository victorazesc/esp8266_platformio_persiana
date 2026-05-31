# Persiana MQTT Final

Firmware ESP8266 para a persiana da sala com controle via MQTT e discovery do Home Assistant.

## Arquivos

- `platformio.ini`: ambientes PlatformIO para USB e OTA.
- `src/app_esp8266_persiana_mqtt.cpp`: firmware final MQTT.
- `src/local_secrets_esp8266.example.h`: exemplo de secrets.

## Configurar secrets

Copie o exemplo:

```bash
cp src/local_secrets_esp8266.example.h src/local_secrets_esp8266.h
```

Depois edite:

```cpp
constexpr char kWifiSsid[] = "NOME_DA_REDE";
constexpr char kWifiPassword[] = "SENHA_DA_REDE";
constexpr char kDeviceId[] = "persiana-sala";
```

## MQTT

Config padrão no `platformio.ini`:

```ini
-DMQTT_HOST=\"192.168.1.136\"
-DMQTT_PORT=1883
-DMQTT_BASE_TOPIC=\"mini-has/devices\"
```

## Tópicos

Estado JSON:

```text
mini-has/devices/persiana-sala/state
```

Comando JSON:

```text
mini-has/devices/persiana-sala/command
```

Home Assistant cover:

```text
mini-has/devices/persiana-sala/cover/set
mini-has/devices/persiana-sala/cover/position/set
```

Discovery:

```text
homeassistant/cover/persiana-sala/config
```

## Comandos

Abrir:

```json
{"command":"open"}
```

Fechar:

```json
{"command":"close"}
```

Parar:

```json
{"command":"stop"}
```

Posição:

```json
{"command":"set_position","position":45}
```

ou:

```json
{"desiredPosition":45}
```

## Build

```bash
pio run -e esp8266-persiana-mqtt
```

## Upload USB

```bash
pio run -e esp8266-persiana-mqtt -t upload
```

## Upload OTA

```bash
pio run -e esp8266-persiana-mqtt-ota -t upload
```
