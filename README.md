# Persiana MQTT Final

Firmware ESP8266 da persiana da sala com controle via MQTT, encoder incremental,
discovery Home Assistant e OTA. A escala é `0% = aberta` e `100% = fechada`.

## Hardware

Mapeamento usado pelo firmware:

| Função | Pino |
| --- | --- |
| Ponte H IN1 | D5 |
| Ponte H IN2 | D6 |
| PWM/enable do motor | D7 |
| Encoder A | D1 |
| Encoder B | D2 |

O ESP8266 conecta somente em redes Wi-Fi de `2,4 GHz`.

## Arquivos

- `platformio.ini`: ambientes PlatformIO para USB e OTA.
- `src/app_esp8266_persiana_mqtt.cpp`: firmware MQTT da persiana.
- `src/local_secrets_esp8266.example.h`: exemplo de secrets locais.

## Configurar secrets

```bash
cp src/local_secrets_esp8266.example.h src/local_secrets_esp8266.h
```

Preencha:

```cpp
constexpr char kWifiSsid[] = "NOME_DA_REDE";
constexpr char kWifiPassword[] = "SENHA_DA_REDE";
constexpr char kDeviceId[] = "persiana";
```

`src/local_secrets_esp8266.h` é ignorado pelo Git. Nunca publique a senha do
Wi-Fi ou credenciais MQTT.

## MQTT

Config atual no `platformio.ini`:

```ini
-DMQTT_HOST=\"192.168.1.57\"
-DMQTT_HOST_FALLBACK=\"192.168.1.34\"
-DMQTT_PORT=1883
-DMQTT_BASE_TOPIC=\"mini-has/devices\"
```

O servidor Mini-HAS (`192.168.1.57`) é sempre o principal. Se a persiana usar o
fallback durante uma indisponibilidade, o firmware testa o principal novamente
com intervalo progressivo de até cinco minutos.

## Segurança da rede

Por padrão, o MQTT está sem usuário/senha e o ArduinoOTA sem senha. Use este
firmware apenas em uma rede local confiável e não exponha as portas MQTT ou OTA
à internet. Para outra instalação, configure autenticação antes do uso.

Com `kDeviceId = "persiana"`, os tópicos ficam:

```text
mini-has/devices/persiana/state
mini-has/devices/persiana/command
mini-has/devices/persiana/availability
mini-has/devices/persiana/cover/state
mini-has/devices/persiana/cover/set
mini-has/devices/persiana/cover/position
mini-has/devices/persiana/cover/position/set
homeassistant/cover/persiana/config
```

## Discovery

O firmware publica discovery retido em `homeassistant/cover/<deviceId>/config`.

Campos importantes:

```json
{
  "device_class": "shade",
  "command_topic": "mini-has/devices/persiana/cover/set",
  "state_topic": "mini-has/devices/persiana/cover/state",
  "position_topic": "mini-has/devices/persiana/cover/position",
  "set_position_topic": "mini-has/devices/persiana/cover/position/set",
  "payload_open": "OPEN",
  "payload_close": "CLOSE",
  "payload_stop": "STOP",
  "position_open": 0,
  "position_closed": 100
}
```

## Estados

`position` usa a escala do firmware:

- `0` = aberta
- `100` = fechada

`cover/state` publica:

- `open`
- `closed`
- `opening`
- `closing`
- `stopped`

`state` publica JSON com:

- `state.position`
- `state.targetPosition`
- `state.moving`
- `state.jogMode`
- `state.motionDirection`
- `state.calibrated`
- `state.calibrationState`
- `state.positionKnown`
- `state.positionTrusted`
- `state.rawEncoderTicks`
- `state.normalizedEncoderTicks`
- `state.encoderDirectionSign`
- `state.encoderTicksOpenApplied`
- `state.encoderTicksClosedApplied`
- `state.pwm`
- `state.wifiRssi`
- `state.resetReason`

## Comandos Home Assistant cover

Abrir:

```bash
mosquitto_pub -h 192.168.1.57 -t mini-has/devices/persiana/cover/set -m OPEN
```

Fechar:

```bash
mosquitto_pub -h 192.168.1.57 -t mini-has/devices/persiana/cover/set -m CLOSE
```

Parar:

```bash
mosquitto_pub -h 192.168.1.57 -t mini-has/devices/persiana/cover/set -m STOP
```

Ir para posicao:

```bash
mosquitto_pub -h 192.168.1.57 -t mini-has/devices/persiana/cover/position/set -m 45
```

## Comandos JSON

Publique em `mini-has/devices/persiana/command`.

Abrir:

```json
{"command":"open"}
```

Fechar:

```json
{"command":"close"}
```

Parar e manter posicao atual:

```json
{"command":"stop"}
```

Ir para posicao:

```json
{"command":"set_position","position":45}
```

Tambem aceito:

```json
{"command":"setPosition","position":45}
```

ou:

```json
{"desiredPosition":45}
```

## Jog manual

Move sem alterar o alvo definitivo até receber `stop`. Use apenas com a
persiana visível: jog é movimento contínuo para encontrar os batentes.

```json
{"jog":"open"}
```

```json
{"jog":"close"}
```

```json
{"jog":"stop"}
```

Se o encoder parar de avançar, o firmware corta o motor após `1,2 s`. Qualquer
movimento também é interrompido após `45 s`.

## Calibração segura

Prefira o assistente da tela da persiana no Mini-HAS:

1. Clique em **Iniciar calibração**.
2. Use **Jog para abrir** até a persiana ficar totalmente aberta.
3. Clique em **Parar motor agora** e depois em **Salvar aberto**.
4. Use **Jog para fechar** até a persiana ficar totalmente fechada.
5. Clique em **Parar motor agora** e depois em **Salvar fechado**.
6. Confirme `calibrated=true`, `positionKnown=true` e posição `100`.

Fluxo MQTT equivalente:

Marcar posicao atual como aberta:

```json
{"calibration":{"setOpenHere":true}}
```

Marcar posicao atual como fechada e validar span:

```json
{"calibration":{"setClosedHere":true}}
```

`setClosedHere` só conclui a calibração quando mede um curso real entre `512` e
`120000` ticks depois da referência aberta.

### Comandos avançados

`zeroHere` redefine o ponto atual como aberto e **invalida a calibração
completa**. `maxSteps` altera apenas o curso armazenado; ele não substitui a
medição dos dois batentes. Não use esses comandos no fluxo normal.

```json
{"calibration":{"zeroHere":true}}
{"calibration":{"maxSteps":32000}}
```

## Persistência e recuperação

- Antes de energizar o motor, o firmware marca a posição persistida como não
  confiável.
- Ao parar normalmente, salva a posição normalizada na EEPROM.
- Um reboot com o motor parado restaura a posição e mantém a calibração.
- Uma queda ou exceção durante o movimento inicia com posição desconhecida,
  bloqueia comandos automáticos e exige nova calibração.
- A ISR do encoder lê os GPIOs diretamente em IRAM para evitar exceções por
  acesso à flash durante interrupções.

## Build

Instale o [PlatformIO Core](https://docs.platformio.org/en/latest/core/index.html)
e execute:

```bash
pio run -e esp8266-persiana-mqtt
```

## Upload USB

```bash
pio run -e esp8266-persiana-mqtt -t upload
```

## Upload OTA

O OTA funciona depois do primeiro upload por USB e quando `persiana.local`
está acessível na rede:

```bash
pio run -e esp8266-persiana-mqtt-ota -t upload
```

Para acompanhar boot, Wi-Fi, MQTT, encoder e calibração:

```bash
pio device monitor -b 115200
```
