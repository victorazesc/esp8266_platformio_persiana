#pragma once

// Copie este arquivo para `local_secrets_esp8266.h` e preencha seus valores reais.
// Esse arquivo fica como exemplo para evitar copiar senhas/tokens reais.

constexpr char kWifiSsid[] = "NOME_DA_REDE";
constexpr char kWifiPassword[] = "SENHA_DA_REDE";

// Usado como clientId MQTT, hostname OTA e identificador do dispositivo.
constexpr char kDeviceId[] = "persiana";

// Mantido por compatibilidade com firmwares anteriores.
// O firmware MQTT atual nao usa esse segredo para autenticar comandos.
constexpr char kDeviceSecret[] = "troque-este-segredo";
