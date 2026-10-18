# BME688 Wearable Environmental Monitor

Dispositivo vestível baseado em ESP32 + sensor BME688 para monitoramento de qualidade do ar, com objetivo futuro de detectar vazamentos através da análise da **taxa de variação** das concentrações de gás medidas.

## Status atual

- [x] Firmware base rodando em bancada (ESP32 + BME688)
- [x] Dashboard web local (servido pelo próprio ESP32) com temperatura, umidade, pressão, altitude e índice de qualidade do ar
- [x] Cálculo de taxa de variação (derivada) das leituras de gás
- [x] Sistema de alarme/threshold para detecção de vazamento
- [ ] Modo de baixo consumo (deep sleep) para uso em bateria
- [ ] Migração do formato de bancada para formato vestível
- [ ] Interface remota (app celular via BLE/WiFi, substituindo o dashboard servido localmente)

## Hardware

- ESP32 (DevKit para prototipagem; formato reduzido a definir para versão wearable)
- Sensor BME688 (temperatura, umidade, pressão, resistência de gás/COVs)

## Estrutura do projeto

```
.
├── src/
│   └── main.cpp       # firmware principal
├── data/               # arquivos para SPIFFS (se necessário)
├── platformio.ini      # configuração do PlatformIO
└── README.md
```

## Como rodar

1. Instale a extensão [PlatformIO](https://platformio.org/) no VSCode
2. Abra a pasta do projeto no VSCode
3. Ajuste `ssid` e `password` em `src/main.cpp` (ou migre para `secrets.h`, ignorado pelo git)
4. Conecte o ESP32 via USB
5. Use o botão de Upload do PlatformIO (ou `pio run --target upload`)
6. Abra o Monitor Serial (115200 baud) para ver os logs

## Aviso de segurança

⚠️ As credenciais de WiFi estão hardcoded em `main.cpp` neste estágio inicial do projeto — isso é aceitável para prototipagem em rede isolada/de bancada, mas **não deve ir para produção assim**. Antes de qualquer uso real (e especialmente antes de decisões de segurança envolvendo detecção de vazamento), migrar para um arquivo `secrets.h` fora do controle de versão.

## Licença

A definir.
