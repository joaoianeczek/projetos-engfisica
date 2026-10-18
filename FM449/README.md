# BME688 Wearable Environmental Monitor

Monitor ambiental baseado em **ESP32 + sensor BME688** para acompanhamento da qualidade do ar. O dispositivo calibra uma resistência de referência (R0) em ar limpo, suaviza as leituras de gás com uma **média móvel** e classifica o ambiente em quatro níveis, sinalizados por **LED RGB** e **buzzer**. O objetivo futuro é evoluir para um dispositivo vestível capaz de detectar vazamentos a partir da **taxa de variação** das concentrações de gás.

## Status atual

- [x] Firmware base rodando em bancada (ESP32 + BME688)
- [x] Leitura de temperatura, umidade, pressão, resistência de gás, altitude e ponto de orvalho
- [x] Calibração automática do sensor (R0) na inicialização
- [x] Média móvel das leituras de gás usando fila em lista ligada (soma acumulada, média em O(1))
- [x] Classificação da qualidade do ar em 4 níveis (razão R_médio / R0)
- [x] Alertas visuais (LED RGB) e sonoros (buzzer), sem bloqueio do `loop()`
- [x] Log completo via Serial (115200 baud)
- [ ] Cálculo da taxa de variação (derivada) das leituras de gás
- [ ] Sistema de threshold/alarme para detecção de vazamento
- [ ] Dashboard web local servido pelo ESP32
- [ ] Modo de baixo consumo (deep sleep) para uso em bateria
- [ ] Migração do formato de bancada para formato vestível
- [ ] Interface remota (app celular via BLE/WiFi)

## Hardware

| Componente | Detalhes |
|---|---|
| Microcontrolador | ESP32 (DevKit para prototipagem; formato reduzido a definir para a versão vestível) |
| Sensor | BME688 via I2C (temperatura, umidade, pressão, resistência de gás/COVs) |
| LED RGB | Ânodo comum (nível `LOW` acende, `HIGH` apaga) |
| Buzzer | Ativo, acionado por nível `HIGH` |

### Pinagem

| Função | GPIO |
|---|---|
| LED vermelho | 13 |
| LED verde | 12 |
| LED azul | 14 |
| Buzzer | 15 |
| BME688 (I2C) | SDA / SCL padrão do ESP32 |

## Como funciona

### 1. Inicialização e configuração do sensor

Ao ligar, o firmware inicia o BME688 e aplica a configuração abaixo, seguindo a ordem recomendada no datasheet (umidade, temperatura e pressão):

- Oversampling: umidade 4x, temperatura 8x, pressão 4x
- Filtro IIR: tamanho 3
- Aquecedor de gás: 320 °C por 150 ms

Se o sensor não for encontrado, o firmware informa o erro pela Serial e fica parado aguardando correção das conexões.

### 2. Calibração (R0)

Com o **LED branco aceso**, o firmware descarta a primeira leitura (geralmente fria) e coleta **30 leituras, uma por segundo**. A média aritmética dessas leituras define a resistência de referência **R0**, que deve ser obtida em **ar limpo**. A calibração leva cerca de 30 segundos. Ao final, a fila da média móvel é limpa para começar do zero.

### 3. Média móvel (fila em lista ligada)

Cada nova leitura de gás é inserida em uma fila de capacidade **N = 5** (`TAMANHO_JANELA_MEDIA`). Quando a fila está cheia, o elemento mais antigo é removido antes da inserção. Como a soma é mantida em um acumulador, a média é calculada em tempo constante.

### 4. Classificação da qualidade do ar

A razão `R = R_médio / R0` define o nível. Quanto menor a resistência de gás em relação à referência, pior o ar.

| Nível | Razão R | Classificação | LED | Buzzer |
|---|---|---|---|---|
| 1 | ≥ 0,75 | Bom | Verde | — |
| 2 | ≥ 0,40 | Moderado | Amarelo | — |
| 3 | ≥ 0,20 | Ruim | Vermelho | — |
| 4 | < 0,20 | Crítico | Magenta | Sim (3 s) |

### 5. Ciclo de operação

- Uma nova leitura é feita a cada **20 segundos**.
- Após cada leitura, o LED indica o nível por **5 segundos** e depois apaga.
- LED e buzzer são controlados com `millis()` (sem `delay()` no controle de alertas), então o `loop()` continua responsivo.

### 6. Saída pela Serial

A cada leitura são impressos: temperatura, umidade, pressão, resistência de gás instantânea e média móvel (com ocupação da fila, por exemplo `[3/5]`), altitude, ponto de orvalho, razão R e nível de alerta.

- **Altitude**: calculada com pressão ao nível do mar de 1013,25 hPa (`SEALEVELPRESSURE_HPA`).
- **Ponto de orvalho**: calculado pela aproximação de Magnus.

## Estrutura do projeto

```
.
├── src/
│   └── main.cpp       # firmware principal
├── platformio.ini     # configuração do PlatformIO
└── README.md
```

## Dependências

- Framework Arduino para ESP32
- [Adafruit BME680 Library](https://github.com/adafruit/Adafruit_BME680)
- [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor)

Exemplo de `platformio.ini`:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    adafruit/Adafruit BME680 Library
    adafruit/Adafruit Unified Sensor
```

## Como rodar

1. Instale a extensão [PlatformIO](https://platformio.org/) no VSCode
2. Abra a pasta do projeto no VSCode
3. Monte o circuito conforme a tabela de pinagem
4. Conecte o ESP32 via USB
5. Use o botão de Upload do PlatformIO (ou `pio run --target upload`)
6. Abra o Monitor Serial (115200 baud)
7. **Mantenha o ambiente com ar limpo durante os ~30 s de calibração** após ligar

## Limitações conhecidas

- A qualidade da classificação depende de a calibração ser feita em ar limpo. Se for feita em ambiente já poluído, R0 fica baixo e os níveis ficam subestimados.
- O sensor de gás do BME688 tem tempo de aquecimento/estabilização; as primeiras leituras após ligar podem variar mais.
- A leitura usa `delay()` curto durante a medição e a calibração bloqueia o firmware por cerca de 30 s. Isso é aceitável em bancada, mas deve ser revisto para a versão em bateria.
- Os limiares de classificação (0,75 / 0,40 / 0,20) são empíricos e ainda não foram validados contra um gás de referência.
- O sistema **ainda não detecta vazamentos**. Os níveis atuais indicam apenas a qualidade geral do ar. Não deve ser usado como dispositivo de segurança.

## Próximos passos

1. Calcular a taxa de variação da resistência de gás (derivada sobre a janela da fila) para detectar quedas bruscas
2. Definir thresholds de alarme de vazamento com base na derivada
3. Adicionar deep sleep e medir o consumo
4. Reintroduzir conectividade (dashboard web local e, depois, BLE/app celular). Ao fazer isso, manter as credenciais de WiFi em um `secrets.h` fora do controle de versão
5. Projetar o formato vestível (placa reduzida, bateria, case)

## Licença

A definir.