
#define BLYNK_TEMPLATE_ID   "SEU_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Lorax"
#define BLYNK_AUTH_TOKEN    "SEU_DEVICE_AUTH_TOKEN"

#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <math.h>
#include <stdlib.h>

const char WIFI_SSID[] = "NOME_DO_WIFI";
const char WIFI_PASSWORD[] = "SENHA_DO_WIFI";

#define SEALEVELPRESSURE_HPA 1013.25F
#define TAMANHO_JANELA_MEDIA 5

const unsigned long INTERVALO_LEITURA_MS = 5000UL;
const unsigned long INTERVALO_ENVIO_MS = 30000UL;

const unsigned long INTERVALO_WIFI_MS = 15000UL;
const unsigned long INTERVALO_BLYNK_MS = 10000UL;

const unsigned long DURACAO_BUZZER_MS = 1000UL;
const unsigned long DURACAO_LED_MS = 3000UL;

const unsigned long IDADE_MAXIMA_LEITURA_MS = 10000UL;


// ============================================================
// 1. ESTRUTURAS DA FILA
// ============================================================

typedef struct No {
    float dado;
    struct No *prox;
} No;

typedef struct {
    No *inicio;
    No *fim;

    int tamanho;
    int capacidade_max;

    // Permite calcular a media sem percorrer a fila.
    float soma;
} Fila;


// ============================================================
// 2. HARDWARE E VARIAVEIS GLOBAIS
// ============================================================

// Mantidos os pinos do seu circuito.
// A logica assume LED e buzzer acionados com HIGH.
const int pinRED = 13;
const int pinGREEN = 12;
const int pinBLUE = 14;
const int pinBUZZER = 15;

unsigned long BuzzerBeginTime = 0;
unsigned long LedBeginTime = 0;

bool buzzerAtivo = false;
bool ledAtivo = false;

// Indica se a ultima tentativa de leitura teve sucesso.
bool leituraValida = false;

// Resistencia de referencia em kOhms.
float R0 = 0.0F;

// Sensor conectado via I2C.
Adafruit_BME680 bme;

// Gerencia os intervalos de leitura e envio.
BlynkTimer timer;

struct SensorData {
    float temperature;
    float humidity;
    float pressure;

    float gasResistance;
    float gasResistanceMedia;

    float altitude;
    float dewPoint;

    // 0 = sem referencia; 1 a 4 = niveis do projeto.
    int airQualityLevel;

    float razao_R;
    unsigned long lastUpdate;
};

SensorData sensorData = {};
Fila filaResistencia = {};


// ============================================================
// 3. PROTOTIPOS
// ============================================================

// Sensor e calculos.
bool readSensorData();

float dewPointCalculation(float temperature, float humidity);
int calculateAirQualityLevel(float razao_R);

void printSensorData();
void calibrarSensor();

// Monitoramento e perifericos.
void atualizarMonitor();

void activateBuzzer();
void updateBuzzer();
void updateLED();
void apagarLED();

// Comunicacao.
void manterConexao();
void enviarDadosBlynk();

// Fila.
Fila cria_fila(int capacidade_max);

bool enqueue(Fila *fila, float valor);
void dequeue(Fila *fila);

float obter_media_fila(Fila *fila);
void limpar_fila(Fila *fila);


// ============================================================
// 4. SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println();
    Serial.println("LORAX - Monitor BME680/BME688");
    Serial.println("Inicializando...");

    // Inicializa as saidas antes de acessar o sensor.
    pinMode(pinRED, OUTPUT);
    pinMode(pinGREEN, OUTPUT);
    pinMode(pinBLUE, OUTPUT);
    pinMode(pinBUZZER, OUTPUT);

    apagarLED();
    digitalWrite(pinBUZZER, LOW);

    filaResistencia = cria_fila(TAMANHO_JANELA_MEDIA);

    // Usa os pinos I2C padrao da placa selecionada.
    Wire.begin();

    // Tenta os dois enderecos I2C usuais do sensor.
    if (!bme.begin(0x77) && !bme.begin(0x76)) {
        Serial.println("Sensor nao encontrado.");
        Serial.println("Verifique alimentacao, SDA e SCL.");

        while (true) {
            delay(1000);
        }
    }

    bme.setHumidityOversampling(BME680_OS_4X);
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setPressureOversampling(BME680_OS_4X);

    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);

    Serial.println("Sensor configurado.");

    // A referencia inicial acontece antes da conexao.
    // Portanto, seus delays nao interrompem uma sessao Blynk.
    calibrarSensor();

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Configura o Blynk sem aguardar indefinidamente pelo Wi-Fi.
    Blynk.config(BLYNK_AUTH_TOKEN);

    timer.setInterval(INTERVALO_LEITURA_MS, atualizarMonitor);
    timer.setInterval(INTERVALO_ENVIO_MS, enviarDadosBlynk);

    // Primeira amostra de monitoramento.
    atualizarMonitor();

    Serial.println("Monitor iniciado.");
    Serial.println("Conectando ao Wi-Fi e ao Blynk...");
}


// ============================================================
// 5. LOOP PRINCIPAL
// ============================================================

void loop() {
    updateBuzzer();
    updateLED();

    manterConexao();

    // Processa a comunicacao enquanto houver conexao.
    if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
        Blynk.run();
    }

    // Executa as tarefas quando seus intervalos vencem.
    timer.run();

    updateBuzzer();
    updateLED();
}


// ============================================================
// 6. CONEXAO E RECONEXAO
// ============================================================

void manterConexao() {
    static unsigned long ultimaTentativaWiFi = 0;
    static unsigned long ultimaTentativaBlynk = 0;

    const unsigned long agora = millis();

    if (WiFi.status() != WL_CONNECTED) {
        if (Blynk.connected()) {
            Blynk.disconnect();
        }

        if (agora - ultimaTentativaWiFi >= INTERVALO_WIFI_MS) {
            ultimaTentativaWiFi = agora;

            Serial.println("Tentando reconectar ao Wi-Fi...");
            WiFi.reconnect();
        }

        return;
    }

    if (!Blynk.connected() &&
        agora - ultimaTentativaBlynk >= INTERVALO_BLYNK_MS) {

        ultimaTentativaBlynk = agora;

        Serial.println("Wi-Fi conectado. Conectando ao Blynk...");

        // Timeout solicitado de 1 segundo.
        // Operacoes de rede ainda podem introduzir latencia.
        if (Blynk.connect(1000)) {
            Serial.println("Blynk conectado!");
        } else {
            Serial.println("Blynk indisponivel; tentarei novamente.");
        }
    }
}


// ============================================================
// 7. ENVIO DOS DADOS AO BLYNK
// ============================================================

void enviarDadosBlynk() {
    if (WiFi.status() != WL_CONNECTED || !Blynk.connected()) {
        return;
    }

    if (!leituraValida || R0 <= 0.0F) {
        return;
    }

    // Evita enviar uma amostra antiga como se fosse atual.
    if (millis() - sensorData.lastUpdate > IDADE_MAXIMA_LEITURA_MS) {
        return;
    }

    // V0: temperatura em graus Celsius.
    Blynk.virtualWrite(V0, sensorData.temperature);

    // V1: resistencia media dividida pela referencia.
    Blynk.virtualWrite(V1, sensorData.razao_R);

    // V2: nivel de alerta de 1 a 4.
    Blynk.virtualWrite(V2, sensorData.airQualityLevel);

    Serial.println("Dados enviados ao Blynk: V0, V1 e V2.");
}


// ============================================================
// 8. LEITURA DO SENSOR
// ============================================================

bool readSensorData() {
    leituraValida = false;

// Realiza uma amostra completa.
    // Esta chamada aguarda a medicao do sensor terminar.
    if (!bme.performReading()) {
        Serial.println("Falha na leitura do BME680/BME688.");
        return false;
    }

    const float temperatura = bme.temperature;
    const float umidade = bme.humidity;

    // Pa -> hPa.
    const float pressao = bme.pressure / 100.0F;

    // Ohms -> kOhms.
    const float resistencia = bme.gas_resistance / 1000.0F;

    if (!isfinite(temperatura) ||
        !isfinite(umidade) ||
        !isfinite(pressao) ||
        !isfinite(resistencia) ||
        umidade < 0.0F ||
        umidade > 100.0F ||
        pressao <= 0.0F ||
        resistencia <= 0.0F) {

        Serial.println("Amostra invalida; leitura descartada.");
        return false;
    }

    sensorData.temperature = temperatura;
    sensorData.humidity = umidade;
    sensorData.pressure = pressao;
    sensorData.gasResistance = resistencia;

    // Calcula altitude usando a pressao da mesma amostra.
    sensorData.altitude = 44330.0F *
        (1.0F - powf(pressao / SEALEVELPRESSURE_HPA, 0.1903F));

    sensorData.dewPoint =
        dewPointCalculation(temperatura, umidade);

    if (R0 > 0.0F) {
        // Se a insercao falhar, nao publica media desatualizada.
        if (!enqueue(&filaResistencia, resistencia)) {
            Serial.println("Falha ao atualizar a media movel.");
            return false;
        }

        sensorData.gasResistanceMedia =
            obter_media_fila(&filaResistencia);

        sensorData.razao_R =
            sensorData.gasResistanceMedia / R0;

        sensorData.airQualityLevel =
            calculateAirQualityLevel(sensorData.razao_R);
    } else {
        // Durante a referencia inicial nao existe classificacao.
        sensorData.gasResistanceMedia = resistencia;
        sensorData.razao_R = 0.0F;
        sensorData.airQualityLevel = 0;
    }

    sensorData.lastUpdate = millis();
    leituraValida = true;

    return true;
}


// ============================================================
// 9. CALCULOS
// ============================================================

float dewPointCalculation(float temperature, float humidity) {
    // Evita log(0) e entradas fora do dominio.
    if (!isfinite(temperature) ||
        !isfinite(humidity) ||
        humidity <= 0.0F ||
        humidity > 100.0F) {

        return NAN;
    }

    const float a = 17.271F;
    const float b = 237.7F;

    const float termo =
        (a * temperature) / (b + temperature) +
        logf(humidity / 100.0F);

    return (b * termo) / (a - termo);
}

int calculateAirQualityLevel(float razao_R) {
    // Limiares experimentais mantidos do seu projeto.
    // Nao representam um indice oficial de qualidade do ar.
    if (razao_R >= 0.75F) return 1;
    if (razao_R >= 0.40F) return 2;
    if (razao_R >= 0.20F) return 3;

    return 4;
}


// ============================================================
// 10. REFERENCIA INICIAL DO SENSOR
// ============================================================

void calibrarSensor() {
    Serial.println();
    Serial.println("Iniciando referencia do sensor...");
    Serial.println("Mantenha o sensor em ambiente limpo.");

    digitalWrite(pinRED, HIGH);
    digitalWrite(pinGREEN, HIGH);
    digitalWrite(pinBLUE, HIGH);

    R0 = 0.0F;
    limpar_fila(&filaResistencia);

    const int numeroLeituras = 10;
    const int maxTentativas = 20;

    int validas = 0;
    float somaDasLeituras = 0.0F;

    // Descarta a primeira amostra.
    readSensorData();
    delay(500);

    for (int tentativa = 0;
         tentativa < maxTentativas && validas < numeroLeituras;
         tentativa++) {

        if (readSensorData()) {
            somaDasLeituras += sensorData.gasResistance;
            validas++;

            Serial.printf(
                "Referencia: %d/%d | Resistencia: %.2f kOhms\n",
                validas,
                numeroLeituras,
                sensorData.gasResistance
            );
        } else {
            Serial.println("Amostra ignorada na referencia.");
        }

        delay(5000);
    }

    apagarLED();

    if (validas == numeroLeituras) {
        R0 = somaDasLeituras / validas;

        Serial.printf(
            "Referencia concluida: R0 = %.2f kOhms\n",
            R0
        );
    } else {
        Serial.println("Nao foi possivel obter a referencia.");
        Serial.println("Verifique o sensor e reinicie.");
    }

    limpar_fila(&filaResistencia);

    // Exige uma nova amostra antes de enviar ao Blynk.
    leituraValida = false;
}


// ============================================================
// 11. ATUALIZACAO DO MONITOR E DOS ALERTAS
// ============================================================

void atualizarMonitor() {
    if (!readSensorData()) {
        return;
    }

    printSensorData();

    // Sem referencia, nao aciona alertas de qualidade do ar.
    if (R0 <= 0.0F) {
        return;
    }

    apagarLED();

    switch (sensorData.airQualityLevel) {
        case 1:
            // Bom: verde.
            digitalWrite(pinGREEN, HIGH);
            break;

        case 2:
            // Moderado: amarelo.
            digitalWrite(pinRED, HIGH);
            digitalWrite(pinGREEN, HIGH);
            break;

        case 3:
            // Ruim: vermelho.
            digitalWrite(pinRED, HIGH);
            break;

        case 4:
            // Critico: magenta e buzzer.
            digitalWrite(pinRED, HIGH);
            digitalWrite(pinBLUE, HIGH);
            activateBuzzer();
            break;
    }

    LedBeginTime = millis();
    ledAtivo = true;
}


// ============================================================
// 12. IMPRESSAO NO MONITOR SERIAL
// ============================================================

void printSensorData() {
    Serial.println();
    Serial.println("--- Novas leituras do sensor ---");

    Serial.printf(
        "Temperatura: %.2f C\n",
        sensorData.temperature
    );

    Serial.printf(
        "Umidade: %.2f %%\n",
        sensorData.humidity
    );

    Serial.printf(
        "Pressao: %.2f hPa\n",
        sensorData.pressure
    );

    Serial.printf(
        "Gas instantaneo: %.2f kOhms\n",
        sensorData.gasResistance
    );

    Serial.printf(
        "Gas medio [%d/%d]: %.2f kOhms\n",
        filaResistencia.tamanho,
        filaResistencia.capacidade_max,
        sensorData.gasResistanceMedia
    );

    Serial.printf(
        "Altitude estimada: %.2f m\n",
        sensorData.altitude
    );

    if (isfinite(sensorData.dewPoint)) {
        Serial.printf(
            "Ponto de orvalho: %.2f C\n",
            sensorData.dewPoint
        );
    } else {
        Serial.println("Ponto de orvalho: indisponivel");
    }

    if (R0 > 0.0F) {
        Serial.printf("R0: %.2f kOhms\n", R0);

        Serial.printf(
            "Razao R_medio/R0: %.3f\n",
            sensorData.razao_R
        );
    }

    switch (sensorData.airQualityLevel) {
        case 1:
            Serial.println("Nivel de alerta: Bom");
            break;

        case 2:
            Serial.println("Nivel de alerta: Moderado");
            break;

        case 3:
            Serial.println("Nivel de alerta: Ruim");
            break;

        case 4:
            Serial.println("Nivel de alerta: Critico");
            break;

        default:
            Serial.println("Nivel de alerta: sem referencia");
            break;
    }

    Serial.printf(
        "Wi-Fi: %s | Blynk: %s\n",
        WiFi.status() == WL_CONNECTED ? "conectado" : "desconectado",
        Blynk.connected() ? "conectado" : "desconectado"
    );

    Serial.println("--------------------------------");
}


// ============================================================
// 13. CONTROLE DO BUZZER E DO LED
// ============================================================

void activateBuzzer() {
    digitalWrite(pinBUZZER, HIGH);

    BuzzerBeginTime = millis();
    buzzerAtivo = true;
}

void updateBuzzer() {
    if (buzzerAtivo &&
        millis() - BuzzerBeginTime >= DURACAO_BUZZER_MS) {

        digitalWrite(pinBUZZER, LOW);
        buzzerAtivo = false;
    }
}

void apagarLED() {
    digitalWrite(pinRED, LOW);
    digitalWrite(pinGREEN, LOW);
    digitalWrite(pinBLUE, LOW);

    ledAtivo = false;
}

void updateLED() {
    if (ledAtivo &&
        millis() - LedBeginTime >= DURACAO_LED_MS) {

        apagarLED();
    }
}


// ============================================================
// 14. FILA ENCADEADA E MEDIA MOVEL
// ============================================================

Fila cria_fila(int capacidade_max) {
    Fila fila;

    fila.inicio = NULL;
    fila.fim = NULL;
    fila.tamanho = 0;
    fila.capacidade_max = capacidade_max;
    fila.soma = 0.0F;

    return fila;
}

bool enqueue(Fila *fila, float valor) {
    if (fila == NULL || fila->capacidade_max <= 0) {
        return false;
    }

    // Aloca antes de remover a amostra antiga.
    // Assim, uma falha de memoria preserva a fila existente.
    No *novo = (No *)malloc(sizeof(No));

    if (novo == NULL) {
        Serial.println("Erro: memoria insuficiente para a fila.");
        return false;
    }

    novo->dado = valor;
    novo->prox = NULL;

    if (fila->tamanho >= fila->capacidade_max) {
        dequeue(fila);
    }

    if (fila->fim == NULL) {
        fila->inicio = novo;
        fila->fim = novo;
    } else {
        fila->fim->prox = novo;
        fila->fim = novo;
    }

    fila->soma += valor;
    fila->tamanho++;

    return true;
}

void dequeue(Fila *fila) {
    if (fila == NULL || fila->inicio == NULL) {
        return;
    }

    No *temp = fila->inicio;

    fila->soma -= temp->dado;
    fila->inicio = temp->prox;
    fila->tamanho--;

    if (fila->inicio == NULL) {
        fila->fim = NULL;
        fila->soma = 0.0F;
    }

    free(temp);
}

float obter_media_fila(Fila *fila) {
    if (fila == NULL || fila->tamanho == 0) {
        return 0.0F;
    }

    return fila->soma / fila->tamanho;
}

void limpar_fila(Fila *fila) {
    if (fila == NULL) {
        return;
    }

    while (fila->inicio != NULL) {
        dequeue(fila);
    }
}