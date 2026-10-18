#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <stdio.h>
#include <stdlib.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define TAMANHO_JANELA_MEDIA 5 // Quantidade 'N' de leituras na fila para a media movel

// ---------------- ESTRUTURAS DA FILA ----------------
typedef struct No {
    float dado;
    struct No *prox;
} No;

typedef struct {
    No *inicio;
    No *fim;
    int tamanho;
    int capacidade_max;
    float soma; // Acumulador para calcular a media em O(1)
} Fila;

// ---------------- HARDWARE E VARIAVEIS GLOBAIS ----------------
// Pinos do LED e do Buzzer
const int pinRED = 13;
const int pinGREEN = 12;
const int pinBLUE = 14;
const int pinBUZZER = 15;

// Variaveis para o controle do Buzzer e do LED
unsigned long BuzzerBeginTime = 0;
bool buzzerAtivo = false;
unsigned long LedBeginTime = 0;
bool ledAtivo = false;

// Resistencia de referencia do sensor
float R0 = 0.0;

Adafruit_BME680 bme; // I2C

// Estrutura de dados do sensor
struct SensorData {
    float temperature;
    float humidity;
    float pressure;
    float gasResistance;      // Resistencia instantanea lida (kOhms)
    float gasResistanceMedia; // Media movel das ultimas N resistencias (kOhms)
    float altitude;
    float dewPoint;
    int airQualityLevel;
    float razao_R;
    unsigned long lastUpdate;
};

SensorData sensorData;
Fila filaResistencia; // Instancia global da fila de media movel

// ---------------- PROTOTIPOS DAS FUNCOES ----------------
// Funcoes do Sensor e Perifericos
void readSensorData();
float dewPointCalculation(float temperature, float humidity);
void printSensorData();
void activateBuzzer();
void updateBuzzer();
void updateLED();
void calibrarSensor();
int calculateAirQualityLevel(float razao_R);

// Funcoes da Fila (Lista Ligada)
Fila cria_fila(int capacidade_max);
void enqueue(Fila *fila, float valor);
void dequeue(Fila *fila);
float obter_media_fila(Fila *fila);
void limpar_fila(Fila *fila);

// ---------------- SETUP E LOOP ----------------
void setup() {
    Serial.begin(115200);
    delay(100);
    
    Serial.println(F("BME688 Environmental Monitor - Inicializando..."));
    
    if (!bme.begin()) {
        Serial.println("Nao foi possivel encontrar um sensor BME688 valido, verifique as conexoes!");
        while (1) {
            delay(1000);
        }
    }

    // Inicializa a fila com o tamanho definido para a media movel
    filaResistencia = cria_fila(TAMANHO_JANELA_MEDIA);

    // Configuracao dos pinos do LED e do buzzer
    pinMode(pinRED, OUTPUT);
    pinMode(pinGREEN, OUTPUT);
    pinMode(pinBLUE, OUTPUT);
    pinMode(pinBUZZER, OUTPUT);

    digitalWrite(pinRED, HIGH);
    digitalWrite(pinGREEN, HIGH);
    digitalWrite(pinBLUE, HIGH);
    digitalWrite(pinBUZZER, LOW);
    
    // Configuracao do sensor
    // Segundo o datasheet, e recomendado iniciar umidade, temperatura e pressao nesta ordem
    bme.setHumidityOversampling(BME680_OS_4X);
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320 C por 150 ms
    Serial.println("Configuracao do sensor BME688 concluida.\n");

    calibrarSensor(); // Realiza a calibracao com o LED branco aceso
    readSensorData(); // Primeira leitura ja aplicando a fila
    printSensorData();
}

void loop() {
    static unsigned long lastReadTime = 0;
    
    // Realiza a leitura a cada 20 segundos (20000 ms)
    if (millis() - lastReadTime > 20000) {
        readSensorData();
        printSensorData();

        // Reseta o LED (apaga todas as cores)
        digitalWrite(pinRED, HIGH);
        digitalWrite(pinGREEN, HIGH);
        digitalWrite(pinBLUE, HIGH);

        // Aciona o hardware conforme o nivel
        // 1 = bom (verde), 2 = moderado (amarelo), 3 = ruim (vermelho), 4 = critico (magenta + buzzer)
        switch (sensorData.airQualityLevel) {
            case 1:
                // LED verde
                digitalWrite(pinGREEN, LOW);
                break;
            case 2:
                // LED amarelo
                digitalWrite(pinRED, LOW);
                digitalWrite(pinGREEN, LOW);
                break;
            case 3:
                // LED vermelho
                digitalWrite(pinRED, LOW);
                break;
            case 4:
                // LED magenta + Buzzer
                digitalWrite(pinRED, LOW);
                digitalWrite(pinBLUE, LOW);
                activateBuzzer();
                break;
        }
        // Inicia os parametros de controle do LED
        LedBeginTime = millis();
        ledAtivo = true;

        lastReadTime = millis();
    }

    updateBuzzer();
    updateLED();
}

// ---------------- FUNCOES DO SENSOR E HARDWARE ----------------
void readSensorData() {
    unsigned long endTime = bme.beginReading();
    if (endTime == 0) {
        Serial.println("Falha ao iniciar leitura :(");
        return;
    }
    
    delay(50); // Tempo para medicao do sensor

    if (!bme.endReading()) {
        Serial.println("Falha ao completar leitura :(");
        return;
    }

    // Coleta e calculo dos dados basicos
    sensorData.humidity = bme.humidity;
    sensorData.temperature = bme.temperature;
    sensorData.pressure = bme.pressure / 100.0F;
    sensorData.gasResistance = bme.gas_resistance / 1000.0F; // kOhms
    sensorData.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    sensorData.dewPoint = dewPointCalculation(sensorData.temperature, sensorData.humidity);

    // Se o sensor ja foi calibrado (R0 > 0), alimenta a fila e usa a media movel
    if (R0 == 0.0F) {
        sensorData.gasResistanceMedia = sensorData.gasResistance;
        sensorData.razao_R = 0.0F;
    } else {
        enqueue(&filaResistencia, sensorData.gasResistance);
        sensorData.gasResistanceMedia = obter_media_fila(&filaResistencia);
        sensorData.razao_R = sensorData.gasResistanceMedia / R0;
    }

    sensorData.airQualityLevel = calculateAirQualityLevel(sensorData.razao_R);
    sensorData.lastUpdate = millis();
}

float dewPointCalculation(float temperature, float humidity) {
    float a = 17.271;
    float b = 237.7;
    float temp = (a * temperature) / (b + temperature) + log(humidity / 100.0);
    return (b * temp) / (a - temp);
}

int calculateAirQualityLevel(float razao_R) {
    if (razao_R >= 0.75) return 1;      // Excelente / Bom
    else if (razao_R >= 0.40) return 2; // Moderado
    else if (razao_R >= 0.20) return 3; // Ruim
    else return 4;                      // Critico
}

void printSensorData() {
    Serial.println("--- Novas Leituras do Sensor ---");
    Serial.printf("Temperatura: %.2f C\n", sensorData.temperature);
    Serial.printf("Umidade: %.2f %%\n", sensorData.humidity);
    Serial.printf("Pressao: %.2f hPa\n", sensorData.pressure);
    Serial.printf("Gas (Instantaneo): %.2f kOhms\n", sensorData.gasResistance);
    Serial.printf("Gas (Media Movel [%d/%d]): %.2f kOhms\n",
                  filaResistencia.tamanho, filaResistencia.capacidade_max, sensorData.gasResistanceMedia);
    Serial.printf("Altitude: %.2f m\n", sensorData.altitude);
    Serial.printf("Ponto de Orvalho: %.2f C\n", sensorData.dewPoint);
    Serial.printf("Razao entre as resistencias (R_medio / R0): %.2f\n", sensorData.razao_R);

    switch (sensorData.airQualityLevel) {
        case 1:
            Serial.printf("Nivel de Alerta: Bom\n");
            break;
        case 2:
            Serial.printf("Nivel de Alerta: Moderado\n");
            break;
        case 3:
            Serial.printf("Nivel de Alerta: Ruim\n");
            break;
        case 4:
            Serial.printf("Nivel de Alerta: Critico\n");
            break;
    }
    Serial.println("--------------------------------\n");
}

void activateBuzzer() {
    digitalWrite(pinBUZZER, HIGH);
    BuzzerBeginTime = millis();
    buzzerAtivo = true;
}

void updateBuzzer() {
    if (buzzerAtivo && millis() - BuzzerBeginTime > 3000) {
        digitalWrite(pinBUZZER, LOW);
        buzzerAtivo = false;
    }
}

void calibrarSensor() {
    Serial.println("Iniciando calibracao do BME688...");
    Serial.println("Mantenha o ambiente limpo. Aguarde alguns segundos.");

    // Acende o LED em Branco (Vermelho + Verde + Azul ativos) durante a calibracao
    digitalWrite(pinRED, LOW);
    digitalWrite(pinGREEN, LOW);
    digitalWrite(pinBLUE, LOW);

    int numeroLeituras = 30;
    float somaDasLeituras = 0.0;
    float leituraAtual;

    // Descarta a primeirissima leitura, que costuma vir com lixo de memoria ou fria
    readSensorData(); 
    delay(500);

    for (int i = 0; i < numeroLeituras; i++) {
        readSensorData();
        leituraAtual = sensorData.gasResistance; // Resistencia instantanea em kOhms
        somaDasLeituras += leituraAtual; 

        Serial.print("#");
        delay(1000);
    }

    // Apaga o LED branco ao finalizar a calibracao
    digitalWrite(pinRED, HIGH);
    digitalWrite(pinGREEN, HIGH);
    digitalWrite(pinBLUE, HIGH);

    // Garante que a fila comece limpa apos a calibracao
    limpar_fila(&filaResistencia);

    // Calcula a media aritmetica de referencia
    R0 = somaDasLeituras / numeroLeituras;
    
    Serial.println();
    Serial.print("Calibracao concluida! R0 = ");
    Serial.print(R0);
    Serial.println(" kOhms\n");
}

void updateLED() {
    if (ledAtivo && (millis() - LedBeginTime > 5000)) {
        digitalWrite(pinRED, HIGH);
        digitalWrite(pinGREEN, HIGH);
        digitalWrite(pinBLUE, HIGH);
        ledAtivo = false;
    }
}

// ---------------- IMPLEMENTACAO DAS FUNCOES DA FILA ----------------
Fila cria_fila(int capacidade_max) {
    Fila fila;
    fila.inicio = NULL;
    fila.fim = NULL;
    fila.tamanho = 0;
    fila.capacidade_max = capacidade_max;
    fila.soma = 0.0F;
    return fila;
}

void enqueue(Fila *fila, float valor) {
    if (fila == NULL) return;

    // Se a fila ja atingiu o limite de N elementos, remove o mais antigo antes de inserir
    if (fila->tamanho >= fila->capacidade_max) {
        dequeue(fila);
    }

    No *novo = (No *)malloc(sizeof(No));
    if (novo == NULL) {
        Serial.println("Erro: Falha ao alocar memoria para a fila!");
        return;
    }
    
    novo->dado = valor;
    novo->prox = NULL;

    if (fila->fim == NULL) { // Fila estava vazia
        fila->inicio = novo;
        fila->fim = novo;
    } else {
        fila->fim->prox = novo;
        fila->fim = novo;
    }

    fila->soma += valor;
    fila->tamanho++;
}

void dequeue(Fila *fila) {
    if (fila == NULL || fila->inicio == NULL) return;

    No *temp = fila->inicio;
    fila->soma -= temp->dado; // Subtrai da soma acumulada o valor que esta saindo

    fila->inicio = temp->prox;

    // Caso critico: se a fila ficou vazia apos a remocao, aterra o ponteiro 'fim'
    if (fila->inicio == NULL) {
        fila->fim = NULL;
        fila->soma = 0.0F; // Evita residuos de arredondamento de ponto flutuante
    }

    free(temp);
    fila->tamanho--;
}

float obter_media_fila(Fila *fila) {
    if (fila == NULL || fila->tamanho == 0) {
        return 0.0F;
    }
    return fila->soma / fila->tamanho;
}

void limpar_fila(Fila *fila) {
    if (fila == NULL) return;
    while (fila->inicio != NULL) {
        dequeue(fila);
    }
}