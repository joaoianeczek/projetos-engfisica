#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#define SEALEVELPRESSURE_HPA (1013.25)

// Pinos do LED e do Buzzer
const int pinRED = 13;
const int pinGREEN = 12;
const int pinBLUE = 14;
const int pinBUZZER = 15;

// Variáveis para o controle do Buzzer e do LED
unsigned long BuzzerBeginTime = 0;
bool buzzerAtivo = false;
unsigned long LedBeginTime = 0;
bool ledAtivo = false;

// Resistência de refrência do sensor
float R0 = 0.0;

Adafruit_BME680 bme; // I2C

// Estrutura de dados do sensor
struct SensorData {
    float temperature;
    float humidity;
    float pressure;
    float gasResistance;
    float altitude;
    float dewPoint;
    int airQualityLevel;
    float razao_R;
    unsigned long lastUpdate;
};

SensorData sensorData;

// Protótipos das funções
void readSensorData();
float dewPointCalculation(float temperature, float humidity);
void printSensorData();
void activateBuzzer();
void updateBuzzer();
void updateLED();
void calibrarSensor();
int calculateAirQualityLevel(float razao_R);

void setup() {
    Serial.begin(115200);
    delay(100);
    
    Serial.println(F("BME688 Environmental Monitor - Inicializando..."));
    
    if (!bme.begin()) {
        Serial.println("Não foi possível encontrar um sensor BME688 válido, verifique as conexões!");
        while (1) {
            delay(1000);
        }
    }

    // Configuração dos pinos do LED e do buzzer
    pinMode(pinRED, OUTPUT);
    pinMode(pinGREEN, OUTPUT);
    pinMode(pinBLUE, OUTPUT);
    pinMode(pinBUZZER, OUTPUT);

    digitalWrite(pinRED, HIGH);
    digitalWrite(pinGREEN, HIGH);
    digitalWrite(pinBLUE, HIGH);
    digitalWrite(pinBUZZER, LOW);
    
    // Configuração do sensor

    // Segundo o datasheet, é recomendado iniciar humidade, temperatura e pressão nesta ordem
    bme.setHumidityOversampling(BME680_OS_4X); // define para fazer 4 medições e tomar a média para uma medida
    bme.setTemperatureOversampling(BME680_OS_8X); // define para fazer 8 medições e tomar a média para uma medida
    bme.setPressureOversampling(BME680_OS_4X); // define para fazer 4 medições e tomar a média para uma medida
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // define a temperatura do sensor do gás para 320°C por 150 ms
    Serial.println("Configuração do sensor BME688 concluída.\n");

    calibrarSensor(); // Realiza a calibração da resistência de referência
    readSensorData(); // Primeira leitura
    printSensorData();
}

void loop() {
    static unsigned long lastReadTime = 0; // static faz essa linha ser chamada apenas no primeiro ciclo
    
    // Realiza a leitura a cada 20 segundos (20000 ms)
    if (millis() - lastReadTime > 20000) {
        readSensorData();
        printSensorData();

        // Reseta o LED
        digitalWrite(pinRED, HIGH);
        digitalWrite(pinGREEN, HIGH);
        digitalWrite(pinBLUE, HIGH);

        // Aciona o hardware conforme o nível
        // 1 = bom, 2 = moderado, 3 = ruim, 4 = crítico
        switch(sensorData.airQualityLevel) {
            case 1:
                // Led verde
                digitalWrite(pinGREEN, LOW);
                break;
            case 2:
                // led amarelo
                digitalWrite(pinRED, LOW);
                digitalWrite(pinGREEN, LOW);
                break;
            case 3:
                // led magenta
                digitalWrite(pinRED, LOW);
                digitalWrite(pinBLUE, LOW);
                break;
            case 4:
                digitalWrite(pinRED, LOW);
                activateBuzzer();
                break;
        }
        // Inicia os parâmetros de controle do LED
        LedBeginTime = millis();
        ledAtivo = true;

        lastReadTime = millis();
    }

    updateBuzzer();
    updateLED();
}

void readSensorData() {
    unsigned long endTime = bme.beginReading();
    if (endTime == 0) {
        Serial.println("Falha ao iniciar leitura :(");
        return;
    }
    
    delay(50); // Tempo para medição do sensor

    if (!bme.endReading()) {
        Serial.println("Falha ao completar leitura :(");
        return;
    }

    // Coleta e cálculo dos dados
    sensorData.humidity = bme.humidity;
    sensorData.temperature = bme.temperature;
    sensorData.pressure = bme.pressure / 100.0F;
    sensorData.gasResistance = bme.gas_resistance / 1000.0;
    sensorData.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    sensorData.dewPoint = dewPointCalculation(sensorData.temperature, sensorData.humidity);
    if (R0 == 0)
        sensorData.razao_R = 0.0;
    else sensorData.razao_R = sensorData.gasResistance/R0;
    sensorData.airQualityLevel = calculateAirQualityLevel(sensorData.razao_R);
    sensorData.lastUpdate = millis();
}

float dewPointCalculation(float temperature, float humidity) {
    float a = 17.271;
    float b = 237.7;
    float temp = (a * temperature) / (b + temperature) + log(humidity / 100);
    return (b * temp) / (a - temp);
}

int calculateAirQualityLevel(float razao_R) {
    if (razao_R >= 0.75) return 1;      // Excelente
    else if (razao_R >= 0.40) return 2; // Moderado
    else if (razao_R >= 0.20) return 3; // Ruim
    else return 4;                       // Crítico
}

void printSensorData() {
    // Exibição no Monitor Serial
    Serial.println("--- Novas Leituras do Sensor ---");
    Serial.printf("Temperatura: %.2f °C\n", sensorData.temperature);
    Serial.printf("Umidade: %.2f %%\n", sensorData.humidity);
    Serial.printf("Pressão: %.2f hPa\n", sensorData.pressure);
    Serial.printf("Gás: %.2f KOhms\n", sensorData.gasResistance);
    Serial.printf("Altitude: %.2f m\n", sensorData.altitude);
    Serial.printf("Ponto de Orvalho: %.2f °C\n", sensorData.dewPoint);
    Serial.printf("Razão entre as resistências: %.2f\n", sensorData.razao_R);
    switch(sensorData.airQualityLevel){
        case 1:
            Serial.printf("Nível de Alerta: Bom\n");
            break;
        case 2:
            Serial.printf("Nível de Alerta: Moderado\n");
            break;
        case 3:
            Serial.printf("Nível de ALerta: Ruim\n");
            break;
        case 4:
            Serial.printf("Nível de Alerta: Crítico\n");
            break;
    }
    Serial.println("--------------------------------\n");
}

// Função para ligar o Buzzer
void activateBuzzer() {
    digitalWrite(pinBUZZER, HIGH);
    BuzzerBeginTime = millis();
    buzzerAtivo = true;
}

// Função para checar se já deve desligar o buzzer
void updateBuzzer() {
    // Checa se o buzzer está ativo por mais de 3 segundos
    if (buzzerAtivo && millis() - BuzzerBeginTime > 3000) {
        digitalWrite(pinBUZZER, LOW);
        buzzerAtivo = false;
    }
}

void calibrarSensor() {
    Serial.println("Iniciando calibração do BME688...");
    Serial.println("Mantenha o ambiente limpo. Aguarde alguns segundos.");

    int numeroLeituras = 30;
    float somaDasLeituras = 0.0;
    float leituraAtual;

    // Descarta a primeiríssima leitura, que costuma vir com lixo de memória ou fria
    readSensorData(); 
    delay(500);

    for (int i = 0; i < numeroLeituras; i++) {
        readSensorData();
        leituraAtual = sensorData.gasResistance; // fornece a resistência em kOhms
        somaDasLeituras += leituraAtual; 

        Serial.print("#"); // feedback visual para a leitura
        delay(1000); // Aguarda 1 segundo a cada leitura
    }

    // Calcula a média aritmética
    R0 = somaDasLeituras / numeroLeituras;
    
    Serial.println();
    Serial.print("Calibração concluída! R0 = ");
    Serial.print(R0);
    Serial.println(" kOhms");
}

void updateLED() {
    // Checa se o LED está ativo por mais de 5 segundos
    if (ledAtivo && (millis() - LedBeginTime > 5000)) {
        digitalWrite(pinRED, HIGH);
        digitalWrite(pinGREEN, HIGH);
        digitalWrite(pinBLUE, HIGH);
        ledAtivo = false;
    }
}

