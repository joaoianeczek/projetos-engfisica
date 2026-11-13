#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

// =====================================================================
//  INCLUDES EXATOS DA SUA PASTA "include/"
// =====================================================================
extern "C" {
    #include "bme68x.h"
    #include "bme68x_defs.h"
    #include "bsec_datatypes.h"
    #include "bsec_interface.h"
    #include "bsec_iaq.h"
}

// ---------------- CONFIGURACAO DO HARDWARE ----------------
constexpr uint8_t PIN_R      = 4;
constexpr uint8_t PIN_G      = 5;
constexpr uint8_t PIN_B      = 18;
constexpr uint8_t PIN_BUZZER = 15;

// Limites SUPERIORES de IAQ (0-500) para Bom, Moderado e Ruim. Acima = Critico.
constexpr float   LIMITE_SUP[3]   = {100.0f, 200.0f, 300.0f};
constexpr float   HISTERESE       = 10.0f;
constexpr uint8_t ACURACIA_MINIMA = 1;

// Intervalo de backup da calibracao na bancada: 30 minutos (em milissegundos)
constexpr uint32_t INTERVALO_SALVAR_MS = 30UL * 60UL * 1000UL;

// Tamanhos seguros de buffer (previne erro caso bsec_datatypes.h seja de versao diferente da .a)
constexpr uint32_t SAFE_WORKBUFFER_SIZE = (BSEC_MAX_WORKBUFFER_SIZE > 4096) ? BSEC_MAX_WORKBUFFER_SIZE : 4096;
constexpr uint32_t SAFE_STATE_BLOB_SIZE = (BSEC_MAX_STATE_BLOB_SIZE > 1024) ? BSEC_MAX_STATE_BLOB_SIZE : 1024;

// ---------------- ESTADOS E VARIAVEIS GLOBAIS ----------------
enum Estado : uint8_t { AQUECENDO, BOM, MODERADO, RUIM, CRITICO, ERRO };
const char *NOMES[] = {"Aquecendo", "Bom", "Moderado", "Ruim", "Critico", "Erro"};

Estado estado   = AQUECENDO;
bool   sensorOk = false;

// Variaveis das leituras (prontas para enviar ao Blynk quando reativar!)
float   val_iaq       = 0.0f; // IAQ Adaptativo (memoria recente - detecta picos)
float   val_staticIaq = 0.0f; // IAQ Estatico (absoluto - mede exposicao continua)
float   val_bVoc      = 0.0f; // Concentracao equivalente de COVs em ppm
float   val_temp      = 0.0f; // Temperatura compensada (C)
float   val_umidade   = 0.0f; // Umidade compensada (%)
uint8_t val_acuracia  = 0;    // Status da calibracao (0 a 3)

// Controle do BME688 e da BSEC
struct bme68x_dev   bme;
bsec_bme_settings_t sensor_settings;
uint8_t             EnderecoI2C = BME68X_I2C_ADDR_HIGH; // Sera descoberto automaticamente
uint8_t             work_buffer[SAFE_WORKBUFFER_SIZE];
uint8_t             serialized_state[SAFE_STATE_BLOB_SIZE];

// Controle da Memoria Flash (NVS) e Relogio de 64 bits em nanossegundos
Preferences preferencias;
uint32_t    ultimoSaveMs         = 0;
uint8_t     maiorAcuraciaSalva   = 0;
uint8_t     maiorAcuraciaTentada = 0;
uint32_t    totalSavesSessao     = 0;
uint32_t    ultimoMillis         = 0;
int64_t     tempoAcumuladoNs     = 0;

// =====================================================================
//  1. CALLBACKS I2C E AUTO-DETECCAO DE ENDERECO (0x77 E 0x76)
// =====================================================================
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) return -1;
    Wire.requestFrom(dev_addr, (uint8_t)len);
    for (uint32_t i = 0; i < len && Wire.available(); i++) {
        reg_data[i] = Wire.read();
    }
    return BME68X_OK;
}

BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.write(reg_data, len);
    if (Wire.endTransmission() != 0) return -1;
    return BME68X_OK;
}

void bme68x_delay_us(uint32_t period, void *intf_ptr) {
    delayMicroseconds(period);
}

// Testa automaticamente os dois enderecos I2C possiveis do BME688 (0x77 e 0x76)
int8_t inicializarSensorAutoI2C() {
    const uint8_t enderecosPossiveis[2] = {BME68X_I2C_ADDR_HIGH, BME68X_I2C_ADDR_LOW}; // 0x77 e 0x76

    for (uint8_t i = 0; i < 2; i++) {
        EnderecoI2C = enderecosPossiveis[i];

        Wire.beginTransmission(EnderecoI2C);
        if (Wire.endTransmission() == 0) {
            bme.intf     = BME68X_I2C_INTF;
            bme.read     = bme68x_i2c_read;
            bme.write    = bme68x_i2c_write;
            bme.delay_us = bme68x_delay_us;
            bme.intf_ptr = &EnderecoI2C;
            bme.amb_temp = 25;

            int8_t rslt = bme68x_init(&bme);
            if (rslt == BME68X_OK) {
                Serial.printf("[I2C] BME688 encontrado e inicializado no endereco 0x%02X!\n", EnderecoI2C);
                return BME68X_OK;
            }
        }
    }

    Serial.println("[I2C] ERRO: BME688 nao respondeu nem em 0x77 nem em 0x76!");
    return BME68X_E_DEV_NOT_FOUND;
}

// Gera timestamp em nanossegundos (int64_t) sem risco de overflow do millis()
int64_t getTimestampNs() {
    uint32_t agora = millis();
    tempoAcumuladoNs += (int64_t)(agora - ultimoMillis) * 1000000LL;
    ultimoMillis = agora;
    return tempoAcumuladoNs;
}

// =====================================================================
//  2. FUNCOES DE MEMORIA FLASH (CARREGAR NO BOOT E SALVAR CALIBRACAO)
// =====================================================================
void carregarCalibracaoDaMemoria() {
    if (!preferencias.begin("bsec_mem", false)) {
        Serial.println("[MEMORIA] Erro ao abrir o namespace NVS.");
        return;
    }

    if (preferencias.isKey("state")) {
        size_t tam = preferencias.getBytesLength("state");
        if (tam > 0 && tam <= SAFE_STATE_BLOB_SIZE) {
            preferencias.getBytes("state", serialized_state, tam);
            bsec_library_return_t status = bsec_set_state(
                serialized_state, tam, work_buffer, SAFE_WORKBUFFER_SIZE
            );
            if (status >= BSEC_OK) {
                maiorAcuraciaSalva   = preferencias.getUChar("saved_acc", 1);
                maiorAcuraciaTentada = maiorAcuraciaSalva;
                ultimoSaveMs         = millis();
                Serial.println("==========================================================");
                Serial.printf("[MEMORIA] Calibracao restaurada da Flash! (%u bytes | acc salva: %u | status: %d)\n",
                              (uint32_t)tam, maiorAcuraciaSalva, status);
                Serial.println("==========================================================");
            } else {
                Serial.printf("[MEMORIA] Estado salvo invalido (erro %d). Iniciando do zero.\n", status);
            }
        }
    } else {
        Serial.println("[MEMORIA] Nenhuma calibracao salva encontrada. Iniciando do zero.");
    }
    preferencias.end();
}

void salvarCalibracaoNaMemoria(bool forcarAgora = false) {
    uint32_t agora = millis();

    // Usa maiorAcuraciaTentada para nao ficar preso num loop infinito se bsec_get_state falhar
    bool subiuDeNivel    = (val_acuracia > maiorAcuraciaTentada);
    bool salvarPeriodico = (val_acuracia >= 1 && (agora - ultimoSaveMs >= INTERVALO_SALVAR_MS));

    if (forcarAgora || subiuDeNivel || salvarPeriodico) {
        if (val_acuracia > maiorAcuraciaTentada) {
            maiorAcuraciaTentada = val_acuracia;
        }
        ultimoSaveMs = agora;

        uint32_t n_serialized_state = 0;

        // Tentativa 1: Usa os buffers expandidos seguros (SAFE_STATE_BLOB_SIZE / SAFE_WORKBUFFER_SIZE)
        bsec_library_return_t status = bsec_get_state(
            0, serialized_state, SAFE_STATE_BLOB_SIZE,
            work_buffer, SAFE_WORKBUFFER_SIZE, &n_serialized_state
        );

        // Tentativa 2: Se a lib exigir exatamente a constante do header, tenta com BSEC_MAX_STATE_BLOB_SIZE
        if (status < BSEC_OK) {
            status = bsec_get_state(
                0, serialized_state, BSEC_MAX_STATE_BLOB_SIZE,
                work_buffer, BSEC_MAX_WORKBUFFER_SIZE, &n_serialized_state
            );
        }

        if (status >= BSEC_OK && n_serialized_state > 0) {
            if (preferencias.begin("bsec_mem", false)) {
                size_t gravados = preferencias.putBytes("state", serialized_state, n_serialized_state);
                if (val_acuracia > maiorAcuraciaSalva) {
                    maiorAcuraciaSalva = val_acuracia;
                }
                preferencias.putUChar("saved_acc", maiorAcuraciaSalva);
                preferencias.end();

                totalSavesSessao++;

                const char *motivo = forcarAgora ? "Comando manual ('s')" :
                                     (subiuDeNivel ? "Subiu de nivel de acuracia" : "Backup periodico (30 min)");

                Serial.println("\n**********************************************************");
                Serial.printf(">>> [FLASH NVS] CALIBRACAO SALVA COM SUCESSO! (Save #%u) <<<\n", totalSavesSessao);
                Serial.printf(">>> Motivo: %s\n", motivo);
                Serial.printf(">>> Status BSEC: %d | Acuracia atual: %u | Gravados: %u/%u bytes\n",
                              status, val_acuracia, (uint32_t)gravados, n_serialized_state);
                Serial.println("**********************************************************\n");
            } else {
                Serial.println("\n[ERRO FLASH] Falha ao abrir NVS 'bsec_mem' para escrita!\n");
            }
        } else {
            Serial.printf("\n[AVISO BSEC] bsec_get_state indisponivel (status=%d | tam=%u | Header MAX_STATE=%u, MAX_WORK=%u).\n",
                          status, n_serialized_state, (uint32_t)BSEC_MAX_STATE_BLOB_SIZE, (uint32_t)BSEC_MAX_WORKBUFFER_SIZE);
        }
    }
}

void inspecionarMemoriaFlash() {
    if (!preferencias.begin("bsec_mem", true)) {
        Serial.println("\n[INSPECAO FLASH] Namespace 'bsec_mem' ainda nao existe na NVS.\n");
        return;
    }
    bool temState = preferencias.isKey("state");
    size_t tam    = temState ? preferencias.getBytesLength("state") : 0;
    uint8_t acc   = preferencias.getUChar("saved_acc", 0);
    preferencias.end();

    Serial.println("\n------------------ STATUS DA FLASH NVS ------------------");
    Serial.printf("Chave 'state' existe? %s | Tamanho: %u bytes | Maior acc salva: %u | Saves nesta sessao: %u\n",
                  temState ? "SIM" : "NAO", (uint32_t)tam, acc, totalSavesSessao);
    Serial.printf("Constantes do Header -> BSEC_MAX_STATE_BLOB_SIZE: %u | BSEC_MAX_WORKBUFFER_SIZE: %u\n",
                  (uint32_t)BSEC_MAX_STATE_BLOB_SIZE, (uint32_t)BSEC_MAX_WORKBUFFER_SIZE);
    Serial.println("---------------------------------------------------------\n");
}

// =====================================================================
//  3. LOGICA DE CLASSIFICACAO, HISTERESE, LEDS, BUZZER E SERIAL
// =====================================================================
bool ciclo(uint32_t agora, uint32_t periodo, uint32_t ligado) {
    return (agora % periodo) < ligado;
}

Estado classificar(float iaq, Estado atual) {
    Estado novo = CRITICO;
    for (uint8_t i = 0; i < 3; i++) {
        if (iaq <= LIMITE_SUP[i]) {
            novo = (Estado)(BOM + i);
            break;
        }
    }
    if (atual >= BOM && atual <= CRITICO && novo < atual &&
        iaq > LIMITE_SUP[atual - BOM - 1] - HISTERESE) {
        novo = atual;
    }
    return novo;
}

void atualizarEstado() {
    if (val_acuracia < ACURACIA_MINIMA) {
        estado = AQUECENDO;
    } else {
        float iaqParaAlerta = (val_iaq > val_staticIaq) ? val_iaq : val_staticIaq;
        estado = classificar(iaqParaAlerta, estado);
    }
}

void imprimir() {
    uint32_t decorridoMs = millis() - ultimoSaveMs;
    uint32_t restanteSeg = (decorridoMs < INTERVALO_SALVAR_MS)
                           ? ((INTERVALO_SALVAR_MS - decorridoMs) / 1000UL)
                           : 0;
    uint32_t minRest = restanteSeg / 60UL;
    uint32_t segRest = restanteSeg % 60UL;

    Serial.printf("IAQ: %.0f | sIAQ: %.0f (acc %u) | bVOC: %.2f ppm | T: %.1f C | UR: %.1f%% | %s | [Flash: acc %u | Save #%u | Prox: %02um%02us]\n",
                  val_iaq, val_staticIaq, val_acuracia, val_bVoc,
                  val_temp, val_umidade, NOMES[estado],
                  maiorAcuraciaSalva, totalSavesSessao, minRest, segRest);
}

void atualizarSaidas(uint32_t agora) {
    bool r = false, g = false, b = false, buzzer = false;

    switch (estado) {
        case AQUECENDO:
            b = ciclo(agora, 2000, 1000);
            break;
        case BOM:
            g = true;
            break;
        case MODERADO:
            r = true;
            g = true;
            break;
        case RUIM:
            r = true;
            break;
        case CRITICO:
            r      = ciclo(agora, 1000, 500);
            buzzer = ciclo(agora, 1000, 200);
            break;
        case ERRO:
            r = g = b = ciclo(agora, 400, 200);
            break;
    }

    digitalWrite(PIN_R, r);
    digitalWrite(PIN_G, g);
    digitalWrite(PIN_B, b);
    digitalWrite(PIN_BUZZER, buzzer);
}

// =====================================================================
//  4. PASSO DE LEITURA DO SENSOR + PROCESSAMENTO BSEC
// =====================================================================
bool executarPassoBsec() {
    int64_t time_stamp = getTimestampNs();

    if (time_stamp < sensor_settings.next_call) {
        return false;
    }

    bsec_library_return_t bsec_status = bsec_sensor_control(time_stamp, &sensor_settings);
    if (bsec_status < BSEC_OK) {
        Serial.printf("[ERRO] bsec_sensor_control: %d\n", bsec_status);
        sensorOk = false;
        return false;
    }

    if (sensor_settings.trigger_measurement == 1) {
        struct bme68x_conf       conf;
        struct bme68x_heatr_conf heatr_conf;

        conf.os_hum  = sensor_settings.humidity_oversampling;
        conf.os_temp = sensor_settings.temperature_oversampling;
        conf.os_pres = sensor_settings.pressure_oversampling;
        conf.filter  = BME68X_FILTER_OFF;
        conf.odr     = BME68X_ODR_NONE;
        bme68x_set_conf(&conf, &bme);

        heatr_conf.enable     = sensor_settings.run_gas;
        heatr_conf.heatr_temp = sensor_settings.heater_temperature;
        heatr_conf.heatr_dur  = sensor_settings.heater_duration;
        bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);

        bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);

        uint32_t duracao_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme)
                              + ((uint32_t)heatr_conf.heatr_dur * 1000UL);
        delay((duracao_us / 1000UL) + 5);
    }

    if (sensor_settings.process_data != 0) {
        struct bme68x_data data;
        uint8_t n_fields = 0;

        if (bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme) == BME68X_OK && n_fields > 0) {
            bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];
            uint8_t n_inputs = 0;

            if (sensor_settings.process_data & BSEC_PROCESS_TEMPERATURE) {
                inputs[n_inputs].sensor_id  = BSEC_INPUT_TEMPERATURE;
                inputs[n_inputs].signal     = data.temperature;
                inputs[n_inputs].time_stamp = time_stamp;
                n_inputs++;
            }
            if (sensor_settings.process_data & BSEC_PROCESS_HUMIDITY) {
                inputs[n_inputs].sensor_id  = BSEC_INPUT_HUMIDITY;
                inputs[n_inputs].signal     = data.humidity;
                inputs[n_inputs].time_stamp = time_stamp;
                n_inputs++;
            }
            if (sensor_settings.process_data & BSEC_PROCESS_PRESSURE) {
                inputs[n_inputs].sensor_id  = BSEC_INPUT_PRESSURE;
                inputs[n_inputs].signal     = data.pressure;
                inputs[n_inputs].time_stamp = time_stamp;
                n_inputs++;
            }
            if ((sensor_settings.process_data & BSEC_PROCESS_GAS) && (data.status & BME68X_GASM_VALID_MSK)) {
                inputs[n_inputs].sensor_id  = BSEC_INPUT_GASRESISTOR;
                inputs[n_inputs].signal     = data.gas_resistance;
                inputs[n_inputs].time_stamp = time_stamp;
                n_inputs++;
            }

            if (n_inputs > 0) {
                bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
                uint8_t n_outputs = BSEC_NUMBER_OUTPUTS;

                bsec_status = bsec_do_steps(inputs, n_inputs, outputs, &n_outputs);
                if (bsec_status >= BSEC_OK && n_outputs > 0) {
                    for (uint8_t i = 0; i < n_outputs; i++) {
                        switch (outputs[i].sensor_id) {
                            case BSEC_OUTPUT_IAQ:
                                val_iaq      = outputs[i].signal;
                                val_acuracia = outputs[i].accuracy;
                                break;
                            case BSEC_OUTPUT_STATIC_IAQ:
                                val_staticIaq = outputs[i].signal;
                                break;
                            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                                val_bVoc = outputs[i].signal;
                                break;
                            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                                val_temp = outputs[i].signal;
                                break;
                            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                                val_umidade = outputs[i].signal;
                                break;
                        }
                    }
                    return true;
                }
            }
        }
    }
    return false;
}

// =====================================================================
//  5. SETUP E LOOP PRINCIPAL
// =====================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_R, OUTPUT);
    pinMode(PIN_G, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);

    Wire.begin();

    int8_t rslt = inicializarSensorAutoI2C();

    bsec_library_return_t bsec_status = bsec_init();
    bsec_version_t ver;
    bsec_get_version(&ver);
    Serial.printf("[BSEC] Versao da lib: %u.%u.%u.%u | bsec_init status: %d\n",
                  ver.major, ver.minor, ver.major_bugfix, ver.minor_bugfix, bsec_status);

    if (rslt == BME68X_OK && bsec_status == BSEC_OK) {
        bsec_status = bsec_set_configuration(
            bsec_config_iaq, sizeof(bsec_config_iaq),
            work_buffer, SAFE_WORKBUFFER_SIZE
        );

        if (bsec_status != BSEC_OK && sizeof(bsec_config_iaq) > 4) {
            bsec_status = bsec_set_configuration(
                &bsec_config_iaq[4], sizeof(bsec_config_iaq) - 4,
                work_buffer, SAFE_WORKBUFFER_SIZE
            );
        }

        if (bsec_status != BSEC_OK) {
            Serial.printf("[BSEC] Aviso: bsec_iaq.h incompativel com a lib v%u.%u.%u.%u (cod %d). Usando config interna padrao.\n",
                          ver.major, ver.minor, ver.major_bugfix, ver.minor_bugfix, bsec_status);
        } else {
            Serial.println("[BSEC] Configuracao bsec_iaq.h aplicada com sucesso!");
        }

        carregarCalibracaoDaMemoria();

        bsec_sensor_configuration_t requested_virtual_sensors[5];
        bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
        uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

        requested_virtual_sensors[0].sensor_id   = BSEC_OUTPUT_IAQ;
        requested_virtual_sensors[0].sample_rate = BSEC_SAMPLE_RATE_LP;

        requested_virtual_sensors[1].sensor_id   = BSEC_OUTPUT_STATIC_IAQ;
        requested_virtual_sensors[1].sample_rate = BSEC_SAMPLE_RATE_LP;

        requested_virtual_sensors[2].sensor_id   = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
        requested_virtual_sensors[2].sample_rate = BSEC_SAMPLE_RATE_LP;

        requested_virtual_sensors[3].sensor_id   = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
        requested_virtual_sensors[3].sample_rate = BSEC_SAMPLE_RATE_LP;

        requested_virtual_sensors[4].sensor_id   = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
        requested_virtual_sensors[4].sample_rate = BSEC_SAMPLE_RATE_LP;

        bsec_status = bsec_update_subscription(
            requested_virtual_sensors, 5,
            required_sensor_settings, &n_required_sensor_settings
        );
        Serial.printf("[BSEC] bsec_update_subscription status: %d\n", bsec_status);

        if (bsec_status >= BSEC_OK) {
            sensorOk = true;
            sensor_settings.next_call = 0;
        }
    }

    estado = sensorOk ? AQUECENDO : ERRO;
    if (sensorOk) {
        Serial.println("BSEC iniciado! Aguardando leituras (a cada 3s)...");
        Serial.println("[DICA] Digite 's' no terminal para forcar o salvamento na Flash ou 'r' para inspecionar a memoria.");
    } else {
        Serial.printf("Falha ao iniciar (I2C rslt=%d | BSEC status=%d).\n", rslt, bsec_status);
    }
}

void loop() {
    while (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 's' || cmd == 'S') {
            Serial.println("\n[COMANDO] Forcando salvamento manual na Flash...");
            salvarCalibracaoNaMemoria(true);
        } else if (cmd == 'r' || cmd == 'R') {
            inspecionarMemoriaFlash();
        }
    }

    if (sensorOk) {
        if (executarPassoBsec()) {
            atualizarEstado();
            salvarCalibracaoNaMemoria(false);
            imprimir();
        } else if (!sensorOk) {
            estado = ERRO;
        }
    }
    atualizarSaidas(millis());
}