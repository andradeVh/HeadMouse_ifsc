/*
  HX710B (sensor de pressão) + ESP32-C3 Super Mini + BLE HID Mouse
  Versão NimBLE-Arduino 2.x com soma acumulada O(1) na média móvel.
*/

#include <Arduino.h>
#include <esp_wifi.h>
#include <esp32-hal-cpu.h>

#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>

#include <driver/gpio.h>

// ================== DEBUG SERIAL ==================
#define DEBUG_SERIAL 1

#if DEBUG_SERIAL
  #define LOG_BEGIN(baud) Serial.begin(baud)
  #define LOG(x)          Serial.println(x)
  #define LOGF(...)       Serial.printf(__VA_ARGS__)
  #define LOG_FLUSH()     Serial.flush()
#else
  #define LOG_BEGIN(baud) ((void)0)
  #define LOG(x)          ((void)0)
  #define LOGF(...)       ((void)0)
  #define LOG_FLUSH()     ((void)0)
#endif

#ifndef HID_MOUSE
#define HID_MOUSE 0x03C2   // Appearance: Mouse
#endif

// Protótipos
void setupBLE();
void enviarReport(uint8_t botoes, int8_t x, int8_t y);
void enviarClique();
void entrarDeepSleep();
long lerHX710B();

// ================== PINOS (ESP32-C3 Super Mini) ==================
const int PIN_OUT = 2; // DOUT do HX710B
const int PIN_SCK = 3; // SCK do HX710B
const int PIN_WAKE = 4; // Botão para GND que acorda do Deep Sleep (GPIO 0-5 no C3)

// ================== PARÂMETROS BLE (economia de energia) ==================
const unsigned long ADV_TIMEOUT_MS = 60000; // Sem conexão por 60 s -> Deep Sleep
const uint16_t ADV_INTERVALO = 160;         // Unidades de 0,625 ms -> 100 ms
const uint16_t CONN_INTERVALO_MIN = 12;     // Unidades de 1,25 ms  -> 15 ms
const uint16_t CONN_INTERVALO_MAX = 24;     // 30 ms
const uint16_t CONN_LATENCIA = 10;          // Eventos que o periférico pode pular sem dados
const uint16_t CONN_TIMEOUT = 400;          // Unidades de 10 ms    -> 4 s (> 3 x 30 ms x 11)

// ================== PARÂMETROS DE DETECÇÃO ==================
const long DELTA = 60000;
const int TAMANHO_BUFFER = 20;
const unsigned long DEBOUNCE_MS = 100;
const unsigned long TIMEOUT_PRESSAO_MS = 200;
const float FATOR_RELEASE = 0.35;

// ================== BUFFER DE MÉDIA MÓVEL (SOMA ACUMULADA) ==================
long leituras[TAMANHO_BUFFER] = {0};
long somaLeituras = 0; // Guardará o acumulado O(1)
int indiceBuffer = 0;
int totalLeituras = 0;
bool bufferCheio = false;

// ================== ESTADO DO CLIQUE / DEBOUNCE ==================
bool emEstadoPressionado = false;
unsigned long ultimoCliqueMillis = 0;
unsigned long inicioPressaoMillis = 0;

// ================== BLE HID (NimBLE) ==================
NimBLEHIDDevice* hid;
NimBLECharacteristic* inputMouse;
NimBLEServer* pServer;
volatile bool conectado = false;
volatile unsigned long inicioAdvMillis = 0;

static const uint8_t reportMap[] = {
  0x05, 0x01,       // USAGE_PAGE (Generic Desktop)
  0x09, 0x02,       // USAGE (Mouse)
  0xA1, 0x01,       // COLLECTION (Application)
  0x09, 0x01,       //   USAGE (Pointer)
  0xA1, 0x00,       //   COLLECTION (Physical)
  0x85, 0x01,       //     REPORT_ID (1)
  0x05, 0x09,       //     USAGE_PAGE (Button)
  0x19, 0x01,       //     USAGE_MINIMUM (Button 1)
  0x29, 0x03,       //     USAGE_MAXIMUM (Button 3)
  0x15, 0x00,       //     LOGICAL_MINIMUM (0)
  0x25, 0x01,       //     LOGICAL_MAXIMUM (1)
  0x95, 0x03,       //     REPORT_COUNT (3)
  0x75, 0x01,       //     REPORT_SIZE (1)
  0x81, 0x02,       //     INPUT (Data,Var,Abs)
  0x95, 0x01,       //     REPORT_COUNT (1)
  0x75, 0x05,       //     REPORT_SIZE (5)
  0x81, 0x03,       //     INPUT (Cnst,Var,Abs)  -- padding
  0x05, 0x01,       //     USAGE_PAGE (Generic Desktop)
  0x09, 0x30,       //     USAGE (X)
  0x09, 0x31,       //     USAGE (Y)
  0x09, 0x38,       //     USAGE (Wheel)
  0x15, 0x81,       //     LOGICAL_MINIMUM (-127)
  0x25, 0x7F,       //     LOGICAL_MAXIMUM (127)
  0x75, 0x08,       //     REPORT_SIZE (8)
  0x95, 0x03,       //     REPORT_COUNT (3)
  0x81, 0x06,       //     INPUT (Data,Var,Rel)
  0xC0,             //   END_COLLECTION
  0xC0              // END_COLLECTION
};

class MinhasServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    conectado = true;
    LOG("\n[BLE STATUS] >>> DISPOSITIVO CONECTADO E PRONTO <<<");
    server->updateConnParams(connInfo.getConnHandle(), CONN_INTERVALO_MIN, CONN_INTERVALO_MAX,
                             CONN_LATENCIA, CONN_TIMEOUT);
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    conectado = false;
    LOGF("\n[BLE STATUS] >>> DESCONECTADO (motivo: %d). Reiniciando Advertising... <<<\n", reason);
    inicioAdvMillis = millis();
    NimBLEDevice::startAdvertising();
  }
};

void setupBLE() {
  NimBLEDevice::init("Botao Bola");

  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MinhasServerCallbacks());

  hid = new NimBLEHIDDevice(pServer);
  inputMouse = hid->getInputReport(1);

  hid->setManufacturer("Francisco");
  hid->setPnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->setHidInfo(0x00, 0x01);
  hid->setReportMap((uint8_t*)reportMap, sizeof(reportMap));
  hid->startServices();
  hid->setBatteryLevel(100);

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setAppearance(HID_MOUSE);
  pAdvertising->addServiceUUID(hid->getHidService()->getUUID());
  pAdvertising->enableScanResponse(true);
  pAdvertising->setMinInterval(ADV_INTERVALO);
  pAdvertising->setMaxInterval(ADV_INTERVALO);
  pAdvertising->start();
  inicioAdvMillis = millis();

  LOG("[BLE] Stack HID (NimBLE) pronta. Aguardando conexão do Host...");
}

void enviarReport(uint8_t botoes, int8_t x, int8_t y) {
  if (!conectado) {
    LOG("  [BLE ERR] Tentativa de envio sem conexão BLE ativa!");
    return;
  }

  uint8_t report[4] = { botoes, (uint8_t)x, (uint8_t)y, 0 };
  inputMouse->setValue(report, sizeof(report));
  inputMouse->notify();

  LOGF("  [HID TX] Payload enviado: [Botões: 0x%02X | X: %d | Y: %d]\n", report[0], x, y);
}

void enviarClique() {
  LOG("\n--------------------------------------------------");
  LOG(">>> EVENTO DE CLIQUE DETECTADO (HX710B) <<<");
  LOGF("    Instante: %lu ms | Desde o último clique: %lu ms\n", millis(), millis() - ultimoCliqueMillis);

  if (!conectado) {
    LOG(">>> [AVISO] Clique ignorado: Nenhum dispositivo BLE conectado! <<<");
    LOG("--------------------------------------------------\n");
    return;
  }

  LOG("1. Enviando Pressionar Botão Esquerdo (0x01)...");
  enviarReport(0x01, 0, 0);

  delay(30);

  LOG("2. Enviando Soltar Botão (0x00)...");
  enviarReport(0x00, 0, 0);

  LOG(">>> CLIQUE ENVIADO COM SUCESSO AO HOST BLE <<<");
  LOG("--------------------------------------------------\n");
}

void entrarDeepSleep() {
  LOGF("\n[SLEEP] Sem conexão há %lu s. Entrando em Deep Sleep...\n", ADV_TIMEOUT_MS / 1000);
  LOG_FLUSH();

  NimBLEDevice::getAdvertising()->stop();
  NimBLEDevice::deinit(true);

  digitalWrite(PIN_SCK, HIGH);
  delayMicroseconds(100);
  gpio_hold_en((gpio_num_t)PIN_SCK);
  gpio_deep_sleep_hold_en();

  gpio_pullup_en((gpio_num_t)PIN_WAKE);
  gpio_pulldown_dis((gpio_num_t)PIN_WAKE);
  esp_deep_sleep_enable_gpio_wakeup(1ULL << PIN_WAKE, ESP_GPIO_WAKEUP_GPIO_LOW);

  esp_deep_sleep_start();
}

void setup() {
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)PIN_SCK);
  gpio_reset_pin((gpio_num_t)PIN_WAKE);

  pinMode(PIN_OUT, INPUT);
  pinMode(PIN_SCK, OUTPUT);
  digitalWrite(PIN_SCK, LOW);

  LOG_BEGIN(115200);
#if DEBUG_SERIAL
  delay(1000);
#endif

  LOG("\n=== HX710B + ESP32-C3 Super Mini + NimBLE Mouse Debug ===");

  esp_wifi_stop();
  esp_wifi_deinit();
  setCpuFrequencyMhz(80);

  LOG("[INIT] Aguardando preenchimento do buffer (20 leituras)...");
  setupBLE();
}

long IRAM_ATTR lerHX710B() {
  unsigned long timeout = millis();
  while (digitalRead(PIN_OUT) == HIGH) {
    if (millis() - timeout > 500) {
      return 0;
    }
    delay(1);
  }
  long resultado = 0;
  portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&myMutex);
  for (int i = 0; i < 24; i++) {
    digitalWrite(PIN_SCK, HIGH);
    esp_rom_delay_us(1);
    resultado = resultado << 1;
    digitalWrite(PIN_SCK, LOW);
    esp_rom_delay_us(1);
    if (digitalRead(PIN_OUT) == HIGH) {
      resultado++;
    }
  }
  digitalWrite(PIN_SCK, HIGH);
  esp_rom_delay_us(1);
  digitalWrite(PIN_SCK, LOW);
  esp_rom_delay_us(1);
  portEXIT_CRITICAL(&myMutex);
  if (resultado & 0x800000) {
    resultado |= 0xFF000000;
  }
  return resultado;
}

void loop() {
  if (!conectado && (millis() - inicioAdvMillis > ADV_TIMEOUT_MS)) {
    entrarDeepSleep();
  }

  long ultimaLeitura = lerHX710B();

  if (ultimaLeitura != 0) {
    if (!bufferCheio) {
      // Preenchimento inicial do buffer
      leituras[indiceBuffer] = ultimaLeitura;
      somaLeituras += ultimaLeitura;
      indiceBuffer = (indiceBuffer + 1) % TAMANHO_BUFFER;
      totalLeituras++;

      LOGF("[BUFFER] Carregando: %d/%d (Leitura: %ld)\n", totalLeituras, TAMANHO_BUFFER, ultimaLeitura);

      if (totalLeituras >= TAMANHO_BUFFER) {
        bufferCheio = true;
        LOG("--> [BUFFER] Cheio! Monitoramento ativado.\n");
      }
    } else {
      // Atualização O(1) usando a soma acumulada
      somaLeituras -= leituras[indiceBuffer];
      leituras[indiceBuffer] = ultimaLeitura;
      somaLeituras += ultimaLeitura;
      indiceBuffer = (indiceBuffer + 1) % TAMANHO_BUFFER;

      long media = somaLeituras / TAMANHO_BUFFER;
      long diferenca = abs(ultimaLeitura - media);
      bool acimaDoLimiar = (diferenca > DELTA);
      unsigned long agora = millis();

      LOGF("Raw: %ld | Média: %ld | Diff: %ld | Limiar: %ld | Press: %s | BLE: %s\n",
                    ultimaLeitura, media, diferenca, DELTA,
                    emEstadoPressionado ? "SIM" : "NAO",
                    conectado ? "CONECTADO" : "DESCONECTADO");

      if (acimaDoLimiar && !emEstadoPressionado) {
        if (agora - ultimoCliqueMillis > DEBOUNCE_MS) {
          LOGF("\n[ALERTA DETECTADO] Diferença (%ld) > DELTA (%ld)!\n", diferenca, DELTA);
          enviarClique();
          ultimoCliqueMillis = agora;
        } else {
          LOGF("  [DEBOUNCE] Clique bloqueado por tempo. Decorrido: %lu ms (Mínimo: %lu ms)\n",
                        agora - ultimoCliqueMillis, DEBOUNCE_MS);
        }
        emEstadoPressionado = true;
        inicioPressaoMillis = agora;
      } else if (emEstadoPressionado) {
        bool voltouAoNormal = (diferenca < (DELTA * FATOR_RELEASE));
        bool estourouTimeout = (agora - inicioPressaoMillis > TIMEOUT_PRESSAO_MS);

        if (voltouAoNormal || estourouTimeout) {
          LOGF("  [REARME] Destravando estado pressionado (Voltou ao Normal: %s | Timeout: %s)\n",
                        voltouAoNormal ? "SIM" : "NAO", estourouTimeout ? "SIM" : "NAO");
          emEstadoPressionado = false;
        }
      }
    }
  }
  delay(5);
}