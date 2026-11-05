/*
  HX710B (sensor de pressão) + ESP32-C3 Super Mini + BLE HID Mouse
  Versão NimBLE-Arduino 2.x com média O(1) e leitura contínua.
*/

#include <Arduino.h>
#include <esp_wifi.h>
#include <esp32-hal-cpu.h>

#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>

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
#define HID_MOUSE 0x03C2
#endif

// Protótipos
void setupBLE();
void enviarReport(uint8_t botoes, int8_t x, int8_t y);
void enviarClique();
long lerHX710B();

// ================== PINOS (ESP32-C3 Super Mini) ==================
const int PIN_OUT = 2; // DOUT do HX710B
const int PIN_SCK = 3; // SCK do HX710B

// ================== PARÂMETROS BLE ==================
const uint16_t ADV_INTERVALO = 160;         // 100 ms
const uint16_t CONN_INTERVALO_MIN = 12;     // 15 ms
const uint16_t CONN_INTERVALO_MAX = 24;     // 30 ms
const uint16_t CONN_LATENCIA = 10;          
const uint16_t CONN_TIMEOUT = 400;          // 4 s

// ================== PARÂMETROS DE DETECÇÃO ==================
const long DELTA = 60000;
const int TAMANHO_BUFFER = 20;
const unsigned long DEBOUNCE_MS = 100;
const unsigned long TIMEOUT_PRESSAO_MS = 200;
const float FATOR_RELEASE = 0.35;

// ================== BUFFER DE MÉDIA MÓVEL (SOMA ACUMULADA) ==================
long leituras[TAMANHO_BUFFER] = {0};
long somaLeituras = 0;
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

static const uint8_t reportMap[] = {
  0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01,
  0xA1, 0x00, 0x85, 0x01, 0x05, 0x09, 0x19, 0x01,
  0x29, 0x03, 0x15, 0x00, 0x25, 0x01, 0x95, 0x03,
  0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05,
  0x81, 0x03, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
  0x09, 0x38, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08,
  0x95, 0x03, 0x81, 0x06, 0xC0, 0xC0
};

class MinhasServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    conectado = true;
    LOG("\n[BLE STATUS] >>> DISPOSITIVO CONECTADO <<<");
    server->updateConnParams(connInfo.getConnHandle(), CONN_INTERVALO_MIN, CONN_INTERVALO_MAX,
                             CONN_LATENCIA, CONN_TIMEOUT);
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    conectado = false;
    LOGF("\n[BLE STATUS] >>> DESCONECTADO (motivo: %d) <<<\n", reason);
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

  LOG("[BLE] Stack HID pronta.");
}

void enviarReport(uint8_t botoes, int8_t x, int8_t y) {
  if (!conectado) return;
  uint8_t report[4] = { botoes, (uint8_t)x, (uint8_t)y, 0 };
  inputMouse->setValue(report, sizeof(report));
  inputMouse->notify();
}

void enviarClique() {
  LOG("\n>>> CLIQUE DETECTADO (HX710B) <<<");
  if (!conectado) return;

  enviarReport(0x01, 0, 0);
  delay(30);
  enviarReport(0x00, 0, 0);
}

void setup() {
  pinMode(PIN_OUT, INPUT);
  pinMode(PIN_SCK, OUTPUT);
  digitalWrite(PIN_SCK, LOW);

  LOG_BEGIN(115200);
#if DEBUG_SERIAL
  delay(1000);
#endif

  LOG("\n=== HX710B + ESP32-C3 Super Mini + NimBLE Mouse ===");

  esp_wifi_stop();
  esp_wifi_deinit();
  setCpuFrequencyMhz(80);

  setupBLE();
}

// Leitura não-bloqueante do HX710B
long lerHX710B() {
  if (digitalRead(PIN_OUT) == HIGH) {
    return 0;
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

  // 25º pulso para manter a taxa de amostragem em 10Hz
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
  long ultimaLeitura = lerHX710B();

  if (ultimaLeitura != 0) {
    if (!bufferCheio) {
      leituras[indiceBuffer] = ultimaLeitura;
      somaLeituras += ultimaLeitura;
      indiceBuffer = (indiceBuffer + 1) % TAMANHO_BUFFER;
      totalLeituras++;

      if (totalLeituras >= TAMANHO_BUFFER) {
        bufferCheio = true;
        LOG("--> Buffer cheio! Monitoramento ativo.");
      }
    } else {
      somaLeituras -= leituras[indiceBuffer];
      leituras[indiceBuffer] = ultimaLeitura;
      somaLeituras += ultimaLeitura;
      indiceBuffer = (indiceBuffer + 1) % TAMANHO_BUFFER;

      long media = somaLeituras / TAMANHO_BUFFER;
      long diferenca = abs(ultimaLeitura - media);
      bool acimaDoLimiar = (diferenca > DELTA);
      unsigned long agora = millis();

      if (acimaDoLimiar && !emEstadoPressionado) {
        if (agora - ultimoCliqueMillis > DEBOUNCE_MS) {
          enviarClique();
          ultimoCliqueMillis = agora;
        }
        emEstadoPressionado = true;
        inicioPressaoMillis = agora;
      } else if (emEstadoPressionado) {
        bool voltouAoNormal = (diferenca < (DELTA * FATOR_RELEASE));
        bool estourouTimeout = (agora - inicioPressaoMillis > TIMEOUT_PRESSAO_MS);

        if (voltouAoNormal || estourouTimeout) {
          emEstadoPressionado = false;
        }
      }
    }
  }

  delay(5);
}